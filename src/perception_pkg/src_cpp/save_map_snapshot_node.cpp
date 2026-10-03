#include "perception_pkg/save_map_snapshot_node.hpp"

#include <cmath>
#include <filesystem>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

namespace perception_pkg
{

MapSnapshotSaver::MapSnapshotSaver(const rclcpp::NodeOptions & options)
: Node("map_snapshot_saver", options)
{
  this->declare_parameter("maps_dir", std::string(""));
  this->declare_parameter("session_dir", std::string(""));
  this->declare_parameter("filename_prefix", std::string("slam_map"));
  this->declare_parameter("update_latest_symlink", true);
  this->declare_parameter("save_on_shutdown", true);
  this->declare_parameter("save_interval_sec", 90.0);
  this->declare_parameter("cache_interval_sec", 5.0);
  this->declare_parameter("save_settle_sec", 2.5);
  this->declare_parameter("stop_robot_before_save", true);
  this->declare_parameter("map_frame", std::string("map"));
  this->declare_parameter("base_frame", std::string("base_footprint"));

  maps_root_ = this->get_parameter("maps_dir").as_string();
  session_dir_ = this->get_parameter("session_dir").as_string();
  prefix_ = this->get_parameter("filename_prefix").as_string();
  update_latest_ = this->get_parameter("update_latest_symlink").as_bool();
  save_on_shutdown_ = this->get_parameter("save_on_shutdown").as_bool();
  save_interval_ = this->get_parameter("save_interval_sec").as_double();
  cache_interval_ = this->get_parameter("cache_interval_sec").as_double();
  save_settle_sec_ = std::max(this->get_parameter("save_settle_sec").as_double(), 0.0);
  stop_robot_before_save_ = this->get_parameter("stop_robot_before_save").as_bool();
  map_frame_ = this->get_parameter("map_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();

  if (maps_root_.empty()) {
    const char * env = std::getenv("PERCEPTION_MAPS_DIR");
    if (env && env[0] != '\0') {
      maps_root_ = env;
    } else {
      try {
        auto pkg_share = ament_index_cpp::get_package_share_directory("perception_pkg");
        fs::path ws_root = fs::path(pkg_share).parent_path().parent_path().parent_path().parent_path();
        auto src_maps = ws_root / "src" / "perception_pkg" / "maps";
        if (fs::is_directory(src_maps)) {
          maps_root_ = src_maps.string();
        } else {
          maps_root_ = (fs::path(pkg_share) / "maps").string();
        }
      } catch (...) {
        maps_root_ = (fs::current_path() / "maps").string();
      }
    }
  }

  if (session_dir_.empty()) {
    const char * env = std::getenv("PERCEPTION_MAP_SESSION");
    if (env && env[0] != '\0') {
      session_dir_ = env;
    }
  }

  cache_dir_ = session_dir_.empty()
    ? (fs::path(maps_root_) / "_cache").string()
    : (fs::path(session_dir_) / "_cache").string();

  if (!session_dir_.empty()) {
    fs::create_directories(session_dir_);
    write_session_meta(session_dir_, maps_root_);
    setenv("PERCEPTION_MAP_SESSION", session_dir_.c_str(), 1);
    RCLCPP_INFO(this->get_logger(), "Session dir: %s", session_dir_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Maps dir: %s", maps_root_.c_str());
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  finalize_pub_ = this->create_publisher<std_msgs::msg::Empty>("/perception/mapping_finalized", 10);
  auto stop_qos = rclcpp::QoS(1).reliability(rclcpp::ReliabilityPolicy::Reliable);
  cmd_vel_teleop_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_teleop", stop_qos);
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", stop_qos);

  auto latched = rclcpp::QoS(1)
    .durability(rclcpp::DurabilityPolicy::TransientLocal)
    .reliability(rclcpp::ReliabilityPolicy::Reliable);
  auto default_qos = rclcpp::QoS(10)
    .durability(rclcpp::DurabilityPolicy::Volatile)
    .reliability(rclcpp::ReliabilityPolicy::Reliable);

  map_sub_latched_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", latched, std::bind(&MapSnapshotSaver::on_map, this, std::placeholders::_1));
  map_sub_default_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", default_qos, std::bind(&MapSnapshotSaver::on_map, this, std::placeholders::_1));

  save_service_ = this->create_service<std_srvs::srv::Trigger>(
    "save_map",
    std::bind(&MapSnapshotSaver::handle_save_map, this, std::placeholders::_1, std::placeholders::_2));

  if (save_interval_ > 0.0) {
    save_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(save_interval_),
      [this]() {
        if (last_map_) {
          RCLCPP_INFO(this->get_logger(), "Periodic map backup...");
          save_now(false, false);
        }
      });
  }

  pose_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&MapSnapshotSaver::update_pose_cache, this));
}

void MapSnapshotSaver::on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  last_map_ = msg;
  double now = this->get_clock()->now().seconds();
  if (now - last_cache_time_ >= cache_interval_) {
    last_cache_time_ = now;
    try {
      fs::create_directories(cache_dir_);
      auto pgm_path = (fs::path(cache_dir_) / "latest.pgm").string();
      auto yaml_path = (fs::path(cache_dir_) / "latest.yaml").string();
      write_pgm(pgm_path, *msg);
      write_yaml(yaml_path, "latest.pgm", *msg);
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Cache write failed: %s", e.what());
    }
  }
}

void MapSnapshotSaver::update_pose_cache()
{
  if (session_dir_.empty()) return;

  auto pose = lookup_pose(*tf_buffer_, map_frame_, base_frame_);
  if (!pose.has_value()) return;
  last_pose_ = pose.value();

  auto out = (fs::path(session_dir_) / "initial_pose.yaml").string();
  write_initial_pose_yaml(out, pose->x, pose->y, pose->yaw);
  save_map_odom_offset();
}

bool MapSnapshotSaver::handle_save_map(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  bool ok = save_now(true, true);
  res->success = ok;
  if (ok && !session_dir_.empty()) {
    res->message = "saved to " + session_dir_;
  } else {
    res->message = ok ? "saved" : "no /map yet";
  }
  return ok;
}

bool MapSnapshotSaver::save_now(bool write_pose, bool finalize)
{
  if (finalize) stop_robot_and_settle();
  if (!last_map_) {
    RCLCPP_WARN(this->get_logger(), "No /map received, skipping save");
    return false;
  }

  auto out_dir = session_dir_.empty() ? maps_root_ : session_dir_;
  fs::create_directories(out_dir);

  auto stem = "slam_map";
  auto pgm_name = std::string(stem) + ".pgm";
  auto pgm_path = (fs::path(out_dir) / pgm_name).string();
  auto yaml_path = (fs::path(out_dir) / (std::string(stem) + ".yaml")).string();

  try {
    write_pgm(pgm_path, *last_map_);
    write_yaml(yaml_path, pgm_name, *last_map_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Map save failed: %s", e.what());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Map saved: %s", yaml_path.c_str());
  save_map_odom_offset();

  if (write_pose) save_initial_pose();
  if (finalize) signal_finalize();
  return true;
}

void MapSnapshotSaver::save_initial_pose()
{
  if (session_dir_.empty()) return;
  auto out = (fs::path(session_dir_) / "initial_pose.yaml").string();
  if (last_pose_.has_value()) {
    write_initial_pose_yaml(out, last_pose_->x, last_pose_->y, last_pose_->yaw);
    RCLCPP_INFO(this->get_logger(), "Initial pose saved: %s", out.c_str());
    return;
  }
  auto pose = lookup_pose(*tf_buffer_, map_frame_, base_frame_);
  if (pose.has_value()) {
    last_pose_ = pose.value();
    write_initial_pose_yaml(out, pose->x, pose->y, pose->yaw);
    RCLCPP_INFO(this->get_logger(), "Initial pose saved: %s", out.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "Cannot save initial_pose: TF lookup failed");
  }
}

void MapSnapshotSaver::save_map_odom_offset()
{
  if (session_dir_.empty()) return;
  try {
    auto tf = tf_buffer_->lookupTransform(
      map_frame_, "odom", tf2::TimePointZero,
      tf2::durationFromSec(0.5));
    double x = tf.transform.translation.x;
    double y = tf.transform.translation.y;
    auto & q = tf.transform.rotation;
    double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    auto out = (fs::path(session_dir_) / "map_odom_offset.yaml").string();
    write_map_odom_offset_yaml(out, x, y, yaw);
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "Cannot save map_odom_offset: %s", e.what());
  }
}

void MapSnapshotSaver::signal_finalize()
{
  if (!session_dir_.empty()) {
    auto flag_path = fs::path(session_dir_) / ".mapping_finalized";
    try {
      auto now = std::chrono::system_clock::now();
      auto t_c = std::chrono::system_clock::to_time_t(now);
      std::stringstream ss;
      ss << std::put_time(std::localtime(&t_c), "%Y-%m-%dT%H:%M:%S");
      std::ofstream ofs(flag_path.string());
      ofs << ss.str();
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "Cannot write finalize flag: %s", e.what());
    }
  }
  try {
    finalize_pub_->publish(std_msgs::msg::Empty());
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "Finalize topic publish failed: %s", e.what());
  }
}

void MapSnapshotSaver::stop_robot_and_settle()
{
  if (!stop_robot_before_save_) return;
  auto zero = geometry_msgs::msg::Twist();
  for (int i = 0; i < 12; ++i) {
    cmd_vel_teleop_pub_->publish(zero);
    cmd_vel_pub_->publish(zero);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

// --- Static file I/O helpers ---

void MapSnapshotSaver::write_pgm(const std::string & path, const nav_msgs::msg::OccupancyGrid & msg)
{
  uint32_t w = msg.info.width, h = msg.info.height;
  std::vector<uint8_t> rows(w * h);
  for (uint32_t y = 0; y < h; ++y) {
    for (uint32_t x = 0; x < w; ++x) {
      uint32_t src = y * w + x;
      uint32_t dst = (h - y - 1) * w + x;
      int8_t occ = msg.data[src];
      if (occ < 0) rows[dst] = 205;
      else if (occ == 0) rows[dst] = 254;
      else rows[dst] = 0;
    }
  }
  std::ofstream ofs(path, std::ios::binary);
  ofs << "P5\n" << w << " " << h << "\n255\n";
  ofs.write(reinterpret_cast<const char *>(rows.data()), rows.size());
}

void MapSnapshotSaver::write_yaml(
  const std::string & path, const std::string & pgm_name,
  const nav_msgs::msg::OccupancyGrid & msg)
{
  auto o = msg.info.origin.position;
  std::ofstream ofs(path);
  ofs << "image: " << pgm_name << "\n"
      << "mode: trinary\n"
      << "resolution: " << std::fixed << std::setprecision(3) << msg.info.resolution << "\n"
      << "origin: [" << std::fixed << std::setprecision(3) << o.x << ", " << o.y << ", " << o.z << "]\n"
      << "negate: 0\n"
      << "occupied_thresh: 0.65\n"
      << "free_thresh: 0.196\n";
}

void MapSnapshotSaver::write_initial_pose_yaml(
  const std::string & path, double x, double y, double yaw)
{
  auto now = std::chrono::system_clock::now();
  auto t_c = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&t_c), "%Y-%m-%dT%H:%M:%S");

  std::ofstream ofs(path);
  ofs << "frame_id: map\n"
      << "x: " << std::fixed << std::setprecision(4) << x << "\n"
      << "y: " << y << "\n"
      << "yaw: " << yaw << "\n"
      << "cov_xx: 0.05\n"
      << "cov_yy: 0.05\n"
      << "cov_aa: 0.02\n"
      << "saved_at: " << ss.str() << "\n"
      << "source: tf_map_base_footprint\n";
}

void MapSnapshotSaver::write_map_odom_offset_yaml(
  const std::string & path, double x, double y, double yaw)
{
  auto now = std::chrono::system_clock::now();
  auto t_c = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&t_c), "%Y-%m-%dT%H:%M:%S");

  std::ofstream ofs(path);
  ofs << "map_frame: map\n"
      << "odom_frame: odom\n"
      << "x: " << std::fixed << std::setprecision(4) << x << "\n"
      << "y: " << y << "\n"
      << "yaw: " << yaw << "\n"
      << "saved_at: " << ss.str() << "\n"
      << "note: 'GT in odom frame -> map: p_map = R(yaw)*p_odom + [x,y]'\n";
}

void MapSnapshotSaver::write_session_meta(
  const std::string & session_dir, const std::string & maps_root)
{
  auto now = std::chrono::system_clock::now();
  auto t_c = std::chrono::system_clock::to_time_t(now);
  std::stringstream ss;
  ss << std::put_time(std::localtime(&t_c), "%Y-%m-%dT%H:%M:%S");

  auto meta_path = (fs::path(session_dir) / "session_meta.json").string();
  std::ofstream ofs(meta_path);
  ofs << "{\n"
      << "  \"session_id\": \"" << fs::path(session_dir).filename().string() << "\",\n"
      << "  \"created_at\": \"" << ss.str() << "\",\n"
      << "  \"maps_dir\": \"" << session_dir << "\",\n"
      << "  \"bag_dir\": \"" << (fs::path(session_dir) / "bag").string() << "\",\n"
      << "  \"map_yaml\": \"slam_map.yaml\",\n"
      << "  \"map_png\": \"slam_map.png\",\n"
      << "  \"waypoints_yaml\": \"waypoints.yaml\",\n"
      << "  \"map_waypoints_png\": \"slam_map_waypoints.png\",\n"
      << "  \"initial_pose_yaml\": \"initial_pose.yaml\"\n"
      << "}\n";

  update_map_latest_symlink(maps_root, fs::path(session_dir).filename().string());
}

void MapSnapshotSaver::update_map_latest_symlink(
  const std::string & maps_root, const std::string & session_name)
{
  auto link = fs::path(maps_root) / "map_latest";
  std::error_code ec;
  if (fs::is_symlink(link) || fs::exists(link, ec)) {
    fs::remove(link, ec);
  }
  fs::create_directory_symlink(session_name, link, ec);
}

std::optional<Pose3> MapSnapshotSaver::lookup_pose(
  tf2_ros::Buffer & tf_buffer, const std::string & map_frame,
  const std::string & base_frame)
{
  try {
    auto tf = tf_buffer.lookupTransform(
      map_frame, base_frame, tf2::TimePointZero,
      tf2::durationFromSec(0.1));
    double x = tf.transform.translation.x;
    double y = tf.transform.translation.y;
    auto & q = tf.transform.rotation;
    double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    return Pose3{x, y, yaw};
  } catch (...) {
    return std::nullopt;
  }
}

}  // namespace perception_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<perception_pkg::MapSnapshotSaver>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
