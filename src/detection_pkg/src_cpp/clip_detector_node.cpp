#include "detection_pkg/clip_detector_node.hpp"

#include "detection_pkg/map_overlay.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

#include <opencv2/imgproc.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

namespace detection_pkg
{

namespace
{

double monotonic_now()
{
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

cv::Mat image_msg_to_bgr(const sensor_msgs::msg::Image & msg)
{
  const int height = static_cast<int>(msg.height);
  const int width = static_cast<int>(msg.width);
  std::string encoding = msg.encoding;
  std::transform(encoding.begin(), encoding.end(), encoding.begin(), ::tolower);

  if (encoding == "rgb8") {
    cv::Mat rgb(height, width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()));
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    return bgr.clone();
  }
  if (encoding == "bgr8") {
    cv::Mat bgr(height, width, CV_8UC3, const_cast<uint8_t *>(msg.data.data()));
    return bgr.clone();
  }
  if (encoding == "rgba8") {
    cv::Mat rgba(height, width, CV_8UC4, const_cast<uint8_t *>(msg.data.data()));
    cv::Mat bgr;
    cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
    return bgr.clone();
  }
  if (encoding == "mono8") {
    cv::Mat gray(height, width, CV_8UC1, const_cast<uint8_t *>(msg.data.data()));
    cv::Mat bgr;
    cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
    return bgr.clone();
  }
  throw std::runtime_error("Unsupported image encoding: " + msg.encoding);
}

sensor_msgs::msg::Image bgr_to_image_msg(const cv::Mat & bgr, const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::Image msg;
  msg.header = header;
  msg.height = static_cast<uint32_t>(bgr.rows);
  msg.width = static_cast<uint32_t>(bgr.cols);
  msg.encoding = "bgr8";
  msg.is_bigendian = false;
  msg.step = static_cast<uint32_t>(bgr.cols * 3);
  msg.data.assign(bgr.datastart, bgr.dataend);
  return msg;
}

std::string format_timestamp()
{
  std::time_t t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S");
  return oss.str();
}

}  // namespace

ClipDetectorNode::ClipDetectorNode(const rclcpp::NodeOptions & options)
: Node("clip_detector", options),
  yolo_(0.5, 640),
  camera_qos_(rclcpp::QoS(10).reliable().keep_last(10))
{
  this->declare_parameter<std::string>("session_dir", "");
  this->declare_parameter<std::string>("weights", "");
  this->declare_parameter<double>("conf_threshold", 0.5);
  this->declare_parameter<double>("max_fps", 8.0);
  this->declare_parameter<double>("ground_z", 0.002);
  this->declare_parameter<double>("dedup_radius_m", 0.45);
  this->declare_parameter<double>("save_dedup_radius_m", 0.65);
  this->declare_parameter<double>("bearing_merge_deg", 6.0);
  this->declare_parameter<double>("merge_max_blend", 0.2);
  this->declare_parameter<double>("outlier_reject_m", 0.5);
  this->declare_parameter<double>("max_angular_for_wp", 0.12);
  this->declare_parameter<double>("min_linear_for_wp", 0.04);
  this->declare_parameter<double>("max_linear_for_wp", 0.20);
  this->declare_parameter<double>("motion_settle_sec", 0.4);
  this->declare_parameter<bool>("enable_motion_gate", false);
  this->declare_parameter<bool>("use_odom_motion_gate", false);
  this->declare_parameter<double>("max_odom_angular_for_wp", 0.15);
  this->declare_parameter<bool>("tf_prefer_image_stamp", true);
  this->declare_parameter<double>("stereo_sync_slop_sec", 0.12);
  this->declare_parameter<bool>("allow_ground_fallback", true);
  this->declare_parameter<double>("stereo_ground_crosscheck_m", 0.6);
  this->declare_parameter<int>("max_tf_lag_ms", 250);
  this->declare_parameter<bool>("enable_motion_compensation", true);
  this->declare_parameter<double>("odom_buffer_duration_sec", 2.0);
  this->declare_parameter<int>("odom_interp_max_gap_ms", 80);
  this->declare_parameter<int>("max_tf_extrapolate_ms", 120);
  this->declare_parameter<double>("waypoint_save_interval_sec", 1.0);
  this->declare_parameter<double>("marker_scale_m", 0.08);
  this->declare_parameter<bool>("enable_reid", true);
  this->declare_parameter<std::string>("reid_backend", "osnet");
  this->declare_parameter<double>("reid_cosine_min", 0.42);
  this->declare_parameter<int>("reid_vocab_size", 48);
  this->declare_parameter<double>("reid_match_score_min", 0.42);
  this->declare_parameter<int>("track_buffer_size", 30);
  this->declare_parameter<double>("sigma_floor_m", 0.08);
  this->declare_parameter<double>("pixel_merge_u_px", 48.0);
  this->declare_parameter<double>("pixel_merge_v_px", 36.0);
  this->declare_parameter<double>("track_uv_ema", 0.15);
  this->declare_parameter<double>("track_map_ema", 0.3);
  this->declare_parameter<double>("stereo_baseline_m", 0.0755);
  this->declare_parameter<double>("min_depth_m", 0.6);
  this->declare_parameter<double>("max_depth_m", 3.0);
  this->declare_parameter<double>("min_disparity_px", 2.0);
  this->declare_parameter<double>("max_ground_z_error", 0.15);
  this->declare_parameter<double>("max_map_radius_m", 8.0);
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<std::string>("base_frame", "base_footprint");

  std::string session = this->get_parameter("session_dir").as_string();
  if (session.empty()) {
    const char * env = std::getenv("PERCEPTION_MAP_SESSION");
    if (env) {
      session = env;
    }
  }
  if (!session.empty()) {
    session_dir_ = std::filesystem::weakly_canonical(std::filesystem::path(session));
  }

  conf_ = this->get_parameter("conf_threshold").as_double();
  max_fps_ = this->get_parameter("max_fps").as_double();
  ground_z_ = this->get_parameter("ground_z").as_double();
  dedup_radius_ = this->get_parameter("dedup_radius_m").as_double();
  save_dedup_radius_ = std::max(
    this->get_parameter("save_dedup_radius_m").as_double(), dedup_radius_);
  bearing_merge_deg_ = std::max(this->get_parameter("bearing_merge_deg").as_double(), 1.0);
  merge_max_blend_ = std::min(this->get_parameter("merge_max_blend").as_double(), 1.0);
  outlier_reject_m_ = std::max(this->get_parameter("outlier_reject_m").as_double(), 0.1);
  max_angular_for_wp_ = std::abs(this->get_parameter("max_angular_for_wp").as_double());
  min_linear_for_wp_ = std::max(this->get_parameter("min_linear_for_wp").as_double(), 0.0);
  max_linear_for_wp_ = std::max(this->get_parameter("max_linear_for_wp").as_double(), 0.05);
  motion_settle_sec_ = std::max(this->get_parameter("motion_settle_sec").as_double(), 0.0);
  enable_motion_gate_ = this->get_parameter("enable_motion_gate").as_bool();
  use_odom_motion_gate_ = this->get_parameter("use_odom_motion_gate").as_bool();
  max_odom_angular_for_wp_ = std::max(
    this->get_parameter("max_odom_angular_for_wp").as_double(), 0.05);
  tf_prefer_image_stamp_ = this->get_parameter("tf_prefer_image_stamp").as_bool();
  stereo_sync_slop_ = std::max(this->get_parameter("stereo_sync_slop_sec").as_double(), 0.01);
  allow_ground_fallback_ = this->get_parameter("allow_ground_fallback").as_bool();
  stereo_ground_crosscheck_m_ = std::max(
    this->get_parameter("stereo_ground_crosscheck_m").as_double(), 0.1);
  max_tf_lag_ms_ = std::max<int>(this->get_parameter("max_tf_lag_ms").as_int(), 100);
  enable_motion_compensation_ = this->get_parameter("enable_motion_compensation").as_bool();

  if (enable_motion_compensation_) {
    const double odom_buffer_sec = std::max(
      this->get_parameter("odom_buffer_duration_sec").as_double(), 0.5);
    const int64_t interp_gap = std::max(
      static_cast<int64_t>(this->get_parameter("odom_interp_max_gap_ms").as_int()), static_cast<int64_t>(10));
    const int64_t max_extrap = std::max(
      static_cast<int64_t>(this->get_parameter("max_tf_extrapolate_ms").as_int()), static_cast<int64_t>(0));
    odom_buffer_ = std::make_unique<OdomRingBuffer>(
      odom_buffer_sec, interp_gap * 1'000'000LL, max_extrap * 1'000'000LL);
  }

  waypoint_save_interval_ = std::max(
    this->get_parameter("waypoint_save_interval_sec").as_double(), 0.5);
  map_frame_ = this->get_parameter("map_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  marker_scale_ = this->get_parameter("marker_scale_m").as_double();
  enable_reid_ = this->get_parameter("enable_reid").as_bool();
  track_buffer_size_ = std::max<int>(this->get_parameter("track_buffer_size").as_int(), 4);
  sigma_floor_m_ = std::max(this->get_parameter("sigma_floor_m").as_double(), 0.01);
  pixel_merge_u_ = std::max(this->get_parameter("pixel_merge_u_px").as_double(), 4.0);
  pixel_merge_v_ = std::max(this->get_parameter("pixel_merge_v_px").as_double(), 4.0);
  track_uv_ema_ = std::clamp(this->get_parameter("track_uv_ema").as_double(), 0.01, 1.0);
  track_map_ema_ = std::clamp(this->get_parameter("track_map_ema").as_double(), 0.05, 1.0);
  stereo_baseline_m_ = std::max(this->get_parameter("stereo_baseline_m").as_double(), 0.01);
  min_depth_m_ = std::max(this->get_parameter("min_depth_m").as_double(), 0.1);
  max_depth_m_ = std::max(
    this->get_parameter("max_depth_m").as_double(), min_depth_m_ + 0.1);
  min_disparity_px_ = std::max(this->get_parameter("min_disparity_px").as_double(), 0.5);
  max_ground_z_error_ = std::max(this->get_parameter("max_ground_z_error").as_double(), 0.01);
  max_map_radius_m_ = std::max(this->get_parameter("max_map_radius_m").as_double(), 1.0);

  if (enable_reid_) {
    reid_.emplace(
      this->get_parameter("reid_backend").as_string(),
      this->get_parameter("reid_cosine_min").as_double(),
      this->get_parameter("reid_vocab_size").as_int(),
      this->get_parameter("reid_match_score_min").as_double());
    std::string msg = "re-ID enabled (backend=" + reid_->active_backend();
    if (!reid_->fallback_reason().empty()) {
      msg += ", orb fallback: " + reid_->fallback_reason();
    }
    msg += ")";
    RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
  }

  if (this->has_parameter("use_sim_time")) {
    use_sim_time_ = this->get_parameter("use_sim_time").as_bool();
  } else {
    use_sim_time_ = true;
  }
  yolo_.set_conf_threshold(conf_);
  node_start_mono_ = monotonic_now();
  const std::string weights = this->get_parameter("weights").as_string();
  start_yolo_init_async(weights);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  wp_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/detection/waypoints", 10);
  img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/detection/annotated_image", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/detection/markers", 10);

  finalize_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "/perception/mapping_finalized", 10,
    std::bind(&ClipDetectorNode::on_finalize, this, std::placeholders::_1));

  left_preview_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/left/image_raw", camera_qos_,
    std::bind(&ClipDetectorNode::on_left_preview, this, std::placeholders::_1));

  left_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    this, "/camera/left/image_raw", camera_qos_.get_rmw_qos_profile());
  right_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
    this, "/camera/right/image_raw", camera_qos_.get_rmw_qos_profile());
  stereo_sync_ = std::make_shared<message_filters::Synchronizer<StereoSyncPolicy>>(
    StereoSyncPolicy(30), *left_sub_, *right_sub_);
  stereo_sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(stereo_sync_slop_));
  stereo_sync_->registerCallback(
    std::bind(
      &ClipDetectorNode::on_stereo_images, this, std::placeholders::_1, std::placeholders::_2));

  left_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/left/camera_info", camera_qos_,
    std::bind(&ClipDetectorNode::on_left_info, this, std::placeholders::_1));
  right_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/right/camera_info", camera_qos_,
    std::bind(&ClipDetectorNode::on_right_info, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&ClipDetectorNode::on_odom, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, std::bind(&ClipDetectorNode::on_cmd_vel, this, std::placeholders::_1));
  teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_teleop", 10, std::bind(&ClipDetectorNode::on_cmd_vel_teleop, this, std::placeholders::_1));

  if (waypoint_save_interval_ > 0.0) {
    flush_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(waypoint_save_interval_),
      std::bind(&ClipDetectorNode::periodic_waypoint_flush, this));
  }
  marker_timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&ClipDetectorNode::republish_markers, this));

  load_waypoints_from_disk();
  if (!waypoints_.empty()) {
    publish_waypoints(true);
  }

  if (session_dir_.has_value()) {
    RCLCPP_INFO(this->get_logger(), "Session dir: %s", session_dir_->string().c_str());
  }
  RCLCPP_INFO(
    this->get_logger(),
    "clip_detector_cpp ready (use_sim_time=%s, motion_gate=%s, motion_comp=%s)",
    use_sim_time_ ? "true" : "false",
    enable_motion_gate_ ? "on" : "off",
    enable_motion_compensation_ ? "on" : "off");
}

cv::Mat ClipDetectorNode::tf_to_matrix(const geometry_msgs::msg::Transform & tf) const
{
  tf2::Quaternion q(
    tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w);
  tf2::Matrix3x3 m(q);
  cv::Mat rot = (cv::Mat_<double>(3, 3) <<
    m[0][0], m[0][1], m[0][2],
    m[1][0], m[1][1], m[1][2],
    m[2][0], m[2][1], m[2][2]);
  cv::Mat trans = (cv::Mat_<double>(3, 1) << tf.translation.x, tf.translation.y, tf.translation.z);
  return make_transform(rot, trans);
}

rclcpp::Time ClipDetectorNode::query_time_from_stamp(const builtin_interfaces::msg::Time & stamp)
{
  const rclcpp::Time stamp_time(stamp);
  const rclcpp::Time now = this->get_clock()->now();
  return stamp_time <= now ? stamp_time : now;
}

std::pair<rclcpp::Time, int> ClipDetectorNode::sync_query_time_for(
  const std::string & target, const std::string & source,
  const builtin_interfaces::msg::Time & stamp)
{
  const rclcpp::Time stamp_time = query_time_from_stamp(stamp);
  const int64_t stamp_ns = stamp_time.nanoseconds();
  try {
    const rclcpp::Time common = tf_buffer_->lookupTransform(
      target, source, tf2::TimePointZero).header.stamp;
    const int64_t common_ns = rclcpp::Time(common).nanoseconds();
    if (common_ns > 0 && stamp_ns > common_ns) {
      return {rclcpp::Time(common_ns), static_cast<int>((stamp_ns - common_ns) / 1'000'000)};
    }
  } catch (const tf2::TransformException &) {
  }
  return {stamp_time, 0};
}

cv::Mat ClipDetectorNode::tf_matrix_at(
  const std::string & target, const std::string & source, const rclcpp::Time & query_time)
{
  const auto tf = tf_buffer_->lookupTransform(
    target, source, query_time, tf2::durationFromSec(0.25));
  return tf_to_matrix(tf.transform);
}

std::optional<cv::Mat> ClipDetectorNode::static_base_to_camera_tf(const std::string & camera_link)
{
  if (t_base_cam_cache_.count(camera_link) > 0) {
    return t_base_cam_cache_.at(camera_link);
  }
  try {
    const cv::Mat mat = tf_matrix_at(base_frame_, camera_link, rclcpp::Time(0, 0, RCL_ROS_TIME));
    t_base_cam_cache_[camera_link] = mat;
    return mat;
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

std::optional<cv::Mat> ClipDetectorNode::lookup_camera_tf_composed(
  const builtin_interfaces::msg::Time & stamp, const std::string & camera_link)
{
  if (!odom_buffer_ || odom_buffer_->size() == 0) {
    return std::nullopt;
  }
  const auto t_base_cam = static_base_to_camera_tf(camera_link);
  if (!t_base_cam.has_value()) {
    return std::nullopt;
  }
  const rclcpp::Time stamp_time = query_time_from_stamp(stamp);
  const auto t_odom_base = odom_buffer_->pose_at(stamp_time.nanoseconds());
  if (!t_odom_base.has_value()) {
    return std::nullopt;
  }
  last_odom_extrapolate_ms_ = odom_buffer_->last_extrapolate_ms;
  const auto [q_mo, lag_map_odom] = sync_query_time_for(map_frame_, "odom", stamp);
  try {
    const cv::Mat t_map_odom = tf_matrix_at(map_frame_, "odom", q_mo);
    last_tf_lag_ms_ = std::max(last_tf_lag_ms_, lag_map_odom);
    return compose_map_camera_tf(t_map_odom, t_odom_base.value(), t_base_cam.value());
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

std::tuple<std::optional<cv::Mat>, std::string, std::string, int, int, int>
ClipDetectorNode::lookup_camera_tf_legacy(
  const builtin_interfaces::msg::Time & stamp, const std::string & camera_link)
{
  std::optional<cv::Mat> result;
  std::string strategy;
  std::string stamp_err;
  int lag_map_cam = 0;
  int lag_odom_cam = 0;
  int lag_map_odom = 0;

  if (tf_prefer_image_stamp_) {
    const auto [q_full, lag] = sync_query_time_for(map_frame_, camera_link, stamp);
    lag_map_cam = lag;
    try {
      result = tf_matrix_at(map_frame_, camera_link, q_full);
      strategy = "sync_full";
    } catch (const tf2::TransformException & exc) {
      stamp_err = exc.what();
    }
  }

  if (!result.has_value() && tf_prefer_image_stamp_) {
    try {
      const auto [q_oc, lag1] = sync_query_time_for("odom", camera_link, stamp);
      const auto [q_mo, lag2] = sync_query_time_for(map_frame_, "odom", stamp);
      lag_odom_cam = lag1;
      lag_map_odom = lag2;
      const cv::Mat t_odom_cam = tf_matrix_at("odom", camera_link, q_oc);
      const cv::Mat t_map_odom = tf_matrix_at(map_frame_, "odom", q_mo);
      result = t_map_odom * t_odom_cam;
      strategy = "split_per_link";
      if (!warned_tf_split_) {
        warned_tf_split_ = true;
        RCLCPP_INFO(this->get_logger(), "Using split-chain TF lookup");
      }
    } catch (const tf2::TransformException & exc) {
      if (stamp_err.empty()) {
        stamp_err = exc.what();
      }
    }
  }

  if (!result.has_value()) {
    try {
      result = tf_matrix_at(map_frame_, camera_link, this->get_clock()->now());
      strategy = "latest_full";
      if (tf_prefer_image_stamp_ && !warned_tf_fallback_) {
        warned_tf_fallback_ = true;
        RCLCPP_WARN(
          this->get_logger(), "%s->%s split failed, falling back to latest TF",
          map_frame_.c_str(), camera_link.c_str());
      }
    } catch (const tf2::TransformException & exc) {
      stamp_err = stamp_err.empty() ? exc.what() : stamp_err;
    }
  }

  return {result, strategy, stamp_err, lag_map_cam, lag_odom_cam, lag_map_odom};
}

std::optional<cv::Mat> ClipDetectorNode::lookup_camera_tf(
  const builtin_interfaces::msg::Time & stamp, const std::string & camera_link)
{
  const int64_t stamp_ns = stamp_to_ns(stamp);
  const int64_t clock_ns = this->get_clock()->now().nanoseconds();
  last_pipeline_delay_ms_ = static_cast<int>(std::max<int64_t>(0, (clock_ns - stamp_ns) / 1'000'000));
  last_odom_extrapolate_ms_ = 0;
  last_tf_lag_ms_ = 0;

  std::optional<cv::Mat> result;
  if (enable_motion_compensation_) {
    result = lookup_camera_tf_composed(stamp, camera_link);
  }
  if (!result.has_value()) {
    const auto [mat, strategy, err, lag1, lag2, lag3] = lookup_camera_tf_legacy(stamp, camera_link);
    (void)strategy;
    (void)err;
    result = mat;
    last_tf_lag_ms_ = std::max({last_tf_lag_ms_, lag1, lag2, lag3});
  }
  return result;
}

void ClipDetectorNode::on_left_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  std::array<double, 9> k{};
  std::copy(msg->k.begin(), msg->k.end(), k.begin());
  k_left_ = camera_matrix_from_info(k);
  have_k_left_ = true;
  if (!warned_missing_info_ && have_k_right_) {
    warned_missing_info_ = true;
    RCLCPP_INFO(this->get_logger(), "camera_info ready (left+right)");
  }
}

void ClipDetectorNode::on_right_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  std::array<double, 9> k{};
  std::copy(msg->k.begin(), msg->k.end(), k.begin());
  k_right_ = camera_matrix_from_info(k);
  have_k_right_ = true;
  if (!warned_missing_info_ && have_k_left_) {
    warned_missing_info_ = true;
    RCLCPP_INFO(this->get_logger(), "camera_info ready (left+right)");
  }
}

void ClipDetectorNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  odom_omega_ = msg->twist.twist.angular.z;
  odom_linear_ = msg->twist.twist.linear.x;
  if (odom_buffer_) {
    odom_buffer_->add_from_odometry(*msg);
  }
  mark_fast_motion_if_needed();
}

void ClipDetectorNode::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  cmd_linear_ = msg->linear.x;
  cmd_angular_ = msg->angular.z;
  mark_fast_motion_if_needed();
}

void ClipDetectorNode::on_cmd_vel_teleop(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  teleop_linear_ = msg->linear.x;
  teleop_angular_ = msg->angular.z;
  mark_fast_motion_if_needed();
}

std::pair<double, double> ClipDetectorNode::effective_motion() const
{
  return {
    std::max(std::abs(cmd_linear_), std::abs(teleop_linear_)),
    std::max(std::abs(cmd_angular_), std::abs(teleop_angular_)),
  };
}

void ClipDetectorNode::mark_fast_motion_if_needed()
{
  const auto [v, w] = effective_motion();
  if (v > max_linear_for_wp_ || w > max_angular_for_wp_) {
    last_fast_motion_mono_ = monotonic_now();
    return;
  }
  if (use_odom_motion_gate_ && std::abs(odom_omega_) > max_odom_angular_for_wp_) {
    last_fast_motion_mono_ = monotonic_now();
  }
}

bool ClipDetectorNode::motion_allows_waypoint()
{
  if (!enable_motion_gate_) {
    return true;
  }
  const auto [v, w] = effective_motion();
  if (w > max_angular_for_wp_) {
    return false;
  }
  if (v > max_linear_for_wp_) {
    return false;
  }
  if (use_odom_motion_gate_ && std::abs(odom_omega_) > max_odom_angular_for_wp_) {
    return false;
  }
  if (motion_settle_sec_ > 0.0 &&
    monotonic_now() - last_fast_motion_mono_ < motion_settle_sec_)
  {
    return false;
  }
  return true;
}

void ClipDetectorNode::load_waypoints_from_disk()
{
  if (!session_dir_.has_value()) {
    return;
  }
  const auto wp_path = session_dir_.value() / "waypoints.yaml";
  if (!std::filesystem::is_regular_file(wp_path)) {
    return;
  }
  try {
    const YAML::Node data = YAML::LoadFile(wp_path.string());
    if (data["waypoints"] && data["waypoints"].IsSequence()) {
      for (const auto & item : data["waypoints"]) {
        MapWaypoint wp;
        wp.x = item["x"].as<double>();
        wp.y = item["y"].as<double>();
        wp.confidence = item["confidence"].as<double>(0.5);
        wp.source = item["source"].as<std::string>("file");
        waypoints_.push_back(wp);
        tracks_.emplace_back(track_buffer_size_, track_uv_ema_, track_map_ema_);
      }
    }
    if (!waypoints_.empty()) {
      const auto merged = deduplicate_waypoints(waypoints_, dedup_radius_);
      if (merged.size() < waypoints_.size()) {
        waypoints_ = merged;
        tracks_.resize(waypoints_.size());
      }
      RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from disk", waypoints_.size());
    }
  } catch (const std::exception & exc) {
    RCLCPP_WARN(this->get_logger(), "Failed to load waypoints.yaml: %s", exc.what());
  }
}

void ClipDetectorNode::health_check()
{
  if (!yolo_ready_.load() && !yolo_init_failed_.load()) {
    if (monotonic_now() - node_start_mono_ >= 10.0 && !warned_yolo_loading_) {
      warned_yolo_loading_ = true;
      RCLCPP_WARN(
        this->get_logger(),
        "YOLO still loading in background (OpenCV DNN / TensorRT). "
        "Prebuild engine: bash scripts/convert_to_trt_engine.sh");
    }
    return;
  }
  if (yolo_init_failed_.load()) {
    return;
  }
  if (stereo_frame_count_ > 0 || warned_no_frames_) {
    return;
  }
  if (monotonic_now() - node_start_mono_ < 10.0) {
    return;
  }
  warned_no_frames_ = true;
  RCLCPP_ERROR(
    this->get_logger(),
    "No stereo frames received in 10s. Check Gazebo camera topics and use_sim_time.");
}

void ClipDetectorNode::republish_markers()
{
  health_check();
  if (!waypoints_.empty()) {
    publish_waypoints(true);
  }
}

void ClipDetectorNode::on_left_preview(const sensor_msgs::msg::Image::SharedPtr msg)
{
  ++left_preview_count_;
  if (left_preview_count_ == 1) {
    RCLCPP_INFO(this->get_logger(), "Received left preview; publishing /detection/annotated_image");
  }
  const double now = monotonic_now();
  if (now - last_preview_time_ < 0.4) {
    return;
  }
  last_preview_time_ = now;
  try {
    const cv::Mat bgr = image_msg_to_bgr(*msg);
    publish_annotated(bgr, msg->header);
  } catch (const std::exception &) {
  }
}

std::vector<DetectionBox> ClipDetectorNode::nms_boxes(const std::vector<DetectionBox> & boxes)
{
  return nms_detection_boxes(boxes);
}

WaypointTrack ClipDetectorNode::new_track()
{
  return WaypointTrack(track_buffer_size_, track_uv_ema_, track_map_ema_);
}

WaypointTrack & ClipDetectorNode::ensure_track(std::size_t index)
{
  while (tracks_.size() <= index) {
    tracks_.push_back(new_track());
  }
  return tracks_[index];
}

std::pair<std::optional<std::size_t>, double> ClipDetectorNode::nearest_waypoint_index(
  double map_x, double map_y)
{
  if (waypoints_.empty()) {
    return {std::nullopt, std::numeric_limits<double>::infinity()};
  }
  std::size_t best_idx = 0;
  double best_dist = std::hypot(map_x - waypoints_[0].x, map_y - waypoints_[0].y);
  for (std::size_t i = 1; i < waypoints_.size(); ++i) {
    const double dist = std::hypot(map_x - waypoints_[i].x, map_y - waypoints_[i].y);
    if (dist < best_dist) {
      best_dist = dist;
      best_idx = i;
    }
  }
  return {best_idx, best_dist};
}

bool ClipDetectorNode::apply_fused_track(std::size_t index, const Observation & obs)
{
  auto & track = ensure_track(index);
  const std::optional<MapWaypoint> prev =
    (index < waypoints_.size()) ? std::optional<MapWaypoint>(waypoints_[index]) : std::nullopt;
  if (prev.has_value()) {
    const double jump = std::hypot(obs.map_x - prev->x, obs.map_y - prev->y);
    if (jump > outlier_reject_m_) {
      ++merge_outlier_blocked_;
      return false;
    }
  }
  waypoints_[index] = track.fuse_map_1sigma(obs, sigma_floor_m_, prev);
  return true;
}

void ClipDetectorNode::add_waypoint(
  double u, double v, double range_m, double confidence, const std::string & source,
  const std::optional<cv::Mat> & crop_bgr, const cv::Mat & t_map_camera)
{
  if (last_tf_lag_ms_ > max_tf_lag_ms_) {
    ++tf_lag_high_;
  }
  if (!motion_allows_waypoint()) {
    ++motion_blocked_frames_;
    return;
  }
  if (!have_k_left_) {
    return;
  }

  const auto obs = observation_from_detection(
    u, v, range_m, confidence, source, k_left_, t_map_camera);
  if (!obs.has_value()) {
    ++wp_obs_none_;
    return;
  }

  const auto [best_idx, best_dist] = nearest_waypoint_index(obs->map_x, obs->map_y);
  if (best_idx.has_value() && best_dist <= dedup_radius_) {
    if (apply_fused_track(best_idx.value(), obs.value())) {
      ++merge_spatial_;
      if (reid_.has_value() && crop_bgr.has_value() && !crop_bgr->empty()) {
        reid_->register_waypoint(static_cast<int>(best_idx.value()), crop_bgr.value());
      }
    }
    return;
  }

  if (best_idx.has_value() && best_dist < save_dedup_radius_) {
    ++merge_ambiguous_skip_;
    return;
  }

  auto track = new_track();
  const MapWaypoint wp = track.fuse_map_1sigma(obs.value(), sigma_floor_m_, std::nullopt);
  waypoints_.push_back(wp);
  tracks_.push_back(std::move(track));
  ++merge_new_;
  const int new_idx = static_cast<int>(waypoints_.size()) - 1;
  if (reid_.has_value() && crop_bgr.has_value() && !crop_bgr->empty()) {
    reid_->register_waypoint(new_idx, crop_bgr.value());
  }
}

std::optional<std::tuple<double, double, double, std::string>> ClipDetectorNode::ground_hit_map(
  double u, double v, const cv::Mat & t_map_camera)
{
  const cv::Vec3d ray_optical = pixel_to_ray_optical(u, v, k_left_);
  const cv::Mat t_map_optical = t_map_camera * camera_optical_fix();
  const cv::Vec3d origin(
    t_map_optical.at<double>(0, 3),
    t_map_optical.at<double>(1, 3),
    t_map_optical.at<double>(2, 3));
  cv::Mat rot = t_map_optical(cv::Rect(0, 0, 3, 3));
  cv::Mat dir_mat = rot * (cv::Mat_<double>(3, 1) << ray_optical[0], ray_optical[1], ray_optical[2]);
  const cv::Vec3d direction(dir_mat.at<double>(0), dir_mat.at<double>(1), dir_mat.at<double>(2));
  const auto hit = ray_ground_intersection(origin, direction, ground_z_);
  if (!hit.has_value()) {
    return std::nullopt;
  }
  const auto range_m = validate_map_point(
    hit.value(), t_map_camera, min_depth_m_, max_depth_m_, max_ground_z_error_, ground_z_,
    max_map_radius_m_);
  if (!range_m.has_value()) {
    return std::nullopt;
  }
  return std::make_tuple(hit->operator[](0), hit->operator[](1), range_m.value(), std::string("ground"));
}

bool ClipDetectorNode::stereo_range_crosscheck(
  const DetectionBox & lb, const DetectionBox & rb, const cv::Mat & t_left,
  const cv::Mat & t_right, double ground_range_m)
{
  const double fx = k_left_.at<double>(0, 0);
  const auto point = triangulate_stereo(
    lb.center_u(), lb.center_v(), rb.center_u(), rb.center_v(),
    k_left_, k_right_, t_left, t_right);
  if (!point.has_value()) {
    return true;
  }
  const auto stereo_range = validate_stereo_point(
    point.value(), lb.center_u(), rb.center_u(), fx, stereo_baseline_m_, t_left,
    min_depth_m_, max_depth_m_, max_ground_z_error_, min_disparity_px_, ground_z_, max_map_radius_m_);
  if (!stereo_range.has_value()) {
    return true;
  }
  if (std::abs(stereo_range.value() - ground_range_m) > stereo_ground_crosscheck_m_) {
    ++stereo_reject_frames_;
    return false;
  }
  return true;
}

void ClipDetectorNode::process_stereo_detection(
  const DetectionBox & lb, const DetectionBox & rb, const cv::Mat & left_bgr,
  const cv::Mat & t_left, const cv::Mat & t_right)
{
  const cv::Mat patch = crop_detection_patch(left_bgr, lb.x1, lb.y1, lb.x2, lb.y2);
  if (!allow_ground_fallback_) {
    ++wp_depth_skip_;
    return;
  }
  const auto mapped = ground_hit_map(lb.center_u(), lb.center_v(), t_left);
  if (!mapped.has_value()) {
    ++wp_depth_skip_;
    return;
  }
  const double range_m = std::get<2>(mapped.value());
  if (!stereo_range_crosscheck(lb, rb, t_left, t_right, range_m)) {
    return;
  }
  const std::optional<cv::Mat> patch_opt = patch.empty() ? std::nullopt : std::optional<cv::Mat>(patch);
  add_waypoint(
    lb.center_u(), lb.center_v(), range_m, 0.5 * (lb.confidence + rb.confidence),
    std::get<3>(mapped.value()), patch_opt, t_left);
}

void ClipDetectorNode::process_mono_detection(
  const DetectionBox & lb, const cv::Mat & left_bgr, const cv::Mat & t_left)
{
  if (!allow_ground_fallback_) {
    ++wp_depth_skip_;
    return;
  }
  const cv::Mat patch = crop_detection_patch(left_bgr, lb.x1, lb.y1, lb.x2, lb.y2);
  const auto mapped = ground_hit_map(lb.center_u(), lb.center_v(), t_left);
  if (!mapped.has_value()) {
    ++wp_depth_skip_;
    return;
  }
  const std::optional<cv::Mat> patch_opt = patch.empty() ? std::nullopt : std::optional<cv::Mat>(patch);
  add_waypoint(
    lb.center_u(), lb.center_v(), std::get<2>(mapped.value()), lb.confidence,
    std::get<3>(mapped.value()), patch_opt, t_left);
}

cv::Mat ClipDetectorNode::draw_boxes(const cv::Mat & bgr, const std::vector<DetectionBox> & boxes)
{
  cv::Mat annotated = bgr.clone();
  for (const auto & box : boxes) {
    cv::rectangle(
      annotated,
      cv::Point(static_cast<int>(box.x1), static_cast<int>(box.y1)),
      cv::Point(static_cast<int>(box.x2), static_cast<int>(box.y2)),
      cv::Scalar(0, 255, 0), 2);
    const std::string label = cv::format("%.2f", box.confidence);
    cv::putText(
      annotated, label,
      cv::Point(static_cast<int>(box.x1), std::max(0, static_cast<int>(box.y1) - 4)),
      cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
  }
  return annotated;
}

void ClipDetectorNode::publish_annotated(const cv::Mat & bgr, const std_msgs::msg::Header & header)
{
  try {
    img_pub_->publish(bgr_to_image_msg(bgr, header));
  } catch (const std::exception & exc) {
    RCLCPP_WARN(this->get_logger(), "annotated image publish failed: %s", exc.what());
  }
}

void ClipDetectorNode::on_stereo_images(
  const sensor_msgs::msg::Image::ConstSharedPtr & left_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & right_msg)
{
  if (!yolo_ready_.load()) {
    return;
  }
  if (!have_k_left_ || !have_k_right_) {
    if (!warned_missing_info_) {
      RCLCPP_WARN(this->get_logger(), "Waiting for camera_info from both cameras");
      warned_missing_info_ = true;
    }
    return;
  }

  ++stereo_frame_count_;
  const double now = monotonic_now();
  const double min_interval = 1.0 / std::max(max_fps_, 0.1);
  if (now - last_infer_time_ < min_interval) {
    return;
  }
  last_infer_time_ = now;

  cv::Mat left_bgr;
  cv::Mat right_bgr;
  try {
    left_bgr = image_msg_to_bgr(*left_msg);
    right_bgr = image_msg_to_bgr(*right_msg);
  } catch (const std::exception & exc) {
    RCLCPP_WARN(this->get_logger(), "image decode failed: %s", exc.what());
    return;
  }

  auto left_boxes = nms_boxes(yolo_.detect(left_bgr));
  (void)right_bgr;
  auto right_boxes = nms_boxes(yolo_.detect(right_bgr));
  publish_annotated(draw_boxes(left_bgr, left_boxes), left_msg->header);

  const auto t_left = lookup_camera_tf(left_msg->header.stamp, "camera_left_link");
  const auto t_right = lookup_camera_tf(right_msg->header.stamp, "camera_right_link");
  if (!t_left.has_value() || !t_right.has_value()) {
    if (!warned_tf_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Cannot query %s->camera TF; detections visible but waypoints need SLAM map frame",
        map_frame_.c_str());
      warned_tf_ = true;
    }
    publish_waypoints(true);
    return;
  }

  const double fx = k_left_.at<double>(0, 0);
  const double disp_max = max_disparity_px(fx, stereo_baseline_m_, min_depth_m_);
  const auto pairs = match_stereo_detections(
    left_boxes, right_boxes, 8.0, 0.5, 2.0, min_disparity_px_, disp_max);

  if (stereo_frame_count_ == 1 || stereo_frame_count_ % 40 == 0) {
    const auto [v, w] = effective_motion();
    RCLCPP_INFO(
      this->get_logger(),
      "stereo frame %d: L=%zu R=%zu pairs=%zu wps=%zu motion=%s v=%.2f w=%.2f",
      stereo_frame_count_, left_boxes.size(), right_boxes.size(), pairs.size(), waypoints_.size(),
      motion_allows_waypoint() ? "OK" : "BLOCK", v, w);
    motion_blocked_frames_ = 0;
    stereo_reject_frames_ = 0;
    merge_spatial_ = 0;
    merge_ambiguous_skip_ = 0;
    merge_new_ = 0;
    merge_outlier_blocked_ = 0;
    wp_obs_none_ = 0;
    wp_depth_skip_ = 0;
    tf_lag_high_ = 0;
  }

  std::vector<bool> paired_left(left_boxes.size(), false);
  for (std::size_t i = 0; i < pairs.size(); ++i) {
    const auto & lb = pairs[i].first;
    const auto & rb = pairs[i].second;
    for (std::size_t j = 0; j < left_boxes.size(); ++j) {
      if (left_boxes[j].center_u() == lb.center_u() && left_boxes[j].center_v() == lb.center_v()) {
        paired_left[j] = true;
        break;
      }
    }
    process_stereo_detection(lb, rb, left_bgr, t_left.value(), t_right.value());
  }

  for (std::size_t j = 0; j < left_boxes.size(); ++j) {
    if (!paired_left[j]) {
      process_mono_detection(left_boxes[j], left_bgr, t_left.value());
    }
  }

  publish_waypoints(true);
}

void ClipDetectorNode::publish_waypoints(bool live)
{
  (void)live;
  geometry_msgs::msg::PoseArray msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = map_frame_;
  for (const auto & wp : waypoints_) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = wp.x;
    pose.position.y = wp.y;
    pose.position.z = ground_z_;
    pose.orientation.w = 1.0;
    msg.poses.push_back(pose);
  }
  wp_pub_->publish(msg);
  publish_markers(waypoints_);
}

void ClipDetectorNode::publish_markers(const std::vector<MapWaypoint> & wps)
{
  visualization_msgs::msg::MarkerArray arr;
  const auto stamp = this->get_clock()->now();
  for (std::size_t idx = 0; idx < wps.size(); ++idx) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = stamp;
    m.ns = "clip_waypoints";
    m.id = static_cast<int>(idx);
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = wps[idx].x;
    m.pose.position.y = wps[idx].y;
    m.pose.position.z = ground_z_;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = marker_scale_;
    m.color.r = 0.1f;
    m.color.g = 0.95f;
    m.color.b = 0.1f;
    m.color.a = 0.95f;
    arr.markers.push_back(m);
  }
  for (int idx = static_cast<int>(wps.size()); idx < marker_slot_count_; ++idx) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = stamp;
    m.ns = "clip_waypoints";
    m.id = idx;
    m.action = visualization_msgs::msg::Marker::DELETE;
    arr.markers.push_back(m);
  }
  marker_slot_count_ = static_cast<int>(wps.size());
  if (!arr.markers.empty()) {
    marker_pub_->publish(arr);
  }
}

void ClipDetectorNode::on_finalize(const std_msgs::msg::Empty::SharedPtr)
{
  RCLCPP_INFO(this->get_logger(), "Mapping finalized — saving waypoints");
  finalize_waypoints(true);
}

int ClipDetectorNode::write_waypoints_to_disk()
{
  if (!session_dir_.has_value()) {
    return 0;
  }
  const auto final_wps = deduplicate_waypoints(waypoints_, save_dedup_radius_);
  std::filesystem::create_directories(session_dir_.value());
  YAML::Node root;
  root["frame_id"] = map_frame_;
  root["ground_z"] = ground_z_;
  root["dedup_radius_m"] = dedup_radius_;
  root["saved_at"] = format_timestamp();
  YAML::Node wp_list;
  for (std::size_t i = 0; i < final_wps.size(); ++i) {
    YAML::Node item;
    item["id"] = static_cast<int>(i + 1);
    item["x"] = std::round(final_wps[i].x * 10000.0) / 10000.0;
    item["y"] = std::round(final_wps[i].y * 10000.0) / 10000.0;
    item["confidence"] = std::round(final_wps[i].confidence * 10000.0) / 10000.0;
    item["source"] = final_wps[i].source;
    wp_list.push_back(item);
  }
  root["waypoints"] = wp_list;
  const auto out = session_dir_.value() / "waypoints.yaml";
  std::ofstream ofs(out);
  ofs << root;
  return static_cast<int>(final_wps.size());
}

void ClipDetectorNode::periodic_waypoint_flush()
{
  if (!session_dir_.has_value()) {
    return;
  }
  const double now = monotonic_now();
  if (now - last_waypoint_flush_time_ < waypoint_save_interval_ - 0.05) {
    return;
  }
  last_waypoint_flush_time_ = now;
  write_waypoints_to_disk();
}

void ClipDetectorNode::save_map_odom_offset()
{
  if (!session_dir_.has_value()) {
    return;
  }
  try {
    const auto tf = tf_buffer_->lookupTransform(
      map_frame_, "odom", tf2::TimePointZero, tf2::durationFromSec(0.5));
    const auto & t = tf.transform.translation;
    const auto & q = tf.transform.rotation;
    const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    YAML::Node root;
    root["map_frame"] = map_frame_;
    root["odom_frame"] = "odom";
    root["x"] = std::round(t.x * 10000.0) / 10000.0;
    root["y"] = std::round(t.y * 10000.0) / 10000.0;
    root["yaw"] = std::round(yaw * 10000.0) / 10000.0;
    root["saved_at"] = format_timestamp();
    const auto out = session_dir_.value() / "map_odom_offset.yaml";
    std::ofstream ofs(out);
    ofs << root;
  } catch (const tf2::TransformException & exc) {
    RCLCPP_WARN(this->get_logger(), "Cannot save map_odom_offset: %s", exc.what());
  }
}

void ClipDetectorNode::render_waypoints_overlay()
{
  if (finalize_overlay_done_ || !session_dir_.has_value()) {
    return;
  }
  try {
    save_map_odom_offset();
    const auto overlay = ::detection_pkg::render_waypoints_overlay(session_dir_.value());
    if (overlay.has_value()) {
      RCLCPP_INFO(this->get_logger(), "Waypoints map: %s", overlay->string().c_str());
    }
  } catch (const std::exception & exc) {
    RCLCPP_WARN(this->get_logger(), "waypoints map overlay failed: %s", exc.what());
  }
  finalize_overlay_done_ = true;
}

void ClipDetectorNode::finalize_waypoints(bool publish, bool force)
{
  const int count = write_waypoints_to_disk();
  if (count == 0 && waypoints_.empty() && !force) {
    return;
  }
  if (session_dir_.has_value()) {
    RCLCPP_INFO(
      this->get_logger(), "Waypoints saved: %s (%d points)",
      (session_dir_.value() / "waypoints.yaml").string().c_str(), count);
  }
  render_waypoints_overlay();
  if (publish) {
    try {
      publish_waypoints(false);
    } catch (const std::exception & exc) {
      RCLCPP_WARN(this->get_logger(), "waypoint publish skipped on shutdown: %s", exc.what());
    }
  }
}

void ClipDetectorNode::start_yolo_init_async(const std::string & weights)
{
  RCLCPP_INFO(
    this->get_logger(),
    "Loading YOLO in background (OpenCV DNN if no .engine; "
    "TensorRT: bash scripts/convert_to_trt_engine.sh)...");
  join_yolo_init_thread();
  yolo_ready_ = false;
  yolo_init_failed_ = false;
  yolo_init_thread_ = std::thread([this, weights]() {
    const bool ok = yolo_.init(weights);
    if (!ok) {
      yolo_init_failed_ = true;
      RCLCPP_ERROR(
        this->get_logger(),
        "YOLO init failed (OpenCV DNN cannot load best.onnx Split ops). "
        "Use: ros2 launch detection_pkg detection_cpp.launch.py (auto Python fallback) "
        "or: bash scripts/convert_to_trt_engine.sh");
      return;
    }
    const char * backend_name = "none";
    switch (yolo_.backend()) {
      case YoloBackend::TensorRT: backend_name = "TensorRT"; break;
      case YoloBackend::OpenCVDnn: backend_name = "OpenCV DNN"; break;
      default: break;
    }
    RCLCPP_INFO(this->get_logger(), "YOLO backend ready: %s", backend_name);
    yolo_ready_ = true;
  });
}

void ClipDetectorNode::join_yolo_init_thread()
{
  if (yolo_init_thread_.joinable()) {
    yolo_init_thread_.join();
  }
}

void ClipDetectorNode::detach_yolo_init_thread()
{
  if (yolo_init_thread_.joinable()) {
    yolo_init_thread_.detach();
  }
}

ClipDetectorNode::~ClipDetectorNode()
{
  detach_yolo_init_thread();
}

void ClipDetectorNode::stop_ros_io()
{
  flush_timer_.reset();
  marker_timer_.reset();
  stereo_sync_.reset();
  left_sub_.reset();
  right_sub_.reset();
  left_preview_sub_.reset();
  left_info_sub_.reset();
  right_info_sub_.reset();
  odom_sub_.reset();
  cmd_vel_sub_.reset();
  teleop_sub_.reset();
  finalize_sub_.reset();
  tf_listener_.reset();
}

void ClipDetectorNode::shutdown_save()
{
  if (shutdown_done_) {
    return;
  }
  shutdown_done_ = true;
  stop_ros_io();
  detach_yolo_init_thread();
  finalize_waypoints(false, true);
}

}  // namespace detection_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<detection_pkg::ClipDetectorNode> node;
  node = std::make_shared<detection_pkg::ClipDetectorNode>();
  rclcpp::on_shutdown([node]() {
    if (node) {
      node->shutdown_save();
    }
  });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
