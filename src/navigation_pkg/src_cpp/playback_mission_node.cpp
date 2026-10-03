#include "navigation_pkg/playback_mission_node.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>

#include "navigation_pkg/waypoint_io.hpp"

namespace navigation_pkg
{

namespace
{

geometry_msgs::msg::Quaternion yaw_to_quat(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

rclcpp::QoS static_viz_qos()
{
  return rclcpp::QoS(1)
    .transient_local()
    .reliable();
}

struct CliArgs
{
  std::filesystem::path mission_dir;
  double rate{2.0};
  bool loop{false};
};

}  // namespace

std::optional<CliArgs> parse_playback_args(int argc, char ** argv)
{
  CliArgs args;
  args.mission_dir = resolve_path_latest();

  for (int i = 1; i < argc; ++i) {
    const std::string flag = argv[i];
    if (flag == "--mission" && i + 1 < argc) {
      args.mission_dir = std::filesystem::absolute(std::filesystem::path(argv[++i]));
      continue;
    }
    if (flag == "--rate" && i + 1 < argc) {
      args.rate = std::stod(argv[++i]);
      continue;
    }
    if (flag == "--loop") {
      args.loop = true;
      continue;
    }
    if (flag == "--help" || flag == "-h") {
      std::cout << "usage: playback_mission_cpp [--mission DIR] [--rate 2.0] [--loop]\n";
      return std::nullopt;
    }
    std::cerr << "unknown argument: " << flag << "\n";
    return std::nullopt;
  }
  return args;
}

MissionPlaybackNode::MissionPlaybackNode(
  const std::filesystem::path & mission_dir,
  double rate,
  bool loop)
: Node("mission_playback"),
  rate_(std::max(rate, 0.1)),
  loop_(loop)
{
  const auto mission_file = mission_dir / "mission.yaml";
  if (!std::filesystem::is_regular_file(mission_file)) {
    throw std::runtime_error("Missing " + mission_file.string());
  }

  const YAML::Node data = YAML::LoadFile(mission_file.string());
  frame_id_ = data["frame_id"] ? data["frame_id"].as<std::string>() : "map";

  if (data["trajectory"]) {
    for (const auto & item : data["trajectory"]) {
      TrajectorySample sample;
      sample.t_sec = item["t_sec"].as<double>();
      sample.x = item["x"].as<double>();
      sample.y = item["y"].as<double>();
      sample.yaw = item["yaw"] ? item["yaw"].as<double>() : 0.0;
      samples_.push_back(sample);
    }
  }

  if (data["waypoints"]) {
    for (const auto & item : data["waypoints"]) {
      WaypointSample wp;
      wp.id = item["id"].as<int>();
      wp.x = item["x"].as<double>();
      wp.y = item["y"].as<double>();
      waypoints_.push_back(wp);
    }
  }

  if (data["visits"]) {
    for (const auto & item : data["visits"]) {
      visited_ids_.insert(item["waypoint_id"].as<int>());
    }
  }

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/navigation/playback_pose", 10);
  mission_path_pub_ = create_publisher<nav_msgs::msg::Path>("/navigation/mission_path", static_viz_qos());
  driven_pub_ = create_publisher<nav_msgs::msg::Path>("/navigation/driven_path", 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/navigation/waypoint_markers", static_viz_qos());
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  full_path_.header.frame_id = frame_id_;
  played_path_.header.frame_id = frame_id_;
  for (const auto & s : samples_) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = frame_id_;
    ps.pose.position.x = s.x;
    ps.pose.position.y = s.y;
    ps.pose.orientation = yaw_to_quat(s.yaw);
    full_path_.poses.push_back(ps);
  }
  mission_path_pub_->publish(full_path_);
  publish_markers();

  idx_ = 0;
  wall_start_ = get_clock()->now();
  mission_start_t_ = samples_.empty() ? 0.0 : samples_.front().t_sec;
  timer_ = create_wall_timer(std::chrono::milliseconds(50), std::bind(&MissionPlaybackNode::tick, this));
  static_viz_timer_ = create_wall_timer(
    std::chrono::milliseconds(2000),
    std::bind(&MissionPlaybackNode::republish_static_viz, this));

  if (!samples_.empty()) {
    const auto & s0 = samples_.front();
    publish_tf(s0.x, s0.y, s0.yaw, get_clock()->now());
  }

  RCLCPP_INFO(
    get_logger(), "Playing %zu samples from %s at %.1fx",
    samples_.size(), mission_dir.string().c_str(), rate_);
  if (samples_.empty()) {
    RCLCPP_WARN(get_logger(), "Mission has no trajectory samples");
    finish_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MissionPlaybackNode::finish_playback, this));
  } else {
    const double duration = samples_.back().t_sec - samples_.front().t_sec;
    RCLCPP_INFO(
      get_logger(), "Mission duration %.1fs -> playback ~%.1fs",
      duration, duration / rate_);
  }
}

void MissionPlaybackNode::republish_static_viz()
{
  const auto stamp = get_clock()->now();
  full_path_.header.stamp = stamp;
  mission_path_pub_->publish(full_path_);
  publish_markers();
}

void MissionPlaybackNode::publish_markers()
{
  visualization_msgs::msg::MarkerArray arr;
  for (const auto & wp : waypoints_) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame_id_;
    m.header.stamp = get_clock()->now();
    m.ns = "detected_paperclips";
    m.id = wp.id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = wp.x;
    m.pose.position.y = wp.y;
    m.pose.position.z = 0.05;
    m.scale.x = 0.15;
    m.scale.y = 0.15;
    m.scale.z = 0.15;
    if (visited_ids_.count(wp.id) > 0) {
      m.color.r = 0.1f;
      m.color.g = 0.95f;
      m.color.b = 0.1f;
      m.color.a = 0.95f;
    } else {
      m.color.r = 0.95f;
      m.color.g = 0.1f;
      m.color.b = 0.1f;
      m.color.a = 0.95f;
    }
    arr.markers.push_back(m);
  }
  marker_pub_->publish(arr);
}

void MissionPlaybackNode::publish_tf(double x, double y, double yaw, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = frame_id_;
  t.child_frame_id = "base_footprint";
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = 0.0;
  t.transform.rotation = yaw_to_quat(yaw);
  tf_broadcaster_->sendTransform(t);
}

void MissionPlaybackNode::reset_playback()
{
  idx_ = 0;
  wall_start_ = get_clock()->now();
  mission_start_t_ = samples_.front().t_sec;
  played_path_ = nav_msgs::msg::Path();
  played_path_.header.frame_id = frame_id_;
}

void MissionPlaybackNode::finish_playback()
{
  if (finished_) {
    return;
  }
  finished_ = true;
  if (static_viz_timer_) {
    static_viz_timer_->cancel();
  }
  RCLCPP_INFO(get_logger(), "Playback finished; closing session");
  shutdown_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MissionPlaybackNode::shutdown_node_once, this));
}

void MissionPlaybackNode::shutdown_node_once()
{
  if (timer_) {
    timer_->cancel();
  }
  rclcpp::shutdown();
}

void MissionPlaybackNode::tick()
{
  if (finished_ || samples_.empty()) {
    return;
  }

  const double elapsed = (get_clock()->now() - wall_start_).seconds();
  const double mission_t = mission_start_t_ + elapsed * rate_;

  while (
    idx_ + 1 < samples_.size() &&
    samples_[idx_ + 1].t_sec <= mission_t)
  {
    ++idx_;
  }

  const bool at_end = idx_ >= samples_.size() - 1 &&
    mission_t >= samples_.back().t_sec;
  if (at_end) {
    if (loop_) {
      reset_playback();
      return;
    }
    finish_playback();
    return;
  }

  const auto & s = samples_[idx_];
  const auto stamp = get_clock()->now();
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = frame_id_;
  msg.header.stamp = stamp;
  msg.pose.position.x = s.x;
  msg.pose.position.y = s.y;
  msg.pose.orientation = yaw_to_quat(s.yaw);
  pose_pub_->publish(msg);
  publish_tf(s.x, s.y, s.yaw, stamp);

  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = frame_id_;
  ps.header.stamp = stamp;
  ps.pose = msg.pose;
  played_path_.poses.push_back(ps);
  played_path_.header.stamp = stamp;
  driven_pub_->publish(played_path_);
}

}  // namespace navigation_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto args = navigation_pkg::parse_playback_args(argc, argv);
  if (!args) {
    rclcpp::shutdown();
    return 1;
  }

  try {
    auto node = std::make_shared<navigation_pkg::MissionPlaybackNode>(
      args->mission_dir, args->rate, args->loop);
    RCLCPP_INFO(node->get_logger(), "Playback mission dir: %s", args->mission_dir.string().c_str());
    rclcpp::spin(node);
  } catch (const std::exception & exc) {
    std::cerr << exc.what() << "\n";
    rclcpp::shutdown();
    return 1;
  }

  return 0;
}
