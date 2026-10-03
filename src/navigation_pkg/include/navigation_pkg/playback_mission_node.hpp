#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace navigation_pkg
{

class MissionPlaybackNode : public rclcpp::Node
{
public:
  MissionPlaybackNode(
    const std::filesystem::path & mission_dir,
    double rate,
    bool loop);

private:
  struct TrajectorySample
  {
    double t_sec{0.0};
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
  };

  struct WaypointSample
  {
    int id{0};
    double x{0.0};
    double y{0.0};
  };

  void republish_static_viz();
  void publish_markers();
  void publish_tf(double x, double y, double yaw, const rclcpp::Time & stamp);
  void reset_playback();
  void finish_playback();
  void shutdown_node_once();
  void tick();

  double rate_;
  bool loop_;
  bool finished_{false};
  std::string frame_id_;
  std::vector<TrajectorySample> samples_;
  std::vector<WaypointSample> waypoints_;
  std::unordered_set<int> visited_ids_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mission_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr driven_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  nav_msgs::msg::Path full_path_;
  nav_msgs::msg::Path played_path_;
  std::size_t idx_{0};
  rclcpp::Time wall_start_;
  double mission_start_t_{0.0};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr static_viz_timer_;
  rclcpp::TimerBase::SharedPtr finish_timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;
};

}  // namespace navigation_pkg
