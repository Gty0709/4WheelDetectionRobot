#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <string>
#include <optional>
#include <fstream>

namespace perception_pkg
{

struct Pose3 {
  double x{0.0}, y{0.0}, yaw{0.0};
};

class MapSnapshotSaver : public rclcpp::Node
{
public:
  explicit MapSnapshotSaver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void update_pose_cache();
  void periodic_save();
  bool handle_save_map(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  bool save_now(bool write_pose, bool finalize);
  void save_initial_pose();
  void save_map_odom_offset();
  void signal_finalize();
  void stop_robot_and_settle();

  // File I/O helpers
  static void write_pgm(const std::string & path, const nav_msgs::msg::OccupancyGrid & msg);
  static void write_yaml(const std::string & path, const std::string & pgm_name,
    const nav_msgs::msg::OccupancyGrid & msg);
  static void write_initial_pose_yaml(const std::string & path, double x, double y, double yaw);
  static void write_map_odom_offset_yaml(const std::string & path, double x, double y, double yaw);
  static void write_session_meta(const std::string & session_dir, const std::string & maps_root);
  static void update_map_latest_symlink(const std::string & maps_root, const std::string & session_name);
  static std::optional<Pose3> lookup_pose(
    tf2_ros::Buffer & tf_buffer, const std::string & map_frame,
    const std::string & base_frame);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_latched_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_default_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr finalize_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_teleop_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr save_timer_;
  rclcpp::TimerBase::SharedPtr pose_timer_;

  nav_msgs::msg::OccupancyGrid::SharedPtr last_map_;
  std::optional<Pose3> last_pose_;
  bool shutdown_save_done_{false};
  bool saving_{false};
  double last_cache_time_{0.0};

  std::string maps_root_;
  std::string session_dir_;
  std::string cache_dir_;
  std::string prefix_{"slam_map"};
  bool update_latest_{true};
  bool save_on_shutdown_{true};
  double save_interval_{90.0};
  double cache_interval_{5.0};
  double save_settle_sec_{2.5};
  bool stop_robot_before_save_{true};
  std::string map_frame_{"map"};
  std::string base_frame_{"base_footprint"};
};

}  // namespace perception_pkg
