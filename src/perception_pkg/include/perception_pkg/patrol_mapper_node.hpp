#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <array>
#include <cmath>
#include <optional>
#include <tuple>
#include <vector>

namespace perception_pkg
{

enum class Phase { WAIT_ODOM, DRIVE, DONE };

class PatrolMapperNode : public rclcpp::Node
{
public:
  explicit PatrolMapperNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  void shutdown();

private:
  // Callbacks
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void control_tick();

  // Helpers
  void safe_publish(double lin, double ang);
  void stop();
  std::tuple<double, double> goal() const;
  std::optional<std::tuple<double, double>> next_goal() const;
  void begin_segment();
  double along_dist(double x, double y) const;
  double dist_to_goal(double x, double y) const;
  bool segment_done(double x, double y) const;
  double cross_track_err(double x, double y) const;
  double xtrack_w(double xte) const;
  double steer_bearing(double x, double y) const;
  bool check_stuck(double x, double y, double now);
  void reset_stuck();
  void advance_waypoint();
  void request_save();

  // ROS interfaces
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  bool save_on_finish_;
  double v_max_;
  double w_max_;
  double goal_tol_;
  double yaw_kp_;
  double xtrack_kp_;

  // State
  std::optional<std::tuple<double, double, double>> pose_;  // (x, y, yaw)
  Phase phase_;
  size_t wp_i_;
  double segment_yaw_;
  bool seg_horizontal_;
  std::tuple<double, double> seg_from_;
  bool running_;
  std::optional<double> stuck_t0_;
  std::optional<std::tuple<double, double>> stuck_xy_;

  // Constants
  static constexpr double CORNER_APPROACH = 0.25;
  static constexpr double TURN_IN_PLACE = 0.50;
  static constexpr double STUCK_SEC = 12.0;
  static constexpr double STUCK_MOVE = 0.04;
};

}  // namespace perception_pkg
