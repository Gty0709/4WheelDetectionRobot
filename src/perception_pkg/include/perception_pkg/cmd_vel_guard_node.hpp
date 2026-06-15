#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace perception_pkg
{

class CmdVelGuard : public rclcpp::Node
{
public:
  explicit CmdVelGuard(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_teleop(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_nav(const geometry_msgs::msg::Twist::SharedPtr msg);
  void tick();

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr out_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist teleop_msg_;
  geometry_msgs::msg::Twist nav_msg_;
  bool have_teleop_{false};
  bool have_nav_{false};
  int64_t last_teleop_ns_{0};
  int64_t last_nav_ns_{0};
  int nav_log_left_{3};

  static constexpr double RATE_HZ = 30.0;
  static constexpr double TELEOP_STALE_SEC = 0.35;
  static constexpr double NAV_STALE_SEC = 0.5;
};

}  // namespace perception_pkg
