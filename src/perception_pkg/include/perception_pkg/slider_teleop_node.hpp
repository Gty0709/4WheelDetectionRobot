#pragma once

#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace perception_pkg
{

class SliderTeleopNode : public rclcpp::Node
{
public:
  explicit SliderTeleopNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SliderTeleopNode() override;

  void set_vel(double linear, double angular);
  void publish_stop();
  void stop_publishing();
  void request_save_map();

  static constexpr double WHEEL_SEPARATION = 0.18;   // m
  static constexpr double WHEEL_RADIUS     = 0.04;   // m
  static constexpr double V_MAX            = 0.8;     // m/s
  static constexpr double W_MAX            = 1.5;     // rad/s
  static constexpr double PUBLISH_HZ       = 20.0;

  struct WheelSpeeds
  {
    double v_left;
    double v_right;
    double w_left;
    double w_right;
  };

  static WheelSpeeds kinematics(double v, double omega);

  void publish_stop_burst(int count = 8);
  void timer_callback();

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub_;

  // Service client
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr save_client_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // State
  std::mutex lock_;
  double linear_{0.0};
  double angular_{0.0};
  bool stopped_{false};
  bool save_on_exit_{true};
};

}  // namespace perception_pkg
