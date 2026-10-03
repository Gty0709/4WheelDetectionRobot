#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <random>
#include <string>

namespace perception_pkg
{

enum class ExploreState { FORWARD, TURN };

class AutoExploreNode : public rclcpp::Node
{
public:
  AutoExploreNode();

private:
  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void control_loop();
  double scan_data_range(const sensor_msgs::msg::LaserScan & scan,
                         double angle_min_deg, double angle_max_deg);

public:
  void stop();

  // Publishers / subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Motion parameters
  double linear_vel_;
  double turn_vel_;
  double safe_dist_;
  double side_dist_;

  // State machine
  ExploreState state_;
  double turn_dir_;
  int turn_count_;
  int turn_target_;

  // Latest scan
  sensor_msgs::msg::LaserScan::SharedPtr scan_data_;

  // Random engine
  std::mt19937 rng_;
};

}  // namespace perception_pkg
