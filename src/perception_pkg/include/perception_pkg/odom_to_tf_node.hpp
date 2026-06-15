#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace perception_pkg
{

class OdomToTF : public rclcpp::Node
{
public:
  explicit OdomToTF(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);

  std::shared_ptr<tf2_ros::TransformBroadcaster> br_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

}  // namespace perception_pkg
