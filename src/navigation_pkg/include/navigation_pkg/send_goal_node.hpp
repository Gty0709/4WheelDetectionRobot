#pragma once

#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

namespace navigation_pkg
{

class GoalSenderNode : public rclcpp::Node
{
public:
  GoalSenderNode();

  int send(
    double x,
    double y,
    double yaw,
    const std::string & frame_id,
    double server_timeout,
    const std::optional<double> & result_timeout);

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback);

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::Time last_feedback_time_;
};

}  // namespace navigation_pkg
