#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace perception_pkg
{

class SaveMapTrigger : public rclcpp::Node
{
public:
  explicit SaveMapTrigger(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void tick();

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool done_{false};
};

}  // namespace perception_pkg
