#pragma once

#include <string>
#include <tuple>

#include <gazebo_msgs/msg/entity_state.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>

namespace navigation_pkg
{

std::tuple<double, double, double, double> yaw_to_quaternion(double yaw);

gazebo_msgs::srv::SetEntityState::Request make_set_entity_state_request(
  const std::string & entity_name,
  double x,
  double y,
  double yaw,
  double z = 0.0,
  const std::string & reference_frame = "world");

}  // namespace navigation_pkg
