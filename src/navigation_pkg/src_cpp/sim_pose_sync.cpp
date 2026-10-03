#include "navigation_pkg/sim_pose_sync.hpp"

#include <cmath>

namespace navigation_pkg
{

std::tuple<double, double, double, double> yaw_to_quaternion(double yaw)
{
  const double half = yaw * 0.5;
  return {0.0, 0.0, std::sin(half), std::cos(half)};
}

gazebo_msgs::srv::SetEntityState::Request make_set_entity_state_request(
  const std::string & entity_name,
  double x,
  double y,
  double yaw,
  double z,
  const std::string & reference_frame)
{
  gazebo_msgs::srv::SetEntityState::Request req;
  req.state.name = entity_name;
  req.state.reference_frame = reference_frame;
  req.state.pose.position.x = x;
  req.state.pose.position.y = y;
  req.state.pose.position.z = z;
  const auto [qx, qy, qz, qw] = yaw_to_quaternion(yaw);
  req.state.pose.orientation.x = qx;
  req.state.pose.orientation.y = qy;
  req.state.pose.orientation.z = qz;
  req.state.pose.orientation.w = qw;
  return req;
}

}  // namespace navigation_pkg
