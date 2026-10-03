#include "navigation_pkg/trajectory_tracker.hpp"

#include <cmath>

namespace navigation_pkg
{

double normalize_yaw(double yaw)
{
  while (yaw > M_PI) {
    yaw -= 2.0 * M_PI;
  }
  while (yaw < -M_PI) {
    yaw += 2.0 * M_PI;
  }
  return yaw;
}

std::tuple<double, double, double> delta_se2(
  double x0, double y0, double yaw0,
  double x1, double y1, double yaw1)
{
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  const double c = std::cos(-yaw0);
  const double s = std::sin(-yaw0);
  return {c * dx - s * dy, s * dx + c * dy, normalize_yaw(yaw1 - yaw0)};
}

Pose3 compose_se2(
  double x0, double y0, double yaw0,
  double dx, double dy, double dyaw)
{
  const double c = std::cos(yaw0);
  const double s = std::sin(yaw0);
  return {
    x0 + c * dx - s * dy,
    y0 + s * dx + c * dy,
    normalize_yaw(yaw0 + dyaw)};
}

OdomTrajectoryTracker::OdomTrajectoryTracker(
  double max_odom_step_m,
  double max_odom_yaw_step)
: max_odom_step_m_(max_odom_step_m),
  max_odom_yaw_step_(max_odom_yaw_step)
{
}

Pose3 OdomTrajectoryTracker::reset_anchor(const Pose3 & map_pose, const Pose3 & odom_pose)
{
  pose_ = map_pose;
  last_odom_ = odom_pose;
  anchored_ = true;
  return *pose_;
}

std::optional<Pose3> OdomTrajectoryTracker::update_odom(const Pose3 & odom_pose)
{
  if (!anchored_ || !pose_ || !last_odom_) {
    return pose_;
  }

  const auto [dx, dy, dyaw] = delta_se2(
    std::get<0>(*last_odom_), std::get<1>(*last_odom_), std::get<2>(*last_odom_),
    std::get<0>(odom_pose), std::get<1>(odom_pose), std::get<2>(odom_pose));

  if (std::hypot(dx, dy) > max_odom_step_m_ || std::abs(dyaw) > max_odom_yaw_step_) {
    last_odom_ = odom_pose;
    return pose_;
  }

  pose_ = compose_se2(
    std::get<0>(*pose_), std::get<1>(*pose_), std::get<2>(*pose_),
    dx, dy, dyaw);
  last_odom_ = odom_pose;
  return pose_;
}

std::vector<TrajectorySample> filter_trajectory_outliers(
  const std::vector<TrajectorySample> & samples,
  double max_step_m)
{
  if (samples.empty()) {
    return {};
  }
  std::vector<TrajectorySample> out;
  out.push_back(samples.front());
  for (std::size_t i = 1; i < samples.size(); ++i) {
    const auto & prev = out.back();
    const auto & s = samples[i];
    if (std::hypot(s.x - prev.x, s.y - prev.y) <= max_step_m) {
      out.push_back(s);
    }
  }
  return out;
}

}  // namespace navigation_pkg
