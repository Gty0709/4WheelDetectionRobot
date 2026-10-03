#pragma once

#include <cmath>
#include <optional>
#include <tuple>
#include <vector>

namespace navigation_pkg
{

using Pose3 = std::tuple<double, double, double>;

double normalize_yaw(double yaw);

std::tuple<double, double, double> delta_se2(
  double x0, double y0, double yaw0,
  double x1, double y1, double yaw1);

Pose3 compose_se2(
  double x0, double y0, double yaw0,
  double dx, double dy, double dyaw);

class OdomTrajectoryTracker
{
public:
  explicit OdomTrajectoryTracker(
    double max_odom_step_m = 0.35,
    double max_odom_yaw_step = 0.9);

  bool anchored() const { return anchored_; }

  Pose3 reset_anchor(const Pose3 & map_pose, const Pose3 & odom_pose);
  std::optional<Pose3> update_odom(const Pose3 & odom_pose);

private:
  double max_odom_step_m_;
  double max_odom_yaw_step_;
  std::optional<Pose3> last_odom_;
  std::optional<Pose3> pose_;
  bool anchored_{false};
};

struct TrajectorySample
{
  double x{0.0};
  double y{0.0};
};

std::vector<TrajectorySample> filter_trajectory_outliers(
  const std::vector<TrajectorySample> & samples,
  double max_step_m = 0.45);

}  // namespace navigation_pkg
