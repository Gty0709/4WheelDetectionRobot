#include "navigation_pkg/collision_check.hpp"

#include <cmath>

namespace navigation_pkg
{

namespace
{

const Footprint kDefaultFootprint = {
  {0.12, 0.10},
  {0.12, -0.10},
  {-0.12, -0.10},
  {-0.12, 0.10},
};

}  // namespace

const Footprint & default_footprint()
{
  return kDefaultFootprint;
}

std::vector<std::pair<double, double>> rotate_footprint(
  const Footprint & footprint,
  double x,
  double y,
  double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  std::vector<std::pair<double, double>> out;
  out.reserve(footprint.size());
  for (const auto & [fx, fy] : footprint) {
    out.emplace_back(x + fx * c - fy * s, y + fx * s + fy * c);
  }
  return out;
}

std::pair<int, int> world_to_map(
  double wx,
  double wy,
  double origin_x,
  double origin_y,
  double resolution)
{
  const int mx = static_cast<int>((wx - origin_x) / resolution);
  const int my = static_cast<int>((wy - origin_y) / resolution);
  return {mx, my};
}

std::optional<int> cost_at(
  int mx,
  int my,
  int width,
  int height,
  const std::vector<int8_t> & data)
{
  if (mx < 0 || my < 0 || mx >= width || my >= height) {
    return LETHAL_COST;
  }
  return static_cast<int>(data[static_cast<std::size_t>(my) * static_cast<std::size_t>(width) + mx]);
}

int footprint_max_cost(
  double x,
  double y,
  double yaw,
  int width,
  int height,
  double resolution,
  double origin_x,
  double origin_y,
  const std::vector<int8_t> & data,
  const Footprint & footprint)
{
  int worst = 0;
  for (const auto & [wx, wy] : rotate_footprint(footprint, x, y, yaw)) {
    const auto [mx, my] = world_to_map(wx, wy, origin_x, origin_y, resolution);
    const auto c = cost_at(mx, my, width, height, data);
    if (!c) {
      return LETHAL_COST;
    }
    worst = std::max(worst, *c);
  }
  return worst;
}

bool footprint_free(
  double x,
  double y,
  double yaw,
  int width,
  int height,
  double resolution,
  double origin_x,
  double origin_y,
  const std::vector<int8_t> & data,
  const Footprint & footprint,
  int max_cost)
{
  return footprint_max_cost(
           x, y, yaw, width, height, resolution, origin_x, origin_y, data, footprint) < max_cost;
}

double estimate_free_heading(
  double x,
  double y,
  double yaw,
  int width,
  int height,
  double resolution,
  double origin_x,
  double origin_y,
  const std::vector<int8_t> & data,
  const Footprint & footprint,
  double probe_radius,
  int samples)
{
  double best_heading = yaw;
  double best_cost = static_cast<double>(footprint_max_cost(
    x, y, yaw, width, height, resolution, origin_x, origin_y, data, footprint));

  for (int i = 0; i < samples; ++i) {
    const double heading = (2.0 * M_PI * i) / samples;
    const double px = x + probe_radius * std::cos(heading);
    const double py = y + probe_radius * std::sin(heading);
    const double cost = static_cast<double>(footprint_max_cost(
      px, py, yaw, width, height, resolution, origin_x, origin_y, data, footprint));
    if (cost < best_cost) {
      best_cost = cost;
      best_heading = heading;
    }
  }
  return best_heading;
}

double backup_target_yaw(double /*current_yaw*/, double free_heading)
{
  return std::atan2(
    std::sin(free_heading - M_PI),
    std::cos(free_heading - M_PI));
}

}  // namespace navigation_pkg
