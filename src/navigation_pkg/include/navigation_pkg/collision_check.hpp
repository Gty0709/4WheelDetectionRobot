#pragma once

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace navigation_pkg
{

constexpr int LETHAL_COST = 253;
constexpr int INSCRIBED_COST = 252;

using Footprint = std::vector<std::pair<double, double>>;

const Footprint & default_footprint();

std::vector<std::pair<double, double>> rotate_footprint(
  const Footprint & footprint,
  double x,
  double y,
  double yaw);

std::pair<int, int> world_to_map(
  double wx,
  double wy,
  double origin_x,
  double origin_y,
  double resolution);

std::optional<int> cost_at(
  int mx,
  int my,
  int width,
  int height,
  const std::vector<int8_t> & data);

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
  const Footprint & footprint = default_footprint(),
  int max_cost = INSCRIBED_COST);

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
  const Footprint & footprint = default_footprint());

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
  const Footprint & footprint = default_footprint(),
  double probe_radius = 0.55,
  int samples = 16);

double backup_target_yaw(double current_yaw, double free_heading);

}  // namespace navigation_pkg
