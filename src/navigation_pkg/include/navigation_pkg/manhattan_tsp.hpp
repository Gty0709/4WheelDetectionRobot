#pragma once

#include <cstddef>
#include <optional>
#include <tuple>
#include <vector>

#include "navigation_pkg/waypoint_io.hpp"

namespace navigation_pkg
{

double manhattan(const Waypoint & a, const Waypoint & b);
double manhattan_xy(double ax, double ay, double bx, double by);

std::vector<std::vector<double>> distance_matrix(const std::vector<Waypoint> & waypoints);
std::vector<std::vector<int>> adjacency_matrix(std::size_t n);

std::size_t nearest_index(const std::vector<Waypoint> & waypoints, double x, double y);

std::vector<int> tsp_fixed_start(
  const std::vector<std::vector<double>> & distance,
  int start);

double path_cost(
  const std::vector<int> & order,
  const std::vector<std::vector<double>> & distance);

double path_cost_from_pose(
  const std::vector<Waypoint> & waypoints,
  double start_x,
  double start_y,
  const std::vector<int> & wp_order);

std::tuple<std::vector<int>, double> brute_force_wp_order_from_pose(
  const std::vector<Waypoint> & waypoints,
  double start_x,
  double start_y);

std::tuple<std::vector<int>, std::vector<std::vector<double>>, std::vector<std::vector<int>>, double>
tsp_from_start_pose(
  const std::vector<Waypoint> & waypoints,
  double start_x,
  double start_y);

std::tuple<std::vector<int>, std::vector<std::vector<double>>, std::vector<std::vector<int>>>
tsp_order_after_first(const std::vector<Waypoint> & waypoints, int start_idx);

}  // namespace navigation_pkg
