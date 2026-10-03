#include "navigation_pkg/manhattan_tsp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace navigation_pkg
{

double manhattan(const Waypoint & a, const Waypoint & b)
{
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

double manhattan_xy(double ax, double ay, double bx, double by)
{
  return std::abs(ax - bx) + std::abs(ay - by);
}

std::vector<std::vector<double>> distance_matrix(const std::vector<Waypoint> & waypoints)
{
  const std::size_t n = waypoints.size();
  std::vector<std::vector<double>> d(n, std::vector<double>(n, 0.0));
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n; ++j) {
      if (i != j) {
        d[i][j] = manhattan(waypoints[i], waypoints[j]);
      }
    }
  }
  return d;
}

std::vector<std::vector<int>> adjacency_matrix(std::size_t n)
{
  std::vector<std::vector<int>> adj(n, std::vector<int>(n, 0));
  for (std::size_t i = 0; i < n; ++i) {
    for (std::size_t j = 0; j < n; ++j) {
      adj[i][j] = (i == j) ? 0 : 1;
    }
  }
  return adj;
}

std::size_t nearest_index(const std::vector<Waypoint> & waypoints, double x, double y)
{
  std::size_t best = 0;
  double best_d = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    const double d = manhattan_xy(x, y, waypoints[i].x, waypoints[i].y);
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}

std::vector<int> tsp_fixed_start(
  const std::vector<std::vector<double>> & distance,
  int start)
{
  const int n = static_cast<int>(distance.size());
  if (n == 0) {
    return {};
  }
  if (n == 1) {
    return {start};
  }

  const int start_mask = 1 << start;
  const double inf = std::numeric_limits<double>::infinity();
  const int mask_count = 1 << n;
  std::vector<std::vector<double>> dp(mask_count, std::vector<double>(n, inf));
  std::vector<std::vector<int>> parent(mask_count, std::vector<int>(n, -1));

  for (int j = 0; j < n; ++j) {
    if (j == start) {
      continue;
    }
    const int mask = start_mask | (1 << j);
    dp[mask][j] = distance[start][j];
    parent[mask][j] = start;
  }

  for (int mask = 0; mask < mask_count; ++mask) {
    if ((mask & start_mask) == 0) {
      continue;
    }
    for (int j = 0; j < n; ++j) {
      if ((mask & (1 << j)) == 0) {
        continue;
      }
      const double cost_j = dp[mask][j];
      if (cost_j == inf) {
        continue;
      }
      for (int k = 0; k < n; ++k) {
        if (mask & (1 << k)) {
          continue;
        }
        const int nmask = mask | (1 << k);
        const double nc = cost_j + distance[j][k];
        if (nc < dp[nmask][k]) {
          dp[nmask][k] = nc;
          parent[nmask][k] = j;
        }
      }
    }
  }

  const int full = (1 << n) - 1;
  int end = 0;
  double best_end = dp[full][0];
  for (int j = 1; j < n; ++j) {
    if (dp[full][j] < best_end) {
      best_end = dp[full][j];
      end = j;
    }
  }

  std::vector<int> order;
  int mask = full;
  int cur = end;
  while (cur != -1) {
    order.push_back(cur);
    const int prev = parent[mask][cur];
    mask ^= 1 << cur;
    cur = prev;
  }
  std::reverse(order.begin(), order.end());
  return order;
}

double path_cost(
  const std::vector<int> & order,
  const std::vector<std::vector<double>> & distance)
{
  if (order.size() < 2) {
    return 0.0;
  }
  double total = 0.0;
  for (std::size_t i = 0; i + 1 < order.size(); ++i) {
    total += distance[order[i]][order[i + 1]];
  }
  return total;
}

double path_cost_from_pose(
  const std::vector<Waypoint> & waypoints,
  double start_x,
  double start_y,
  const std::vector<int> & wp_order)
{
  if (wp_order.empty()) {
    return 0.0;
  }
  double total = manhattan_xy(start_x, start_y, waypoints[wp_order[0]].x, waypoints[wp_order[0]].y);
  for (std::size_t i = 0; i + 1 < wp_order.size(); ++i) {
    total += manhattan(waypoints[wp_order[i]], waypoints[wp_order[i + 1]]);
  }
  return total;
}

std::tuple<std::vector<int>, double> brute_force_wp_order_from_pose(
  const std::vector<Waypoint> & waypoints,
  double start_x,
  double start_y)
{
  const std::size_t n = waypoints.size();
  if (n == 0) {
    return std::make_tuple(std::vector<int>{}, 0.0);
  }
  if (n == 1) {
    return {{0}, manhattan_xy(start_x, start_y, waypoints[0].x, waypoints[0].y)};
  }

  std::vector<int> perm(n);
  std::iota(perm.begin(), perm.end(), 0);

  std::vector<int> best_order;
  double best_cost = std::numeric_limits<double>::infinity();

  do {
    const double cost = path_cost_from_pose(waypoints, start_x, start_y, perm);
    if (cost < best_cost) {
      best_cost = cost;
      best_order = perm;
    }
  } while (std::next_permutation(perm.begin(), perm.end()));

  return {best_order, best_cost};
}

std::tuple<std::vector<int>, std::vector<std::vector<double>>, std::vector<std::vector<int>>, double>
tsp_from_start_pose(
  const std::vector<Waypoint> & waypoints,
  double start_x,
  double start_y)
{
  const int n_wp = static_cast<int>(waypoints.size());
  const auto wp_d = distance_matrix(waypoints);
  const auto wp_a = adjacency_matrix(waypoints.size());

  if (n_wp == 0) {
    return {{}, wp_d, wp_a, 0.0};
  }
  if (n_wp == 1) {
    return {
      {0},
      wp_d,
      wp_a,
      manhattan_xy(start_x, start_y, waypoints[0].x, waypoints[0].y)};
  }

  const int n = n_wp + 1;
  std::vector<std::vector<double>> ext(n, std::vector<double>(n, 0.0));
  for (int j = 0; j < n_wp; ++j) {
    const double c = manhattan_xy(start_x, start_y, waypoints[j].x, waypoints[j].y);
    ext[0][j + 1] = c;
    ext[j + 1][0] = c;
  }
  for (int i = 0; i < n_wp; ++i) {
    for (int j = 0; j < n_wp; ++j) {
      if (i != j) {
        ext[i + 1][j + 1] = wp_d[i][j];
      }
    }
  }

  const std::vector<int> full_order = tsp_fixed_start(ext, 0);
  std::vector<int> wp_order;
  wp_order.reserve(n_wp);
  for (const int idx : full_order) {
    if (idx != 0) {
      wp_order.push_back(idx - 1);
    }
  }
  const double total_cost = path_cost(full_order, ext);
  return {wp_order, wp_d, wp_a, total_cost};
}

std::tuple<std::vector<int>, std::vector<std::vector<double>>, std::vector<std::vector<int>>>
tsp_order_after_first(const std::vector<Waypoint> & waypoints, int start_idx)
{
  const auto d = distance_matrix(waypoints);
  const auto a = adjacency_matrix(waypoints.size());
  const auto order = tsp_fixed_start(d, start_idx);
  return {order, d, a};
}

}  // namespace navigation_pkg
