#include <gtest/gtest.h>

#include "navigation_pkg/manhattan_tsp.hpp"

TEST(ManhattanTsp, FixedStartThreeWaypoints)
{
  std::vector<navigation_pkg::Waypoint> wps = {
    {1, 0.0, 0.0},
    {2, 1.0, 0.0},
    {3, 1.0, 1.0},
  };
  const auto dist = navigation_pkg::distance_matrix(wps);
  const auto order = navigation_pkg::tsp_fixed_start(dist, 0);
  ASSERT_EQ(order.size(), 3u);
  EXPECT_DOUBLE_EQ(navigation_pkg::path_cost(order, dist), 2.0);
}

TEST(ManhattanTsp, FromStartPoseUsesDepot)
{
  std::vector<navigation_pkg::Waypoint> wps = {
    {1, 2.0, 0.0},
    {2, 2.0, 2.0},
  };
  const auto [order, dmat, amat, cost] = navigation_pkg::tsp_from_start_pose(wps, 0.0, 0.0);
  ASSERT_FALSE(order.empty());
  EXPECT_GT(cost, 0.0);
  EXPECT_EQ(dmat.size(), order.size());
}
