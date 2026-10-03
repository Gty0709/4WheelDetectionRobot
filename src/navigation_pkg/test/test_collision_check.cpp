#include <gtest/gtest.h>

#include "navigation_pkg/collision_check.hpp"

TEST(CollisionCheck, OutOfBoundsIsLethal)
{
  const auto cost = navigation_pkg::cost_at(
    100, 100, 10, 10, std::vector<int8_t>(100, 0));
  ASSERT_TRUE(cost.has_value());
  EXPECT_EQ(*cost, navigation_pkg::LETHAL_COST);
}

TEST(CollisionCheck, DefaultFootprintHasFourVertices)
{
  const auto & fp = navigation_pkg::default_footprint();
  EXPECT_EQ(fp.size(), 4u);
}

TEST(CollisionCheck, FreeCellFootprintFree)
{
  std::vector<int8_t> data(100 * 100, 0);
  const bool ok = navigation_pkg::footprint_free(
    0.5, 0.5, 0.0, 100, 100, 0.05, 0.0, 0.0, data);
  EXPECT_TRUE(ok);
}
