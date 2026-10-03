#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>

namespace navigation_pkg
{

constexpr int INSCRIBED_INFLATED_OBSTACLE = 253;

extern const std::vector<std::pair<double, double>> GAZEBO_DYNAMIC_OBSTACLE_XY;

class StaticMapIndex
{
public:
  explicit StaticMapIndex(const std::filesystem::path & session_dir);

  bool is_occupied(double wx, double wy) const;

  double resolution() const { return resolution_; }
  double origin_x() const { return origin_x_; }
  double origin_y() const { return origin_y_; }
  int width() const { return width_; }
  int height() const { return height_; }

private:
  double resolution_{0.05};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double occupied_thresh_{0.65};
  int negate_{0};
  int width_{0};
  int height_{0};
  std::vector<uint8_t> gray_;
};

std::vector<std::pair<double, double>> extract_dynamic_obstacles(
  const nav_msgs::msg::OccupancyGrid & costmap,
  const StaticMapIndex & static_map,
  tf2_ros::Buffer * tf_buffer = nullptr,
  const std::string & target_frame = "map",
  int min_cost = INSCRIBED_INFLATED_OBSTACLE,
  int cell_stride = 2);

struct ObstacleRecord
{
  double x{0.0};
  double y{0.0};
  std::string source;
};

std::vector<ObstacleRecord> obstacles_for_png(
  const std::vector<ObstacleRecord> & recorded,
  const StaticMapIndex & static_map,
  double merge_grid_m = 0.25);

}  // namespace navigation_pkg
