#include "navigation_pkg/obstacle_detect.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <yaml-cpp/yaml.h>

namespace navigation_pkg
{

const std::vector<std::pair<double, double>> GAZEBO_DYNAMIC_OBSTACLE_XY = {
  {-3.0, 2.5},
  {3.0, 2.5},
  {-3.0, -2.5},
  {3.0, -2.5},
};

namespace
{

struct PairHash
{
  std::size_t operator()(const std::pair<int, int> & p) const noexcept
  {
    return (static_cast<std::size_t>(p.first) << 32) ^ static_cast<std::size_t>(p.second);
  }
};

std::pair<double, double> parse_origin(const YAML::Node & origin_field)
{
  if (origin_field.IsSequence() && origin_field.size() >= 2) {
    return {origin_field[0].as<double>(), origin_field[1].as<double>()};
  }
  if (origin_field.IsMap()) {
    return {
      origin_field["x"] ? origin_field["x"].as<double>() : 0.0,
      origin_field["y"] ? origin_field["y"].as<double>() : 0.0};
  }
  return {0.0, 0.0};
}

std::optional<std::pair<double, double>> transform_xy(
  tf2_ros::Buffer * tf_buffer,
  const std::string & from_frame,
  const std::string & to_frame,
  double x,
  double y)
{
  if (from_frame == to_frame) {
    return std::make_pair(x, y);
  }
  if (!tf_buffer) {
    return std::nullopt;
  }
  try {
    const auto transform = tf_buffer->lookupTransform(
      to_frame, from_frame, tf2::TimePointZero, tf2::durationFromSec(0.15));
    const double tx = transform.transform.translation.x;
    const double ty = transform.transform.translation.y;
    const double yaw = tf2::getYaw(transform.transform.rotation);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return std::make_pair(c * x - s * y + tx, s * x + c * y + ty);
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

}  // namespace

StaticMapIndex::StaticMapIndex(const std::filesystem::path & session_dir)
{
  const std::filesystem::path yaml_path = session_dir / "slam_map.yaml";
  const YAML::Node meta = YAML::LoadFile(yaml_path.string());

  resolution_ = meta["resolution"] ? meta["resolution"].as<double>() : 0.05;
  const auto origin = parse_origin(meta["origin"]);
  origin_x_ = origin.first;
  origin_y_ = origin.second;
  occupied_thresh_ = meta["occupied_thresh"] ? meta["occupied_thresh"].as<double>() : 0.65;
  negate_ = meta["negate"] ? meta["negate"].as<int>() : 0;

  std::string image_name = "slam_map.pgm";
  if (meta["image"]) {
    image_name = meta["image"].as<std::string>();
  }
  std::filesystem::path img_path = session_dir / image_name;
  if (!std::filesystem::is_regular_file(img_path)) {
    img_path = session_dir / "slam_map.pgm";
  }

  const cv::Mat gray = cv::imread(img_path.string(), cv::IMREAD_GRAYSCALE);
  if (gray.empty()) {
    throw std::runtime_error("cannot load static map image: " + img_path.string());
  }

  height_ = gray.rows;
  width_ = gray.cols;
  gray_.resize(static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_));
  std::memcpy(gray_.data(), gray.data, gray_.size());
}

bool StaticMapIndex::is_occupied(double wx, double wy) const
{
  const int px = static_cast<int>((wx - origin_x_) / resolution_);
  const int py = static_cast<int>(height_ - 1 - (wy - origin_y_) / resolution_);
  if (px < 0 || py < 0 || px >= width_ || py >= height_) {
    return true;
  }
  const int val = static_cast<int>(gray_[static_cast<std::size_t>(py) * static_cast<std::size_t>(width_) + px]);
  double occ_prob = 0.0;
  if (negate_) {
    occ_prob = val / 255.0;
  } else {
    occ_prob = (255 - val) / 255.0;
  }
  return occ_prob >= occupied_thresh_;
}

std::vector<std::pair<double, double>> extract_dynamic_obstacles(
  const nav_msgs::msg::OccupancyGrid & costmap,
  const StaticMapIndex & static_map,
  tf2_ros::Buffer * tf_buffer,
  const std::string & target_frame,
  int min_cost,
  int cell_stride)
{
  const auto & info = costmap.info;
  const auto & data = costmap.data;
  const int w = static_cast<int>(info.width);
  const int h = static_cast<int>(info.height);
  if (w == 0 || h == 0 || data.empty()) {
    return {};
  }

  const double res = info.resolution;
  const double ox = info.origin.position.x;
  const double oy = info.origin.position.y;
  const std::string src_frame = costmap.header.frame_id.empty() ? "map" : costmap.header.frame_id;

  std::vector<std::pair<double, double>> out;
  for (int my = 0; my < h; my += cell_stride) {
    const int row = my * w;
    for (int mx = 0; mx < w; mx += cell_stride) {
      const int cost = static_cast<int>(data[static_cast<std::size_t>(row + mx)]);
      if (cost < min_cost) {
        continue;
      }
      double wx = ox + (mx + 0.5) * res;
      double wy = oy + (my + 0.5) * res;
      if (tf_buffer != nullptr && src_frame != target_frame) {
        const auto mapped = transform_xy(tf_buffer, src_frame, target_frame, wx, wy);
        if (!mapped) {
          continue;
        }
        wx = mapped->first;
        wy = mapped->second;
      } else if (src_frame != target_frame) {
        continue;
      }
      if (!static_map.is_occupied(wx, wy)) {
        out.emplace_back(wx, wy);
      }
    }
  }
  return out;
}

std::vector<ObstacleRecord> obstacles_for_png(
  const std::vector<ObstacleRecord> & recorded,
  const StaticMapIndex & static_map,
  double merge_grid_m)
{
  std::vector<ObstacleRecord> merged = recorded;
  std::unordered_set<std::pair<int, int>, PairHash> seen;
  seen.reserve(merged.size());
  for (const auto & o : merged) {
    seen.insert({
      static_cast<int>(std::lround(o.x / merge_grid_m)),
      static_cast<int>(std::lround(o.y / merge_grid_m))});
  }

  for (const auto & [x, y] : GAZEBO_DYNAMIC_OBSTACLE_XY) {
    if (static_map.is_occupied(x, y)) {
      continue;
    }
    const auto key = std::make_pair(
      static_cast<int>(std::lround(x / merge_grid_m)),
      static_cast<int>(std::lround(y / merge_grid_m)));
    if (seen.count(key) > 0) {
      continue;
    }
    merged.push_back({x, y, "local_costmap_or_gazebo"});
    seen.insert(key);
  }
  return merged;
}

}  // namespace navigation_pkg
