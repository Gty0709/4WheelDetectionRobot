#include "navigation_pkg/mission_plot.hpp"

#include <cmath>
#include <fstream>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include "navigation_pkg/trajectory_tracker.hpp"

namespace navigation_pkg
{

namespace
{

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

std::pair<int, int> world_to_pixel(
  double x, double y,
  const std::pair<double, double> & origin_xy,
  double resolution,
  int height)
{
  const int px = static_cast<int>(std::lround((x - origin_xy.first) / resolution));
  const int py = static_cast<int>(std::lround(height - 1 - (y - origin_xy.second) / resolution));
  return {px, py};
}

cv::Mat load_map_bgr(
  const std::filesystem::path & session_dir,
  double & resolution,
  std::pair<double, double> & origin_xy)
{
  const auto yaml_path = session_dir / "slam_map.yaml";
  YAML::Node meta = YAML::LoadFile(yaml_path.string());
  resolution = meta["resolution"] ? meta["resolution"].as<double>() : 0.05;
  origin_xy = parse_origin(meta["origin"]);

  const std::string image_name = meta["image"] ? meta["image"].as<std::string>() : "slam_map.pgm";
  const std::vector<std::filesystem::path> candidates = {
    session_dir / image_name,
    session_dir / "slam_map.pgm",
    session_dir / "slam_map.png",
  };

  for (const auto & candidate : candidates) {
    if (!std::filesystem::is_regular_file(candidate)) {
      continue;
    }
    if (candidate.extension() == ".pgm") {
      cv::Mat gray = cv::imread(candidate.string(), cv::IMREAD_GRAYSCALE);
      if (!gray.empty()) {
        cv::Mat bgr;
        cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
      }
    }
    cv::Mat img = cv::imread(candidate.string(), cv::IMREAD_COLOR);
    if (!img.empty()) {
      return img;
    }
  }
  throw std::runtime_error("no map image under " + session_dir.string());
}

}  // namespace

std::filesystem::path render_mission_trajectory_png(
  const std::filesystem::path & map_session_dir,
  const std::filesystem::path & out_path,
  const std::vector<MissionTrajectoryPoint> & trajectory_in,
  const std::vector<MissionWaypointPoint> & waypoints,
  const std::optional<std::unordered_set<int>> & visited_ids)
{
  const std::unordered_set<int> visited = visited_ids.value_or(std::unordered_set<int>{});

  double resolution = 0.05;
  std::pair<double, double> origin_xy{0.0, 0.0};
  cv::Mat img = load_map_bgr(map_session_dir, resolution, origin_xy);
  const int h = img.rows;
  const int w = img.cols;

  std::vector<TrajectorySample> raw;
  raw.reserve(trajectory_in.size());
  for (const auto & s : trajectory_in) {
    raw.push_back({s.x, s.y});
  }
  const auto filtered = filter_trajectory_outliers(raw);

  std::vector<cv::Point> pts;
  pts.reserve(filtered.size());
  for (const auto & s : filtered) {
    const auto [px, py] = world_to_pixel(s.x, s.y, origin_xy, resolution, h);
    if (px >= 0 && px < w && py >= 0 && py < h) {
      pts.emplace_back(px, py);
    }
  }

  if (pts.size() >= 2) {
    cv::polylines(img, pts, false, cv::Scalar(40, 180, 255), 2, cv::LINE_AA);
  }

  for (const auto & wp : waypoints) {
    const auto [px, py] = world_to_pixel(wp.x, wp.y, origin_xy, resolution, h);
    if (px < 0 || px >= w || py < 0 || py >= h) {
      continue;
    }
    const bool vis = visited.count(wp.id) > 0;
    const cv::Scalar color = vis ? cv::Scalar(40, 220, 40) : cv::Scalar(220, 40, 40);
    const cv::Scalar edge = vis ? cv::Scalar(20, 140, 20) : cv::Scalar(140, 20, 20);
    cv::circle(img, {px, py}, 5, color, -1, cv::LINE_AA);
    cv::circle(img, {px, py}, 5, edge, 1, cv::LINE_AA);
  }

  if (!trajectory_in.empty()) {
    const auto & first = trajectory_in.front();
    const auto [sx, sy] = world_to_pixel(first.x, first.y, origin_xy, resolution, h);
    if (sx >= 0 && sx < w && sy >= 0 && sy < h) {
      cv::circle(img, {sx, sy}, 7, cv::Scalar(200, 200, 40), 2, cv::LINE_AA);
    }
    if (trajectory_in.size() > 1) {
      const auto & last = trajectory_in.back();
      const auto [ex, ey] = world_to_pixel(last.x, last.y, origin_xy, resolution, h);
      if (ex >= 0 && ex < w && ey >= 0 && ey < h) {
        const int dx = static_cast<int>(16.0 * std::cos(last.yaw));
        const int dy = static_cast<int>(-16.0 * std::sin(last.yaw));
        cv::arrowedLine(
          img, {ex, ey}, {ex + dx, ey + dy},
          cv::Scalar(40, 80, 200), 2, cv::LINE_AA, 0, 0.35);
      }
    }
  }

  std::filesystem::create_directories(out_path.parent_path());
  cv::imwrite(out_path.string(), img);
  return out_path;
}

}  // namespace navigation_pkg
