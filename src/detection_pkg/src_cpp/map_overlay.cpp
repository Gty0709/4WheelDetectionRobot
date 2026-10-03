#include "detection_pkg/map_overlay.hpp"

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace detection_pkg
{

namespace
{

constexpr const char * kWaypointsPng = "slam_map_waypoints.png";
constexpr const char * kBaseStem = "slam_map";
constexpr const char * kGtConfigName = "paperclips_small_house.yaml";

std::pair<double, double> parse_origin(const YAML::Node & origin_field)
{
  if (origin_field.IsSequence() && origin_field.size() >= 2) {
    return {origin_field[0].as<double>(), origin_field[1].as<double>()};
  }
  if (origin_field.IsMap()) {
    return {origin_field["x"].as<double>(0.0), origin_field["y"].as<double>(0.0)};
  }
  return {0.0, 0.0};
}

std::optional<std::filesystem::path> resolve_gt_config_path()
{
  std::vector<std::filesystem::path> candidates;
  try {
    const auto share = std::filesystem::path(
      ament_index_cpp::get_package_share_directory("perception_pkg"));
    candidates.push_back(share / "config" / kGtConfigName);
    if (share.has_parent_path()) {
      const auto ws_root = share.parent_path().parent_path().parent_path().parent_path();
      candidates.push_back(ws_root / "src" / "perception_pkg" / "config" / kGtConfigName);
    }
  } catch (const std::exception &) {
  }
  for (const auto & path : candidates) {
    if (std::filesystem::is_regular_file(path)) {
      return path;
    }
  }
  return std::nullopt;
}

YAML::Node load_map_meta(const std::filesystem::path & session_dir)
{
  const auto yaml_path = session_dir / (std::string(kBaseStem) + ".yaml");
  if (!std::filesystem::is_regular_file(yaml_path)) {
    throw std::runtime_error("map yaml not found: " + yaml_path.string());
  }
  return YAML::LoadFile(yaml_path.string());
}

std::vector<YAML::Node> load_waypoints(const std::filesystem::path & session_dir)
{
  const auto wp_path = session_dir / "waypoints.yaml";
  if (!std::filesystem::is_regular_file(wp_path)) {
    return {};
  }
  const YAML::Node data = YAML::LoadFile(wp_path.string());
  std::vector<YAML::Node> out;
  if (data["waypoints"] && data["waypoints"].IsSequence()) {
    for (const auto & wp : data["waypoints"]) {
      out.push_back(wp);
    }
  }
  return out;
}

std::optional<YAML::Node> load_initial_pose(const std::filesystem::path & session_dir)
{
  const auto pose_path = session_dir / "initial_pose.yaml";
  if (!std::filesystem::is_regular_file(pose_path)) {
    return std::nullopt;
  }
  const YAML::Node data = YAML::LoadFile(pose_path.string());
  if (!data["x"] || !data["y"]) {
    return std::nullopt;
  }
  return data;
}

std::tuple<double, double, double> load_map_odom_offset(const std::filesystem::path & session_dir)
{
  const auto path = session_dir / "map_odom_offset.yaml";
  if (!std::filesystem::is_regular_file(path)) {
    return {0.0, 0.0, 0.0};
  }
  const YAML::Node data = YAML::LoadFile(path.string());
  return {
    data["x"].as<double>(0.0),
    data["y"].as<double>(0.0),
    data["yaw"].as<double>(0.0),
  };
}

bool map_odom_offset_present(const std::filesystem::path & session_dir)
{
  return std::filesystem::is_regular_file(session_dir / "map_odom_offset.yaml");
}

std::vector<YAML::Node> load_paperclip_gt(const std::optional<std::filesystem::path> & gt_path)
{
  const auto path = gt_path.value_or(resolve_gt_config_path().value_or(std::filesystem::path{}));
  if (path.empty() || !std::filesystem::is_regular_file(path)) {
    return {};
  }
  const YAML::Node data = YAML::LoadFile(path.string());
  std::vector<YAML::Node> out;
  if (data["clips"] && data["clips"].IsSequence()) {
    for (const auto & clip : data["clips"]) {
      out.push_back(clip);
    }
  }
  return out;
}

std::pair<double, double> odom_xy_to_map(
  double x_odom, double y_odom, double offset_x, double offset_y, double offset_yaw)
{
  const double c = std::cos(offset_yaw);
  const double s = std::sin(offset_yaw);
  return {
    offset_x + c * x_odom - s * y_odom,
    offset_y + s * x_odom + c * y_odom,
  };
}

std::pair<int, int> world_to_pixel(
  double x, double y, const std::pair<double, double> & origin_xy, double resolution, int height)
{
  const int px = static_cast<int>(std::lround((x - origin_xy.first) / resolution));
  const int py = static_cast<int>(std::lround(height - 1 - (y - origin_xy.second) / resolution));
  return {px, py};
}

cv::Mat load_base_bgr(const std::filesystem::path & session_dir, const YAML::Node & meta)
{
  const std::string image_name = meta["image"] ?
    meta["image"].as<std::string>() : (std::string(kBaseStem) + ".pgm");
  const auto primary = session_dir / image_name;
  if (std::filesystem::is_regular_file(primary)) {
    if (primary.extension() == ".pgm") {
      cv::Mat gray = cv::imread(primary.string(), cv::IMREAD_GRAYSCALE);
      if (!gray.empty()) {
        cv::Mat bgr;
        cv::cvtColor(gray, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
      }
    } else {
      cv::Mat img = cv::imread(primary.string(), cv::IMREAD_COLOR);
      if (!img.empty()) {
        return img;
      }
    }
  }
  const auto png_path = session_dir / (std::string(kBaseStem) + ".png");
  if (std::filesystem::is_regular_file(png_path)) {
    cv::Mat img = cv::imread(png_path.string(), cv::IMREAD_COLOR);
    if (!img.empty()) {
      return img;
    }
  }
  throw std::runtime_error("no map image for " + session_dir.string());
}

}  // namespace

std::optional<std::filesystem::path> render_waypoints_overlay(
  const std::filesystem::path & session_dir,
  const std::string & out_name,
  bool draw_initial_pose,
  bool draw_gt,
  const std::optional<std::filesystem::path> & gt_config)
{
  const auto resolved = std::filesystem::weakly_canonical(session_dir);
  const YAML::Node meta = load_map_meta(resolved);
  const auto waypoints = load_waypoints(resolved);
  const auto gt_clips = draw_gt ? load_paperclip_gt(gt_config) : std::vector<YAML::Node>{};
  if (waypoints.empty() && gt_clips.empty()) {
    return std::nullopt;
  }

  const double resolution = meta["resolution"].as<double>(0.05);
  const auto origin_xy = parse_origin(meta["origin"]);
  cv::Mat img = load_base_bgr(resolved, meta);
  const int h = img.rows;
  const int w = img.cols;

  for (const auto & wp : waypoints) {
    const auto [px, py] = world_to_pixel(wp["x"].as<double>(), wp["y"].as<double>(), origin_xy, resolution, h);
    if (px >= 0 && px < w && py >= 0 && py < h) {
      cv::circle(img, cv::Point(px, py), 3, cv::Scalar(30, 220, 30), -1, cv::LINE_AA);
      cv::circle(img, cv::Point(px, py), 3, cv::Scalar(10, 120, 10), 1, cv::LINE_AA);
    }
  }

  if (draw_gt && !gt_clips.empty()) {
    if (!map_odom_offset_present(resolved)) {
      std::cerr <<
        "[map_overlay] WARNING: map_odom_offset.yaml missing; GT crosses may be misaligned.\n";
    }
    const auto [ox, oy, oyaw] = load_map_odom_offset(resolved);
    for (const auto & clip : gt_clips) {
      const auto [x_map, y_map] = odom_xy_to_map(
        clip["x"].as<double>(), clip["y"].as<double>(), ox, oy, oyaw);
      const auto [px, py] = world_to_pixel(x_map, y_map, origin_xy, resolution, h);
      if (px >= 0 && px < w && py >= 0 && py < h) {
        cv::circle(img, cv::Point(px, py), 1, cv::Scalar(40, 40, 240), -1, cv::LINE_AA);
      }
    }
  }

  if (draw_initial_pose) {
    const auto pose = load_initial_pose(resolved);
    if (pose.has_value()) {
      const double px_d = pose->operator[]("x").as<double>();
      const double py_d = pose->operator[]("y").as<double>();
      const double yaw = pose->operator[]("yaw").as<double>(0.0);
      const auto [px, py] = world_to_pixel(px_d, py_d, origin_xy, resolution, h);
      if (px >= 0 && px < w && py >= 0 && py < h) {
        cv::circle(img, cv::Point(px, py), 8, cv::Scalar(240, 120, 30), 2, cv::LINE_AA);
        const int dx = static_cast<int>(14 * std::cos(yaw));
        const int dy = static_cast<int>(-14 * std::sin(yaw));
        cv::arrowedLine(
          img, cv::Point(px, py), cv::Point(px + dx, py + dy),
          cv::Scalar(240, 120, 30), 2, cv::LINE_AA, 0, 0.35);
      }
    }
  }

  const auto out_path = resolved / out_name;
  cv::imwrite(out_path.string(), img);
  return out_path;
}

}  // namespace detection_pkg
