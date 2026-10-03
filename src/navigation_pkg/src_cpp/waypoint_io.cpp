#include "navigation_pkg/waypoint_io.hpp"

#include <fstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace navigation_pkg
{

namespace
{

std::filesystem::path normalize_relative_path(
  const std::string & path_str,
  const std::filesystem::path & ws_root)
{
  std::filesystem::path p(path_str);
  p = p.lexically_normal();
  if (p.is_relative()) {
    p = ws_root / p;
  }
  return std::filesystem::absolute(p);
}

}  // namespace

std::filesystem::path resolve_ws_root()
{
  const std::filesystem::path here = std::filesystem::path(__FILE__).parent_path();
  for (std::filesystem::path parent = here; !parent.empty(); parent = parent.parent_path()) {
    if (std::filesystem::is_directory(parent / "src" / "perception_pkg")) {
      return parent;
    }
  }
  return std::filesystem::current_path();
}

std::filesystem::path resolve_session_dir(const std::optional<std::string> & session_dir)
{
  if (session_dir) {
    return normalize_relative_path(*session_dir, resolve_ws_root());
  }
  return std::filesystem::absolute(
    resolve_ws_root() / "src" / "perception_pkg" / "maps" / "map_latest");
}

std::filesystem::path resolve_result_dir(const std::optional<std::string> & result_dir)
{
  if (result_dir) {
    return normalize_relative_path(*result_dir, resolve_ws_root());
  }
  return std::filesystem::absolute(resolve_ws_root() / "src" / "navigation_pkg" / "result");
}

std::filesystem::path resolve_path_latest(const std::optional<std::string> & result_dir)
{
  const std::filesystem::path root = resolve_result_dir(result_dir);
  const std::filesystem::path latest = root / "path_latest";
  if (std::filesystem::is_symlink(latest)) {
    std::filesystem::path target = std::filesystem::read_symlink(latest);
    if (target.is_relative()) {
      target = latest.parent_path() / target;
    }
    return std::filesystem::absolute(target);
  }
  return std::filesystem::absolute(latest);
}

std::vector<Waypoint> load_waypoints(const std::filesystem::path & path)
{
  YAML::Node data = YAML::LoadFile(path.string());
  std::vector<Waypoint> out;
  if (!data["waypoints"]) {
    return out;
  }
  for (const auto & item : data["waypoints"]) {
    Waypoint wp;
    wp.id = item["id"].as<int>();
    wp.x = item["x"].as<double>();
    wp.y = item["y"].as<double>();
    wp.confidence = item["confidence"] ? item["confidence"].as<double>() : 1.0;
    wp.source = item["source"] ? item["source"].as<std::string>() : "ground";
    out.push_back(wp);
  }
  return out;
}

InitialPose load_initial_pose(const std::filesystem::path & path)
{
  YAML::Node data = YAML::LoadFile(path.string());
  InitialPose pose;
  pose.x = data["x"].as<double>();
  pose.y = data["y"].as<double>();
  pose.yaw = data["yaw"].as<double>();
  pose.frame_id = data["frame_id"] ? data["frame_id"].as<std::string>() : "map";
  return pose;
}

std::unordered_map<std::string, std::filesystem::path> session_files(
  const std::filesystem::path & session_dir)
{
  return {
    {"map_yaml", session_dir / "slam_map.yaml"},
    {"waypoints_yaml", session_dir / "waypoints.yaml"},
    {"initial_pose_yaml", session_dir / "initial_pose.yaml"},
  };
}

}  // namespace navigation_pkg
