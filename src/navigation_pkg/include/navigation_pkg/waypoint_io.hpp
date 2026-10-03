#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace navigation_pkg
{

struct Waypoint
{
  int id{0};
  double x{0.0};
  double y{0.0};
  double confidence{1.0};
  std::string source{"ground"};
};

struct InitialPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::string frame_id{"map"};
};

std::filesystem::path resolve_ws_root();
std::filesystem::path resolve_session_dir(const std::optional<std::string> & session_dir = std::nullopt);
std::filesystem::path resolve_result_dir(const std::optional<std::string> & result_dir = std::nullopt);
std::filesystem::path resolve_path_latest(const std::optional<std::string> & result_dir = std::nullopt);

std::vector<Waypoint> load_waypoints(const std::filesystem::path & path);
InitialPose load_initial_pose(const std::filesystem::path & path);

std::unordered_map<std::string, std::filesystem::path> session_files(
  const std::filesystem::path & session_dir);

}  // namespace navigation_pkg
