#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

namespace navigation_pkg
{

inline constexpr const char * MISSION_TRAJECTORY_PNG = "mission_trajectory.png";

struct MissionTrajectoryPoint
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct MissionWaypointPoint
{
  int id{0};
  double x{0.0};
  double y{0.0};
};

std::filesystem::path render_mission_trajectory_png(
  const std::filesystem::path & map_session_dir,
  const std::filesystem::path & out_path,
  const std::vector<MissionTrajectoryPoint> & trajectory,
  const std::vector<MissionWaypointPoint> & waypoints,
  const std::optional<std::unordered_set<int>> & visited_ids = std::nullopt);

}  // namespace navigation_pkg
