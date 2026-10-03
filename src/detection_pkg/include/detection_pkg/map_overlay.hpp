#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace detection_pkg
{

struct MapOverlayWaypoint
{
  double x{0.0};
  double y{0.0};
};

std::optional<std::filesystem::path> render_waypoints_overlay(
  const std::filesystem::path & session_dir,
  const std::string & out_name = "slam_map_waypoints.png",
  bool draw_initial_pose = true,
  bool draw_gt = true,
  const std::optional<std::filesystem::path> & gt_config = std::nullopt);

}  // namespace detection_pkg
