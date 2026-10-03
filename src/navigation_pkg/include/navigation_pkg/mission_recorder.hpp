#pragma once

#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "navigation_pkg/waypoint_io.hpp"

namespace navigation_pkg
{

struct MissionTrajectorySample
{
  double t_sec{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct MissionLegRecord
{
  int leg_index{0};
  int from_id{-1};
  int to_id{-1};
  std::vector<std::pair<double, double>> poses;
};

struct MissionVisitRecord
{
  int waypoint_id{0};
  double visited_at_sec{0.0};
  double dwell_sec{0.0};
};

struct MissionDynamicObstacle
{
  double x{0.0};
  double y{0.0};
  double first_seen_sec{0.0};
  double last_seen_sec{0.0};
  int hits{1};
};

class MissionRecorder
{
public:
  MissionRecorder(
    const std::filesystem::path & session_dir,
    const std::vector<Waypoint> & waypoints,
    const std::optional<std::filesystem::path> & result_dir = std::nullopt);

  const std::filesystem::path & mission_dir() const { return mission_dir_; }

  void start(double t_sec);
  void save_progress(
    double t_sec,
    const std::string & phase,
    const std::map<std::string, std::string> & extra = {});
  double elapsed(double t_sec) const;

  void add_pose_sample(double t_sec, double x, double y, double yaw);
  void add_dynamic_obstacles(double t_sec, const std::vector<std::pair<double, double>> & points);

  void set_tsp(
    const std::vector<int> & order_indices,
    const std::vector<std::vector<double>> & distance_matrix,
    const std::vector<std::vector<int>> & adjacency_matrix,
    const std::optional<std::pair<double, double>> & start_pose = std::nullopt,
    const std::optional<double> & optimal_manhattan_cost = std::nullopt);

  void add_leg(
    int leg_index,
    int from_id,
    int to_id,
    const std::vector<std::pair<double, double>> & poses);

  void record_visit(int waypoint_id, double t_sec, double dwell_sec);

  std::filesystem::path finalize(double t_sec, const std::vector<int> & aborted = {});

private:
  std::vector<std::unordered_map<std::string, double>> trajectory_snapshot() const;
  std::vector<std::unordered_map<std::string, double>> dynamic_obstacle_snapshot() const;
  void write_detected_obstacles_yaml();
  void render_trajectory_png_stub(
    const std::vector<std::unordered_map<std::string, double>> & traj,
    const std::vector<Waypoint> & wp_snapshot,
    const std::vector<int> & visited_ids) const;
  void update_latest_symlink();

  std::filesystem::path session_dir_;
  std::vector<Waypoint> waypoints_;
  std::filesystem::path result_root_;
  std::filesystem::path mission_dir_;
  std::filesystem::path legs_dir_;
  std::optional<double> start_time_;
  std::vector<MissionTrajectorySample> trajectory_;
  std::vector<MissionLegRecord> legs_;
  std::vector<MissionVisitRecord> visits_;
  std::vector<int> tsp_order_;
  std::optional<std::pair<double, double>> start_pose_;
  std::optional<double> optimal_manhattan_cost_;
  std::vector<std::vector<double>> distance_matrix_;
  std::vector<std::vector<int>> adjacency_matrix_;
  bool finalized_{false};
  std::map<std::pair<int, int>, MissionDynamicObstacle> dynamic_obstacles_;
  double obstacle_grid_m_{0.18};
};

}  // namespace navigation_pkg
