#include "navigation_pkg/mission_recorder.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unordered_set>

#include <yaml-cpp/yaml.h>

#include "navigation_pkg/mission_plot.hpp"
#include "navigation_pkg/trajectory_tracker.hpp"

namespace navigation_pkg
{

namespace
{

std::string timestamp_now()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf{};
  localtime_r(&t, &tm_buf);
  std::ostringstream oss;
  oss << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
  return oss.str();
}

std::string iso_now()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf{};
  localtime_r(&t, &tm_buf);
  std::ostringstream oss;
  oss << std::put_time(&tm_buf, "%Y-%m-%dT%H:%M:%S");
  return oss.str();
}

YAML::Node waypoint_snapshot_node(const std::vector<Waypoint> & waypoints)
{
  YAML::Node root;
  root["waypoint_origin"] = "clip_detection";
  root["waypoint_origin_note"] = "NOT ground-truth paperclip positions";
  YAML::Node arr(YAML::NodeType::Sequence);
  for (const auto & w : waypoints) {
    YAML::Node wp;
    wp["id"] = w.id;
    wp["x"] = w.x;
    wp["y"] = w.y;
    wp["confidence"] = w.confidence;
    wp["source"] = w.source;
    arr.push_back(wp);
  }
  root["waypoints"] = arr;
  return root;
}

YAML::Node dynamic_obstacles_yaml(const std::vector<std::unordered_map<std::string, double>> & obs)
{
  YAML::Node arr(YAML::NodeType::Sequence);
  for (const auto & o : obs) {
    YAML::Node row;
    row["x"] = o.at("x");
    row["y"] = o.at("y");
    row["first_seen_sec"] = o.at("first_seen_sec");
    row["last_seen_sec"] = o.at("last_seen_sec");
    row["hits"] = static_cast<int>(o.at("hits"));
    arr.push_back(row);
  }
  return arr;
}

YAML::Node matrix_yaml(const std::vector<std::vector<double>> & mat)
{
  YAML::Node rows(YAML::NodeType::Sequence);
  for (const auto & row : mat) {
    YAML::Node r(YAML::NodeType::Sequence);
    for (double v : row) {
      r.push_back(v);
    }
    rows.push_back(r);
  }
  return rows;
}

YAML::Node matrix_yaml_int(const std::vector<std::vector<int>> & mat)
{
  YAML::Node rows(YAML::NodeType::Sequence);
  for (const auto & row : mat) {
    YAML::Node r(YAML::NodeType::Sequence);
    for (int v : row) {
      r.push_back(v);
    }
    rows.push_back(r);
  }
  return rows;
}

}  // namespace

MissionRecorder::MissionRecorder(
  const std::filesystem::path & session_dir,
  const std::vector<Waypoint> & waypoints,
  const std::optional<std::filesystem::path> & result_dir)
: session_dir_(session_dir),
  waypoints_(waypoints),
  result_root_(result_dir.value_or(resolve_result_dir())),
  mission_dir_(result_root_ / ("path_" + timestamp_now())),
  legs_dir_(mission_dir_ / "legs")
{
}

void MissionRecorder::start(double t_sec)
{
  start_time_ = t_sec;
  std::filesystem::create_directories(mission_dir_);
  std::filesystem::create_directories(legs_dir_);
  update_latest_symlink();
  save_progress(t_sec, "started");
}

void MissionRecorder::save_progress(
  double t_sec,
  const std::string & phase,
  const std::map<std::string, std::string> & extra)
{
  std::filesystem::create_directories(mission_dir_);

  YAML::Node progress;
  progress["phase"] = phase;
  progress["elapsed_sec"] = elapsed(t_sec);
  progress["trajectory_samples"] = static_cast<int>(trajectory_.size());
  progress["visits"] = static_cast<int>(visits_.size());
  progress["map_session_dir"] = session_dir_.string();
  progress["mission_dir"] = mission_dir_.string();
  progress["waypoint_origin"] = "clip_detection";
  progress["waypoint_origin_note"] =
    "waypoints.yaml from perception (CLIP paperclip detection); NOT ground-truth positions";
  progress["dynamic_obstacles"] = static_cast<int>(dynamic_obstacles_.size());
  for (const auto & [key, value] : extra) {
    progress[key] = value;
  }

  {
    std::ofstream f(mission_dir_ / "progress.yaml");
    f << progress;
  }

  const auto traj_maps = trajectory_snapshot();
  YAML::Node traj_root;
  traj_root["frame_id"] = "map";
  traj_root["integration"] = "odom_dead_reckon";
  YAML::Node samples(YAML::NodeType::Sequence);
  for (const auto & s : traj_maps) {
    YAML::Node row;
    row["t_sec"] = s.at("t_sec");
    row["x"] = s.at("x");
    row["y"] = s.at("y");
    row["yaw"] = s.at("yaw");
    samples.push_back(row);
  }
  traj_root["samples"] = samples;
  {
    std::ofstream f(mission_dir_ / "trajectory.yaml");
    f << traj_root;
  }

  {
    std::ofstream f(mission_dir_ / "waypoints_snapshot.yaml");
    f << waypoint_snapshot_node(waypoints_);
  }

  if (!trajectory_.empty() || !dynamic_obstacles_.empty()) {
    std::vector<int> visited_ids;
    for (const auto & v : visits_) {
      visited_ids.push_back(v.waypoint_id);
    }
    write_detected_obstacles_yaml();
    render_trajectory_png_stub(traj_maps, waypoints_, visited_ids);
  }

  YAML::Node mission;
  mission["frame_id"] = "map";
  mission["trajectory_integration"] = "odom_dead_reckon";
  YAML::Node traj_arr(YAML::NodeType::Sequence);
  for (const auto & s : traj_maps) {
    YAML::Node row;
    row["t_sec"] = s.at("t_sec");
    row["x"] = s.at("x");
    row["y"] = s.at("y");
    row["yaw"] = s.at("yaw");
    traj_arr.push_back(row);
  }
  mission["trajectory"] = traj_arr;

  YAML::Node order_ids(YAML::NodeType::Sequence);
  for (int idx : tsp_order_) {
    if (idx >= 0 && idx < static_cast<int>(waypoints_.size())) {
      order_ids.push_back(waypoints_[static_cast<std::size_t>(idx)].id);
    }
  }
  mission["tsp_order_ids"] = order_ids;
  mission["waypoint_origin"] = "clip_detection";
  mission["waypoints"] = waypoint_snapshot_node(waypoints_)["waypoints"];

  YAML::Node visits_node(YAML::NodeType::Sequence);
  for (const auto & v : visits_) {
    YAML::Node row;
    row["waypoint_id"] = v.waypoint_id;
    row["visited_at_sec"] = v.visited_at_sec;
    row["dwell_sec"] = v.dwell_sec;
    visits_node.push_back(row);
  }
  mission["visits"] = visits_node;
  mission["dynamic_obstacles"] = dynamic_obstacles_yaml(dynamic_obstacle_snapshot());

  {
    std::ofstream f(mission_dir_ / "mission.yaml");
    f << mission;
  }
}

double MissionRecorder::elapsed(double t_sec) const
{
  if (!start_time_) {
    return 0.0;
  }
  return t_sec - *start_time_;
}

void MissionRecorder::add_pose_sample(double t_sec, double x, double y, double yaw)
{
  trajectory_.push_back({elapsed(t_sec), x, y, yaw});
}

void MissionRecorder::add_dynamic_obstacles(
  double t_sec,
  const std::vector<std::pair<double, double>> & points)
{
  if (points.empty()) {
    return;
  }
  const double el = elapsed(t_sec);
  const double g = obstacle_grid_m_;
  for (const auto & [x, y] : points) {
    const auto key = std::make_pair(
      static_cast<int>(std::lround(x / g)),
      static_cast<int>(std::lround(y / g)));
    auto it = dynamic_obstacles_.find(key);
    if (it != dynamic_obstacles_.end()) {
      auto & obs = it->second;
      const int n = obs.hits + 1;
      obs.x = (obs.x * obs.hits + x) / n;
      obs.y = (obs.y * obs.hits + y) / n;
      obs.hits = n;
      obs.last_seen_sec = el;
    } else {
      dynamic_obstacles_[key] = MissionDynamicObstacle{x, y, el, el, 1};
    }
  }
}

void MissionRecorder::set_tsp(
  const std::vector<int> & order_indices,
  const std::vector<std::vector<double>> & distance_matrix,
  const std::vector<std::vector<int>> & adjacency_matrix,
  const std::optional<std::pair<double, double>> & start_pose,
  const std::optional<double> & optimal_manhattan_cost)
{
  tsp_order_ = order_indices;
  start_pose_ = start_pose;
  optimal_manhattan_cost_ = optimal_manhattan_cost;
  distance_matrix_ = distance_matrix;
  adjacency_matrix_ = adjacency_matrix;
}

void MissionRecorder::add_leg(
  int leg_index,
  int from_id,
  int to_id,
  const std::vector<std::pair<double, double>> & poses)
{
  MissionLegRecord leg{leg_index, from_id, to_id, poses};
  legs_.push_back(leg);

  char fname[128];
  std::snprintf(fname, sizeof(fname), "leg_%02d_%d_to_%d.yaml", leg_index, from_id, to_id);
  YAML::Node root;
  root["from_id"] = from_id;
  root["to_id"] = to_id;
  YAML::Node pose_arr(YAML::NodeType::Sequence);
  for (const auto & [px, py] : poses) {
    YAML::Node p;
    p["x"] = px;
    p["y"] = py;
    pose_arr.push_back(p);
  }
  root["poses"] = pose_arr;
  std::ofstream f(legs_dir_ / fname);
  f << root;
}

void MissionRecorder::record_visit(int waypoint_id, double t_sec, double dwell_sec)
{
  visits_.push_back({waypoint_id, elapsed(t_sec), dwell_sec});
}

std::filesystem::path MissionRecorder::finalize(
  double t_sec,
  const std::vector<int> & aborted)
{
  if (finalized_) {
    return mission_dir_;
  }
  finalized_ = true;
  std::filesystem::create_directories(mission_dir_);

  {
    std::ofstream f(mission_dir_ / "waypoints_snapshot.yaml");
    f << waypoint_snapshot_node(waypoints_);
  }

  YAML::Node order_ids(YAML::NodeType::Sequence);
  for (int idx : tsp_order_) {
    if (idx >= 0 && idx < static_cast<int>(waypoints_.size())) {
      order_ids.push_back(waypoints_[static_cast<std::size_t>(idx)].id);
    }
  }

  YAML::Node tsp_payload;
  tsp_payload["solver"] = "held_karp";
  tsp_payload["optimality"] = "globally_optimal_manhattan_fixed_start";
  if (start_pose_) {
    YAML::Node sp;
    sp["x"] = start_pose_->first;
    sp["y"] = start_pose_->second;
    tsp_payload["start_pose"] = sp;
  } else {
    tsp_payload["start_pose"] = YAML::Node(YAML::NodeType::Null);
  }
  if (optimal_manhattan_cost_) {
    tsp_payload["optimal_manhattan_cost_m"] = *optimal_manhattan_cost_;
  }
  YAML::Node order_idx(YAML::NodeType::Sequence);
  for (int i : tsp_order_) {
    order_idx.push_back(i);
  }
  tsp_payload["order_indices"] = order_idx;
  tsp_payload["order_ids"] = order_ids;
  tsp_payload["distance_matrix"] = matrix_yaml(distance_matrix_);
  tsp_payload["adjacency_matrix"] = matrix_yaml_int(adjacency_matrix_);
  {
    std::ofstream f(mission_dir_ / "tsp_order.yaml");
    f << tsp_payload;
  }

  const auto traj_maps = trajectory_snapshot();
  YAML::Node traj_root;
  traj_root["frame_id"] = "map";
  traj_root["integration"] = "odom_dead_reckon";
  YAML::Node samples(YAML::NodeType::Sequence);
  for (const auto & s : traj_maps) {
    YAML::Node row;
    row["t_sec"] = s.at("t_sec");
    row["x"] = s.at("x");
    row["y"] = s.at("y");
    row["yaw"] = s.at("yaw");
    samples.push_back(row);
  }
  traj_root["samples"] = samples;
  {
    std::ofstream f(mission_dir_ / "trajectory.yaml");
    f << traj_root;
  }

  YAML::Node summary;
  summary["duration_sec"] = elapsed(t_sec);
  YAML::Node visits_node(YAML::NodeType::Sequence);
  for (const auto & v : visits_) {
    YAML::Node row;
    row["waypoint_id"] = v.waypoint_id;
    row["visited_at_sec"] = v.visited_at_sec;
    row["dwell_sec"] = v.dwell_sec;
    visits_node.push_back(row);
  }
  summary["visits"] = visits_node;
  YAML::Node aborted_node(YAML::NodeType::Sequence);
  for (int id : aborted) {
    aborted_node.push_back(id);
  }
  summary["aborted_waypoint_ids"] = aborted_node;
  summary["leg_count"] = static_cast<int>(legs_.size());
  {
    std::ofstream f(mission_dir_ / "mission_summary.yaml");
    f << summary;
  }

  YAML::Node meta;
  meta["session_id"] = mission_dir_.filename().string();
  meta["map_session_dir"] = session_dir_.string();
  meta["created_at"] = iso_now();
  meta["duration_sec"] = elapsed(t_sec);
  meta["waypoint_count"] = static_cast<int>(waypoints_.size());
  meta["visit_count"] = static_cast<int>(visits_.size());
  meta["waypoint_origin"] = "clip_detection";
  meta["dynamic_obstacle_count"] = static_cast<int>(dynamic_obstacles_.size());
  {
    std::ofstream f(mission_dir_ / "session_meta.json");
    f << "{\n";
    f << "  \"session_id\": \"" << meta["session_id"].as<std::string>() << "\",\n";
    f << "  \"map_session_dir\": \"" << session_dir_.string() << "\",\n";
    f << "  \"created_at\": \"" << iso_now() << "\",\n";
    f << "  \"duration_sec\": " << elapsed(t_sec) << ",\n";
    f << "  \"waypoint_count\": " << waypoints_.size() << ",\n";
    f << "  \"visit_count\": " << visits_.size() << ",\n";
    f << "  \"waypoint_origin\": \"clip_detection\",\n";
    f << "  \"dynamic_obstacle_count\": " << dynamic_obstacles_.size() << "\n";
    f << "}\n";
  }

  YAML::Node mission;
  mission["frame_id"] = "map";
  mission["trajectory_integration"] = "odom_dead_reckon";
  mission["trajectory"] = samples;
  mission["tsp_order_ids"] = order_ids;
  mission["waypoint_origin"] = "clip_detection";
  mission["waypoints"] = waypoint_snapshot_node(waypoints_)["waypoints"];
  mission["visits"] = visits_node;
  mission["dynamic_obstacles"] = dynamic_obstacles_yaml(dynamic_obstacle_snapshot());
  {
    std::ofstream f(mission_dir_ / "mission.yaml");
    f << mission;
  }

  write_detected_obstacles_yaml();
  std::vector<int> visited_ids;
  for (const auto & v : visits_) {
    visited_ids.push_back(v.waypoint_id);
  }
  render_trajectory_png_stub(traj_maps, waypoints_, visited_ids);
  update_latest_symlink();
  return mission_dir_;
}

std::vector<std::unordered_map<std::string, double>> MissionRecorder::trajectory_snapshot() const
{
  std::vector<MissionTrajectorySample> filtered;
  filtered.reserve(trajectory_.size());
  std::vector<TrajectorySample> raw;
  for (const auto & s : trajectory_) {
    raw.push_back({s.x, s.y});
  }
  const auto kept = filter_trajectory_outliers(raw);
  std::size_t j = 0;
  for (const auto & s : trajectory_) {
    if (j < kept.size() && std::abs(s.x - kept[j].x) < 1e-9 && std::abs(s.y - kept[j].y) < 1e-9) {
      filtered.push_back(s);
      ++j;
    }
  }

  std::vector<std::unordered_map<std::string, double>> out;
  out.reserve(filtered.size());
  for (const auto & s : filtered) {
    out.push_back({
        {"t_sec", s.t_sec},
        {"x", s.x},
        {"y", s.y},
        {"yaw", s.yaw},
      });
  }
  return out;
}

std::vector<std::unordered_map<std::string, double>> MissionRecorder::dynamic_obstacle_snapshot() const
{
  std::vector<std::unordered_map<std::string, double>> out;
  for (const auto & [_, o] : dynamic_obstacles_) {
    out.push_back({
        {"x", o.x},
        {"y", o.y},
        {"first_seen_sec", o.first_seen_sec},
        {"last_seen_sec", o.last_seen_sec},
        {"hits", static_cast<double>(o.hits)},
      });
  }
  return out;
}

void MissionRecorder::write_detected_obstacles_yaml()
{
  YAML::Node root;
  root["frame_id"] = "map";
  root["source"] = "local_costmap_vs_static_slam";
  root["note"] = "Orange markers on mission_trajectory.png (local costmap vs static SLAM)";
  YAML::Node arr(YAML::NodeType::Sequence);
  for (const auto & o : dynamic_obstacle_snapshot()) {
    YAML::Node row;
    row["x"] = o.at("x");
    row["y"] = o.at("y");
    row["first_seen_sec"] = o.at("first_seen_sec");
    row["last_seen_sec"] = o.at("last_seen_sec");
    row["hits"] = static_cast<int>(o.at("hits"));
    arr.push_back(row);
  }
  root["obstacles"] = arr;
  std::ofstream f(mission_dir_ / "detected_obstacles.yaml");
  f << root;
}

void MissionRecorder::render_trajectory_png_stub(
  const std::vector<std::unordered_map<std::string, double>> & traj,
  const std::vector<Waypoint> & wp_snapshot,
  const std::vector<int> & visited_ids) const
{
  try {
    std::vector<MissionTrajectoryPoint> tpts;
    for (const auto & s : traj) {
      tpts.push_back({s.at("x"), s.at("y"), s.at("yaw")});
    }
    std::vector<MissionWaypointPoint> wpts;
    for (const auto & w : wp_snapshot) {
      wpts.push_back({w.id, w.x, w.y});
    }
    std::unordered_set<int> vis(visited_ids.begin(), visited_ids.end());
    render_mission_trajectory_png(
      session_dir_,
      mission_dir_ / MISSION_TRAJECTORY_PNG,
      tpts,
      wpts,
      vis);
  } catch (...) {
    // match Python: swallow render errors
  }
}

void MissionRecorder::update_latest_symlink()
{
  const auto latest = result_root_ / "path_latest";
  const auto target = mission_dir_.filename();
  std::error_code ec;
  if (std::filesystem::exists(latest)) {
    std::filesystem::remove(latest, ec);
  }
  std::filesystem::create_directory_symlink(target, latest, ec);
}

}  // namespace navigation_pkg
