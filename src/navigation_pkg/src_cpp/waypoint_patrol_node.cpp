#include "navigation_pkg/waypoint_patrol_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include <action_msgs/msg/goal_status.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <visualization_msgs/msg/marker.hpp>

#include "navigation_pkg/collision_check.hpp"
#include "navigation_pkg/manhattan_tsp.hpp"

namespace navigation_pkg
{

namespace
{

double now_sec(rclcpp::Node & node)
{
  return node.get_clock()->now().seconds();
}

rclcpp::QoS costmap_qos()
{
  return rclcpp::QoS(1).transient_local().reliable();
}

bool parse_double_flag(int & i, int argc, char ** argv, const char * flag, double & out)
{
  if (std::string(argv[i]) != flag || i + 1 >= argc) {
    return false;
  }
  out = std::stod(argv[++i]);
  return true;
}

bool parse_int_flag(int & i, int argc, char ** argv, const char * flag, int & out)
{
  if (std::string(argv[i]) != flag || i + 1 >= argc) {
    return false;
  }
  out = std::stoi(argv[++i]);
  return true;
}

}  // namespace

PatrolParams parse_patrol_args(int argc, char ** argv)
{
  PatrolParams p;
  for (int i = 1; i < argc; ++i) {
    const std::string flag = argv[i];
    if (flag == "--session-dir" && i + 1 < argc) {
      p.session_dir = argv[++i];
      continue;
    }
    if (parse_double_flag(i, argc, argv, "--dwell-sec", p.dwell_sec)) { continue; }
    if (parse_double_flag(i, argc, argv, "--position-tolerance", p.position_tolerance)) { continue; }
    if (parse_double_flag(i, argc, argv, "--approach-tolerance", p.approach_tolerance)) { continue; }
    if (parse_double_flag(i, argc, argv, "--stall-time-sec", p.stall_time_sec)) { continue; }
    if (parse_double_flag(i, argc, argv, "--stall-spin-time-sec", p.stall_spin_time_sec)) { continue; }
    if (parse_double_flag(i, argc, argv, "--stall-min-travel-m", p.stall_min_travel_m)) { continue; }
    if (parse_double_flag(i, argc, argv, "--relocalize-settle-sec", p.relocalize_settle_sec)) { continue; }
    if (parse_double_flag(i, argc, argv, "--relocalize-min-sec", p.relocalize_min_sec)) { continue; }
    if (parse_double_flag(i, argc, argv, "--relocalize-min-shift", p.relocalize_min_shift)) { continue; }
    if (parse_double_flag(i, argc, argv, "--relocalize-timeout-sec", p.relocalize_timeout_sec)) { continue; }
    if (parse_int_flag(i, argc, argv, "--max-nav-retries", p.max_nav_retries)) { continue; }
    if (parse_int_flag(i, argc, argv, "--sweep-max-nav-retries", p.sweep_max_nav_retries)) { continue; }
    if (parse_double_flag(i, argc, argv, "--sweep-approach-tolerance", p.sweep_approach_tolerance)) { continue; }
    if (parse_int_flag(i, argc, argv, "--max-sweep-rounds", p.max_sweep_rounds)) { continue; }
    if (parse_double_flag(i, argc, argv, "--retry-delay-sec", p.retry_delay_sec)) { continue; }
    if (parse_double_flag(i, argc, argv, "--quick-retry-delay-sec", p.quick_retry_delay_sec)) { continue; }
    if (parse_double_flag(i, argc, argv, "--backup-dist", p.backup_dist)) { continue; }
    if (parse_double_flag(i, argc, argv, "--backup-speed", p.backup_speed)) { continue; }
    if (parse_int_flag(i, argc, argv, "--max-backup-attempts", p.max_backup_attempts)) { continue; }
    if (parse_int_flag(i, argc, argv, "--high-cost-threshold", p.high_cost_threshold)) { continue; }
    if (parse_double_flag(i, argc, argv, "--marker-scale", p.marker_scale)) { continue; }
    if (parse_double_flag(i, argc, argv, "--marker-z", p.marker_z)) { continue; }
    if (flag == "--help" || flag == "-h") {
      std::cout <<
        "usage: waypoint_patrol_cpp [--session-dir DIR] [--dwell-sec 0.75] ...\n";
      std::exit(0);
    }
    if (flag.rfind("--ros-args", 0) == 0) {
      break;
    }
  }
  return p;
}

PatrolParams load_patrol_params(rclcpp::Node & node, const PatrolParams & cli_defaults)
{
  PatrolParams p;
  p.session_dir = node.declare_parameter("session_dir", cli_defaults.session_dir);
  p.dwell_sec = node.declare_parameter("dwell_sec", cli_defaults.dwell_sec);
  p.position_tolerance = node.declare_parameter("position_tolerance", cli_defaults.position_tolerance);
  p.approach_tolerance = node.declare_parameter("approach_tolerance", cli_defaults.approach_tolerance);
  p.stall_time_sec = node.declare_parameter("stall_time_sec", cli_defaults.stall_time_sec);
  p.stall_spin_time_sec = node.declare_parameter("stall_spin_time_sec", cli_defaults.stall_spin_time_sec);
  p.stall_min_travel_m = node.declare_parameter("stall_min_travel_m", cli_defaults.stall_min_travel_m);
  p.relocalize_settle_sec = node.declare_parameter("relocalize_settle_sec", cli_defaults.relocalize_settle_sec);
  p.relocalize_min_sec = node.declare_parameter("relocalize_min_sec", cli_defaults.relocalize_min_sec);
  p.relocalize_min_shift = node.declare_parameter("relocalize_min_shift", cli_defaults.relocalize_min_shift);
  p.relocalize_timeout_sec = node.declare_parameter("relocalize_timeout_sec", cli_defaults.relocalize_timeout_sec);
  p.max_nav_retries = node.declare_parameter("max_nav_retries", cli_defaults.max_nav_retries);
  p.sweep_max_nav_retries = node.declare_parameter("sweep_max_nav_retries", cli_defaults.sweep_max_nav_retries);
  p.sweep_approach_tolerance = node.declare_parameter("sweep_approach_tolerance", cli_defaults.sweep_approach_tolerance);
  p.max_sweep_rounds = node.declare_parameter("max_sweep_rounds", cli_defaults.max_sweep_rounds);
  p.retry_delay_sec = node.declare_parameter("retry_delay_sec", cli_defaults.retry_delay_sec);
  p.quick_retry_delay_sec = node.declare_parameter("quick_retry_delay_sec", cli_defaults.quick_retry_delay_sec);
  p.backup_dist = node.declare_parameter("backup_dist", cli_defaults.backup_dist);
  p.backup_speed = node.declare_parameter("backup_speed", cli_defaults.backup_speed);
  p.max_backup_attempts = node.declare_parameter("max_backup_attempts", cli_defaults.max_backup_attempts);
  p.high_cost_threshold = node.declare_parameter("high_cost_threshold", cli_defaults.high_cost_threshold);
  p.marker_scale = node.declare_parameter("marker_scale", cli_defaults.marker_scale);
  p.marker_z = node.declare_parameter("marker_z", cli_defaults.marker_z);
  return p;
}

geometry_msgs::msg::Quaternion WaypointPatrolNode::yaw_to_quat(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double WaypointPatrolNode::quat_to_yaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z));
}

std::string WaypointPatrolNode::phase_name(PatrolPhase phase)
{
  switch (phase) {
    case PatrolPhase::WAIT_NAV2: return "wait_nav2";
    case PatrolPhase::WAIT_RELOCALIZE: return "wait_relocalize";
    case PatrolPhase::DWELL: return "dwell";
    case PatrolPhase::COMPUTE_TSP: return "compute_tsp";
    case PatrolPhase::GO_NEXT: return "go_next";
    case PatrolPhase::BACKUP: return "backup";
    case PatrolPhase::SPIN: return "spin";
    case PatrolPhase::SAVE: return "save";
    case PatrolPhase::DONE: return "done";
  }
  return "unknown";
}

WaypointPatrolNode::WaypointPatrolNode(const PatrolParams & params)
: Node("waypoint_patrol"),
  params_(params),
  session_dir_(resolve_session_dir(
      params.session_dir.empty() ? std::nullopt : std::optional<std::string>(params.session_dir))),
  recorder_(session_dir_, std::vector<Waypoint>{}),
  static_map_(session_dir_)
{
  params_ = load_patrol_params(*this, params);
  session_dir_ = resolve_session_dir(
    params_.session_dir.empty() ? std::nullopt : std::optional<std::string>(params_.session_dir));
  const auto files = session_files(session_dir_);
  waypoints_ = load_waypoints(files.at("waypoints_yaml"));
  if (waypoints_.empty()) {
    throw std::runtime_error("No waypoints in " + files.at("waypoints_yaml").string());
  }
  recorder_ = MissionRecorder(session_dir_, waypoints_);
  static_map_ = StaticMapIndex(session_dir_);

  RCLCPP_INFO(
    get_logger(),
    "Patrol targets: %zu positions from waypoints.yaml "
    "(CLIP paperclip detection on map; NOT ground-truth GT)",
    waypoints_.size());

  if (std::filesystem::is_regular_file(files.at("initial_pose_yaml"))) {
    prior_pose_ = load_initial_pose(files.at("initial_pose_yaml"));
    RCLCPP_INFO(
      get_logger(),
      "AMCL mapping prior (map frame): x=%.2f y=%.2f yaw=%.2f from %s",
      prior_pose_->x, prior_pose_->y, prior_pose_->yaw,
      files.at("initial_pose_yaml").string().c_str());
  }

  const double t0 = now_sec(*this);
  recorder_.start(t0);
  RCLCPP_INFO(get_logger(), "Mission recording: %s", recorder_.mission_dir().string().c_str());

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  backup_client_ = rclcpp_action::create_client<BackUp>(this, "backup");
  spin_client_ = rclcpp_action::create_client<Spin>(this, "spin");
  clear_local_costmap_ = create_client<nav2_msgs::srv::ClearEntireCostmap>(
    "/local_costmap/clear_entirely_local_costmap");
  clear_global_costmap_ = create_client<nav2_msgs::srv::ClearEntireCostmap>(
    "/global_costmap/clear_entirely_global_costmap");

  initialpose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

  costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/global_costmap/costmap", costmap_qos(),
    std::bind(&WaypointPatrolNode::on_costmap, this, std::placeholders::_1));
  local_costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/local_costmap/costmap", costmap_qos(),
    std::bind(&WaypointPatrolNode::on_local_costmap, this, std::placeholders::_1));
  plan_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/plan", 10, std::bind(&WaypointPatrolNode::on_plan, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&WaypointPatrolNode::on_odom, this, std::placeholders::_1));

  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/navigation/waypoint_markers", 10);
  driven_pub_ = create_publisher<nav_msgs::msg::Path>("/navigation/driven_path", 10);
  driven_path_.header.frame_id = "map";

  sample_timer_ = create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&WaypointPatrolNode::sample_pose, this));
  tick_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&WaypointPatrolNode::tick, this));
  autosave_timer_ = create_wall_timer(
    std::chrono::milliseconds(3000),
    std::bind(&WaypointPatrolNode::autosave, this));
  obstacle_scan_timer_ = create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&WaypointPatrolNode::scan_dynamic_obstacles, this));

  RCLCPP_INFO(
    get_logger(), "Loaded %zu waypoints from %s",
    waypoints_.size(), session_dir_.string().c_str());
  publish_markers();
}

WaypointPatrolNode::~WaypointPatrolNode()
{
  if (phase_ != PatrolPhase::DONE && phase_ != PatrolPhase::SAVE) {
    try {
      recorder_.finalize(now_sec(*this), aborted_ids_);
    } catch (...) {
    }
  }
}

void WaypointPatrolNode::flush_on_shutdown()
{
  try {
    const double t = now_sec(*this);
    if (phase_ == PatrolPhase::DONE || phase_ == PatrolPhase::SAVE) {
      recorder_.save_progress(t, "interrupted", {});
    } else {
      recorder_.finalize(t, aborted_ids_);
    }
  } catch (...) {
  }
}

void WaypointPatrolNode::on_costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  costmap_ = msg;
}

void WaypointPatrolNode::on_local_costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  local_costmap_ = msg;
}

void WaypointPatrolNode::on_plan(const nav_msgs::msg::Path::SharedPtr msg)
{
  last_plan_poses_.clear();
  last_plan_poses_.reserve(msg->poses.size());
  for (const auto & p : msg->poses) {
    last_plan_poses_.emplace_back(p.pose.position.x, p.pose.position.y);
  }
}

void WaypointPatrolNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & pos = msg->pose.pose.position;
  last_odom_pose_ = Pose3{pos.x, pos.y, quat_to_yaw(msg->pose.pose.orientation)};
}

void WaypointPatrolNode::scan_dynamic_obstacles()
{
  if (!local_costmap_ || phase_ == PatrolPhase::WAIT_NAV2 || phase_ == PatrolPhase::DONE) {
    return;
  }
  const auto points = extract_dynamic_obstacles(
    *local_costmap_, static_map_, tf_buffer_.get(), "map");
  if (points.empty()) {
    return;
  }
  recorder_.add_dynamic_obstacles(now_sec(*this), points);
}

void WaypointPatrolNode::anchor_trajectory(const Pose3 & map_pose)
{
  if (traj_tracker_.anchored() || !last_odom_pose_) {
    return;
  }
  traj_tracker_.reset_anchor(map_pose, *last_odom_pose_);
  RCLCPP_INFO(
    get_logger(),
    "Driven trace anchored: odom integration in map frame (AMCL jumps filtered)");
}

std::optional<WaypointPatrolNode::Pose3> WaypointPatrolNode::get_pose()
{
  try {
    const auto t = tf_buffer_->lookupTransform(
      "map", "base_footprint", tf2::TimePointZero, tf2::durationFromSec(0.2));
    return Pose3{
      t.transform.translation.x,
      t.transform.translation.y,
      quat_to_yaw(t.transform.rotation)};
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

std::optional<WaypointPatrolNode::Pose3> WaypointPatrolNode::trace_pose()
{
  if (!last_odom_pose_) {
    return std::nullopt;
  }
  if (!traj_tracker_.anchored()) {
    if (phase_ == PatrolPhase::WAIT_NAV2 || phase_ == PatrolPhase::WAIT_RELOCALIZE) {
      return std::nullopt;
    }
    const auto map_pose = get_pose();
    if (map_pose) {
      anchor_trajectory(*map_pose);
    }
    if (!traj_tracker_.anchored()) {
      return std::nullopt;
    }
  }
  return traj_tracker_.update_odom(*last_odom_pose_);
}

void WaypointPatrolNode::sample_pose()
{
  const auto pose = trace_pose();
  if (!pose) {
    return;
  }
  const auto [x, y, yaw] = *pose;
  recorder_.add_pose_sample(now_sec(*this), x, y, yaw);
  publish_driven_path(x, y, yaw);
}

void WaypointPatrolNode::publish_driven_path(double x, double y, double yaw)
{
  if (last_driven_xy_) {
    const double dx = x - last_driven_xy_->first;
    const double dy = y - last_driven_xy_->second;
    if (std::hypot(dx, dy) < 0.05) {
      return;
    }
  }
  last_driven_xy_ = std::make_pair(x, y);
  geometry_msgs::msg::PoseStamped ps;
  ps.header.frame_id = "map";
  ps.header.stamp = get_clock()->now();
  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.orientation = yaw_to_quat(yaw);
  driven_path_.poses.push_back(ps);
  driven_path_.header.stamp = ps.header.stamp;
  driven_pub_->publish(driven_path_);
}

bool WaypointPatrolNode::at_waypoint(const Waypoint & wp)
{
  const auto pose = get_pose();
  if (!pose) {
    return false;
  }
  const auto [x, y, yaw] = *pose;
  if (std::hypot(x - wp.x, y - wp.y) > params_.position_tolerance) {
    return false;
  }
  if (!costmap_) {
    return true;
  }
  const auto & info = costmap_->info;
  return footprint_free(
    wp.x, wp.y, yaw,
    static_cast<int>(info.width), static_cast<int>(info.height),
    info.resolution, info.origin.position.x, info.origin.position.y,
    costmap_->data);
}

void WaypointPatrolNode::publish_markers()
{
  visualization_msgs::msg::MarkerArray arr;
  for (const auto & wp : waypoints_) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = get_clock()->now();
    m.ns = "detected_paperclips";
    m.id = wp.id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = wp.x;
    m.pose.position.y = wp.y;
    m.pose.position.z = params_.marker_z;
    m.scale.x = params_.marker_scale;
    m.scale.y = params_.marker_scale;
    m.scale.z = params_.marker_scale;
    if (visited_ids_.count(wp.id) > 0) {
      m.color.r = 0.1f;
      m.color.g = 0.95f;
      m.color.b = 0.1f;
      m.color.a = 0.95f;
    } else {
      m.color.r = 0.95f;
      m.color.g = 0.1f;
      m.color.b = 0.1f;
      m.color.a = 0.95f;
    }
    arr.markers.push_back(m);
  }
  marker_pub_->publish(arr);
}

void WaypointPatrolNode::autosave()
{
  if (phase_ == PatrolPhase::DONE || phase_ == PatrolPhase::SAVE) {
    return;
  }
  recorder_.save_progress(now_sec(*this), phase_name(phase_), {});
}

bool WaypointPatrolNode::footprint_cost_high()
{
  const auto pose = get_pose();
  if (!pose || !costmap_) {
    return false;
  }
  const auto [x, y, yaw] = *pose;
  const auto & info = costmap_->info;
  const int mc = footprint_max_cost(
    x, y, yaw,
    static_cast<int>(info.width), static_cast<int>(info.height),
    info.resolution, info.origin.position.x, info.origin.position.y,
    costmap_->data);
  if (mc >= params_.high_cost_threshold) {
    return true;
  }
  return !footprint_free(
    x, y, yaw,
    static_cast<int>(info.width), static_cast<int>(info.height),
    info.resolution, info.origin.position.x, info.origin.position.y,
    costmap_->data);
}

void WaypointPatrolNode::backup_to_free(ContinueFn continue_fn)
{
  if (backup_attempts_ >= params_.max_backup_attempts) {
    RCLCPP_WARN(get_logger(), "Backup attempts exhausted; continuing anyway");
    backup_attempts_ = 0;
    continue_fn();
    return;
  }
  if (!backup_client_->wait_for_action_server(std::chrono::seconds(0))) {
    RCLCPP_WARN(get_logger(), "backup action server unavailable; skipping backup");
    continue_fn();
    return;
  }
  backup_continue_ = std::move(continue_fn);
  phase_ = PatrolPhase::BACKUP;

  BackUp::Goal goal;
  goal.target.x = params_.backup_dist;
  goal.target.y = 0.0;
  goal.target.z = 0.0;
  goal.speed = params_.backup_speed;
  goal.time_allowance = builtin_interfaces::msg::Duration();
  goal.time_allowance.sec = 20;

  RCLCPP_INFO(
    get_logger(),
    "Backing up %.2f m (attempt %d/%d)",
    params_.backup_dist, backup_attempts_ + 1, params_.max_backup_attempts);

  auto send_options = rclcpp_action::Client<BackUp>::SendGoalOptions();
  send_options.goal_response_callback =
    std::bind(&WaypointPatrolNode::backup_goal_response_callback, this, std::placeholders::_1);
  send_options.result_callback =
    std::bind(&WaypointPatrolNode::backup_result_callback, this, std::placeholders::_1);
  backup_client_->async_send_goal(goal, send_options);
}

void WaypointPatrolNode::run_backup_again()
{
  BackUp::Goal goal;
  goal.target.x = params_.backup_dist;
  goal.target.y = 0.0;
  goal.target.z = 0.0;
  goal.speed = params_.backup_speed;
  goal.time_allowance = builtin_interfaces::msg::Duration();
  goal.time_allowance.sec = 20;

  auto send_options = rclcpp_action::Client<BackUp>::SendGoalOptions();
  send_options.goal_response_callback =
    std::bind(&WaypointPatrolNode::backup_goal_response_callback, this, std::placeholders::_1);
  send_options.result_callback =
    std::bind(&WaypointPatrolNode::backup_result_callback, this, std::placeholders::_1);
  backup_client_->async_send_goal(goal, send_options);
}

void WaypointPatrolNode::clear_costmaps(bool local_only)
{
  auto req = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
  std::vector<rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr> clients;
  clients.push_back(clear_local_costmap_);
  if (!local_only) {
    clients.push_back(clear_global_costmap_);
  }
  for (const auto & client : clients) {
    if (client->service_is_ready()) {
      client->async_send_request(req);
    }
  }
}

std::optional<double> WaypointPatrolNode::estimate_escape_spin_yaw()
{
  const auto pose = get_pose();
  if (!pose || !costmap_) {
    return std::nullopt;
  }
  const auto [x, y, yaw] = *pose;
  const auto & info = costmap_->info;
  const double free_h = estimate_free_heading(
    x, y, yaw,
    static_cast<int>(info.width), static_cast<int>(info.height),
    info.resolution, info.origin.position.x, info.origin.position.y,
    costmap_->data);
  const double target_yaw = backup_target_yaw(yaw, free_h);
  const double delta = std::atan2(
    std::sin(target_yaw - yaw), std::cos(target_yaw - yaw));
  if (std::abs(delta) < 0.12) {
    return std::nullopt;
  }
  return delta;
}

void WaypointPatrolNode::begin_escape_recovery(ContinueFn continue_fn)
{
  const auto spin_delta = estimate_escape_spin_yaw();
  if (spin_delta) {
    spin_then_backup(*spin_delta, std::move(continue_fn));
    return;
  }
  backup_to_free(std::move(continue_fn));
}

void WaypointPatrolNode::spin_then_backup(double delta_yaw, ContinueFn continue_fn)
{
  if (!spin_client_->wait_for_action_server(std::chrono::seconds(0))) {
    RCLCPP_WARN(get_logger(), "spin action server unavailable; backup directly");
    backup_to_free(std::move(continue_fn));
    return;
  }
  spin_continue_ = std::move(continue_fn);
  phase_ = PatrolPhase::SPIN;

  Spin::Goal goal;
  goal.target_yaw = delta_yaw;
  goal.time_allowance = builtin_interfaces::msg::Duration();
  goal.time_allowance.sec = 12;

  RCLCPP_INFO(get_logger(), "Spin %.2f rad toward freer space before backup", delta_yaw);

  auto send_options = rclcpp_action::Client<Spin>::SendGoalOptions();
  send_options.goal_response_callback =
    std::bind(&WaypointPatrolNode::spin_goal_response_callback, this, std::placeholders::_1);
  send_options.result_callback =
    std::bind(&WaypointPatrolNode::spin_result_callback, this, std::placeholders::_1);
  spin_client_->async_send_goal(goal, send_options);
}

void WaypointPatrolNode::finish_spin(bool continue_anyway)
{
  auto fn = std::move(spin_continue_);
  spin_continue_ = nullptr;
  phase_ = PatrolPhase::GO_NEXT;
  if (!fn) {
    return;
  }
  if (continue_anyway) {
    fn();
    return;
  }
  backup_to_free(std::move(fn));
}

void WaypointPatrolNode::finish_backup(bool continue_anyway)
{
  (void)continue_anyway;
  auto fn = std::move(backup_continue_);
  backup_continue_ = nullptr;
  backup_attempts_ = 0;
  phase_ = PatrolPhase::GO_NEXT;
  if (fn) {
    fn();
  }
}

std::optional<Waypoint> WaypointPatrolNode::current_tsp_wp()
{
  if (tsp_cursor_ < 0 || tsp_cursor_ >= static_cast<int>(tsp_targets_.size())) {
    return std::nullopt;
  }
  return tsp_targets_[tsp_cursor_];
}

std::optional<double> WaypointPatrolNode::dist_to_current_waypoint()
{
  const auto wp = current_tsp_wp();
  if (!wp) {
    return std::nullopt;
  }
  const auto pose = get_pose();
  if (!pose) {
    return std::nullopt;
  }
  return std::hypot(std::get<0>(*pose) - wp->x, std::get<1>(*pose) - wp->y);
}

double WaypointPatrolNode::approach_tolerance_m() const
{
  return in_sweep_pass_ ? params_.sweep_approach_tolerance : params_.approach_tolerance;
}

int WaypointPatrolNode::max_nav_retries() const
{
  return in_sweep_pass_ ? params_.sweep_max_nav_retries : params_.max_nav_retries;
}

std::vector<Waypoint> WaypointPatrolNode::unvisited_targets()
{
  std::vector<Waypoint> out;
  for (const auto & wp : tsp_targets_) {
    if (visited_ids_.count(wp.id) == 0) {
      out.push_back(wp);
    }
  }
  return out;
}

bool WaypointPatrolNode::try_approach_success()
{
  const auto dist = dist_to_current_waypoint();
  const auto wp = current_tsp_wp();
  const double tol = approach_tolerance_m();
  if (!wp || !dist || *dist > tol) {
    return false;
  }
  RCLCPP_INFO(
    get_logger(),
    "Approach success: within %.2f m of TSP leg id=%d (%.2f, %.2f); marking visited",
    *dist, wp->id, wp->x, wp->y);
  mark_visited(*wp);
  nav_retries_ = 0;
  phase_ = PatrolPhase::DWELL;
  dwell_start_ = get_clock()->now();
  return true;
}

void WaypointPatrolNode::send_goal(const std::optional<Waypoint> & wp)
{
  std::optional<Waypoint> target = wp;
  if (!target) {
    target = current_tsp_wp();
  }
  if (!target) {
    RCLCPP_ERROR(get_logger(), "No TSP target for current leg");
    return;
  }

  double yaw = 0.0;
  const auto pose = get_pose();
  if (pose) {
    yaw = std::get<2>(*pose);
  }

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = get_clock()->now();
  goal.pose.pose.position.x = target->x;
  goal.pose.pose.position.y = target->y;
  goal.pose.pose.orientation = yaw_to_quat(yaw);

  stall_anchor_ = std::nullopt;
  ignore_next_nav_result_ = false;
  const int leg_no = tsp_cursor_ + 1;
  const int total = static_cast<int>(tsp_targets_.size());
  RCLCPP_INFO(
    get_logger(), "TSP leg %d/%d: id=%d goal=(%.4f, %.4f)",
    leg_no, total, target->id, target->x, target->y);

  auto send_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_options.goal_response_callback =
    std::bind(&WaypointPatrolNode::nav_goal_response_callback, this, std::placeholders::_1);
  send_options.result_callback =
    std::bind(&WaypointPatrolNode::nav_result_callback, this, std::placeholders::_1);
  nav_client_->async_send_goal(goal, send_options);
}

void WaypointPatrolNode::nav_goal_response_callback(const GoalHandleNavigate::SharedPtr & goal_handle)
{
  nav_goal_handle_ = goal_handle;
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal rejected by Nav2");
    on_nav_failed();
  }
}

void WaypointPatrolNode::nav_result_callback(const GoalHandleNavigate::WrappedResult & result)
{
  if (ignore_next_nav_result_) {
    ignore_next_nav_result_ = false;
    return;
  }
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    const auto wp = current_tsp_wp();
    if (!wp) {
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "Navigation leg succeeded (TSP id=%d at %.2f, %.2f); dwell %.2fs",
      wp->id, wp->x, wp->y, params_.dwell_sec);
    mark_visited(*wp);
    nav_retries_ = 0;
    phase_ = PatrolPhase::DWELL;
    dwell_start_ = get_clock()->now();
    return;
  }
  RCLCPP_WARN(
    get_logger(), "Navigation leg failed status=%d",
    static_cast<int>(result.code));
  on_nav_failed();
}

void WaypointPatrolNode::on_nav_failed()
{
  if (handling_nav_failure_) {
    return;
  }
  handling_nav_failure_ = true;
  try {
    on_nav_failed_impl();
  } catch (...) {
  }
  handling_nav_failure_ = false;
  stall_anchor_ = std::nullopt;
}

void WaypointPatrolNode::cancel_nav_goal()
{
  if (nav_goal_handle_) {
    nav_client_->async_cancel_goal(nav_goal_handle_);
  }
}

void WaypointPatrolNode::schedule_nav_retry(double delay_sec, bool local_only)
{
  if (retry_timer_) {
    retry_timer_->cancel();
    retry_timer_.reset();
  }
  retry_timer_ = create_wall_timer(
    std::chrono::duration<double>(delay_sec),
    [this, local_only]() {
      if (retry_timer_) {
        retry_timer_->cancel();
        retry_timer_.reset();
      }
      if (!current_tsp_wp()) {
        return;
      }
      clear_costmaps(local_only);
      const char * label = local_only ? "local clear" : "clear";
      RCLCPP_INFO(get_logger(), "Re-sending TSP leg goal after %s and wait", label);
      send_goal();
    });
}

void WaypointPatrolNode::on_nav_failed_impl()
{
  cancel_nav_goal();
  if (try_approach_success()) {
    return;
  }
  nav_retries_ += 1;
  const int max_retries = max_nav_retries();
  if (nav_retries_ == 1) {
    RCLCPP_INFO(
      get_logger(),
      "Nav failed; quick local clear + retry in %.1fs (1/%d)",
      params_.quick_retry_delay_sec, max_retries);
    schedule_nav_retry(params_.quick_retry_delay_sec, true);
    return;
  }
  if (nav_retries_ <= max_retries) {
    RCLCPP_INFO(
      get_logger(),
      "Nav failed; directed escape then retry in %.1fs (%d/%d)",
      params_.retry_delay_sec, nav_retries_, max_retries);
    begin_escape_recovery([this]() { schedule_nav_retry(params_.retry_delay_sec, false); });
    return;
  }
  if (try_approach_success()) {
    return;
  }
  const auto wp = current_tsp_wp();
  if (!wp) {
    return;
  }
  if (in_sweep_pass_) {
    handle_sweep_leg_failed(*wp);
    return;
  }
  RCLCPP_ERROR(
    get_logger(),
    "Deferring TSP leg id=%d (%.2f, %.2f) to sweep pass",
    wp->id, wp->x, wp->y);
  nav_retries_ = 0;
  advance_after_dwell(true);
}

void WaypointPatrolNode::check_nav_stall()
{
  if (phase_ != PatrolPhase::GO_NEXT || !nav_goal_handle_) {
    stall_anchor_ = std::nullopt;
    return;
  }
  const auto pose = get_pose();
  const auto dist = dist_to_current_waypoint();
  if (!pose || !dist) {
    return;
  }
  const auto now = get_clock()->now();
  if (!stall_anchor_) {
    stall_anchor_ = StallAnchor{
      std::get<0>(*pose), std::get<1>(*pose), *dist, now, std::get<2>(*pose)};
    return;
  }
  const double elapsed = (now - stall_anchor_->time).seconds();
  const double moved = std::hypot(
    std::get<0>(*pose) - stall_anchor_->x,
    std::get<1>(*pose) - stall_anchor_->y);
  const double dyaw = std::abs(std::atan2(
    std::sin(std::get<2>(*pose) - stall_anchor_->yaw),
    std::cos(std::get<2>(*pose) - stall_anchor_->yaw)));

  if (*dist < stall_anchor_->dist - 0.05) {
    stall_anchor_ = StallAnchor{
      std::get<0>(*pose), std::get<1>(*pose), *dist, now, std::get<2>(*pose)};
    return;
  }

  if (moved < 0.15 && dyaw > 0.35) {
    if (elapsed >= params_.stall_spin_time_sec && *dist >= stall_anchor_->dist - 0.03) {
      const auto wp = current_tsp_wp();
      if (!wp) {
        return;
      }
      RCLCPP_WARN(
        get_logger(),
        "Nav spin-stall on TSP leg id=%d: dyaw %.2f rad in %.0fs, goal dist %.2f m",
        wp->id, dyaw, elapsed, *dist);
      stall_anchor_ = std::nullopt;
      ignore_next_nav_result_ = true;
      cancel_nav_goal();
      on_nav_failed();
      return;
    }
    stall_anchor_->x = std::get<0>(*pose);
    stall_anchor_->y = std::get<1>(*pose);
    stall_anchor_->dist = *dist;
    stall_anchor_->yaw = std::get<2>(*pose);
    return;
  }

  if (
    elapsed >= params_.stall_time_sec &&
    moved >= params_.stall_min_travel_m &&
    *dist >= stall_anchor_->dist - 0.03)
  {
    const auto wp = current_tsp_wp();
    if (!wp) {
      return;
    }
    RCLCPP_WARN(
      get_logger(),
      "Nav stall on TSP leg id=%d (%.2f, %.2f): moved %.2f m in %.0fs but goal dist %.2f m "
      "(was %.2f m)",
      wp->id, wp->x, wp->y, moved, elapsed, *dist, stall_anchor_->dist);
    stall_anchor_ = std::nullopt;
    ignore_next_nav_result_ = true;
    cancel_nav_goal();
    on_nav_failed();
  }
}

void WaypointPatrolNode::save_leg(int to_id)
{
  const int from_id = last_waypoint_id_;
  recorder_.add_leg(leg_index_, from_id, to_id, last_plan_poses_);
  leg_index_ += 1;
  last_plan_poses_.clear();
  last_waypoint_id_ = to_id;
  std::map<std::string, std::string> extra;
  extra["to_id"] = std::to_string(to_id);
  recorder_.save_progress(now_sec(*this), "leg_saved", extra);
}

void WaypointPatrolNode::mark_visited(const Waypoint & wp)
{
  visited_ids_.insert(wp.id);
  recorder_.record_visit(wp.id, now_sec(*this), params_.dwell_sec);
  std::map<std::string, std::string> extra;
  extra["waypoint_id"] = std::to_string(wp.id);
  recorder_.save_progress(now_sec(*this), "visited", extra);
  publish_markers();
}

void WaypointPatrolNode::begin_sweep_leg_for_wp(const Waypoint & wp)
{
  for (std::size_t i = 0; i < tsp_targets_.size(); ++i) {
    if (tsp_targets_[i].id == wp.id) {
      tsp_cursor_ = static_cast<int>(i);
      current_goal_idx_ = tsp_order_[i];
      break;
    }
  }
  in_sweep_pass_ = true;
  nav_retries_ = 0;
  phase_ = PatrolPhase::GO_NEXT;
  if (footprint_cost_high()) {
    RCLCPP_INFO(
      get_logger(), "Robot in inflated cost before sweep leg id=%d; backup first", wp.id);
    begin_escape_recovery([this, wp]() { send_goal(wp); });
    return;
  }
  send_goal(wp);
}

bool WaypointPatrolNode::start_next_sweep_round()
{
  const auto unvisited = unvisited_targets();
  if (unvisited.empty()) {
    return false;
  }
  if (sweep_round_ >= params_.max_sweep_rounds) {
    return false;
  }
  sweep_round_ += 1;
  std::string ids_str;
  for (const auto & wp : unvisited) {
    if (!ids_str.empty()) {
      ids_str += ", ";
    }
    ids_str += std::to_string(wp.id);
  }
  RCLCPP_WARN(
    get_logger(),
    "TSP sweep round %d/%d: %zu unvisited ids=[%s]",
    sweep_round_, params_.max_sweep_rounds, unvisited.size(), ids_str.c_str());
  clear_costmaps(false);
  begin_sweep_leg_for_wp(unvisited.front());
  return true;
}

void WaypointPatrolNode::handle_sweep_leg_failed(const Waypoint & wp)
{
  clear_costmaps(false);
  nav_retries_ = 0;
  const auto unvisited = unvisited_targets();
  std::vector<Waypoint> remaining;
  for (const auto & u : unvisited) {
    if (u.id != wp.id) {
      remaining.push_back(u);
    }
  }
  if (!remaining.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "Sweep could not reach id=%d; trying next unvisited id=%d",
      wp.id, remaining.front().id);
    begin_escape_recovery([this, next = remaining.front()]() { begin_sweep_leg_for_wp(next); });
    return;
  }
  in_sweep_pass_ = false;
  if (start_next_sweep_round()) {
    return;
  }
  finalize_mission_incomplete();
}

void WaypointPatrolNode::finalize_mission_incomplete()
{
  const auto unvisited = unvisited_targets();
  aborted_ids_.clear();
  for (const auto & wp : unvisited) {
    aborted_ids_.push_back(wp.id);
  }
  std::string detail;
  for (const auto & wp : unvisited) {
    detail += "(" + std::to_string(wp.id) + ", " +
      std::to_string(wp.x) + ", " + std::to_string(wp.y) + ") ";
  }
  RCLCPP_ERROR(
    get_logger(),
    "Mission finished with %zu unvisited TSP targets after %d sweep round(s): %s",
    unvisited.size(), sweep_round_, detail.c_str());
  phase_ = PatrolPhase::SAVE;
}

void WaypointPatrolNode::start_next_tsp_leg()
{
  while (tsp_cursor_ < static_cast<int>(tsp_targets_.size())) {
    const auto & wp = tsp_targets_[tsp_cursor_];
    current_goal_idx_ = tsp_order_[tsp_cursor_];
    if (visited_ids_.count(wp.id) > 0) {
      tsp_cursor_ += 1;
      continue;
    }
    in_sweep_pass_ = false;
    phase_ = PatrolPhase::GO_NEXT;
    nav_retries_ = 0;
    if (footprint_cost_high()) {
      RCLCPP_INFO(
        get_logger(), "Robot in inflated cost before TSP leg id=%d; backup first", wp.id);
      begin_escape_recovery([this, wp]() { send_goal(wp); });
      return;
    }
    send_goal(wp);
    return;
  }

  const auto unvisited = unvisited_targets();
  if (!unvisited.empty()) {
    if (start_next_sweep_round()) {
      return;
    }
    finalize_mission_incomplete();
    return;
  }

  RCLCPP_INFO(get_logger(), "All TSP legs visited; saving mission");
  phase_ = PatrolPhase::SAVE;
}

void WaypointPatrolNode::advance_after_dwell(bool skipped)
{
  const auto wp = current_tsp_wp();
  if (!skipped && wp) {
    save_leg(wp->id);
  }

  if (in_sweep_pass_) {
    const auto unvisited = unvisited_targets();
    if (!unvisited.empty()) {
      begin_sweep_leg_for_wp(unvisited.front());
      return;
    }
    in_sweep_pass_ = false;
    if (start_next_sweep_round()) {
      return;
    }
    if (!unvisited_targets().empty()) {
      finalize_mission_incomplete();
      return;
    }
    RCLCPP_INFO(get_logger(), "All TSP legs visited after sweep; saving mission");
    phase_ = PatrolPhase::SAVE;
    return;
  }

  tsp_cursor_ += 1;
  start_next_tsp_leg();
}

void WaypointPatrolNode::publish_amcl_prior()
{
  if (!prior_pose_ || initialpose_sent_) {
    return;
  }
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = prior_pose_->frame_id;
  msg.header.stamp = get_clock()->now();
  msg.pose.pose.position.x = prior_pose_->x;
  msg.pose.pose.position.y = prior_pose_->y;
  msg.pose.pose.orientation = yaw_to_quat(prior_pose_->yaw);
  msg.pose.covariance[0] = 0.05;
  msg.pose.covariance[7] = 0.05;
  msg.pose.covariance[35] = 0.02;
  initialpose_pub_->publish(msg);
  initialpose_sent_ = true;
  RCLCPP_INFO(get_logger(), "Published /initialpose with mapping prior for AMCL relocalization");
}

bool WaypointPatrolNode::relocalize_ready(const Pose3 & pose, const rclcpp::Time & now)
{
  if (!relocalize_start_) {
    return false;
  }
  const double elapsed = (now - *relocalize_start_).seconds();
  if (elapsed < params_.relocalize_min_sec) {
    return false;
  }

  if (!prior_pose_) {
    return elapsed >= params_.relocalize_settle_sec;
  }

  const double shift = std::hypot(
    std::get<0>(pose) - prior_pose_->x,
    std::get<1>(pose) - prior_pose_->y);

  if (shift < params_.relocalize_min_shift) {
    if (!shift_stable_start_) {
      shift_stable_start_ = now;
    }
    const double stable = (now - *shift_stable_start_).seconds();
    if (stable >= params_.relocalize_settle_sec) {
      RCLCPP_INFO(
        get_logger(),
        "AMCL aligned with mapping prior (shift=%.2f m); starting navigation", shift);
      return true;
    }
    return false;
  }

  if (!shift_stable_start_) {
    shift_stable_start_ = now;
    RCLCPP_INFO(
      get_logger(), "AMCL shifted %.2f m from mapping prior; waiting stable...", shift);
    return false;
  }
  const double stable = (now - *shift_stable_start_).seconds();
  if (stable >= params_.relocalize_settle_sec) {
    RCLCPP_INFO(
      get_logger(), "Relocalization converged: shift=%.2f m from prior", shift);
    return true;
  }
  return false;
}

void WaypointPatrolNode::begin_tsp_from_pose(const Pose3 & pose)
{
  anchor_trajectory(pose);
  start_pose_ = pose;
  phase_ = PatrolPhase::COMPUTE_TSP;
  std::map<std::string, std::string> extra;
  extra["start_x"] = std::to_string(std::get<0>(pose));
  extra["start_y"] = std::to_string(std::get<1>(pose));
  extra["start_yaw"] = std::to_string(std::get<2>(pose));
  recorder_.save_progress(now_sec(*this), "relocalized", extra);
  RCLCPP_INFO(
    get_logger(),
    "Relocalized at (%.2f, %.2f); computing optimal TSP from here",
    std::get<0>(pose), std::get<1>(pose));
}

void WaypointPatrolNode::compute_and_start_tsp()
{
  if (!start_pose_ || tsp_computing_) {
    return;
  }
  tsp_computing_ = true;
  const double sx = std::get<0>(*start_pose_);
  const double sy = std::get<1>(*start_pose_);
  const auto waypoints_copy = waypoints_;

  std::thread([this, sx, sy, waypoints_copy]() {
    TspWorkerResult result;
    try {
      const auto [order, d_mat, a_mat, total_cost] = tsp_from_start_pose(waypoints_copy, sx, sy);
      result.order = order;
      result.distance_matrix = d_mat;
      result.adjacency_matrix = a_mat;
      result.total_cost = total_cost;
      result.failed = false;
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(get_logger(), "TSP computation failed: %s", exc.what());
      result.failed = true;
    }
    std::lock_guard<std::mutex> lock(tsp_mutex_);
    tsp_result_ = std::move(result);
  }).detach();
}

void WaypointPatrolNode::apply_tsp_result()
{
  std::lock_guard<std::mutex> lock(tsp_mutex_);
  if (!tsp_result_ || !start_pose_) {
    return;
  }
  auto result = std::move(*tsp_result_);
  tsp_result_ = std::nullopt;
  tsp_computing_ = false;

  tsp_order_ = result.order;
  tsp_targets_.clear();
  for (int idx : tsp_order_) {
    if (idx >= 0 && idx < static_cast<int>(waypoints_.size())) {
      tsp_targets_.push_back(waypoints_[idx]);
    }
  }
  in_sweep_pass_ = false;
  sweep_round_ = 0;
  const double sx = std::get<0>(*start_pose_);
  const double sy = std::get<1>(*start_pose_);
  recorder_.set_tsp(
    tsp_order_, result.distance_matrix, result.adjacency_matrix,
    std::make_pair(sx, sy), result.total_cost);

  std::string ids_str;
  std::string coords_str;
  for (const auto & wp : tsp_targets_) {
    if (!ids_str.empty()) {
      ids_str += ", ";
      coords_str += " ";
    }
    ids_str += std::to_string(wp.id);
    coords_str += "id=" + std::to_string(wp.id) + "(" +
      std::to_string(wp.x).substr(0, std::to_string(wp.x).find('.') + 3) + "," +
      std::to_string(wp.y).substr(0, std::to_string(wp.y).find('.') + 3) + ")";
  }
  RCLCPP_INFO(
    get_logger(),
    "Optimal Manhattan TSP (Held-Karp, globally exact): ids=[%s] total=%.3f m",
    ids_str.c_str(), result.total_cost);
  RCLCPP_INFO(get_logger(), "TSP targets frozen:%s", coords_str.c_str());

  tsp_cursor_ = 0;
  if (tsp_targets_.empty()) {
    phase_ = PatrolPhase::SAVE;
    return;
  }
  start_next_tsp_leg();
}

void WaypointPatrolNode::backup_goal_response_callback(const GoalHandleBackUp::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_WARN(get_logger(), "Backup goal rejected");
    finish_backup(true);
    return;
  }
  (void)goal_handle;
}

void WaypointPatrolNode::backup_result_callback(const GoalHandleBackUp::WrappedResult & result)
{
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_WARN(get_logger(), "Backup failed status=%d", static_cast<int>(result.code));
    finish_backup(true);
    return;
  }
  backup_attempts_ += 1;
  if (footprint_cost_high() && backup_attempts_ < params_.max_backup_attempts) {
    RCLCPP_INFO(get_logger(), "Still near inflated cost after backup; backing up again");
    run_backup_again();
    return;
  }
  RCLCPP_INFO(get_logger(), "Backup complete; footprint in freer space");
  finish_backup(false);
}

void WaypointPatrolNode::spin_goal_response_callback(const GoalHandleSpin::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_WARN(get_logger(), "Spin goal rejected; backup directly");
    finish_spin(true);
  }
}

void WaypointPatrolNode::spin_result_callback(const GoalHandleSpin::WrappedResult & result)
{
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_WARN(
      get_logger(), "Spin failed status=%d; backup anyway",
      static_cast<int>(result.code));
  }
  finish_spin(false);
}

void WaypointPatrolNode::tick()
{
  if (phase_ == PatrolPhase::WAIT_NAV2) {
    if (nav_client_->wait_for_action_server(std::chrono::seconds(0))) {
      RCLCPP_INFO(get_logger(), "Nav2 ready; waiting for AMCL relocalization");
      phase_ = PatrolPhase::WAIT_RELOCALIZE;
      relocalize_start_ = get_clock()->now();
      publish_amcl_prior();
      recorder_.save_progress(now_sec(*this), "wait_relocalize", {});
    } else {
      const auto now = get_clock()->now();
      if (!nav2_wait_log_time_) {
        nav2_wait_log_time_ = now;
      } else if ((now - *nav2_wait_log_time_).seconds() >= 10.0) {
        RCLCPP_WARN(
          get_logger(),
          "Still waiting for Nav2 navigate_to_pose action server. "
          "Terminal 2 must show lifecycle_manager_navigation: "
          "Managed nodes are active (wait ~25–30s after terminal 2 start).");
        nav2_wait_log_time_ = now;
      }
    }
    return;
  }

  if (phase_ == PatrolPhase::WAIT_RELOCALIZE) {
    const auto pose = get_pose();
    if (!pose) {
      const auto now = get_clock()->now();
      if (!relocalize_log_time_) {
        relocalize_log_time_ = now;
      } else if ((now - *relocalize_log_time_).seconds() >= 8.0) {
        RCLCPP_WARN(get_logger(), "Waiting for map→base_footprint TF (AMCL not publishing yet)");
        relocalize_log_time_ = now;
      }
      return;
    }
    const auto now = get_clock()->now();
    const double elapsed = (now - *relocalize_start_).seconds();
    if (elapsed >= params_.relocalize_timeout_sec) {
      RCLCPP_WARN(
        get_logger(),
        "Relocalization timeout (%.0fs); starting TSP with current AMCL pose", elapsed);
      begin_tsp_from_pose(*pose);
      return;
    }
    if (relocalize_ready(*pose, now)) {
      begin_tsp_from_pose(*pose);
    }
    return;
  }

  if (phase_ == PatrolPhase::COMPUTE_TSP) {
    bool ready = false;
    {
      std::lock_guard<std::mutex> lock(tsp_mutex_);
      ready = tsp_result_.has_value();
    }
    if (ready) {
      apply_tsp_result();
      return;
    }
    if (!tsp_computing_) {
      compute_and_start_tsp();
    }
    return;
  }

  if (phase_ == PatrolPhase::GO_NEXT) {
    check_nav_stall();
    return;
  }

  if (phase_ == PatrolPhase::DWELL) {
    if (!dwell_start_) {
      dwell_start_ = get_clock()->now();
      return;
    }
    const double elapsed = (get_clock()->now() - *dwell_start_).seconds();
    if (elapsed >= params_.dwell_sec) {
      dwell_start_ = std::nullopt;
      advance_after_dwell(false);
    }
    return;
  }

  if (phase_ == PatrolPhase::SAVE) {
    const auto out = recorder_.finalize(now_sec(*this), aborted_ids_);
    RCLCPP_INFO(get_logger(), "Mission saved to %s", out.string().c_str());
    phase_ = PatrolPhase::DONE;
  }
}

}  // namespace navigation_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto cli = navigation_pkg::parse_patrol_args(argc, argv);

  std::shared_ptr<navigation_pkg::WaypointPatrolNode> node;
  try {
    node = std::make_shared<navigation_pkg::WaypointPatrolNode>(cli);
  } catch (const std::exception & exc) {
    std::cerr << exc.what() << "\n";
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
