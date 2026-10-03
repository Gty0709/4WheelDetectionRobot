#pragma once

#include <atomic>
#include <filesystem>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <tuple>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nav2_msgs/action/back_up.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>

#include "navigation_pkg/mission_recorder.hpp"
#include "navigation_pkg/obstacle_detect.hpp"
#include "navigation_pkg/trajectory_tracker.hpp"
#include "navigation_pkg/waypoint_io.hpp"

namespace navigation_pkg
{

enum class PatrolPhase
{
  WAIT_NAV2,
  WAIT_RELOCALIZE,
  DWELL,
  COMPUTE_TSP,
  GO_NEXT,
  BACKUP,
  SPIN,
  SAVE,
  DONE
};

struct PatrolParams
{
  std::string session_dir;
  double dwell_sec{0.75};
  double position_tolerance{0.12};
  double approach_tolerance{0.40};
  double stall_time_sec{12.0};
  double stall_spin_time_sec{8.0};
  double stall_min_travel_m{0.25};
  double relocalize_settle_sec{2.0};
  double relocalize_min_sec{5.0};
  double relocalize_min_shift{0.25};
  double relocalize_timeout_sec{25.0};
  int max_nav_retries{3};
  int sweep_max_nav_retries{6};
  double sweep_approach_tolerance{0.55};
  int max_sweep_rounds{4};
  double retry_delay_sec{1.0};
  double quick_retry_delay_sec{0.8};
  double backup_dist{0.40};
  double backup_speed{0.18};
  int max_backup_attempts{3};
  int high_cost_threshold{140};
  double marker_scale{0.15};
  double marker_z{0.05};
};

struct TspWorkerResult
{
  std::vector<int> order;
  std::vector<std::vector<double>> distance_matrix;
  std::vector<std::vector<int>> adjacency_matrix;
  double total_cost{0.0};
  bool failed{false};
};

PatrolParams parse_patrol_args(int argc, char ** argv);
PatrolParams load_patrol_params(rclcpp::Node & node, const PatrolParams & cli_defaults);

class WaypointPatrolNode : public rclcpp::Node
{
public:
  explicit WaypointPatrolNode(const PatrolParams & params);

  ~WaypointPatrolNode() override;

  void flush_on_shutdown();

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using BackUp = nav2_msgs::action::BackUp;
  using Spin = nav2_msgs::action::Spin;
  using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using GoalHandleBackUp = rclcpp_action::ClientGoalHandle<BackUp>;
  using GoalHandleSpin = rclcpp_action::ClientGoalHandle<Spin>;

  using Pose3 = std::tuple<double, double, double>;
  using ContinueFn = std::function<void()>;

  struct StallAnchor
  {
    double x{0.0};
    double y{0.0};
    double dist{0.0};
    rclcpp::Time time;
    double yaw{0.0};
  };

  static geometry_msgs::msg::Quaternion yaw_to_quat(double yaw);
  static double quat_to_yaw(const geometry_msgs::msg::Quaternion & q);
  static std::string phase_name(PatrolPhase phase);

  void on_costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void on_local_costmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void on_plan(const nav_msgs::msg::Path::SharedPtr msg);
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scan_dynamic_obstacles();
  void sample_pose();
  void tick();
  void autosave();

  void anchor_trajectory(const Pose3 & map_pose);
  std::optional<Pose3> get_pose();
  std::optional<Pose3> trace_pose();
  void publish_driven_path(double x, double y, double yaw);
  bool at_waypoint(const Waypoint & wp);
  void publish_markers();

  bool footprint_cost_high();
  void backup_to_free(ContinueFn continue_fn);
  void run_backup_again();
  void clear_costmaps(bool local_only = false);
  std::optional<double> estimate_escape_spin_yaw();
  void begin_escape_recovery(ContinueFn continue_fn);
  void spin_then_backup(double delta_yaw, ContinueFn continue_fn);
  void finish_spin(bool continue_anyway);
  void finish_backup(bool continue_anyway);

  std::optional<Waypoint> current_tsp_wp();
  std::optional<double> dist_to_current_waypoint();
  double approach_tolerance_m() const;
  int max_nav_retries() const;
  std::vector<Waypoint> unvisited_targets();
  bool try_approach_success();

  void send_goal(const std::optional<Waypoint> & wp = std::nullopt);
  void on_nav_failed();
  void on_nav_failed_impl();
  void cancel_nav_goal();
  void schedule_nav_retry(double delay_sec, bool local_only = false);

  void check_nav_stall();
  void save_leg(int to_id);
  void mark_visited(const Waypoint & wp);
  void begin_sweep_leg_for_wp(const Waypoint & wp);
  bool start_next_sweep_round();
  void handle_sweep_leg_failed(const Waypoint & wp);
  void finalize_mission_incomplete();
  void start_next_tsp_leg();
  void advance_after_dwell(bool skipped = false);
  void publish_amcl_prior();
  bool relocalize_ready(const Pose3 & pose, const rclcpp::Time & now);
  void begin_tsp_from_pose(const Pose3 & pose);
  void compute_and_start_tsp();
  void apply_tsp_result();

  void nav_goal_response_callback(const GoalHandleNavigate::SharedPtr & goal_handle);
  void nav_result_callback(const GoalHandleNavigate::WrappedResult & result);
  void backup_goal_response_callback(const GoalHandleBackUp::SharedPtr & goal_handle);
  void backup_result_callback(const GoalHandleBackUp::WrappedResult & result);
  void spin_goal_response_callback(const GoalHandleSpin::SharedPtr & goal_handle);
  void spin_result_callback(const GoalHandleSpin::WrappedResult & result);

  PatrolParams params_;
  std::filesystem::path session_dir_;
  std::vector<Waypoint> waypoints_;
  MissionRecorder recorder_;
  StaticMapIndex static_map_;

  PatrolPhase phase_{PatrolPhase::WAIT_NAV2};
  std::unordered_set<int> visited_ids_;
  std::vector<int> tsp_order_;
  std::vector<Waypoint> tsp_targets_;
  int tsp_cursor_{0};
  bool in_sweep_pass_{false};
  int sweep_round_{0};
  int current_goal_idx_{-1};
  std::optional<Pose3> start_pose_;
  std::optional<rclcpp::Time> dwell_start_;
  std::optional<rclcpp::Time> relocalize_start_;
  int nav_retries_{0};
  std::atomic<bool> tsp_computing_{false};
  std::mutex tsp_mutex_;
  std::optional<TspWorkerResult> tsp_result_;
  int backup_attempts_{0};
  std::vector<int> aborted_ids_;
  std::vector<std::pair<double, double>> last_plan_poses_;
  int leg_index_{0};
  int last_waypoint_id_{-1};
  std::optional<InitialPose> prior_pose_;
  bool initialpose_sent_{false};
  std::optional<rclcpp::Time> shift_stable_start_;
  std::optional<rclcpp::Time> nav2_wait_log_time_;
  std::optional<rclcpp::Time> relocalize_log_time_;
  OdomTrajectoryTracker traj_tracker_;
  std::optional<Pose3> last_odom_pose_;
  std::optional<StallAnchor> stall_anchor_;
  bool ignore_next_nav_result_{false};
  bool handling_nav_failure_{false};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<BackUp>::SharedPtr backup_client_;
  rclcpp_action::Client<Spin>::SharedPtr spin_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_local_costmap_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_global_costmap_;

  GoalHandleNavigate::SharedPtr nav_goal_handle_;
  ContinueFn backup_continue_;
  ContinueFn spin_continue_;

  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;
  nav_msgs::msg::OccupancyGrid::SharedPtr local_costmap_;
  nav_msgs::msg::Path driven_path_;
  std::optional<std::pair<double, double>> last_driven_xy_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr driven_pub_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::TimerBase::SharedPtr sample_timer_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::TimerBase::SharedPtr autosave_timer_;
  rclcpp::TimerBase::SharedPtr obstacle_scan_timer_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
};

}  // namespace navigation_pkg
