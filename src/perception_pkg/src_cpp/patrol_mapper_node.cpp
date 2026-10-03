#include "perception_pkg/patrol_mapper_node.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

namespace perception_pkg
{

// ── Constants (mirroring Python) ──────────────────────────────────────

static constexpr double WALL_CLEAR = 0.68;
static constexpr double PILLAR_CLEAR = 0.82;

static constexpr double WALL_XE = 5.0 - 0.075;
static constexpr double WALL_XW = -5.0 + 0.075;
static constexpr double WALL_YN = 4.0 - 0.075;
static constexpr double WALL_YS = -4.0 + 0.075;

static constexpr double X_E = WALL_XE - WALL_CLEAR;
static constexpr double X_W = WALL_XW + WALL_CLEAR;
static constexpr double Y_N = WALL_YN - WALL_CLEAR;
static constexpr double Y_S = WALL_YS + WALL_CLEAR;
static constexpr double X_INNER_E = -3.0 + 0.075 + WALL_CLEAR;
static constexpr double X_INNER_W = -3.0 - 0.075 - WALL_CLEAR;
static constexpr double Y_INNER_HALF = 2.075;
static constexpr double Y_S_BRIDGE = -(Y_INNER_HALF + 0.10);
static constexpr double Y_N_BRIDGE = Y_INNER_HALF + 0.10;

// Four pillar centres
struct Obs { double x, y; };
static const Obs OBS_SE{3.0, -2.5};
static const Obs OBS_SW{-3.0, -2.5};
static const Obs OBS_NE{3.0, 2.5};
static const Obs OBS_NW{-3.0, 2.5};

// ── Helper functions ──────────────────────────────────────────────────

static double axis_bearing(double x0, double y0, double x1, double y1)
{
  double dx = x1 - x0, dy = y1 - y0;
  if (std::fabs(dx) >= std::fabs(dy)) {
    return dx >= 0 ? 0.0 : M_PI;
  }
  return dy >= 0 ? M_PI / 2.0 : -M_PI / 2.0;
}

using Pt = std::pair<double, double>;

static std::vector<Pt> expand_manhattan(const std::vector<Pt> & pts)
{
  if (pts.empty()) { return pts; }
  std::vector<Pt> out;
  out.reserve(pts.size() + pts.size() / 2);
  out.push_back(pts[0]);
  for (size_t i = 1; i < pts.size(); ++i) {
    double x = pts[i].first, y = pts[i].second;
    double px = out.back().first, py = out.back().second;
    if (std::fabs(x - px) > 0.05 && std::fabs(y - py) > 0.05) {
      out.emplace_back(x, py);
    }
    if (out.back().first != x || out.back().second != y) {
      out.emplace_back(x, y);
    }
  }
  return out;
}

static double norm_angle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

static double clip_val(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

static Pt nearest_outer_corner(double ob_x, double ob_y)
{
  const Pt corners[] = {
    {WALL_XE, WALL_YS}, {WALL_XE, WALL_YN},
    {WALL_XW, WALL_YS}, {WALL_XW, WALL_YN},
  };
  Pt best = corners[0];
  double best_d = std::numeric_limits<double>::max();
  for (const auto & c : corners) {
    double d = (ob_x - c.first) * (ob_x - c.first) + (ob_y - c.second) * (ob_y - c.second);
    if (d < best_d) { best_d = d; best = c; }
  }
  return best;
}

static Pt patrol_point(double ob_x, double ob_y)
{
  auto [cx, cy] = nearest_outer_corner(ob_x, ob_y);
  double dx = cx - ob_x, dy = cy - ob_y;
  double lane_y, x;
  if (std::fabs(dy) < 1e-9) {
    lane_y = ob_y < 0 ? Y_S : Y_N;
    x = ob_x;
  } else {
    lane_y = ob_y < 0 ? Y_S : Y_N;
    double t = (lane_y - ob_y) / dy;
    x = ob_x + t * dx;
  }
  x = clip_val(x, X_W, X_E);
  constexpr double rad = 0.25;
  if (ob_x > 0) {
    x = std::max(x, ob_x + rad + PILLAR_CLEAR);
  } else {
    x = std::min(x, ob_x - rad - PILLAR_CLEAR);
  }
  return {x, lane_y};
}

static std::vector<Pt> inner_south_zigzag()
{
  return {
    {X_INNER_W, Y_S},
    {X_INNER_W, Y_S_BRIDGE},
    {X_INNER_E, Y_S_BRIDGE},
    {X_INNER_E, Y_S},
  };
}

static std::vector<Pt> inner_north_zigzag()
{
  return {
    {X_INNER_E, Y_N},
    {X_INNER_E, Y_N_BRIDGE},
    {X_INNER_W, Y_N_BRIDGE},
    {X_INNER_W, Y_N},
  };
}

static std::vector<Pt> build_waypoints()
{
  auto p_se = patrol_point(OBS_SE.x, OBS_SE.y);
  auto p_sw = patrol_point(OBS_SW.x, OBS_SW.y);
  auto p_ne = patrol_point(OBS_NE.x, OBS_NE.y);
  auto p_nw = patrol_point(OBS_NW.x, OBS_NW.y);

  std::vector<Pt> pts = {
    {0.0, 0.0},
    {X_E, 0.0},
    {X_E, Y_S},
    p_se,
    p_sw,
  };
  auto sz = inner_south_zigzag();
  pts.insert(pts.end(), sz.begin(), sz.end());
  pts.push_back(p_se);
  pts.emplace_back(X_E, Y_S);
  pts.emplace_back(X_E, Y_N);
  pts.push_back(p_ne);
  auto nz = inner_north_zigzag();
  pts.insert(pts.end(), nz.begin(), nz.end());
  pts.push_back(p_nw);
  pts.emplace_back(X_W, Y_N);
  pts.emplace_back(X_W, Y_S);
  pts.emplace_back(X_W, 0.0);
  pts.emplace_back(0.0, 0.0);

  return expand_manhattan(pts);
}

static const std::vector<Pt> SMALL_HOUSE_WAYPOINTS = build_waypoints();

static double yaw_from_quat(double qw, double qx, double qy, double qz)
{
  return std::atan2(
    2.0 * (qw * qz + qx * qy),
    1.0 - 2.0 * (qy * qy + qz * qz));
}

// ── PatrolMapperNode implementation ──────────────────────────────────

PatrolMapperNode::PatrolMapperNode(const rclcpp::NodeOptions & options)
: Node("patrol_mapper", options),
  phase_(Phase::WAIT_ODOM),
  wp_i_(1),
  segment_yaw_(0.0),
  seg_horizontal_(true),
  seg_from_(0.0, 0.0),
  running_(true)
{
  this->declare_parameter("save_map_on_finish", true);
  this->declare_parameter("linear_speed", 0.11);
  this->declare_parameter("angular_speed", 0.48);
  this->declare_parameter("goal_tolerance", 0.20);
  this->declare_parameter("yaw_kp", 2.4);
  this->declare_parameter("xtrack_kp", 0.9);

  save_on_finish_ = this->get_parameter("save_map_on_finish").as_bool();
  v_max_ = this->get_parameter("linear_speed").as_double();
  w_max_ = this->get_parameter("angular_speed").as_double();
  goal_tol_ = this->get_parameter("goal_tolerance").as_double();
  yaw_kp_ = this->get_parameter("yaw_kp").as_double();
  xtrack_kp_ = this->get_parameter("xtrack_kp").as_double();

  auto qos = rclcpp::QoS(10).reliable();
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_teleop", 10);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", qos,
    std::bind(&PatrolMapperNode::on_odom, this, std::placeholders::_1));
  save_client_ = this->create_client<std_srvs::srv::Trigger>("/map_snapshot_saver/save_map");

  RCLCPP_INFO(
    this->get_logger(),
    "贴墙巡逻 %zu 段，墙距 %.2fm，kp=%.1f",
    SMALL_HOUSE_WAYPOINTS.size() - 1, WALL_CLEAR, yaw_kp_);
  for (size_t i = 0; i < SMALL_HOUSE_WAYPOINTS.size(); ++i) {
    RCLCPP_INFO(
      this->get_logger(),
      "  wp%zu: (%.2f, %.2f)", i,
      SMALL_HOUSE_WAYPOINTS[i].first, SMALL_HOUSE_WAYPOINTS[i].second);
  }

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(0.05),
    std::bind(&PatrolMapperNode::control_tick, this));
}

void PatrolMapperNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & p = msg->pose.pose.position;
  const auto & q = msg->pose.pose.orientation;
  pose_ = {p.x, p.y, yaw_from_quat(q.w, q.x, q.y, q.z)};
  if (phase_ == Phase::WAIT_ODOM) {
    begin_segment();
    phase_ = Phase::DRIVE;
    const auto & [gx, gy] = SMALL_HOUSE_WAYPOINTS[wp_i_];
    RCLCPP_INFO(this->get_logger(), "开始巡逻 → (%.2f, %.2f)", gx, gy);
  }
}

void PatrolMapperNode::safe_publish(double lin, double ang)
{
  if (!rclcpp::ok() || !running_) { return; }
  geometry_msgs::msg::Twist msg;
  msg.linear.x = lin;
  msg.angular.z = ang;
  try {
    cmd_pub_->publish(msg);
  } catch (...) {}
}

void PatrolMapperNode::stop()
{
  safe_publish(0.0, 0.0);
}

std::tuple<double, double> PatrolMapperNode::goal() const
{
  return {SMALL_HOUSE_WAYPOINTS[wp_i_].first, SMALL_HOUSE_WAYPOINTS[wp_i_].second};
}

std::optional<std::tuple<double, double>> PatrolMapperNode::next_goal() const
{
  if (wp_i_ + 1 >= SMALL_HOUSE_WAYPOINTS.size()) { return std::nullopt; }
  const auto & n = SMALL_HOUSE_WAYPOINTS[wp_i_ + 1];
  return std::make_tuple(n.first, n.second);
}

void PatrolMapperNode::begin_segment()
{
  seg_from_ = {SMALL_HOUSE_WAYPOINTS[wp_i_ - 1].first,
               SMALL_HOUSE_WAYPOINTS[wp_i_ - 1].second};
  const auto [fx, fy] = seg_from_;
  const auto [gx, gy] = goal();
  segment_yaw_ = axis_bearing(fx, fy, gx, gy);
  seg_horizontal_ = std::fabs(gx - fx) >= std::fabs(gy - fy);
}

double PatrolMapperNode::along_dist(double x, double y) const
{
  const auto [gx, gy] = goal();
  if (seg_horizontal_) { return std::fabs(gx - x); }
  return std::fabs(gy - y);
}

double PatrolMapperNode::dist_to_goal(double x, double y) const
{
  const auto [gx, gy] = goal();
  return std::hypot(gx - x, gy - y);
}

bool PatrolMapperNode::segment_done(double x, double y) const
{
  const auto [gx, gy] = goal();
  if (seg_horizontal_) { return std::fabs(gx - x) < goal_tol_; }
  return std::fabs(gy - y) < goal_tol_;
}

double PatrolMapperNode::cross_track_err(double x, double y) const
{
  const auto [gx, gy] = goal();
  if (seg_horizontal_) { return y - gy; }
  return x - gx;
}

double PatrolMapperNode::xtrack_w(double xte) const
{
  double sign;
  if (seg_horizontal_) {
    sign = std::fabs(segment_yaw_) < 0.01 ? -1.0 : 1.0;
  } else {
    sign = segment_yaw_ > 0 ? 1.0 : -1.0;
  }
  return sign * xtrack_kp_ * xte;
}

double PatrolMapperNode::steer_bearing(double x, double y) const
{
  auto nxt = next_goal();
  if (!nxt.has_value()) { return segment_yaw_; }
  if (segment_done(x, y) || dist_to_goal(x, y) < CORNER_APPROACH) {
    const auto [gx, gy] = goal();
    const auto [nx, ny] = nxt.value();
    return axis_bearing(gx, gy, nx, ny);
  }
  return segment_yaw_;
}

bool PatrolMapperNode::check_stuck(double x, double y, double now)
{
  if (!stuck_xy_.has_value()) {
    stuck_xy_ = std::make_tuple(x, y);
    stuck_t0_ = now;
    return false;
  }
  const auto [sx, sy] = stuck_xy_.value();
  double moved = std::hypot(x - sx, y - sy);
  if (moved > STUCK_MOVE) {
    stuck_xy_ = std::make_tuple(x, y);
    stuck_t0_ = now;
    return false;
  }
  if (now - stuck_t0_.value() > STUCK_SEC) {
    return true;
  }
  return false;
}

void PatrolMapperNode::reset_stuck()
{
  stuck_t0_.reset();
  stuck_xy_.reset();
}

void PatrolMapperNode::advance_waypoint()
{
  wp_i_++;
  if (wp_i_ >= SMALL_HOUSE_WAYPOINTS.size()) {
    phase_ = Phase::DONE;
    stop();
    RCLCPP_INFO(this->get_logger(), "墙边巡逻完成");
    std::thread(&PatrolMapperNode::request_save, this).detach();
    return;
  }
  begin_segment();
  reset_stuck();
  const auto [gx, gy] = goal();
  RCLCPP_INFO(this->get_logger(), "→ 航点 (%.2f, %.2f)", gx, gy);
}

void PatrolMapperNode::control_tick()
{
  if (!running_ || phase_ == Phase::WAIT_ODOM || phase_ == Phase::DONE) { return; }
  if (!pose_.has_value()) { return; }

  const auto [x, y, yaw] = pose_.value();
  double dist = along_dist(x, y);
  double now = this->now().seconds();

  if (check_stuck(x, y, now)) {
    double d = dist_to_goal(x, y);
    RCLCPP_WARN(
      this->get_logger(),
      "卡住 %.0fs，强制换点 (距目标 %.2fm, pos=(%.2f,%.2f))",
      STUCK_SEC, d, x, y);
    advance_waypoint();
    return;
  }

  if (segment_done(x, y)) {
    advance_waypoint();
    return;
  }

  double target_yaw = steer_bearing(x, y);
  double herr = norm_angle(target_yaw - yaw);
  double xte = cross_track_err(x, y);
  double w_corr = clip_val(yaw_kp_ * herr + xtrack_w(xte), -w_max_, w_max_);

  double v;
  if (std::fabs(herr) > TURN_IN_PLACE) {
    v = std::fabs(herr) > 0.85 ? 0.0 : std::min(0.04, v_max_ * 0.25);
  } else if (std::fabs(herr) > 0.18) {
    v = std::min(0.06, v_max_ * 0.45);
  } else {
    v = std::min(v_max_, std::max(0.06, dist * 0.75));
  }

  safe_publish(v, w_corr);
}

void PatrolMapperNode::request_save()
{
  if (!save_on_finish_ || !rclcpp::ok()) { return; }
  std::this_thread::sleep_for(std::chrono::duration<double>(2.5));
  if (!save_client_->wait_for_service(std::chrono::seconds(20))) {
    RCLCPP_WARN(this->get_logger(), "save_map 服务不可用");
    return;
  }
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = save_client_->async_send_request(req);
  // Spin until future complete (on this separate thread)
  if (rclcpp::spin_until_future_complete(
        this->shared_from_this(), future,
        std::chrono::seconds(30)) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto result = future.get();
    if (result->success) {
      RCLCPP_INFO(this->get_logger(), "巡逻结束，地图已保存");
    }
  }
}

void PatrolMapperNode::shutdown()
{
  running_ = false;
  stop();
}

}  // namespace perception_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<perception_pkg::PatrolMapperNode>();
  try {
    rclcpp::spin(node);
  } catch (...) {}
  node->shutdown();
  rclcpp::shutdown();
  return 0;
}
