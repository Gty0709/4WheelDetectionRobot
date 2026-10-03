#include "perception_pkg/auto_explore_node.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace perception_pkg
{

AutoExploreNode::AutoExploreNode()
: Node("auto_explorer"),
  linear_vel_(0.2),
  turn_vel_(0.5),
  safe_dist_(0.5),
  side_dist_(0.3),
  state_(ExploreState::FORWARD),
  turn_dir_(1.0),
  turn_count_(0),
  turn_target_(0),
  rng_(std::random_device{}())
{
  pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,
    std::bind(&AutoExploreNode::scan_cb, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(0.1),
    std::bind(&AutoExploreNode::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "AutoExplorer started, press Ctrl+C to stop");
}

double AutoExploreNode::scan_data_range(
  const sensor_msgs::msg::LaserScan & scan,
  double angle_min_deg, double angle_max_deg)
{
  int n = static_cast<int>(scan.ranges.size());
  double total_deg = (scan.angle_max - scan.angle_min) * 180.0 / M_PI;

  int idx_min = static_cast<int>(
    (angle_min_deg - scan.angle_min * 180.0 / M_PI) / total_deg * n);
  int idx_max = static_cast<int>(
    (angle_max_deg - scan.angle_min * 180.0 / M_PI) / total_deg * n);

  idx_min = std::max(0, std::min(idx_min, n - 1));
  idx_max = std::max(0, std::min(idx_max, n - 1));

  if (idx_min > idx_max) {
    std::swap(idx_min, idx_max);
  }

  std::vector<double> values;
  for (int i = idx_min; i <= idx_max; ++i) {
    double r = scan.ranges[i];
    if (r > scan.range_min && r < scan.range_max) {
      values.push_back(r);
    }
  }

  if (values.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  return *std::min_element(values.begin(), values.end());
}

void AutoExploreNode::scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  scan_data_ = msg;
}

void AutoExploreNode::control_loop()
{
  if (!scan_data_) {
    return;
  }

  const auto & scan = *scan_data_;

  double front   = scan_data_range(scan, -30,  30);
  double front_l = scan_data_range(scan,  30,  60);
  double front_r = scan_data_range(scan, -60, -30);

  geometry_msgs::msg::Twist cmd;

  if (state_ == ExploreState::FORWARD) {
    if (front < safe_dist_) {
      state_ = ExploreState::TURN;
      if (front_l >= front_r) {
        turn_dir_ = 1.0;
      } else {
        turn_dir_ = -1.0;
      }
      std::uniform_real_distribution<double> angle_dist(M_PI / 3.0, M_PI * 5.0 / 6.0);
      double angle = angle_dist(rng_);
      turn_target_ = static_cast<int>(angle / turn_vel_ / 0.1);
      turn_count_ = 0;
    } else {
      cmd.linear.x = linear_vel_;
      if (front_l < side_dist_) {
        cmd.angular.z = -0.3;
      } else if (front_r < side_dist_) {
        cmd.angular.z = 0.3;
      } else {
        std::uniform_real_distribution<double> prob(0.0, 1.0);
        if (prob(rng_) < 0.05) {
          std::uniform_real_distribution<double> angular_dist(-0.2, 0.2);
          cmd.angular.z = angular_dist(rng_);
        }
      }
    }
  }

  if (state_ == ExploreState::TURN) {
    cmd.angular.z = turn_dir_ * turn_vel_;
    turn_count_++;
    if (turn_count_ >= turn_target_) {
      state_ = ExploreState::FORWARD;
    }
  }

  pub_->publish(cmd);
}

void AutoExploreNode::stop()
{
  pub_->publish(geometry_msgs::msg::Twist());
}

}  // namespace perception_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<perception_pkg::AutoExploreNode>();
  try {
    rclcpp::spin(node);
  } catch (...) {
    node->stop();
    RCLCPP_INFO(node->get_logger(), "Stopped");
  }
  node->stop();
  rclcpp::shutdown();
  return 0;
}
