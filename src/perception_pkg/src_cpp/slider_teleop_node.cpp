#include "perception_pkg/slider_teleop_node.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <functional>
#include <iostream>
#include <string>

#include <ncurses.h>
#include <rclcpp/qos.hpp>

namespace perception_pkg
{

SliderTeleopNode::SliderTeleopNode(const rclcpp::NodeOptions & options)
: Node("slider_teleop", options)
{
  // Parameter
  this->declare_parameter("save_map_on_exit", true);
  save_on_exit_ = this->get_parameter("save_map_on_exit").as_bool();

  // QoS: RELIABLE + VOLATILE (matches Python)
  auto cmd_vel_qos = rclcpp::QoS(10)
    .reliable()
    .durability_volatile();

  // Publishers
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel_teleop", cmd_vel_qos);
  wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/teleop/wheel_speeds", rclcpp::QoS(10));

  // Service client
  save_client_ = this->create_client<std_srvs::srv::Trigger>(
    "/map_snapshot_saver/save_map");

  // Timer at 20 Hz
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / PUBLISH_HZ),
    std::bind(&SliderTeleopNode::timer_callback, this));

  // Send stop burst on startup
  publish_stop_burst();
}

SliderTeleopNode::~SliderTeleopNode()
{
  stop_publishing();
  publish_stop();
}

SliderTeleopNode::WheelSpeeds SliderTeleopNode::kinematics(double v, double omega)
{
  double v_left  = v - omega * WHEEL_SEPARATION / 2.0;
  double v_right = v + omega * WHEEL_SEPARATION / 2.0;
  double w_left  = v_left  / WHEEL_RADIUS;
  double w_right = v_right / WHEEL_RADIUS;
  return {v_left, v_right, w_left, w_right};
}

void SliderTeleopNode::set_vel(double linear, double angular)
{
  // Dead zone: suppress small omega when driving straight
  if (std::abs(linear) >= 0.05 && std::abs(angular) < 0.08) {
    angular = 0.0;
  }
  // Dead zone: very small omega -> 0
  if (std::abs(angular) < 0.02) {
    angular = 0.0;
  }
  // Dead zone: very small linear -> 0
  if (std::abs(linear) < 0.01) {
    linear = 0.0;
  }
  std::lock_guard<std::mutex> lk(lock_);
  linear_  = linear;
  angular_ = angular;
}

void SliderTeleopNode::publish_stop()
{
  geometry_msgs::msg::Twist stop;
  cmd_pub_->publish(stop);
}

void SliderTeleopNode::publish_stop_burst(int count)
{
  geometry_msgs::msg::Twist stop;
  for (int i = 0; i < count; ++i) {
    cmd_pub_->publish(stop);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

void SliderTeleopNode::stop_publishing()
{
  stopped_ = true;
  if (timer_) {
    timer_->cancel();
    timer_ = nullptr;
  }
}

void SliderTeleopNode::request_save_map()
{
  if (!save_on_exit_ || !rclcpp::ok()) {
    return;
  }
  if (!save_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_WARN(this->get_logger(), "save_map service unavailable (start SLAM first)");
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = save_client_->async_send_request(request);
  // Spin until future complete (up to 10 seconds)
  auto end_time = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < end_time) {
    auto status = future.wait_for(std::chrono::milliseconds(100));
    if (status == std::future_status::ready) {
      break;
    }
    // Process callbacks so the service response can arrive
    if (rclcpp::ok()) {
      rclcpp::spin_some(this->shared_from_this());
    }
  }
  if (future.valid()) {
    auto result = future.get();
    if (result->success) {
      RCLCPP_INFO(this->get_logger(), "Map saved: %s", result->message.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(), "Map save failed");
    }
  }
}

void SliderTeleopNode::timer_callback()
{
  if (stopped_) {
    return;
  }
  double v, w;
  {
    std::lock_guard<std::mutex> lk(lock_);
    v = linear_;
    w = angular_;
  }

  // Publish Twist
  geometry_msgs::msg::Twist twist;
  twist.linear.x  = v;
  twist.angular.z = w;
  cmd_pub_->publish(twist);

  // Publish wheel speeds
  auto ws = kinematics(v, w);
  std_msgs::msg::Float64MultiArray arr;
  arr.data = {ws.v_left, ws.v_right, ws.w_left, ws.w_right};
  wheel_pub_->publish(arr);
}

// ---------------------------------------------------------------------------
// Terminal-based keyboard input with ncurses
// ---------------------------------------------------------------------------

static constexpr double LIN_STEP = 0.1;   // m/s per key press
static constexpr double ANG_STEP = 0.2;   // rad/s per key press

/// Run the ncurses input loop. Returns when user presses 'q'.
static void ncurses_input_loop(SliderTeleopNode & node)
{
  double linear  = 0.0;
  double angular = 0.0;

  initscr();
  cbreak();
  noecho();
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  curs_set(0);

  // Print initial help
  mvprintw(0, 0, "=== MickRobot Terminal Teleop ===");
  mvprintw(1, 0, "w/Up: forward  s/Down: backward  a/Left: left  d/Right: right");
  mvprintw(2, 0, "space: stop    q: quit (save map & exit)");
  mvprintw(3, 0, "----------------------------------------------------------");

  bool running = true;
  while (running && rclcpp::ok()) {
    int ch = getch();
    if (ch != ERR) {
      switch (ch) {
        // Linear increment
        case 'w':
        case KEY_UP:
          linear += LIN_STEP;
          break;
        case 's':
        case KEY_DOWN:
          linear -= LIN_STEP;
          break;
        // Angular increment
        case 'a':
        case KEY_LEFT:
          angular += ANG_STEP;
          break;
        case 'd':
        case KEY_RIGHT:
          angular -= ANG_STEP;
          break;
        // Stop
        case ' ':
          linear  = 0.0;
          angular = 0.0;
          break;
        // Quit
        case 'q':
          linear  = 0.0;
          angular = 0.0;
          running = false;
          break;
        default:
          break;
      }

      // Clamp
      if (linear > SliderTeleopNode::V_MAX)  linear = SliderTeleopNode::V_MAX;
      if (linear < -SliderTeleopNode::V_MAX) linear = -SliderTeleopNode::V_MAX;
      if (angular > SliderTeleopNode::W_MAX)  angular = SliderTeleopNode::W_MAX;
      if (angular < -SliderTeleopNode::W_MAX) angular = -SliderTeleopNode::W_MAX;

      node.set_vel(linear, angular);

      // Re-read the actual values after dead-zone filtering
      {
        // We read back from node state (linear/angular in the node reflect
        // the filtered values). For display we use the filtered output.
        // Since set_vel already applied dead zones, compute wheel speeds
        // from the filtered values.
        double fv = linear, fw = angular;
        // Apply same dead-zone logic for display
        if (std::abs(fv) >= 0.05 && std::abs(fw) < 0.08) fw = 0.0;
        if (std::abs(fw) < 0.02) fw = 0.0;
        if (std::abs(fv) < 0.01) fv = 0.0;

        auto ws = SliderTeleopNode::kinematics(fv, fw);
        mvprintw(4, 0, "v = %+.3f m/s    w = %+.3f rad/s              ", fv, fw);
        mvprintw(5, 0, "v_L=%+.3f  v_R=%+.3f m/s                      ", ws.v_left, ws.v_right);
        mvprintw(6, 0, "w_L=%+.2f   w_R=%+.2f  rad/s                   ", ws.w_left, ws.w_right);
        refresh();
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
  }

  endwin();
}

}  // namespace perception_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<perception_pkg::SliderTeleopNode>();

  // Spin ROS in a separate thread
  std::thread spin_thread([&node]() {
    rclcpp::spin(node);
  });

  // Run ncurses input loop (blocks until 'q')
  perception_pkg::ncurses_input_loop(*node);

  // Save map on exit
  node->request_save_map();

  // Shutdown
  node->stop_publishing();
  node->publish_stop();
  rclcpp::shutdown();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }

  return 0;
}
