#include "navigation_pkg/send_goal_node.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <future>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

namespace navigation_pkg
{

namespace
{

const char * result_code_text(rclcpp_action::ResultCode code)
{
  using rclcpp_action::ResultCode;
  switch (code) {
    case ResultCode::UNKNOWN: return "UNKNOWN";
    case ResultCode::SUCCEEDED: return "SUCCEEDED";
    case ResultCode::CANCELED: return "CANCELED";
    case ResultCode::ABORTED: return "ABORTED";
    default: return "UNKNOWN";
  }
}

std::tuple<double, double, double, double> yaw_to_quaternion(double yaw)
{
  const double half_yaw = yaw * 0.5;
  return {0.0, 0.0, std::sin(half_yaw), std::cos(half_yaw)};
}

struct CliArgs
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  std::string frame{"map"};
  double server_timeout{30.0};
  std::optional<double> result_timeout;
  bool degrees{false};
};

bool parse_double(const std::string & text, double & out)
{
  try {
    out = std::stod(text);
    return true;
  } catch (...) {
    return false;
  }
}

}  // namespace

std::optional<CliArgs> parse_send_goal_args(int argc, char ** argv)
{
  if (argc < 3) {
    std::cerr << "usage: send_goal_cpp x y [yaw] [--degrees] [--frame map] "
              << "[--server-timeout 30] [--result-timeout SEC]\n";
    return std::nullopt;
  }

  CliArgs args;
  if (!parse_double(argv[1], args.x) || !parse_double(argv[2], args.y)) {
    std::cerr << "x and y must be numeric\n";
    return std::nullopt;
  }

  int positional = 3;
  if (argc > 3 && argv[3][0] != '-') {
    if (!parse_double(argv[3], args.yaw)) {
      std::cerr << "yaw must be numeric\n";
      return std::nullopt;
    }
    positional = 4;
  }

  for (int i = positional; i < argc; ++i) {
    const std::string flag = argv[i];
    if (flag == "--degrees") {
      args.degrees = true;
      continue;
    }
    if (flag == "--frame" && i + 1 < argc) {
      args.frame = argv[++i];
      continue;
    }
    if (flag == "--server-timeout" && i + 1 < argc) {
      if (!parse_double(argv[++i], args.server_timeout)) {
        std::cerr << "--server-timeout must be numeric\n";
        return std::nullopt;
      }
      continue;
    }
    if (flag == "--result-timeout" && i + 1 < argc) {
      double timeout = 0.0;
      if (!parse_double(argv[++i], timeout)) {
        std::cerr << "--result-timeout must be numeric\n";
        return std::nullopt;
      }
      args.result_timeout = timeout;
      continue;
    }
    std::cerr << "unknown argument: " << flag << "\n";
    return std::nullopt;
  }

  if (args.degrees) {
    args.yaw = args.yaw * M_PI / 180.0;
  }
  return args;
}

GoalSenderNode::GoalSenderNode()
: Node("navigation_goal_sender"),
  client_(rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose")),
  last_feedback_time_(get_clock()->now())
{
}

int GoalSenderNode::send(
  double x,
  double y,
  double yaw,
  const std::string & frame_id,
  double server_timeout,
  const std::optional<double> & result_timeout)
{
  RCLCPP_INFO(get_logger(), "Waiting for Nav2 navigate_to_pose action server...");
  if (!client_->wait_for_action_server(std::chrono::duration<double>(server_timeout))) {
    RCLCPP_ERROR(get_logger(), "navigate_to_pose action server is not available.");
    return 2;
  }

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = frame_id;
  goal.pose.header.stamp = get_clock()->now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  const auto [qx, qy, qz, qw] = yaw_to_quaternion(yaw);
  goal.pose.pose.orientation.x = qx;
  goal.pose.pose.orientation.y = qy;
  goal.pose.pose.orientation.z = qz;
  goal.pose.pose.orientation.w = qw;

  RCLCPP_INFO(
    get_logger(),
    "Sending goal: frame=%s, x=%.3f, y=%.3f, yaw=%.3f rad",
    frame_id.c_str(), x, y, yaw);

  auto send_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_options.feedback_callback =
    std::bind(
    &GoalSenderNode::feedback_callback, this,
    std::placeholders::_1, std::placeholders::_2);

  auto goal_future = client_->async_send_goal(goal, send_options);
  if (rclcpp::spin_until_future_complete(shared_from_this(), goal_future) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Failed to send goal.");
    return 3;
  }

  const auto goal_handle = goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by Nav2.");
    return 3;
  }

  RCLCPP_INFO(get_logger(), "Goal accepted; navigating...");
  auto result_future = client_->async_get_result(goal_handle);
  const auto started = get_clock()->now();

  while (rclcpp::ok() && result_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
    rclcpp::spin_some(shared_from_this());
    if (result_timeout) {
      const auto elapsed = get_clock()->now() - started;
      if (elapsed.seconds() > *result_timeout) {
        RCLCPP_WARN(get_logger(), "Result timeout reached; canceling goal.");
        auto cancel_future = client_->async_cancel_goal(goal_handle);
        rclcpp::spin_until_future_complete(shared_from_this(), cancel_future);
        return 4;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  const auto wrapped = result_future.get();
  if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(get_logger(), "Navigation succeeded.");
    return 0;
  }

  RCLCPP_ERROR(
    get_logger(),
    "Navigation finished with status=%s",
    result_code_text(wrapped.code));
  return 5;
}

void GoalSenderNode::feedback_callback(
  GoalHandleNavigateToPose::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  const auto now = get_clock()->now();
  if ((now - last_feedback_time_).seconds() < 1.0) {
    return;
  }
  last_feedback_time_ = now;
  const double eta_sec = feedback->estimated_time_remaining.sec +
    feedback->estimated_time_remaining.nanosec * 1e-9;
  RCLCPP_INFO(
    get_logger(),
    "distance_remaining=%.2f m, eta=%.1f s",
    feedback->distance_remaining, eta_sec);
}

}  // namespace navigation_pkg

int main(int argc, char ** argv)
{
  const auto args = navigation_pkg::parse_send_goal_args(argc, argv);
  if (!args) {
    return 1;
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<navigation_pkg::GoalSenderNode>();
  int exit_code = 0;
  try {
    exit_code = node->send(
      args->x, args->y, args->yaw, args->frame,
      args->server_timeout, args->result_timeout);
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(node->get_logger(), "Exception: %s", exc.what());
    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}
