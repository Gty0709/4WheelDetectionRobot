#include "perception_pkg/cmd_vel_guard_node.hpp"

namespace perception_pkg
{

CmdVelGuard::CmdVelGuard(const rclcpp::NodeOptions & options)
: Node("cmd_vel_guard", options)
{
  auto cmd_qos = rclcpp::QoS(10)
    .reliability(rclcpp::ReliabilityPolicy::Reliable)
    .durability(rclcpp::DurabilityPolicy::Volatile);

  out_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", cmd_qos);
  teleop_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_teleop", cmd_qos,
    std::bind(&CmdVelGuard::on_teleop, this, std::placeholders::_1));
  nav_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel_nav", cmd_qos,
    std::bind(&CmdVelGuard::on_nav, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / RATE_HZ),
    std::bind(&CmdVelGuard::tick, this));

  // Publish zero burst on startup
  auto zero = geometry_msgs::msg::Twist();
  for (int i = 0; i < 40; ++i) {
    out_pub_->publish(zero);
  }

  RCLCPP_INFO(this->get_logger(),
    "Forwarding /cmd_vel_nav (Nav2) and /cmd_vel_teleop -> /cmd_vel; teleop wins");
}

void CmdVelGuard::on_teleop(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  teleop_msg_ = *msg;
  have_teleop_ = true;
  last_teleop_ns_ = this->get_clock()->now().nanoseconds();
  out_pub_->publish(*msg);
}

void CmdVelGuard::on_nav(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  nav_msg_ = *msg;
  have_nav_ = true;
  last_nav_ns_ = this->get_clock()->now().nanoseconds();

  if (nav_log_left_ > 0 && (std::abs(msg->linear.x) > 1e-4 || std::abs(msg->angular.z) > 1e-4)) {
    RCLCPP_INFO(this->get_logger(),
      "Nav2 cmd_vel_nav: vx=%.3f wz=%.3f", msg->linear.x, msg->angular.z);
    --nav_log_left_;
  }

  // If teleop is not active, forward nav
  auto now_ns = this->get_clock()->now().nanoseconds();
  double teleop_age = (now_ns - last_teleop_ns_) * 1e-9;
  if (!have_teleop_ || teleop_age > TELEOP_STALE_SEC) {
    out_pub_->publish(*msg);
  }
}

void CmdVelGuard::tick()
{
  auto now_ns = this->get_clock()->now().nanoseconds();

  // Check if teleop is still active
  if (have_teleop_) {
    double age = (now_ns - last_teleop_ns_) * 1e-9;
    if (age <= TELEOP_STALE_SEC) {
      out_pub_->publish(teleop_msg_);
      return;
    }
  }

  // Fall back to nav
  if (have_nav_) {
    double age = (now_ns - last_nav_ns_) * 1e-9;
    if (age <= NAV_STALE_SEC) {
      out_pub_->publish(nav_msg_);
      return;
    }
  }

  // Neither active -- publish zero
  out_pub_->publish(geometry_msgs::msg::Twist());
}

}  // namespace perception_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<perception_pkg::CmdVelGuard>();
  rclcpp::spin(node);
  // Publish zero burst on shutdown
  try {
    auto zero = geometry_msgs::msg::Twist();
    for (int i = 0; i < 10; ++i) {
      node->out_pub_->publish(zero);
    }
  } catch (...) {}
  node.reset();
  rclcpp::shutdown();
  return 0;
}
