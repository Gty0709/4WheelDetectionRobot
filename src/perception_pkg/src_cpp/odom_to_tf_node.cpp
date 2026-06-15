#include "perception_pkg/odom_to_tf_node.hpp"

namespace perception_pkg
{

OdomToTF::OdomToTF(const rclcpp::NodeOptions & options)
: Node("odom_to_tf", options)
{
  br_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  auto qos = rclcpp::QoS(10)
    .reliability(rclcpp::ReliabilityPolicy::BestEffort)
    .durability(rclcpp::DurabilityPolicy::Volatile);

  sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", qos, std::bind(&OdomToTF::on_odom, this, std::placeholders::_1));
}

void OdomToTF::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = msg->header.stamp;
  t.header.frame_id = msg->header.frame_id;        // "odom"
  t.child_frame_id = msg->child_frame_id;           // "base_footprint"
  t.transform.translation.x = msg->pose.pose.position.x;
  t.transform.translation.y = msg->pose.pose.position.y;
  t.transform.translation.z = msg->pose.pose.position.z;
  t.transform.rotation = msg->pose.pose.orientation;
  br_->sendTransform(t);
}

}  // namespace perception_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<perception_pkg::OdomToTF>();
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
