#include "perception_pkg/save_map_trigger_node.hpp"

namespace perception_pkg
{

SaveMapTrigger::SaveMapTrigger(const rclcpp::NodeOptions & options)
: Node("save_map_trigger", options)
{
  client_ = this->create_client<std_srvs::srv::Trigger>("/map_snapshot_saver/save_map");
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
    std::bind(&SaveMapTrigger::tick, this));
}

void SaveMapTrigger::tick()
{
  if (done_) return;
  if (!client_->wait_for_service(std::chrono::seconds(0))) {
    RCLCPP_INFO(this->get_logger(), "Waiting for map_snapshot_saver service...");
    return;
  }
  done_ = true;
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client_->async_send_request(request,
    [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      try {
        auto res = future.get();
        if (res->success) {
          RCLCPP_INFO(this->get_logger(), "Map saved: %s", res->message.c_str());
        } else {
          RCLCPP_WARN(this->get_logger(), "Map save failed: %s", res->message.c_str());
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "save_map call failed: %s", e.what());
      }
      rclcpp::shutdown();
    });
}

}  // namespace perception_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<perception_pkg::SaveMapTrigger>();
  try {
    rclcpp::spin(node);
  } catch (...) {}
  node.reset();
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
