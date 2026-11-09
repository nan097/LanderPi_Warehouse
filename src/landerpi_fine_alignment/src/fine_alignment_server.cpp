#include <rclcpp/rclcpp.hpp>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fine_alignment_server");
  RCLCPP_WARN(node->get_logger(), "NOT_IMPLEMENTED: hardware/backend integration pending; no motion interfaces enabled");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
