#include <rclcpp/rclcpp.hpp>
#include <landerpi_task_interfaces/msg/robot_status.hpp>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("robot_monitor");
  auto pub = node->create_publisher<landerpi_task_interfaces::msg::RobotStatus>("/robot/status", 10);
  auto timer = node->create_wall_timer(std::chrono::milliseconds(200), [node, pub]() {
    landerpi_task_interfaces::msg::RobotStatus status;
    status.header.stamp = node->now();
    status.robot_ready = false;
    status.active_error_code = 504;
    status.active_error_message = "NOT_IMPLEMENTED: live readiness checks pending";
    pub->publish(status);
  });
  rclcpp::spin(node); rclcpp::shutdown(); return 0;
}
