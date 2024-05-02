#include <JAINode.h>

#include <rclcpp/rclcpp.hpp>

int main() {
  rclcpp::spin(std::make_shared<JAINode>());
  rclcpp::shutdown();
  return 0;
}