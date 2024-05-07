#include <JAINode.h>
#include <PvSampleUtils.h>

#include <cstdio>
#include <rclcpp/rclcpp.hpp>

PV_INIT_SIGNAL_HANDLER();

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<JAINode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  node->join_thread();
  return 0;
}
