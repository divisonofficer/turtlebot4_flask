#include <JAINode.h>
#include <PvSampleUtils.h>
#include <signal.h>

#include <cstdio>
#include <rclcpp/rclcpp.hpp>

PV_INIT_SIGNAL_HANDLER();

void signalHandler(int signum) {
  // SIGBUS 시그널이 발생하면 종료
  if (signum == SIGINT) {
    // 프로그램 종료
    exit(signum);
  }
  if (signum == SIGKILL) {
    exit(signum);
  }
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

  signal(SIGINT, signalHandler);
  signal(SIGKILL, signalHandler);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<JAINode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  node->join_thread();
  return 0;
}
