#include <JAIHDRNode.h>
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

  try {
    // 전역 객체를 사용하여 설정 로드
    config->load_from_json("appconfig.json");

    // 설정 값 출력
    std::cout << "Device Left Name: " << config->DEVICE_LEFT_NAME << std::endl;
    std::cout << "Frame Rate: " << config->FRAME_RATE << " fps" << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }

  signal(SIGINT, signalHandler);
  signal(SIGKILL, signalHandler);

  rclcpp::init(argc, argv);
  if (config->NODE_MODE == 0) {
    auto node = std::make_shared<JAINode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    node->join_thread();
  }
  if (config->NODE_MODE == 1) {
    auto node = std::make_shared<JAIHDRNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
  }

  return 0;
}
