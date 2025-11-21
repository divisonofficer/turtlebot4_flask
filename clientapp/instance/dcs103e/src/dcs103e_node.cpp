#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "dcs103e_controller.hpp"

// Custom service messages (these would normally be generated from .srv files)
namespace dcs103e_interfaces {
namespace srv {

struct ControlChannel {
  struct Request {
    int32_t channel;
    bool enable;
    double current;
  };

  struct Response {
    bool success;
    std::string message;
  };
};

struct SetChannelMode {
  struct Request {
    int32_t channel;
    std::string mode;
    double current;
    double pulse_width;
    double pulse_delay;
  };

  struct Response {
    bool success;
    std::string message;
  };
};

}  // namespace srv
}  // namespace dcs103e_interfaces

class DCS103eNode : public rclcpp::Node {
 public:
  DCS103eNode() : Node("dcs103e_controller") {
    // Declare parameters
    this->declare_parameter<std::string>("ip_address", "");
    this->declare_parameter<bool>("auto_connect", false);

    // Initialize controller
    controller_ = std::make_unique<dcs103e_controller::DCS103eController>();

    // Create services
    control_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/control_channel",
        std::bind(&DCS103eNode::controlChannelCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Create service for connecting to device
    connect_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/connect", std::bind(&DCS103eNode::connectCallback, this,
                               std::placeholders::_1, std::placeholders::_2));

    disconnect_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/disconnect",
        std::bind(&DCS103eNode::disconnectCallback, this, std::placeholders::_1,
                  std::placeholders::_2));

    // Create individual channel services
    for (int i = 0; i < 3; ++i) {
      std::string service_name = "~/channel_" + std::to_string(i) + "/enable";
      channel_enable_services_[i] =
          this->create_service<std_srvs::srv::Trigger>(
              service_name,
              [this, i](
                  const std::shared_ptr<std_srvs::srv::Trigger::Request>
                      request,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                this->channelEnableCallback(i, request, response);
              });

      service_name = "~/channel_" + std::to_string(i) + "/disable";
      channel_disable_services_[i] =
          this->create_service<std_srvs::srv::Trigger>(
              service_name,
              [this, i](
                  const std::shared_ptr<std_srvs::srv::Trigger::Request>
                      request,
                  std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                this->channelDisableCallback(i, request, response);
              });
    }

    // Create status publisher
    status_publisher_ =
        this->create_publisher<std_msgs::msg::String>("~/status", 10);

    // Create timer for status publishing
    status_timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&DCS103eNode::publishStatus, this));

    // Auto-connect if IP address is specified
    std::string ip_address = this->get_parameter("ip_address").as_string();
    bool auto_connect = this->get_parameter("auto_connect").as_bool();

    // Connect automatically if IP address is provided
    if (!ip_address.empty()) {
      RCLCPP_INFO(this->get_logger(),
                  "Attempting to connect to DCS103e at %s...",
                  ip_address.c_str());
      if (controller_->connect(ip_address)) {
        RCLCPP_INFO(this->get_logger(),
                    "✓ Successfully connected to DCS103e at %s",
                    ip_address.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "✗ Failed to connect to DCS103e at %s",
                    ip_address.c_str());
        RCLCPP_INFO(
            this->get_logger(),
            "You can try to connect manually using the connect service");
      }
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "No IP address specified. Use 'ip_address' parameter or call "
                  "connect service manually");
    }

    RCLCPP_INFO(this->get_logger(), "DCS103e controller node started");
  }

  ~DCS103eNode() {
    if (controller_) {
      controller_->disconnect();
    }
  }

 private:
  std::unique_ptr<dcs103e_controller::DCS103eController> controller_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr control_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disconnect_service_;
  std::array<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr, 3>
      channel_enable_services_;
  std::array<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr, 3>
      channel_disable_services_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;

  // Timers
  rclcpp::TimerBase::SharedPtr status_timer_;

  void controlChannelCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /* request */,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    // This is a simple trigger service - in real implementation, you'd want
    // custom services
    response->success = controller_->isConnected();
    response->message = controller_->isConnected() ? "DCS103e is connected"
                                                   : "DCS103e is not connected";
  }

  void connectCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /* request */,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    std::string ip_address = this->get_parameter("ip_address").as_string();

    if (ip_address.empty()) {
      response->success = false;
      response->message =
          "No IP address specified. Set 'ip_address' parameter.";
      return;
    }

    bool success = controller_->connect(ip_address);
    response->success = success;
    response->message =
        success ? "Successfully connected to DCS103e at " + ip_address
                : "Failed to connect to DCS103e at " + ip_address;

    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  void disconnectCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /* request */,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    controller_->disconnect();
    response->success = true;
    response->message = "Disconnected from DCS103e";
    RCLCPP_INFO(this->get_logger(), "Disconnected from DCS103e");
  }

  void channelEnableCallback(
      int channel,
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /* request */,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (!controller_->isConnected()) {
      response->success = false;
      response->message = "DCS103e is not connected";
      return;
    }

    // Use default current of 50% of max continuous current
    double max_current = controller_->getMaxContinuousCurrent(channel);
    double current = max_current * 0.5;

    bool success = controller_->enableChannel(channel, current);
    response->success = success;
    response->message =
        success ? "Channel " + std::to_string(channel) +
                      " enabled with current " + std::to_string(current) + "A"
                : "Failed to enable channel " + std::to_string(channel);

    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  void channelDisableCallback(
      int channel,
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /* request */,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    if (!controller_->isConnected()) {
      response->success = false;
      response->message = "DCS103e is not connected";
      return;
    }

    bool success = controller_->disableChannel(channel);
    response->success = success;
    response->message =
        success ? "Channel " + std::to_string(channel) + " disabled"
                : "Failed to disable channel " + std::to_string(channel);

    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  void publishStatus() {
    if (!controller_) return;

    auto message = std_msgs::msg::String();

    if (controller_->isConnected()) {
      std::string status = "Connected - Channels: ";
      size_t channel_count = controller_->getChannelCount();

      for (size_t i = 0; i < channel_count; ++i) {
        if (i > 0) status += ", ";
        status += std::to_string(i) + ":";

        if (controller_->isChannelEnabled(i)) {
          double current = controller_->getChannelCurrent(i);
          status += "ON(" + std::to_string(current) + "A)";
        } else {
          status += "OFF";
        }
      }

      message.data = status;
    } else {
      message.data = "Disconnected";
    }

    status_publisher_->publish(message);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DCS103eNode>();

  RCLCPP_INFO(node->get_logger(), "Starting DCS103e controller node...");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
