#include <JAINode.h>
#include <Logger.h>

std::vector<std::string> splitString(std::string str, std::string delimiter) {
  std::vector<std::string> parts;
  size_t pos = 0;
  std::string token;
  while ((pos = str.find(delimiter)) != std::string::npos) {
    token = str.substr(0, pos);
    parts.push_back(token);
    str.erase(0, pos + delimiter.length());
  }
  parts.push_back(str);
  return parts;
}

JAINode::JAINode() : Node("jai_node") {
  createPublishers();
  createServiceClient();
}

JAINode::~JAINode() { closeStream(); }

void JAINode::join_thread() {
  subscription_thread.join();

  subscription_thread.detach();
}

void JAINode::createPublishers() {
  enlistJAIDevice(0, "jai_1600", 2);

  Info << "JAI camera node is created"
       << "\n"
       << "Prepare Callbacks and Streams"
       << "\n";

  initMultispectralCamera(0);
  Info << "Callback and Stream Done."
       << "\n"
       << "Begin Listenning"
       << "\n";
  openStream(0);
  subscription_thread =
      std::thread([this]() { this->cameras.back()->runUntilInterrupted(); });
}

void JAINode::enlistJAIDevice(int device_num, std::string device_name,
                              int channel_count) {
  if (imagePublishers.size() <= device_num) {
    imagePublishers.resize(device_num + 1);
    cameraDeviceParamPublishers.resize(device_num + 1);
    cameraDeviceParamSubscribers.resize(device_num + 1);
  }

  if (imagePublishers[device_num].size() < channel_count) {
    imagePublishers[device_num].resize(channel_count);
    cameraDeviceParamPublishers[device_num].resize(channel_count);
    cameraDeviceParamSubscribers[device_num].resize(channel_count);
  }

  for (int i = 0; i < channel_count; i++) {
    imagePublishers[device_num][i] =
        this->create_publisher<sensor_msgs::msg::Image>(
            device_name + "/channel_" + std::to_string(i), 10);
    cameraDeviceParamPublishers[device_num][i] =
        this->create_publisher<std_msgs::msg::String>(
            device_name + "/channel_" + std::to_string(i) + "/device_param",
            10);
  }
}

std::function<void(PvBuffer*)> JAINode::getEbusRosCallback(
    const int device_num, const int source_num) {
  return [&](PvBuffer* buffer) {
    // todo : ros logger
  };
}

void JAINode::emitRosImageMsg(int device_num, int source_num,
                              PvBuffer* buffer) {
  if (!buffer) return;
  auto imageRosMsg = sensor_msgs::msg::Image();
  imageRosMsg.header.stamp = rclcpp::Time(buffer->GetTimestamp());
  imageRosMsg.header.frame_id = "jai_camera";
  imageRosMsg.height = buffer->GetImage()->GetHeight();
  imageRosMsg.width = buffer->GetImage()->GetWidth();
  imageRosMsg.encoding =
      source_num == 1 ? "mono8"
                      : "bayer_rggb8";  // todo : buffer pixel type 을 변환
  if (source_num == 0) {
    imageRosMsg.step = buffer->GetImage()->GetBitsPerPixel() / 8 *
                       buffer->GetImage()->GetWidth();
  } else if (source_num == 1) {
    imageRosMsg.step = buffer->GetImage()->GetBitsPerPixel() / 8 *
                       buffer->GetImage()->GetWidth();
  }
  imageRosMsg.data =
      std::vector<uint8_t>(buffer->GetDataPointer(),
                           buffer->GetDataPointer() + buffer->GetPayloadSize());
  imagePublishers[device_num][source_num]->publish(imageRosMsg);
}

void JAINode::initMultispectralCamera(int camera_num) {
  cameras.push_back(new MultiSpectralCamera());

  Info << "Prepare Callback for Stream 0";
  cameras.back()->addStreamCallback(0, [this, camera_num](PvBuffer* buffer) {
    emitRosImageMsg(camera_num, 0, buffer);
  });
  Info << "Prepare Callback for Stream 1";
  cameras.back()->addStreamCallback(1, [this, camera_num](PvBuffer* buffer) {
    emitRosImageMsg(camera_num, 1, buffer);
  });
  Info << "Prepare Callback for Stream Done";
}

void JAINode::openStream(int camera_num) {
  if (camera_num < 0) {
    for (int i = 0; i < cameras.size(); i++) {
      openStream(i);
    }
    return;
  }
  cameras[camera_num]->openStream();
}

void JAINode::closeStream(int camera_num) {
  if (camera_num < 0) {
    for (int i = 0; i < cameras.size(); i++) {
      closeStream(i);
    }
    return;
  }
  cameras[camera_num]->closeStream();
}

void JAINode::createServiceClient() {
  for (int i = 0; i < cameras.size(); i++) {
    createCameraConfigureService(i, 0);
    createCameraConfigureService(i, 1);
  }
}

void JAINode::createCameraConfigureService(int device_num, int channel_num) {
  std::string service_namespace =
      "jai_1600/channel_" + std::to_string(channel_num);
  this->cameraDeviceParamSubscribers[device_num][channel_num] =
      this->create_subscription<std_msgs::msg::String>(
          service_namespace + "/manual_configure", 10,
          [this, device_num,
           channel_num](const std_msgs::msg::String::SharedPtr msg) {
            this->cameras[device_num]->closeStream();
            std::string command = msg->data;
            std::vector<std::string> commands = splitString(command, ";");
            for (const auto& cmd : commands) {
              std::vector<std::string> parts = splitString(cmd, "=");
              if (parts.size() != 2) {
                continue;
              }

              std::string commandName = parts[0];
              std::string value = parts[1];
              size_t start = value.find('>') + 1;
              size_t end = value.size();
              std::string extractedValue = value.substr(start, end - start);
              std::string extractedType = value.substr(1, start - 2);
              this->cameras[device_num]->configureDevice(
                  channel_num, commandName, extractedType, extractedValue);
            }
            this->cameras[device_num]->openStream();
            this->emitRosDeviceParamMsg(device_num, channel_num);
          });
}

void JAINode::emitRosDeviceParamMsg(int device_num, int source_num) {
  std_msgs::msg::String msg;
  char buffer[500];

  sprintf(buffer, "ExposureTime=%f;Gain=%f",
          cameras[device_num]->getExposure(source_num),
          cameras[device_num]->getGain(source_num));
  msg.data = buffer;
  cameraDeviceParamPublishers[device_num][source_num]->publish(msg);
}