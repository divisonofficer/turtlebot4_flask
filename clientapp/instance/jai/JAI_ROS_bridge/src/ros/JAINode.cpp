#include <JAINode.h>
#include <Logger.h>

JAINode::JAINode() : Node("jai_node") { createPublishers(); }

JAINode::~JAINode() {
  closeStream();
  subscription_thread.join();
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
  }

  if (imagePublishers[device_num].size() <= channel_count) {
    imagePublishers[device_num].resize(channel_count + 1);
  }

  for (int i = 0; i < channel_count; i++) {
    imagePublishers[device_num][i] =
        this->create_publisher<sensor_msgs::msg::Image>(
            device_name + "/channel_" + std::to_string(i), 10);
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
      source_num == 1 ? "mono8" : "bgr8";  // todo : buffer pixel type 을 변환
  imageRosMsg.step = buffer->GetImage()->GetBitsPerPixel() / 8 *
                     buffer->GetImage()->GetWidth();
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
