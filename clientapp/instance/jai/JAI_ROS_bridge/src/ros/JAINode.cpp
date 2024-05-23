#include <JAINode.h>
#include <Logger.h>
#include <cv_bridge/cv_bridge.h>
#include <jai.pb.h>

#include <opencv4/opencv2/opencv.hpp>

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
  initDevices();
  createPublishers();
  createServiceClient();
}

JAINode::~JAINode() { closeStream(); }
void JAINode::initDevices() {
  initMultispectralCamera(0, DEVICE_LEFT_NAME, DEVICE_LEFT_ADDRESS);
  initMultispectralCamera(1, DEVICE_RIGHT_NAME, DEVICE_RIGHT_ADDRESS);
}
void JAINode::createPublishers() {
  logPublisher =
      this->create_publisher<std_msgs::msg::String>("jai_1600/log", 10);

  enlistJAIDevice(0, cameras[0]->deviceName, 2);
  enlistJAIDevice(1, cameras[1]->deviceName, 2);

  openStream(-1);

  this->cameras[0]->initCameraTimestamp();
  this->timestamp_begin_ros = this->systemTimeNano();

  subscription_thread.push_back(
      std::thread([this]() { this->cameras[0]->runUntilInterrupted(); }));
  subscription_thread.push_back(
      std::thread([this]() { this->cameras.back()->runUntilInterrupted(); }));
}

void JAINode::enlistJAIDevice(int device_num, std::string device_name,
                              int channel_count) {
  // Adjust size of Device Vectors
  if (imagePublishers.size() <= device_num) {
    imagePublishers.resize(device_num + 1);
    cameraDeviceParamPublishers.resize(device_num + 1);
    cameraDeviceParamSubscribers.resize(device_num + 1);
    cameraStreamTriggerSubscribers.resize(device_num + 1);
  }
  // Adjust size of Stream Channel Vectors
  if (imagePublishers[device_num].size() < channel_count) {
    imagePublishers[device_num].resize(channel_count);
    cameraDeviceParamPublishers[device_num].resize(channel_count);
    cameraDeviceParamSubscribers[device_num].resize(channel_count);
  }
  // Camera Stream On/Off Trigger Subscription
  cameraStreamTriggerSubscribers[device_num] =
      this->create_subscription<std_msgs::msg::Bool>(
          device_name + "/stream_trigger", 10,
          [this, device_num](std_msgs::msg::Bool trigger) {
            if (trigger.data) {
              this->cameras[device_num]->openStream();
            } else {
              this->cameras[device_num]->closeStream();
            }
          });
  // Per Stream Publishers
  for (int i = 0; i < channel_count; i++) {
    imagePublishers[device_num][i] =
        this->create_publisher<sensor_msgs::msg::CompressedImage>(
            device_name + "/channel_" + std::to_string(i), 3);
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

bool JAINode::validateBufferTimestamp(int device_num, int source_num,
                                      int64_t buffer_time,
                                      int64_t current_time) {
  if (buffer_time > current_time) return false;
  if (current_time - buffer_time > 4000000000) {
    ErrorLog << "Device " << device_num << " Channel " << source_num << " "
             << (current_time - buffer_time) / 1000000 % 100000 << "ms behinds";
    return false;
  }

  std_msgs::msg::String msg;
  msg.data = "Device " + std::to_string(device_num) + "Channel " +
             std::to_string(source_num) + " " +
             std::to_string((current_time - buffer_time) / 1000000 % 100000) +
             "ms behinds";

  logPublisher->publish(msg);
  return true;
}

void JAINode::emitRosImageMsg(int device_num, int source_num,
                              PvBuffer* buffer) {
  if (!buffer) return;

  auto current_time = this->systemTimeNano();
  auto buffer_time =
      buffer->GetTimestamp() + cameras[device_num]->timestamp_begin;
  if (!validateBufferTimestamp(device_num, source_num, buffer_time,
                               current_time))
    return;

  auto imageRosMsg = sensor_msgs::msg::CompressedImage();
  imageRosMsg.header.stamp = rclcpp::Time(buffer_time);
  cv::Mat image =
      cv::Mat(buffer->GetImage()->GetHeight(), buffer->GetImage()->GetWidth(),
              CV_8UC1, buffer->GetDataPointer());
  std::vector<uint8_t> compressed_data;
  std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY,
                                         95};  // Adjust quality as needed
  cv::imencode(".jpg", image, compressed_data, compression_params);
  imageRosMsg.format = "jpeg";
  imageRosMsg.header.frame_id = source_num == 0 ? "bayer" : "mono";
  imageRosMsg.data = std::move(compressed_data);

  imagePublishers[device_num][source_num]->publish(imageRosMsg);
}

void JAINode::initMultispectralCamera(int camera_num, std::string deviceName,
                                      std::string macAddress) {
  cameras.push_back(new MultiSpectralCamera(deviceName, macAddress));

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
      sleep(1);
    }
    return;
  }
  cameras[camera_num]->openStream();
  this->timestamp_begin_ros = this->systemTimeNano();
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
  std::string service_namespace = cameras[device_num]->deviceName +
                                  "/channel_" + std::to_string(channel_num);
  this->cameraDeviceParamSubscribers[device_num][channel_num] =
      this->create_subscription<std_msgs::msg::String>(
          service_namespace + "/manual_configure", 10,
          [this, device_num,
           channel_num](const std_msgs::msg::String::SharedPtr msg) {
            this->cameras[device_num]->closeStream();
            auto msg_proto = new ParameterUpdate();
            msg_proto->ParseFromString(msg->data);
            for (const auto param : msg_proto->parameters()) {
              this->cameras[device_num]->configureDevice(
                  channel_num, param.name(), param.type(), param.value());
            }
            this->cameras[device_num]->openStream();
            this->emitRosDeviceParamMsg(device_num, channel_num);
            free(msg_proto);
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

void JAINode::join_thread() {
  for (auto& t : subscription_thread) {
    t.join();
  }
  for (auto& t : subscription_thread) {
    t.detach();
  }
}

long long JAINode::systemTimeNano() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::time_point_cast<std::chrono::nanoseconds>(
                 std::chrono::high_resolution_clock::now())
                 .time_since_epoch())
      .count();
}
