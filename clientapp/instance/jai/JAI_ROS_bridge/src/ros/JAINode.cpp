#include <JAINode.h>
#include <Logger.h>
#include <jai.pb.h>
#include <omp.h>

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
      std::thread([this]() { this->cameras[0]->runUntilInterrupted(0); }));
  subscription_thread.push_back(
      std::thread([this]() { this->cameras.back()->runUntilInterrupted(0); }));
  subscription_thread.push_back(
      std::thread([this]() { this->cameras[0]->runUntilInterrupted(1); }));
  subscription_thread.push_back(
      std::thread([this]() { this->cameras.back()->runUntilInterrupted(1); }));
}

void JAINode::enlistJAIDevice(int device_num, std::string device_name,
                              int channel_count) {
  // Adjust size of Device Vectors
  if (imagePublishers.size() <= device_num) {
    imagePublishers.resize(device_num + 1);
    imagePublishers_fusion.resize(device_num + 1);
    imagePublishers_hdr.resize(device_num + 1);
    cameraDeviceParamPublishers.resize(device_num + 1);
    cameraDeviceParamSubscribers.resize(device_num + 1);
    cameraStreamTriggerSubscribers.resize(device_num + 1);
    cameraAutoExposureHoldTriggerSubscribers.resize(device_num + 1);
  }
  // Adjust size of Stream Channel Vectors
  if (imagePublishers[device_num].size() < channel_count) {
    imagePublishers[device_num].resize(channel_count);
    imagePublishers_fusion[device_num].resize(channel_count);
    imagePublishers_hdr[device_num].resize(channel_count);
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

  cameraAutoExposureHoldTriggerSubscribers[device_num] =
      this->create_subscription<std_msgs::msg::Bool>(
          device_name + "/auto_exposure_hold_trigger", 10,
          [this, device_num](std_msgs::msg::Bool trigger) {
            auto ret = this->cameras[device_num]->holdAutoExposureAndGetValue(
                trigger.data);
            // emitRosDeviceParamMsg(device_num, 0, "ExposureTime");
            // emitRosDeviceParamMsg(device_num, 1, "ExposureTime");
          });
  // Per Stream Publishers
  for (int i = 0; i < channel_count; i++) {
    imagePublishers[device_num][i] =
        this->create_publisher<sensor_msgs::msg::CompressedImage>(
            device_name + "/channel_" + std::to_string(i), 3);
    imagePublishers_fusion[device_num][i] =
        this->create_publisher<sensor_msgs::msg::CompressedImage>(
            device_name + "/channel_" + std::to_string(i) + "/fusion", 3);

    imagePublishers_hdr[device_num][i] =
        this->create_publisher<sensor_msgs::msg::CompressedImage>(
            device_name + "/channel_" + std::to_string(i) + "/hdr", 3);

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

// OpenMP 사용을 위한 헤더 파일

void JAINode::convertTo16Bit(const uint8_t* src, int width, int height,
                             cv::Mat& dst, int bitDepth, bool isRGB) {
  dst = cv::Mat(height, width * 2, CV_8UC1);

  int channels = isRGB ? 3 : 1;

  int shift = bitDepth == 10 ? 6 : 4;
#pragma omp parallel for collapse(2)  // OpenMP 병렬 처리
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      for (int c = 0; c < channels; ++c) {
        dst.at<uint8_t>(y, x) = *src;
        dst.at<uint8_t>(y, x + width) =
            (*(src + 1) << shift) + ((*src) >> (8 - shift));

        src += 2;
      }
    }
  }
}

void JAINode::processHdrImage(int dn, int sn, PvBuffer* buffer,
                              unsigned long buffer_time) {
  if (hdr_exposure_image.size() <= dn) {
    hdr_exposure_image.resize(dn + 1);
  }
  if (hdr_exposure_image[dn].size() <= sn) {
    hdr_exposure_image[dn].resize(sn + 1);
  }

  if (hdr_exposure_idx[dn][sn] > -1) {
    hdr_exposure_image[dn][sn].push_back(std::make_pair(
        hdr_exposures[hdr_exposure_idx[dn][sn]], buffer->GetDataPointer()));
  }

  if (hdr_exposure_idx[dn][sn] < 3) {
    hdr_exposure_idx[dn][sn]++;
    cameras[dn]->configureExposure(sn, hdr_exposures[hdr_exposure_idx[dn][sn]]);
  }

  else {
    hdr_exposure_idx[dn][sn] = -1;
    cv::Mat hdr, fusion;
    hdr_fusion.fusion_hdr(hdr_exposure_image[dn][sn], hdr, fusion, sn == 0);

    auto hdrMsg = sensor_msgs::msg::CompressedImage();
    hdrMsg.header.stamp = rclcpp::Time(buffer_time);
    hdrMsg.header.frame_id = "jai_1600_";
    hdrMsg.header.frame_id += sn == 0 ? "left" : "right";
    hdrMsg.data.assign(hdr.data, hdr.data + hdr.total() * hdr.elemSize());
    imagePublishers_hdr[dn][sn]->publish(hdrMsg);

    auto fusionMsg = sensor_msgs::msg::CompressedImage();
    fusionMsg.header.stamp = rclcpp::Time(buffer_time);
    fusionMsg.header.frame_id = "jai_1600_";
    fusionMsg.header.frame_id += sn == 0 ? "left" : "right";
    fusionMsg.data.assign(fusion.data,
                          fusion.data + fusion.total() * fusion.elemSize());
    imagePublishers_fusion[dn][sn]->publish(fusionMsg);
  }
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

  if (hdr_capture_mode) {
    processHdrImage(device_num, source_num, buffer, buffer_time);
    return;
  }

  auto imageRosMsg = sensor_msgs::msg::CompressedImage();
  imageRosMsg.header.stamp = rclcpp::Time(buffer_time);
  auto pixelType = buffer->GetImage()->GetPixelType();
  auto pixelSize = buffer->GetImage()->GetPixelSize(pixelType);

  int compressType;
  std::string imageFormat = "jpg";
  std::string colorType;
  int basePixelNumber = 8;
  switch (pixelType) {
    case PvPixelMono8:
      colorType = "mono";
      compressType = CV_8UC1;
      break;
    case PvPixelBayerRG8:
      colorType = "bayer";
      compressType = CV_8UC1;
      break;
    case PvPixelBayerRG10:
      basePixelNumber = 10;
      colorType = "bayer";
      compressType = CV_16UC1;
      break;
    case PvPixelBayerRG12:
      colorType = "bayer";
      basePixelNumber = 12;
      compressType = CV_16UC1;
      break;
    case PvPixelMono10:
      colorType = "mono";
      basePixelNumber = 10;
      compressType = CV_16UC1;
      break;
    case PvPixelMono12:
      colorType = "mono";
      basePixelNumber = 12;
      compressType = CV_16UC1;
      break;
    case PvPixelRGB8:
      colorType = "rgb";
      compressType = CV_8UC3;
      break;
    case PvPixelRGB12:
      colorType = "rgb";
      basePixelNumber = 12;
      compressType = CV_16UC3;
      break;
    case PvPixelRGB10:

    case PvPixelRGB10p32:
    case PvPixelRGB10p:
      colorType = "rgb";
      basePixelNumber = 10;
      compressType = CV_16UC3;
      break;

    default:
      colorType = "unknown";
      compressType = CV_8UC1;
      break;
  }
  imageRosMsg.format = imageFormat;
  imageRosMsg.header.frame_id =
      colorType + "_" + std::to_string(basePixelNumber) + "bit";
  imageRosMsg.data.assign(buffer->GetDataPointer(),
                          buffer->GetDataPointer() + buffer->GetSize());
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
              if (param.value() != "") {
                this->cameras[device_num]->configureDevice(
                    channel_num, param.name(), param.type(), param.value());
              }
              this->emitRosDeviceParamMsg(device_num, channel_num,
                                          param.name());
            }
            this->cameras[device_num]->openStream();

            free(msg_proto);
          });
}

void JAINode::emitRosDeviceParamMsg(int device_num, int source_num,
                                    std::string parameter) {
  std_msgs::msg::String msg;
  ParameterUpdate param_update;
  ParameterValue* param_info = param_update.add_parameters();
  param_info->set_name(parameter);

  param_info->set_value(
      cameras[device_num]->getParameter(source_num, parameter));
  msg.data = param_update.SerializeAsString();
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
