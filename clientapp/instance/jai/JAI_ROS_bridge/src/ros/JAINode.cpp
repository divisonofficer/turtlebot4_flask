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
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 2; j++) {
      hdr_scenario.getCameraExposureCallback[i][j] = [this, i, j]() {
        return cameras[i]->getExposure(j);
      };
      hdr_scenario.setCameraExposureCallback[i][j] = [this, i,
                                                      j](float exposure) {
        cameras[i]->configureExposure(j, exposure);
      };
      hdr_scenario.publishFusionImageCallback[i][j] =
          [this, i, j](sensor_msgs::msg::CompressedImage fusion) {
            imagePublishers_fusion[i][j]->publish(fusion);
          };
      hdr_scenario.publishHdrImageCallback[i][j] =
          [this, i, j](sensor_msgs::msg::CompressedImage hdr) {
            imagePublishers_hdr[i][j]->publish(hdr);
          };
    }
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
  mergedImagePublisher =
      this->create_publisher<sensor_msgs::msg::CompressedImage>(
          "/jai_1600_stereo/merged", 3);
  openStream(-1);

  this->cameras[0]->initCameraTimestamp();
  this->cameras[1]->initCameraTimestamp();
  this->timestamp_begin_ros = this->systemTimeNano();
  subscription_thread.push_back(
      std::thread([this]() { this->cameras.back()->runUntilInterrupted(0); }));
  subscription_thread.push_back(
      std::thread([this]() { this->cameras[0]->runUntilInterrupted(0); }));

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
    // imagePublishers[device_num][i] =
    //     this->create_publisher<sensor_msgs::msg::CompressedImage>(
    //         device_name + "/channel_" + std::to_string(i), 3);
    // imagePublishers_fusion[device_num][i] =
    //     this->create_publisher<sensor_msgs::msg::CompressedImage>(
    //         device_name + "/channel_" + std::to_string(i) + "/fusion", 3);

    // imagePublishers_hdr[device_num][i] =
    //     this->create_publisher<sensor_msgs::msg::CompressedImage>(
    //         device_name + "/channel_" + std::to_string(i) + "/hdr", 3);

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
    cameras[0]->closeStream();
    cameras[1]->closeStream();
    cameras[0]->timeStampReset(0, 0);
    int64_t timestamp_left;
    cameras[0]->dualDevice->getDevice(0)->GetParameters()->GetIntegerValue(
        "Timestamp", timestamp_left);
    cameras[1]->timeStampReset(cameras[0]->timestamp_begin, timestamp_left);
    cameras[0]->openStream();
    cameras[1]->openStream();
    // Debug << "Timestamp Reset" << timestamp_left;
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

void JAINode::emitRosImageMsg(int device_num, int source_num,
                              PvBuffer* buffer) {
  if (!buffer) return;

  auto current_time = this->systemTimeNano();
  auto buffer_time =
      buffer->GetTimestamp() + cameras[device_num]->timestamp_begin;
  if (!validateBufferTimestamp(device_num, source_num, buffer_time,
                               current_time))
    return;
  source_framerate[device_num][source_num] =
      1000000000.0 / (buffer_time - timestamp_history[device_num][source_num]);
  timestamp_history[device_num][source_num] = buffer_time;
  double timestamp_fastest = timestamp_history[1 - device_num][0];
  double time_diff =
      (double(buffer_time) - double(timestamp_fastest)) / 1000000.0;

  Debug << device_num << "/" << source_num << " Time Diff : " << time_diff;
  if (source_num == 0) {
    if (device_num == 0) {
      Debug << "Frame Rate : " << source_framerate[0][0] << " "
            << source_framerate[1][0];
      if (time_diff < 1000.0) {
        triggerDelayPending = 0;
        while (time_diff > 1000.0 / FRAME_RATE) {
          time_diff -= 1000.0 / FRAME_RATE;
        }
        if (time_diff > MIN_DIFFS && time_diff < 500.0 / FRAME_RATE) {
          triggerDelayPending = time_diff;
        } else if (time_diff > MIN_DIFFS && time_diff < 1000.0 / FRAME_RATE &&
                   1000.0 / FRAME_RATE - time_diff > MIN_DIFFS) {
          triggerDelayPending = 1000.0 / FRAME_RATE - time_diff;
        }
      }
    }
  }

  if (hdr_capture_mode) {
    hdr_scenario.processHdrImage(device_num, source_num, buffer, buffer_time);
    return;
  }

  uint8_t* buffer_data = (uint8_t*)calloc(1440 * 1080, sizeof(uint8_t));
  memcpy(buffer_data, buffer->GetDataPointer(), 1440 * 1080 * sizeof(uint8_t));
  auto exposure_time = cameras[device_num]->getExposure(source_num);
  buffer_queue[device_num][source_num].push(
      std::make_pair(std::make_pair(buffer_time, exposure_time), buffer_data));

  if (device_num == 0 && source_num == 0) {
    while (buffer_queue[0][0].size() && buffer_queue[0][1].size() &&
           buffer_queue[1][0].size() && buffer_queue[1][1].size()) {
      auto time00 = buffer_queue[0][0].front().first.first;
      auto time01 = buffer_queue[0][1].front().first.first;
      auto time10 = buffer_queue[1][0].front().first.first;
      auto time11 = buffer_queue[1][1].front().first.first;
      if (abs(time00 - time01) > 1000000) {
        if (time00 < time01) {
          free(buffer_queue[0][0].front().second);
          buffer_queue[0][0].pop();
        } else {
          free(buffer_queue[0][1].front().second);
          buffer_queue[0][1].pop();
        }
        continue;
      }
      if (abs(time10 - time11) > 1000000) {
        if (time10 < time11) {
          free(buffer_queue[1][0].front().second);
          buffer_queue[1][0].pop();
        } else {
          free(buffer_queue[1][1].front().second);
          buffer_queue[1][1].pop();
        }
        continue;
      }
      if (abs(time00 - time10) > 3000000) {
        if (time00 < time10) {
          free(buffer_queue[0][0].front().second);
          buffer_queue[0][0].pop();
          free(buffer_queue[0][1].front().second);
          buffer_queue[0][1].pop();
        } else {
          free(buffer_queue[1][0].front().second);
          buffer_queue[1][0].pop();
          free(buffer_queue[1][1].front().second);
          buffer_queue[1][1].pop();
        }
        continue;
      }
      uint8_t* buffer_merged =
          (uint8_t*)calloc(ROS_MSG_BUFFER_SIZE * 8, sizeof(uint8_t));
      auto buffer_time = time00;
      sensor_msgs::msg::CompressedImage imageRosMsg =
          sensor_msgs::msg::CompressedImage();
      imageRosMsg.header.stamp = rclcpp::Time(buffer_time);

      imageRosMsg.format = "jpg";
      imageRosMsg.header.frame_id = "jai_1600_stereo";
      for (int i = 0; i < 4; i++) {
        imageRosMsg.header.frame_id += "_";
        imageRosMsg.header.frame_id +=
            std::to_string(buffer_queue[i / 2][i % 2].front().first.second);
      }
      for (int i = 0; i < 4; i++) {
        int sn = i / 2, dn = i % 2;
        cv::Mat img(1080, 1440, CV_8UC1, buffer_queue[dn][sn].front().second);
        if (sn == 0) {
          cv::cvtColor(img, img, cv::COLOR_BayerRG2RGB);
          if (ROS_SCALE_DOWN)
            cv::resize(img, img, cv::Size(720, 540), 0, 0, cv::INTER_LINEAR);
        } else {
          if (ROS_SCALE_DOWN) img = img(cv::Rect(0, 0, 720, 540));
        }
        memcpy(buffer_merged +
                   ROS_MSG_BUFFER_SIZE * (dn * (sn == 0 ? 3 : 1) + sn * 6),
               img.data, ROS_MSG_BUFFER_SIZE * sizeof(uint8_t) * (3 - sn * 2));
        free(buffer_queue[dn][sn].front().second);
        img.release();
        buffer_queue[dn][sn].pop();
      }

      imageRosMsg.data.assign(buffer_merged,
                              buffer_merged + ROS_MSG_BUFFER_SIZE * 8);
      mergedImagePublisher->publish(imageRosMsg);
      free(buffer_merged);
    }
  }

  // emitRosImageMsgPublish(device_num, source_num, buffer, buffer_time);
}

void JAINode::emitRosImageMsgPublishBufferBytes(int dn, int sn, uint8_t* buffer,
                                                uint64_t buffer_time,
                                                uint32_t buffer_size,
                                                std::string frame_id) {
  sensor_msgs::msg::CompressedImage imageRosMsg =
      sensor_msgs::msg::CompressedImage();
  imageRosMsg.header.stamp = rclcpp::Time(buffer_time);
  std::string imageFormat = "jpg";
  imageRosMsg.format = imageFormat;
  imageRosMsg.header.frame_id = frame_id;
  imageRosMsg.data.assign(buffer, buffer + buffer_size);
  imagePublishers[dn][sn]->publish(imageRosMsg);
}

void JAINode::emitRosImageMsgPublish(int device_num, int source_num,
                                     PvBuffer* buffer, uint64_t buffer_time) {
  PvPixelType pixelType = buffer->GetImage()->GetPixelType();
  uint32_t pixelSize = buffer->GetImage()->GetPixelSize(pixelType);
  int compressType;

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

  std::string frame_id =
      colorType + "_" + std::to_string(basePixelNumber) + "bit";
  int buffer_size = buffer->GetImage()->GetWidth() *
                    buffer->GetImage()->GetHeight() * pixelSize / 8;
  emitRosImageMsgPublishBufferBytes(device_num, source_num,
                                    buffer->GetDataPointer(), buffer_time,
                                    buffer_size, frame_id);
}

void JAINode::initMultispectralCamera(int camera_num, std::string deviceName,
                                      std::string macAddress) {
  cameras.push_back(new MultiSpectralCamera(deviceName, macAddress));
  cameras.back()->device_idx = camera_num;
  if (camera_num == 0 && TRIGGER_SYNC) {
    cameras[0]->dualDevice->getDevice(0)->GetParameters()->SetEnumValue(
        "ExposureAuto", 2);
    cameras[0]->dualDevice->getDevice(0)->GetParameters()->SetEnumValue(
        "GainAuto", 0);

    cameras.back()->triggerCallback = [this]() {
      if (triggerDelayPending > 0.0f) {
        cameras[0]->closeStream();
        cameras[1]->closeStream();
        // sleep(0.1);
        Info << "Adjust Stereo Pulse Delay" << triggerDelayPending;
        double delay;
        cameras[0]->dualDevice->getDevice(0)->GetParameters()->SetEnumValue(
            "TriggerSelector", 3);

        cameras[0]->dualDevice->getDevice(0)->GetParameters()->GetFloatValue(
            "TriggerDelay", delay);
        if (triggerDelayPending < 0.01f) {
          delay = 0.0f;
        } else {
          // triggerDelayPending -= 0.1f;
        }
        delay += triggerDelayPending * 1000;
        auto max_delay = 1000000.0 / FRAME_RATE;

        while (delay > max_delay) {
          delay -= max_delay;
        }
        cameras[0]->dualDevice->getDevice(0)->GetParameters()->SetFloatValue(
            "TriggerDelay", delay);

        triggerDelayPending = 0.0f;
        cameras[0]->openStream();
        cameras[1]->openStream();
      }

      if (stereo_exposure_sync) {
        float exposure_left = cameras[0]->getExposure(0);
        // float gain_left = cameras[0]->getGain(0);
        cameras[1]->configureExposure(0, exposure_left);
        // cameras[1]->configureGain(0, gain_left);
        exposure_left = cameras[0]->getExposure(1);
        // gain_left = cameras[0]->getGain(1);
        cameras[1]->configureExposure(1, exposure_left);
        // cameras[1]->configureGain(1, gain_left);
      }

      // cameras[0]->openStream();
      // cameras[1]->openStream();
      // cameras[0]->closeStream();
      // cameras[1]->closeStream();
    };
  }

  if (camera_num == 1 && stereo_exposure_sync) {
    cameras.back()->holdAutoExposureAndGetValue(true);
    cameras[0]->dualDevice->getDevice(0)->GetParameters()->SetEnumValue(
        "ExposureAuto", 2);
  }

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
    for (int i = 0; i < int(cameras.size()); i++) {
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
    for (int i = 0; i < int(cameras.size()); i++) {
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

HdrScenario::HdrScenario() {
  hdr_exposure_image.resize(2);
  for (int i = 0; i < 2; i++) {
    hdr_exposure_image[i].resize(2);
    for (int j = 0; j < 2; j++) {
      hdr_exposure_idx[i][j] = -1;
      hdr_intensity_error_count[i][j] = 0;
    }
  }
  hdr_exposures[0] = 1000.0f;
  hdr_exposures[1] = 4000.0f;
  hdr_exposures[2] = 16000.0f;
  hdr_exposures[3] = 64000.0f;
}

void HdrScenario::processHdrImage(int dn, int sn, PvBuffer* buffer,
                                  unsigned long buffer_time) {
  if (hdr_intensity_error_count[dn][sn] > 10) {
    clearImageVector(dn, sn);
    Info << dn << "/" << sn << "HDR Reset";
  }

  cv::Mat img(1080, 1440, !sn ? CV_8UC3 : CV_8UC1, buffer->GetDataPointer());
  if (!validateImageExposure(dn, sn, img)) {
    return;
  }
  if (hdr_exposure_idx[dn][sn] < 3) {
    configureExposure(dn, sn, hdr_exposures[hdr_exposure_idx[dn][sn] + 1]);
    hdr_config_timestamp[dn][sn] = this->systemTimeNano();
  }
  if (hdr_exposure_idx[dn][sn] > -1) {
    hdr_exposure_image[dn][sn].push_back(
        std::make_pair(hdr_exposures[hdr_exposure_idx[dn][sn]], img));
  }
  if (hdr_exposure_idx[dn][sn] == 3) {
    hdr_exposure_idx[dn][sn] = -1;
    if (!validateImageIntensities(dn, sn)) {
      clearImageVector(dn, sn);
      return;
    }
    cv::Mat hdr, fusion;
    hdr_fusion.fusion_hdr(hdr_exposure_image[dn][sn], hdr, fusion, sn == 0);

    if (fusion.rows != 1080 || fusion.cols != 1440) {
      ErrorLog << dn << "/" << sn << " Fusion Image Size Mismatch"
               << fusion.rows << "x" << fusion.cols;

    } else {
      publishFusionImage(dn, sn, fusion, buffer_time);
      publishHdrImage(dn, sn, hdr, buffer_time);
    }

    clearImageVector(dn, sn);
    return;
  }
  hdr_exposure_idx[dn][sn]++;
}

bool HdrScenario::validateImageIntensities(int dn, int sn) {
  std::vector<double> intensity_mean;
  for (auto& [exposure, img] : hdr_exposure_image[dn][sn]) {
    intensity_mean.push_back(cv::mean(img)[0]);
  }

  if (!(intensity_mean[0] * 1.1 < intensity_mean[1] &&
        intensity_mean[1] * 1.1 < intensity_mean[2] &&
        intensity_mean[2] * 1.2 < intensity_mean[3])) {
    ErrorLog << dn << "/" << sn << "Exposure misaligned" << intensity_mean[0]
             << " " << intensity_mean[1] << " " << intensity_mean[2] << " "
             << intensity_mean[3];
    return false;
  }
  return true;
}

bool HdrScenario::validateImageExposure(int dn, int sn, cv::Mat& img) {
  if (hdr_exposure_idx[dn][sn] >= 0) {
    auto intensity_average = cv::mean(img)[0];
    if (hdr_exposures[hdr_exposure_idx[dn][sn]] != getCameraExposure(dn, sn) ||
        (hdr_exposure_idx[dn][sn] == 0 && intensity_average > 30.0f) ||
        (hdr_exposure_idx[dn][sn] > 0 &&
         intensity_average < hdr_intensity_history[dn][sn] * 1.2f)) {
      img.release();
      configureExposure(dn, sn, hdr_exposures[hdr_exposure_idx[dn][sn]]);
      hdr_intensity_error_count[dn][sn]++;

      return false;
    }
    hdr_intensity_history[dn][sn] = intensity_average;
  }
  return true;
}

void HdrScenario::configureExposure(int device, int source, float exposure) {
  setCameraExposureCallback[device][source](exposure);
}

float HdrScenario::getCameraExposure(int device, int source) {
  return getCameraExposureCallback[device][source]();
}

void HdrScenario::clearImageVector(int dn, int sn) {
  hdr_exposure_image[dn][sn].clear();
  hdr_exposure_idx[dn][sn] = -1;
  hdr_intensity_error_count[dn][sn] = 0;
}

void HdrScenario::publishFusionImage(int dn, int sn, cv::Mat& fusion,
                                     unsigned long buffer_time) {
  auto fusionMsg = sensor_msgs::msg::CompressedImage();
  fusionMsg.header.stamp = rclcpp::Time(buffer_time);
  fusionMsg.header.frame_id = "jai_1600_";
  fusionMsg.header.frame_id += sn == 0 ? "left" : "right";
  fusionMsg.data.assign(fusion.data,
                        fusion.data + fusion.total() * fusion.elemSize());

  publishFusionImageCallback[dn][sn](fusionMsg);
}

void HdrScenario::publishHdrImage(int dn, int sn, cv::Mat& hdr,
                                  unsigned long buffer_time) {
  auto hdrMsg = sensor_msgs::msg::CompressedImage();

  hdrMsg.header.stamp = rclcpp::Time(buffer_time);
  hdrMsg.header.frame_id = "jai_1600_";
  hdrMsg.header.frame_id += sn == 0 ? "left" : "right";
  hdrMsg.data.assign(hdr.data, hdr.data + hdr.total() * hdr.elemSize());
  publishHdrImageCallback[dn][sn](hdrMsg);
}

uint64_t HdrScenario::systemTimeNano() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::time_point_cast<std::chrono::nanoseconds>(
                 std::chrono::high_resolution_clock::now())
                 .time_since_epoch())
      .count();
}