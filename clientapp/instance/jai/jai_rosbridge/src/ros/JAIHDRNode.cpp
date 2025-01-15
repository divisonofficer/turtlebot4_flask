

#include <JAIHDRNode.h>
#include <Logger.h>

#include <chrono>
#include <thread>

double systemTimeNano() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::time_point_cast<std::chrono::nanoseconds>(
                 std::chrono::high_resolution_clock::now())
                 .time_since_epoch())
      .count();
}

JAIRGBNIRCamera::JAIRGBNIRCamera() {}

void JAIRGBNIRCamera::connectCamera() {
  if (!cameras.size()) {
    cameras.push_back(new MultiSpectralCamera(config->DEVICE_LEFT_NAME,
                                              config->DEVICE_LEFT_ADDRESS));

    cameras.push_back(new MultiSpectralCamera(config->DEVICE_RIGHT_NAME,
                                              config->DEVICE_RIGHT_ADDRESS));
    cameras[0]->timeStampReset(0, 0);
    cameras.back()->timeStampReset(0, 0);
    ts_cam_bs = systemTimeNano();
  }
}

void JAIRGBNIRCamera::openStreamAll() {
  for (int i = 0; i < 2; i++) {
    openStream(i);
  }
}

void JAIRGBNIRCamera::openStream(int dn) {
  // cameras[dn]->getStream(0)->queueBuffer(

  // )
  auto deviceParam = cameras[dn]->dualDevice->getDevice(0)->GetParameters();
  deviceParam->SetEnumValue("TriggerSelector", 3);
  deviceParam->ExecuteCommand("AcquisitionStart");
  deviceParam->ExecuteCommand("TriggerSoftware");
}

void JAIRGBNIRCamera::configureExposure(int dn, int sn, float exposure) {
  cameras[dn]->configureExposure(sn, exposure);
}

void JAIRGBNIRCamera::configureExposureAll(float exposure) {
  configureExposure(0, 0, exposure);
  configureExposure(0, 1, exposure);
  configureExposure(1, 0, exposure);
  configureExposure(1, 1, exposure);
  ts_exp_ = systemTimeNano() + config->HDR_EXPOSURE_DELAY * 1000000;
}

void JAIRGBNIRCamera::flushStream() {
  for (int i = 0; i < 2; i++) {
    cameras[i]->closeStream();
    for (int s = 0; s < 2; s++) {
      PvStreamGEV* stream = (PvStreamGEV*)cameras[i]->dualDevice->getStream(s);
      stream = (PvStreamGEV*)cameras[i]->dualDevice->getStream(1);
      stream->FlushPacketQueue();
      stream->AbortQueuedBuffers();
      for (int b = 0; b < stream->GetQueuedBufferCount(); b++) {
        PvBuffer* buffer;
        PvResult result;
        stream->RetrieveBuffer(&buffer, &result, 100);
        buffer->Free();
        buffer->Alloc(static_cast<uint32_t>(config->BUFFER_SIZE));
        stream->QueueBuffer(buffer);
      }
    }
  }
}

int JAIRGBNIRCamera::readImage(std::vector<cv::Mat>& dst,
                               __uint64_t& timestamp) {
  bool buffer_done[2][2] = {{false, false}, {false, false}};
  while (true) {
    bool all_done = true;
    for (int d = 0; d < 2; d++) {
      for (int s = 0; s < 2; s++) {
        auto buffer = AcquireManager::getInstance()->AcquireBuffer(
            cameras[d]->dualDevice->getStream(s));
        if (buffer) {
          if (buffer->GetTimestamp() + ts_cam_bs > ts_exp_) {
            if (!buffer_done[d][s]) {
              dst[d * 2 + s] =
                  cv::Mat(1080, 1440, CV_8UC1, buffer->GetDataPointer())
                      .clone();
              buffer_done[d][s] = true;
              if (s + d == 0) timestamp = buffer->GetTimestamp() + ts_cam_bs;
            }
          } else {
            Debug << "C" << d << "/S" << s << " : expired buffer" << std::fixed
                  << std::setprecision(3)
                  << (buffer->GetTimestamp() + ts_cam_bs - ts_exp_) / 1000000.0;
          }
          buffer->Free();
          buffer->Alloc(static_cast<uint32_t>(config->BUFFER_SIZE));
          AcquireManager::getInstance()->queueBuffer(
              cameras[d]->dualDevice->getStream(s), buffer);
        } else {
          Debug << "C" << d << "/S" << s << " : Buffer is null";
        }
      }
    }
    for (int d = 0; d < 2; d++) {
      if (!(buffer_done[d][0] && buffer_done[d][1])) {
        openStream(d);
        all_done = false;
      }
    }
    if (all_done) {
      break;
    }
    std::this_thread::sleep_for(
        std::chrono::nanoseconds(config->HDR_EXPOSURE_DELAY * 1000000));
  }
  return 0;
}

JAIHDRNode::JAIHDRNode() : Node("jai_hdr_node") {
  connectCamera();
  initNodeService();
}

void JAIHDRNode::connectCamera() { camera.connectCamera(); }

void JAIHDRNode::initNodeService() {
  // ROS2 node initialization
  service_hdr_trigger = this->create_service<std_srvs::srv::Trigger>(
      "jai_hdr_trigger",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        (void)request;
        // this->collectHdrImages();
        response->success = true;
      });

  action_server_hdr_trigger =
      rclcpp_action::create_server<jai_rosbridge::action::HDRTrigger>(
          this, "jai_hdr_trigger",
          std::bind(&JAIHDRNode::action_hdr_trigger_handler, this,
                    std::placeholders::_1, std::placeholders::_2),
          std::bind(&JAIHDRNode::action_hdr_trigger_cancel, this,
                    std::placeholders::_1),
          std::bind(&JAIHDRNode::action_hdr_trigger_accepted, this,
                    std::placeholders::_1));
}

void JAIHDRNode::collectHdrImages(
    const std::shared_ptr<GoalHandleHDRTrigger> goal_handle) {
  std::vector<cv::Mat> images;
  std::vector<__uint64_t> timestamps;
  for (float t : config->HDR_EXPOSURE) {
    // camera.flushStream();
    camera.flushStream();
    camera.configureExposureAll(t);
    // camera.flushStream();
    // std::this_thread::sleep_for(
    //     std::chrono::nanoseconds(config->HDR_EXPOSURE_DELAY));

    camera.openStreamAll();
    Debug << "Collect HDR Images for " << t;
    auto feedback = std::make_shared<HDRTrigger::Feedback>();
    feedback->feedback_message = "Collect HDR Images for " + std::to_string(t);
    goal_handle->publish_feedback(feedback);

    std::vector<cv::Mat> imgr;
    imgr.assign(4, cv::Mat());
    __uint64_t timestamp;
    camera.readImage(imgr, timestamp);
    for (auto img : imgr) {
      images.push_back(img);
    }
    timestamps.push_back(timestamp);
  }
  storage.storeHDRSequence(timestamps, images);

  camera.flushStream();
  for (auto img : images) {
    img.release();
  }
  images.clear();
}

HDRStorage::HDRStorage() {}

void HDRStorage::storeHDRSequence(std::vector<__uint64_t> timestamp,
                                  std::vector<cv::Mat> images) {
  // Store HDR images
  //   Debug << "Store HDR images" << images.size();

  //   for (size_t i = 0; i < images.size(); ++i) {
  //     std::stringstream ss;
  //     ss << "temp/image_" << i << ".png";
  //     cv::imwrite(ss.str(), images[i]);
  //   }
  // // Combine images into a single image
  int rows = 4;
  int cols = 4;
  int img_height = 1080;
  int img_width = 1440;
  cv::Mat combined_image =
      cv::Mat::zeros(rows * img_height, cols * img_width, images[0].type());

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      int index = i * cols + j;
      if (index < images.size()) {
        cv::Mat roi = combined_image(
            cv::Rect(j * img_width, i * img_height, img_width, img_height));
        images[index].copyTo(roi);
      }
    }
  }

  // Save the combined image
  cv::imwrite("temp/combined_image.png", combined_image);
}

rclcpp_action::GoalResponse JAIHDRNode::action_hdr_trigger_handler(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const HDRTrigger::Goal> goal) {
  if (this->hdr_trigger_flag.load()) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  hdr_trigger_flag.store(true);
  cancel_flag.store(false);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse JAIHDRNode::action_hdr_trigger_cancel(
    const std::shared_ptr<GoalHandleHDRTrigger> goal_handle) {
  this->cancel_action();
  hdr_trigger_flag.store(false);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JAIHDRNode::action_hdr_trigger_accepted(
    const std::shared_ptr<GoalHandleHDRTrigger> goal_handle) {
  Debug << "HDR Trigger Accepted";
  std::thread([this, goal_handle]() {
    this->collectHdrImages(goal_handle);
    auto result = std::make_shared<HDRTrigger::Result>();
    if (this->cancel_flag.load()) {
      result->success = false;
      goal_handle->abort(result);
      hdr_trigger_flag.store(false);
      return;
    }
    result->success = true;
    result->result_message = "HDR Capture Finished";
    goal_handle->succeed(result);
    hdr_trigger_flag.store(false);
  }).detach();
}
