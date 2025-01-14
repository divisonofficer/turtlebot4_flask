

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
    cameras[i]->openStream();
  }
}

void JAIRGBNIRCamera::openStream(int dn) {
  // cameras[dn]->getStream(0)->queueBuffer(

  // )
  cameras[dn]->openStream();
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

int JAIRGBNIRCamera::readImage(int dn, cv::Mat& dst, cv::Mat& dst_2,
                               __uint64_t& timestamp) {
  bool rgb_done = false, nir_done = false;
  while (true) {
    for (int s = 0; s < 2; s++) {
      auto buffer = AcquireManager::getInstance()->AcquireBuffer(
          cameras[dn]->dualDevice->getStream(s));
      if (buffer) {
        if (buffer->GetTimestamp() + ts_cam_bs > ts_exp_) {
          if (s == 0) {
            dst =
                cv::Mat(1080, 1440, CV_8UC1, buffer->GetDataPointer()).clone();
            rgb_done = true;
            timestamp = buffer->GetTimestamp() + ts_cam_bs;
          } else {
            dst_2 =
                cv::Mat(1080, 1440, CV_8UC1, buffer->GetDataPointer()).clone();
            nir_done = true;
          }
        } else {
          Debug << "C" << dn << "/S" << s << " : expired buffer"
                << buffer->GetTimestamp() + ts_cam_bs - ts_exp_;
        }
        buffer->Free();
        buffer->Alloc(static_cast<uint32_t>(config->BUFFER_SIZE));
        AcquireManager::getInstance()->queueBuffer(
            cameras[dn]->dualDevice->getStream(s), buffer);
      } else {
        Debug << "C" << dn << "/S" << s << " : Buffer is null";
      }
    }

    if (rgb_done && nir_done) {
      break;
    }  // flushStream();
    openStream(dn);
  }
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
    camera.configureExposureAll(t);
    // camera.flushStream();
    std::this_thread::sleep_for(
        std::chrono::nanoseconds(config->HDR_EXPOSURE_DELAY));
    camera.flushStream();
    camera.openStreamAll();
    Debug << "Collect HDR Images for " << t;
    auto feedback = std::make_shared<HDRTrigger::Feedback>();
    feedback->feedback_message = "Collect HDR Images for " + std::to_string(t);
    goal_handle->publish_feedback(feedback);
    for (int i = 0; i < 2; i++) {
      cv::Mat img, img_nir;
      __uint64_t timestamp;
      camera.readImage(i, img, img_nir, timestamp);
      images.push_back(img);
      images.push_back(img_nir);
      timestamps.push_back(timestamp);
    }
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
      return;
    }
    result->success = true;
    result->result_message = "HDR Capture Finished";
    goal_handle->succeed(result);
  }).detach();
}
