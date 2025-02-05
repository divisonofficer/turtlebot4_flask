
#include <JAIHDRNode.h>
#include <Logger.h>

#include <chrono>
#include <filesystem>
#include <thread>

void sendFeedback(std::shared_ptr<GoalHandleHDRTrigger> goal_handle,
                  std::string feedback_message) {
  auto feedback = std::make_shared<HDRTrigger::Feedback>();
  feedback->feedback_message = feedback_message;
  goal_handle->publish_feedback(feedback);
}

void sendFeedbackPrintf(std::shared_ptr<GoalHandleHDRTrigger> goal_handle,
                        const char* format, ...) {
  va_list args;
  va_start(args, format);
  char buffer[256];
  vsnprintf(buffer, 256, format, args);
  sendFeedback(goal_handle, buffer);
  va_end(args);
}

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

void JAIRGBNIRCamera::triggerFrameCapture(int dn) {
  auto deviceParam = cameras[dn]->dualDevice->getDevice(0)->GetParameters();
  deviceParam->SetEnumValue("TriggerSelector", 3);

  deviceParam->ExecuteCommand("TriggerSoftware");
}

void JAIRGBNIRCamera::openStream(int dn) {
  auto deviceParam = cameras[dn]->dualDevice->getDevice(0)->GetParameters();
  deviceParam->ExecuteCommand("AcquisitionStart");
  // triggerFrameCapture(dn);
}

void JAIRGBNIRCamera::configureExposure(int dn, int sn, float exposure) {
  cameras[dn]->configureExposure(sn, exposure);
}

void JAIRGBNIRCamera::configureExposureAll(float exposure, float nir_exposure) {
  configureExposure(0, 0, exposure);
  configureExposure(0, 1, nir_exposure);
  configureExposure(1, 0, exposure);
  configureExposure(1, 1, nir_exposure);
  ts_exp_ = systemTimeNano();  //+ config->HDR_EXPOSURE_DELAY * 1000000;
}

void JAIRGBNIRCamera::flushStream() {
  std::vector<std::thread> threads;

  for (int i = 0; i < 2; i++) {
    for (int s = 0; s < 2; s++) {
      threads.emplace_back([this, i, s]() {
        PvStreamGEV* stream =
            (PvStreamGEV*)cameras[i]->dualDevice->getStream(s);
        stream->FlushPacketQueue();
        stream->AbortQueuedBuffers();
        std::vector<PvBuffer*> buffers;
        while (stream->GetQueuedBufferCount()) {
          PvBuffer* buffer;
          PvResult result;
          auto bresult = stream->RetrieveBuffer(&buffer, &result, 10);
          if (buffer) buffers.push_back(buffer);
        }
        for (auto buffer : buffers) {
          buffer->Reset();
          stream->QueueBuffer(buffer);
        }
      });
    }
  }
  for (auto& thread : threads) {
    thread.join();
  }

  openStreamAll();
}

int JAIRGBNIRCamera::retrieveBuffer(
    const std::shared_ptr<GoalHandleHDRTrigger> goal_handle, PvStream* stream,
    PvBuffer** buffer) {
  PvResult lResult, aResult;
  PvBuffer* lBuffer = nullptr;
  lResult =
      stream->RetrieveBuffer(&lBuffer, &aResult, 1000 / config->FRAME_RATE);
  if (lResult.IsOK() && aResult.IsOK()) {
    *buffer = lBuffer;
    return 0;
  }

  sendFeedbackPrintf(goal_handle,
                     "{\"type\":\"error_retrieve_buffer\",\"data\":{"
                     "\"lResult\" : \"%s\", \"aResult\" : \"%s\"}}",
                     lResult.GetCodeString().GetAscii(),
                     aResult.GetCodeString().GetAscii());
  if (lBuffer) {
    // lBuffer->Free();
    // lBuffer->Reset();
    // lBuffer->Alloc(static_cast<uint32_t>(config->BUFFER_SIZE));
    lBuffer->ResetChunks();

    stream->QueueBuffer(lBuffer);
    return 1;
  }
  if (lResult == PvResult::Code::TIMEOUT) {
    return -1;
  }
  return 2;
}

void JAIRGBNIRCamera::processStream(
    const std::shared_ptr<GoalHandleHDRTrigger> goal_handle, int d, int s,
    std::vector<cv::Mat>& dst) {
  PvBuffer* buffer = nullptr;
  int ret = retrieveBuffer(goal_handle, cameras[d]->dualDevice->getStream(s),
                           &buffer);
  if (!(hdr_stream_done_flag[d][s]) && buffer) {
    if (ret == 0) {
      if (buffer->GetTimestamp() + ts_cam_bs > ts_exp_) {
        dst[d * 2 + s] =
            cv::Mat(1080, 1440, CV_8UC1, buffer->GetDataPointer()).clone();
        hdr_stream_done_flag[d][s] = true;

      } else {
        sendFeedbackPrintf(
            goal_handle,
            "{\"type\":\"error_process_stream_expire\",\"data\":{\"device\" : "
            "%d, \"stream\" : %d, \"expired_time\" : %.3f }}",
            d, s, (buffer->GetTimestamp() + ts_cam_bs - ts_exp_) / 1000000.0);
      }
    }
  }
  if (buffer) {
    // buffer->Free();
    // buffer->Reset();
    // buffer->Alloc(static_cast<uint32_t>(config->BUFFER_SIZE));
    AcquireManager::getInstance()->queueBuffer(
        cameras[d]->dualDevice->getStream(s), buffer);
  }
  if (ret) {
    sendFeedbackPrintf(goal_handle,
                       "{\"type\":\"error_process_stream\",\"data\":{"
                       "\"device\" : %d, \"stream\" : %d, \"code\" : %d,}}",
                       d, s, ret);
  }
  if (ret == -1 || ret == 2) {
    hdr_stream_buffer_received[d][s] -= 1;
  } else {
    hdr_stream_buffer_received[d][s] -= config->HDR_TIMEOUT_CNT;
  }
}

int JAIRGBNIRCamera::readImage(
    const std::shared_ptr<GoalHandleHDRTrigger> goal_handle,
    std::vector<cv::Mat>& dst, __uint64_t& timestamp) {
  for (int i = 0; i < 4; i++) hdr_stream_done_flag[i / 2][i % 2] = false;

  while (true) {
    bool all_done = true;
    std::vector<std::thread> threads;

    for (int d = 0; d < 2; d++) {
      hdr_stream_buffer_received[d][0] = config->HDR_TIMEOUT_CNT;
      hdr_stream_buffer_received[d][1] = config->HDR_TIMEOUT_CNT;
      if (!(hdr_stream_done_flag[d][0]) || !(hdr_stream_done_flag[d][1])) {
        for (int s = 0; s < 2; s++) {
          threads.emplace_back(&JAIRGBNIRCamera::processStream, this,
                               goal_handle, d, s, std::ref(dst));
        }
      }
    }

    for (auto& thread : threads) {
      thread.join();
    }

    for (int d = 0; d < 2; d++) {
      if (!(hdr_stream_done_flag[d][0] && hdr_stream_done_flag[d][1])) {
        // if (hdr_stream_buffer_received[d][0] > 0 ||
        //     hdr_stream_buffer_received[d][1] > 0) {
        //   Debug << "Some buffer timeout";
        //   sendFeedback(goal_handle, "Some buffer timeout");
        //
        // } else {
        //   Debug << "Some buffer received but not all done";

        //   hdr_stream_buffer_received[d][0] = config->HDR_TIMEOUT_CNT;
        //   hdr_stream_buffer_received[d][1] = config->HDR_TIMEOUT_CNT;
        // }
        std::this_thread::sleep_for(
            std::chrono::nanoseconds(config->HDR_EXPOSURE_DELAY * 1000000));
        triggerFrameCapture(d);
        all_done = false;
      }
    }

    if (all_done) {
      break;
    }

    // std::this_thread::sleep_for(
    //     std::chrono::nanoseconds(config->HDR_EXPOSURE_DELAY * 1000000));
  }
  timestamp = systemTimeNano();

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

void JAIRGBNIRCamera::closeStreamAll() {
  for (int i = 0; i < 2; i++) cameras[i]->closeStream();
}

void JAIHDRNode::collectHdrImages(
    const std::shared_ptr<GoalHandleHDRTrigger> goal_handle) {
  std::vector<cv::Mat> images;
  std::vector<__uint64_t> timestamps;

  for (int t_idx = 0; t_idx < config->HDR_EXPOSURE.size(); t_idx++) {
    int t = config->HDR_EXPOSURE[t_idx];
    int t2 = config->HDR_EXPOSURE_NIR[t_idx];
    camera.configureExposureAll(t, t2);
    camera.openStreamAll();
    camera.flushStream();

    // std::this_thread::sleep_for(
    //     std::chrono::nanoseconds(config->HDR_EXPOSURE_DELAY));

    // camera.openStreamAll();
    camera.triggerFrameCapture(0);
    camera.triggerFrameCapture(1);
    Debug << "Collect HDR Images for " << t;

    sendFeedbackPrintf(goal_handle,
                       "{\"type\" : \"info_collect_hdr_images\", \"data\" : { "
                       "\"exposure\" : %d,"
                       "\"exp_idx\" : %d}}",
                       t, t_idx);

    std::vector<cv::Mat> imgr;
    imgr.assign(4, cv::Mat());
    __uint64_t timestamp;
    camera.readImage(goal_handle, imgr, timestamp);
    for (auto img : imgr) {
      images.push_back(img);
    }
    timestamps.push_back(timestamp);
  }
  camera.closeStreamAll();
  storage.storeHDRSequence(goal_handle->get_goal()->space_id, timestamps,
                           images);

  camera.flushStream();
  for (auto img : images) {
    img.release();
  }
  images.clear();
}

HDRStorage::HDRStorage() {}

void HDRStorage::storeHDRSequence(std::string space_id,
                                  std::vector<__uint64_t> timestamp,
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
  char path[256];

  snprintf(path, 256, "%s/%lld.png", space_id.c_str(), timestamp[0]);
  std::string dir_path = space_id;
  std::filesystem::create_directories(dir_path);
  cv::imwrite(path, combined_image);
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
