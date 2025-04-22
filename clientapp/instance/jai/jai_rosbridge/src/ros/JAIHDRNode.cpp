
#include <JAIHDRNode.h>
#include <Logger.h>

#include <chrono>
#include <filesystem>
#include <rclcpp/executor.hpp>
#include <rclcpp/utilities.hpp>
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
    if (!config->HDR_CAPTURE_SINGLE)
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
  for (int i = 0; i < cameras.size(); i++) {
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
  for (int i = 0; i < cameras.size(); i++) {
    configureExposure(i, 0, exposure);
    configureExposure(i, 1, nir_exposure);
  }
  ts_exp_ = systemTimeNano();  //+ config->HDR_EXPOSURE_DELAY * 1000000;
}

void JAIRGBNIRCamera::flushStream() {
  std::vector<std::thread> threads;

  for (int i = 0; i < cameras.size(); i++) {
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
        if (s == 0) {
          dst[d * 2 + s] =
              cv::Mat(1080, 1440, CV_8UC1, buffer->GetDataPointer()).clone();
        } else {
          dst[d * 2 + s] =
              cv::Mat(1080, 1440, CV_16UC1, buffer->GetDataPointer(), 2880)
                  .clone();
        }
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

    for (int d = 0; d < cameras.size(); d++) {
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

    for (int d = 0; d < cameras.size(); d++) {
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

  this->client_tapo_on = this->create_client<std_srvs::srv::Trigger>("tapo/on");
  this->client_tapo_off =
      this->create_client<std_srvs::srv::Trigger>("tapo/off");
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
  for (int i = 0; i < cameras.size(); i++) cameras[i]->closeStream();
}

void JAIHDRNode::tapoTrigger(bool on) {
  auto client = on ? client_tapo_on : client_tapo_off;
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  // 서비스가 준비될 때까지 최대 1초 대기
  if (!client->wait_for_service(std::chrono::seconds(1))) {
    throw std::runtime_error("Service is not available");
  }
  // 비동기적으로 서비스 호출
  auto future_result = client->async_send_request(request);

  // 1초 내에 future의 완료를 체크하기 위한 시작 시각 기록
  auto start_time = std::chrono::steady_clock::now();

  // 이미 Executor에 등록된 상태이므로 spin_until_future_complete 대신 custom
  // spin loop 사용
  while (rclcpp::ok() && future_result.wait_for(std::chrono::seconds(0)) !=
                             std::future_status::ready) {
    // 1초 이상 대기 시 타임아웃 처리
    if (std::chrono::steady_clock::now() - start_time >
        std::chrono::seconds(1)) {
      throw std::runtime_error("Service call timed out");
    }
  }

  // future가 완료되었으므로 응답을 추출
  auto response = future_result.get();
  if (!response || !response->success) {
    throw std::runtime_error(response ? response->message
                                      : "No response received");
  }
}

void JAIHDRNode::collectHdrImages(
    const std::shared_ptr<GoalHandleHDRTrigger> goal_handle) {
  std::vector<cv::Mat> images;
  std::vector<__uint64_t> timestamps;

  for (int t_idx = 0; t_idx < config->HDR_EXPOSURE.size(); t_idx++) {
    int t = config->HDR_EXPOSURE[t_idx];
    int t2 = config->HDR_EXPOSURE_NIR[t_idx % config->HDR_EXPOSURE_NIR.size()];
    camera.configureExposureAll(t, t2);
    camera.openStreamAll();
    camera.flushStream();

    // std::this_thread::sleep_for(
    //     std::chrono::nanoseconds(config->HDR_EXPOSURE_DELAY));

    // camera.openStreamAll();
    camera.triggerFrameCapture(0);
    if (!config->HDR_CAPTURE_SINGLE) camera.triggerFrameCapture(1);
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
    Debug << "Image Read Done";
    if (t_idx + 1 == config->HDR_EXPOSURE_NIR.size()) {
      Debug << "NIR Image Read Done";
      tapoTrigger(false);
    }
    if (t_idx + 1 == config->HDR_EXPOSURE.size()) {
      tapoTrigger(true);
    }
    Debug << "Prepare pushing data";
    for (auto img : imgr) {
      images.push_back(img);
    }
    Debug << "push image done";
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
  int rows = config->HDR_EXPOSURE.size();
  int cols = 4;
  int img_height = 1080;
  int img_width = 1440;
  for (int j = 0; j < cols; ++j) {
    cv::Mat combined_image;
    if (j % 2 == 0) {
      combined_image = cv::Mat::zeros(rows * img_height, img_width, CV_8UC1);
    } else {
      combined_image = cv::Mat::zeros(rows * img_height, img_width, CV_16UC1);
    }

    for (int i = 0; i < rows; ++i) {
      int index = i * cols + j;
      if (index < images.size()) {
        cv::Mat roi =
            combined_image(cv::Rect(0, i * img_height, img_width, img_height));
        images[index].copyTo(roi);
        images[index].release();
      }
    }

    // Save the combined image
    char path[256];
    snprintf(path, 256, "%s/%lld_col%d.png", space_id.c_str(), timestamp[0], j);
    std::string dir_path = space_id;
    std::filesystem::create_directories(dir_path);
    cv::imwrite(path, combined_image);

    combined_image.release();
  }
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
