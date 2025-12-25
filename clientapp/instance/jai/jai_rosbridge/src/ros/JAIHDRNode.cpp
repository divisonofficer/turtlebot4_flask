
#include <JAIHDRNode.h>
#include <Logger.h>

#include <algorithm>
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
  // Parallelize exposure configuration across cameras for faster setup
  std::vector<std::thread> threads;
  for (int i = 0; i < cameras.size(); i++) {
    threads.emplace_back([this, i, exposure, nir_exposure]() {
      configureExposure(i, 0, exposure);
      configureExposure(i, 1, nir_exposure);
    });
  }
  for (auto& thread : threads) {
    thread.join();
  }
  ts_exp_ = systemTimeNano();  //+ config->HDR_EXPOSURE_DELAY * 1000000;
}

void JAIRGBNIRCamera::flushStream() {
  // Optimized: No need to flush and reallocate buffers at high FPS
  // Timestamp-based filtering in processStream will handle old frames
  // This eliminates buffer reallocation overhead that causes frame drops

  // Just ensure streams are running - no buffer manipulation needed
  // The timestamp validation (ts_exp_) will reject old exposure frames
  openStreamAll();
}

int JAIRGBNIRCamera::retrieveBuffer(
    const std::shared_ptr<GoalHandleHDRTrigger> goal_handle, PvStream* stream,
    PvBuffer** buffer) {
  PvResult lResult, aResult;
  PvBuffer* lBuffer = nullptr;

  // Calculate timeout: max exposure time + overhead (in ms)
  // Max exposure from config array, converted from μs to ms, plus 150ms safety
  // margin (reduced for shorter exposures)
  float max_exposure_us = *std::max_element(config->HDR_EXPOSURE.begin(),
                                            config->HDR_EXPOSURE.end());
  float max_exposure_nir_us = *std::max_element(
      config->HDR_EXPOSURE_NIR.begin(), config->HDR_EXPOSURE_NIR.end());
  float max_exposure_ms =
      std::max(max_exposure_us, max_exposure_nir_us) / 1000.0f + 150.0f;

  // Ensure minimum timeout based on frame rate, but allow longer for long
  // exposures
  int timeout_ms = std::max(static_cast<int>(max_exposure_ms),
                            static_cast<int>(1000 / config->FRAME_RATE));

  lResult = stream->RetrieveBuffer(&lBuffer, &aResult, timeout_ms);
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
      // Add 100ms tolerance window to accept frames near exposure change
      // boundary Optimized for 8 FPS (125ms per frame, max exposure 100ms)
      const __uint64_t TIMESTAMP_TOLERANCE = 100000000;  // 100ms in nanoseconds
      if (buffer->GetTimestamp() + ts_cam_bs > ts_exp_ - TIMESTAMP_TOLERANCE) {
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

  int max_retries = 3;  // 최대 재시도 횟수
  int retry_count = 0;

  while (retry_count < max_retries) {
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
        // Exponential backoff: 300ms -> 600ms -> 1200ms
        int retry_delay = config->HDR_EXPOSURE_DELAY * (1 << retry_count);
        std::this_thread::sleep_for(
            std::chrono::nanoseconds(retry_delay * 1000000));
        triggerFrameCapture(d);
        all_done = false;
        break;  // 하나라도 실패하면 바로 다음 루프로
      }
    }

    if (all_done) {
      break;
    }

    retry_count++;
  }

  // 타임아웃 체크
  if (retry_count >= max_retries) {
    sendFeedback(goal_handle,
                 "{\"type\":\"warning_timeout\",\"data\":{\"message\":\"Some "
                 "buffers timed out but continuing\"}}");
  }

  timestamp = systemTimeNano();
  return 0;
}

JAIHDRNode::JAIHDRNode() : Node("jai_hdr_node") {
  connectCamera();
  initNodeService();

  // DCS103e 서비스 클라이언트 초기화
  this->client_dcs_connect = this->create_client<std_srvs::srv::Trigger>(
      "/dcs103e_controller/connect");
  this->client_dcs_disconnect = this->create_client<std_srvs::srv::Trigger>(
      "/dcs103e_controller/disconnect");

  this->client_dcs_ch0_enable = this->create_client<std_srvs::srv::Trigger>(
      "/dcs103e_controller/channel_0/enable");
  this->client_dcs_ch0_disable = this->create_client<std_srvs::srv::Trigger>(
      "/dcs103e_controller/channel_0/disable");

  this->client_dcs_ch1_enable = this->create_client<std_srvs::srv::Trigger>(
      "/dcs103e_controller/channel_1/enable");
  this->client_dcs_ch1_disable = this->create_client<std_srvs::srv::Trigger>(
      "/dcs103e_controller/channel_1/disable");

  this->client_dcs_ch2_enable = this->create_client<std_srvs::srv::Trigger>(
      "/dcs103e_controller/channel_2/enable");
  this->client_dcs_ch2_disable = this->create_client<std_srvs::srv::Trigger>(
      "/dcs103e_controller/channel_2/disable");
}

void JAIHDRNode::connectCamera() { camera.connectCamera(); }

bool JAIHDRNode::isImageEmpty(const cv::Mat& img) {
  return img.empty() || img.data == nullptr || img.rows == 0 || img.cols == 0;
}

double JAIHDRNode::calculateNCC(const cv::Mat& img1, const cv::Mat& img2) {
  if (isImageEmpty(img1) || isImageEmpty(img2)) {
    return 0.0;
  }

  if (img1.size() != img2.size() || img1.type() != img2.type()) {
    return 0.0;
  }

  cv::Mat float1, float2;
  img1.convertTo(float1, CV_32F);
  img2.convertTo(float2, CV_32F);

  cv::Scalar mean1 = cv::mean(float1);
  cv::Scalar mean2 = cv::mean(float2);

  float1 -= mean1[0];
  float2 -= mean2[0];

  cv::Mat numerator = float1.mul(float2);
  double num = cv::sum(numerator)[0];

  double denom1 = cv::sum(float1.mul(float1))[0];
  double denom2 = cv::sum(float2.mul(float2))[0];
  double denom = std::sqrt(denom1 * denom2);

  if (denom < 1e-10) {
    return 0.0;
  }

  return num / denom;
}

ValidationDetails JAIHDRNode::validateHDRImages(
    const std::vector<cv::Mat>& images, int exposure_count) {
  ValidationDetails details;
  details.result = ValidationResult::SUCCESS;
  details.ncc_value = 0.0;

  int images_per_exposure = config->HDR_CAPTURE_SINGLE ? 2 : 4;

  // Validate image count
  if (images.size() != exposure_count * images_per_exposure) {
    details.result = ValidationResult::ERROR_ALL_RGB_FAILED;
    details.error_type = "error_invalid_image_count";
    details.error_message = "Unexpected image count";
    return details;
  }

  // Track failed images
  std::vector<int> failed_rgb_indices;
  std::vector<int> failed_nir_indices;

  // Check each exposure's left camera images
  for (int exp_idx = 0; exp_idx < exposure_count; exp_idx++) {
    int rgb_idx = exp_idx * images_per_exposure + 0;  // Device 0, RGB
    int nir_idx = exp_idx * images_per_exposure + 1;  // Device 0, NIR

    if (isImageEmpty(images[rgb_idx])) {
      failed_rgb_indices.push_back(exp_idx);
    }

    if (isImageEmpty(images[nir_idx])) {
      failed_nir_indices.push_back(exp_idx);
    }
  }

  // === VALIDATION RULE 1: Brightest RGB missing ===
  if (failed_rgb_indices.size() == 1 &&
      failed_rgb_indices[0] == exposure_count - 1) {
    details.result = ValidationResult::WARNING_BRIGHTEST_RGB_MISSING;
    details.error_type = "warning_brightest_rgb_missing";
    details.error_message =
        "Brightest RGB image (highest exposure) not captured";
    details.failed_indices = failed_rgb_indices;
    return details;
  }

  // === VALIDATION RULE 2: All RGB failed ===
  if (failed_rgb_indices.size() == exposure_count) {
    details.result = ValidationResult::ERROR_ALL_RGB_FAILED;
    details.error_type = "error_all_rgb_failed";
    details.error_message = "All RGB images failed to capture";
    details.failed_indices = failed_rgb_indices;
    return details;
  }

  // === VALIDATION RULE 3: Some NIR failed (but not all) ===
  if (failed_nir_indices.size() > 0 &&
      failed_nir_indices.size() < exposure_count) {
    details.result = ValidationResult::WARNING_SOME_NIR_FAILED;
    details.error_type = "warning_some_nir_failed";
    details.error_message = "Some NIR images failed to capture";
    details.failed_indices = failed_nir_indices;
    return details;
  }

  // === VALIDATION RULE 4: All NIR failed ===
  if (failed_nir_indices.size() == exposure_count) {
    details.result = ValidationResult::ERROR_ALL_NIR_FAILED;
    details.error_type = "error_all_nir_failed";
    details.error_message = "All NIR images failed to capture";
    details.failed_indices = failed_nir_indices;
    return details;
  }

  // === VALIDATION RULE 5: NIR lighting not working ===
  if (failed_nir_indices.empty()) {
    bool lighting_failed = false;
    double max_ncc = 0.0;

    int nir_count = config->HDR_EXPOSURE_NIR.size();

    if (exposure_count == 2 * nir_count) {
      // Compare first half vs second half
      for (int nir_idx = 0; nir_idx < nir_count; nir_idx++) {
        int first_idx = nir_idx * images_per_exposure + 1;
        int second_idx = (nir_idx + nir_count) * images_per_exposure + 1;

        double ncc = calculateNCC(images[first_idx], images[second_idx]);
        max_ncc = std::max(max_ncc, ncc);

        if (ncc > 0.999) {
          lighting_failed = true;
        }
      }
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "NIR lighting check skipped: exposure_count=%d, nir_count=%d",
                  exposure_count, nir_count);
    }

    details.ncc_value = max_ncc;

    if (lighting_failed) {
      details.result = ValidationResult::ERROR_NIR_LIGHTING_FAILED;
      details.error_type = "error_nir_lighting_failed";
      char msg[256];
      snprintf(msg, 256, "NIR lighting not toggled (NCC=%.6f > 0.999)",
               max_ncc);
      details.error_message = msg;
      return details;
    }
  }

  // All validations passed
  details.result = ValidationResult::SUCCESS;
  details.error_type = "validation_success";
  details.error_message = "All validations passed";
  return details;
}

void JAIHDRNode::sendValidationFeedback(
    const std::shared_ptr<GoalHandleHDRTrigger> goal_handle,
    const ValidationDetails& details) {
  std::string failed_indices_str = "[";
  for (size_t i = 0; i < details.failed_indices.size(); i++) {
    failed_indices_str += std::to_string(details.failed_indices[i]);
    if (i < details.failed_indices.size() - 1) {
      failed_indices_str += ",";
    }
  }
  failed_indices_str += "]";

  sendFeedbackPrintf(
      goal_handle,
      "{\"type\":\"%s\",\"data\":{"
      "\"message\":\"%s\","
      "\"failed_indices\":%s,"
      "\"ncc_value\":%.6f}}",
      details.error_type.c_str(), details.error_message.c_str(),
      failed_indices_str.c_str(), details.ncc_value);
}

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

void JAIHDRNode::dcsChannelControl(int channel, bool enable) {
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;

  // 채널별 클라이언트 선택
  switch (channel) {
    case 0:
      client = enable ? client_dcs_ch0_enable : client_dcs_ch0_disable;
      break;
    case 1:
      client = enable ? client_dcs_ch1_enable : client_dcs_ch1_disable;
      break;
    case 2:
      client = enable ? client_dcs_ch2_enable : client_dcs_ch2_disable;
      break;
    default:
      throw std::runtime_error("Invalid channel number: " +
                               std::to_string(channel));
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

  // 서비스가 준비될 때까지 최대 1초 대기
  if (!client->wait_for_service(std::chrono::seconds(1))) {
    throw std::runtime_error("DCS103e service is not available");
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
      throw std::runtime_error("DCS103e service call timed out");
    }
  }

  // future가 완료되었으므로 응답을 추출
  auto response = future_result.get();
  if (!response || !response->success) {
    throw std::runtime_error(response ? response->message
                                      : "No response received from DCS103e");
  }

  RCLCPP_INFO(this->get_logger(), "DCS103e Channel %d %s", channel,
              enable ? "enabled" : "disabled");
}

void JAIHDRNode::collectHdrImages(
    const std::shared_ptr<GoalHandleHDRTrigger> goal_handle) {
  std::vector<cv::Mat> images;
  std::vector<__uint64_t> timestamps;

  // 미리 조명 상태 설정 (NIR 첫 번째 시퀀스용)
  try {
    dcsChannelControl(0, true);  // 채널 0 켜기
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "Failed to turn on DCS103e Channel 0: %s",
                e.what());
  }

  for (int t_idx = 0; t_idx < config->HDR_EXPOSURE.size(); t_idx++) {
    int t = config->HDR_EXPOSURE[t_idx];
    int t2 = config->HDR_EXPOSURE_NIR[t_idx % config->HDR_EXPOSURE_NIR.size()];

    // 다음 조명 상태를 미리 준비 (비동기)
    bool should_toggle_light = (t_idx + 1 == config->HDR_EXPOSURE_NIR.size());
    std::future<void> light_control_future;

    camera.configureExposureAll(t, t2);
    camera.openStreamAll();
    camera.flushStream();

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

    // 조명 제어를 이미지 읽기와 병렬로 처리
    if (should_toggle_light) {
      light_control_future = std::async(std::launch::async, [this]() {
        Debug << "NIR Image Read Done - Turning off DCS103e Channel 0";
        try {
          dcsChannelControl(0, false);  // 채널 0 끄기
        } catch (const std::exception& e) {
          RCLCPP_WARN(this->get_logger(),
                      "Failed to turn off DCS103e Channel 0: %s", e.what());
        }
      });
    }

    // HDR 촬영 완료 후 채널 0 켜기
    if (t_idx + 1 == config->HDR_EXPOSURE.size()) {
      Debug << "HDR Image sequence completed - Turning on DCS103e Channel 0";
      try {
        dcsChannelControl(0, true);  // 채널 0 켜기
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to turn on DCS103e Channel 0: %s", e.what());
      }
    }
    // 이미지 데이터를 즉시 이동하여 복사 오버헤드 감소 (빈 이미지도 포함)
    Debug << "Prepare pushing data";
    for (auto& img : imgr) {
      images.push_back(std::move(img));  // 빈 이미지도 포함하여 인덱스 유지
    }
    Debug << "push image done";
    timestamps.push_back(timestamp);

    // 조명 제어 완료 대기 (비동기로 시작했던 것)
    if (should_toggle_light && light_control_future.valid()) {
      light_control_future.wait();
    }
  }

  // === VALIDATION: Validate captured images ===
  int exposure_count = config->HDR_EXPOSURE.size();
  ValidationDetails validation = validateHDRImages(images, exposure_count);

  // Send validation feedback
  sendValidationFeedback(goal_handle, validation);

  // Determine if we should store based on validation result
  bool should_store = (validation.result == ValidationResult::SUCCESS);

  if (should_store) {
    // 저장을 별도 스레드에서 비동기로 처리
    std::thread storage_thread([this, goal_handle,
                                timestamps = std::move(timestamps),
                                images = std::move(images)]() mutable {
      storage.storeHDRSequence(goal_handle->get_goal()->space_id, timestamps,
                               images);

      // 이미지 메모리 해제
      for (auto& img : images) {
        img.release();
      }
      images.clear();
    });

    storage_thread.detach();  // 저장이 완료될 때까지 기다리지 않음
  } else {
    // Skip storage, but clean up memory
    for (auto& img : images) {
      img.release();
    }
    images.clear();
    timestamps.clear();
  }

  // Clean up streams (always execute)
  camera.closeStreamAll();
  camera.flushStream();

  // Store validation result for goal result setting
  this->last_validation_result = validation;
}

void JAIHDRNode::collectHdrImagesParallel(
    const std::shared_ptr<GoalHandleHDRTrigger> goal_handle) {
  std::vector<cv::Mat> images;
  std::vector<__uint64_t> timestamps;

  // 미리 조명 상태 설정 (NIR 첫 번째 시퀀스용)
  try {
    dcsChannelControl(0, true);  // 채널 0 켜기
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "Failed to turn on DCS103e Channel 0: %s",
                e.what());
  }

  for (int t_idx = 0; t_idx < config->HDR_EXPOSURE.size(); t_idx++) {
    int t = config->HDR_EXPOSURE[t_idx];
    int t2 = config->HDR_EXPOSURE_NIR[t_idx % config->HDR_EXPOSURE_NIR.size()];

    // 다음 조명 상태를 미리 준비 (비동기)
    bool should_toggle_light = (t_idx + 1 == config->HDR_EXPOSURE_NIR.size());
    std::future<void> light_control_future;

    sendFeedbackPrintf(
        goal_handle,
        "{\"type\" : \"info_collect_hdr_images_parallel\", \"data\" : { "
        "\"exposure\" : %d,"
        "\"exp_idx\" : %d}}",
        t, t_idx);

    // 노출 설정과 스트림 준비를 비동기로 처리
    std::future<void> setup_future =
        std::async(std::launch::async, [this, t, t2]() {
          camera.configureExposureAll(t, t2);
          camera.openStreamAll();
          camera.flushStream();
        });

    // 설정 완료 대기
    setup_future.wait();

    // 캡처 시작
    camera.triggerFrameCapture(0);
    if (!config->HDR_CAPTURE_SINGLE) camera.triggerFrameCapture(1);

    Debug << "Parallel Collect HDR Images for " << t;

    // 이미지 읽기와 조명 제어를 병렬로 처리
    std::future<std::pair<std::vector<cv::Mat>, __uint64_t>> image_future =
        std::async(std::launch::async, [this, goal_handle]() {
          std::vector<cv::Mat> imgr;
          imgr.assign(4, cv::Mat());
          __uint64_t timestamp;
          camera.readImage(goal_handle, imgr, timestamp);
          return std::make_pair(std::move(imgr), timestamp);
        });

    // 조명 제어를 이미지 읽기와 병렬로 처리
    if (should_toggle_light) {
      light_control_future = std::async(std::launch::async, [this]() {
        Debug << "Parallel NIR Image Read Done - Turning off DCS103e Channel 0";
        try {
          dcsChannelControl(0, false);  // 채널 0 끄기
        } catch (const std::exception& e) {
          RCLCPP_WARN(this->get_logger(),
                      "Failed to turn off DCS103e Channel 0: %s", e.what());
        }
      });
    }

    // 이미지 읽기 완료 대기
    auto [imgr, timestamp] = image_future.get();

    Debug << "Parallel Image Read Done";

    // 이미지 데이터 저장 (기존과 동일한 방식 - 빈 이미지도 포함)
    Debug << "Adding images for exposure " << t_idx
          << ": imgr.size()=" << imgr.size();
    for (size_t idx = 0; idx < imgr.size(); ++idx) {
      if (!imgr[idx].empty()) {
        Debug << "  Adding image[" << images.size() << "] from imgr[" << idx
              << "] - size: " << imgr[idx].size();
      } else {
        Debug << "  Adding empty image[" << images.size() << "] from imgr["
              << idx << "]";
      }
      images.push_back(
          std::move(imgr[idx]));  // 빈 이미지도 포함하여 인덱스 유지
    }
    timestamps.push_back(timestamp);

    // 조명 제어 완료 대기 (비동기로 시작했던 것)
    if (should_toggle_light && light_control_future.valid()) {
      light_control_future.wait();
    }

    // HDR 촬영 완료 후 채널 0 켜기
    if (t_idx + 1 == config->HDR_EXPOSURE.size()) {
      Debug << "Parallel HDR sequence completed - Turning on DCS103e Channel 0";
      try {
        dcsChannelControl(0, true);  // 채널 0 켜기
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(),
                    "Failed to turn on DCS103e Channel 0: %s", e.what());
      }
    }
  }

  // === VALIDATION: Validate captured images ===
  int exposure_count = config->HDR_EXPOSURE.size();
  ValidationDetails validation = validateHDRImages(images, exposure_count);

  // Send validation feedback
  sendValidationFeedback(goal_handle, validation);

  // Determine if we should store based on validation result
  bool should_store = (validation.result == ValidationResult::SUCCESS);

  if (should_store) {
    // 저장을 별도 스레드에서 비동기로 처리
    std::thread storage_thread([this, goal_handle,
                                timestamps = std::move(timestamps),
                                images = std::move(images)]() mutable {
      storage.storeHDRSequence(goal_handle->get_goal()->space_id, timestamps,
                               images);

      // 이미지 메모리 해제
      for (auto& img : images) {
        img.release();
      }
      images.clear();
    });

    storage_thread.detach();  // 저장이 완료될 때까지 기다리지 않음
  } else {
    // Skip storage, but clean up memory
    for (auto& img : images) {
      img.release();
    }
    images.clear();
    timestamps.clear();
  }

  // Clean up streams (always execute)
  camera.closeStreamAll();
  camera.flushStream();

  // Store validation result for goal result setting
  this->last_validation_result = validation;
}

HDRStorage::HDRStorage() {}

void HDRStorage::storeHDRSequence(std::string space_id,
                                  std::vector<__uint64_t> timestamp,
                                  std::vector<cv::Mat> images) {
  int rgb_exposure_count = config->HDR_EXPOSURE.size();
  int cols = 4;  // 센서 개수(2) × 카메라 개수(2)
  int img_height = 1080;
  int img_width = 1440;
  for (int j = 0; j < cols; ++j) {
    // 촬영 횟수는 HDR_EXPOSURE.size()로 결정됨 (NIR도 동일 횟수 촬영)
    int rows = rgb_exposure_count;
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
    // 설정에 따라 병렬 또는 순차 처리 선택
    if (config->HDR_PARALLEL_MODE) {
      this->collectHdrImagesParallel(goal_handle);
    } else {
      this->collectHdrImages(goal_handle);
    }

    auto result = std::make_shared<HDRTrigger::Result>();

    // Check for cancellation
    if (this->cancel_flag.load()) {
      result->success = false;
      result->result_message = "HDR Capture Cancelled";
      goal_handle->abort(result);
      hdr_trigger_flag.store(false);
      return;
    }

    // Check validation result
    bool is_error =
        (last_validation_result.result == ValidationResult::ERROR_ALL_RGB_FAILED ||
         last_validation_result.result == ValidationResult::ERROR_ALL_NIR_FAILED ||
         last_validation_result.result ==
             ValidationResult::ERROR_NIR_LIGHTING_FAILED);

    if (is_error) {
      // Error case: Set success=false with detailed message
      result->success = false;
      result->result_message = last_validation_result.error_message;
      goal_handle->succeed(result);  // Still call succeed(), but with success=false
    } else {
      // Success or warning case: Set success=true
      result->success = true;
      if (last_validation_result.result == ValidationResult::SUCCESS) {
        result->result_message = "HDR Capture Finished";
      } else {
        // Warning case: Include warning in message
        result->result_message = "HDR Capture Finished (Warning: " +
                                 last_validation_result.error_message + ")";
      }
      goal_handle->succeed(result);
    }

    hdr_trigger_flag.store(false);
  }).detach();
}
