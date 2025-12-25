#pragma once
#include <Acquire.h>
#include <AppConfig.h>
#include <Camera.h>
#include <wrapper.h>

#include <jai_rosbridge/action/hdr_trigger.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp_action/create_server.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <cmath>

using HDRTrigger = jai_rosbridge::action::HDRTrigger;
using GoalHandleHDRTrigger = rclcpp_action::ServerGoalHandle<HDRTrigger>;

// Validation result enum
enum class ValidationResult {
  SUCCESS,
  WARNING_BRIGHTEST_RGB_MISSING,
  WARNING_SOME_NIR_FAILED,
  ERROR_ALL_RGB_FAILED,
  ERROR_ALL_NIR_FAILED,
  ERROR_NIR_LIGHTING_FAILED
};

// Validation details structure
struct ValidationDetails {
  ValidationResult result;
  std::string error_type;
  std::string error_message;
  std::vector<int> failed_indices;
  double ncc_value;
};

class JAIRGBNIRCamera {
 public:
  JAIRGBNIRCamera();

  /**
   * Camera device connect
   *
   */
  void connectCamera();

  /**
   * Publish configuration
   *
   */
  void flushStream();

  void openStreamAll();

  void openStream(int dn);

  void triggerFrameCapture(int dn);

  void configureExposure(int dn, int sn, float exposure);

  void configureExposureAll(float exposure, float nir_exposure);

  void closeStreamAll();

  int readImage(const std::shared_ptr<GoalHandleHDRTrigger> goal_handle,
                std::vector<cv::Mat>& dst, __uint64_t& timestamp);
  double ts_cam_bs;
  double ts_exp_;
  void processStream(const std::shared_ptr<GoalHandleHDRTrigger> goal_handle,
                     int d, int s, std::vector<cv::Mat>& dst);

 private:
  int retrieveBuffer(const std::shared_ptr<GoalHandleHDRTrigger> goal_handle,
                     PvStream* stream, PvBuffer** buffer);
  std::vector<MultiSpectralCamera*> cameras;
  std::atomic_bool hdr_stream_done_flag[2][2];
  std::atomic_int hdr_stream_buffer_received[2][2];
};

class HDRStorage {
 public:
  HDRStorage();
  void storeHDRSequence(std::string space_id, std::vector<__uint64_t> timestamp,
                        std::vector<cv::Mat> images);
};

class JAIHDRNode : public rclcpp::Node {
 public:
  JAIHDRNode();
  /**
   * Camera device connect
   *
   */
  void connectCamera();

  /**
   * DCS103e 채널 제어
   * @param channel 채널 번호 (0, 1, 2)
   * @param enable true: 켜기, false: 끄기
   */
  void dcsChannelControl(int channel, bool enable);

  /**
   * ROS2 node initialization
   *
   */
  void initNodeService();

  /**
   *  Acquisition of HDR images
   */

  void collectHdrImages(
      const std::shared_ptr<GoalHandleHDRTrigger> goal_handle);

  /**
   *  Fast parallel HDR image acquisition
   */
  void collectHdrImagesParallel(
      const std::shared_ptr<GoalHandleHDRTrigger> goal_handle);

  void collectHdrImagesFor(int dn, int sn);
  void cancel_action() { cancel_flag.store(true); }

 private:
  JAIRGBNIRCamera camera;
  HDRStorage storage;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_hdr_trigger;
  rclcpp_action::Server<HDRTrigger>::SharedPtr action_server_hdr_trigger;
  rclcpp_action::GoalResponse action_hdr_trigger_handler(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const HDRTrigger::Goal> goal);
  rclcpp_action::CancelResponse action_hdr_trigger_cancel(
      const std::shared_ptr<GoalHandleHDRTrigger> goal_handle);
  void action_hdr_trigger_accepted(
      const std::shared_ptr<GoalHandleHDRTrigger> goal_handle);

  std::atomic_bool cancel_flag;
  std::atomic_bool hdr_trigger_flag;

  // DCS103e 조명 제어 클라이언트
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_dcs_connect,
      client_dcs_disconnect;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_dcs_ch0_enable,
      client_dcs_ch0_disable;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_dcs_ch1_enable,
      client_dcs_ch1_disable;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_dcs_ch2_enable,
      client_dcs_ch2_disable;

  // Image validation functions
  ValidationDetails validateHDRImages(const std::vector<cv::Mat>& images,
                                      int exposure_count);
  bool isImageEmpty(const cv::Mat& img);
  double calculateNCC(const cv::Mat& img1, const cv::Mat& img2);
  void sendValidationFeedback(
      const std::shared_ptr<GoalHandleHDRTrigger> goal_handle,
      const ValidationDetails& details);

  ValidationDetails last_validation_result;  // Store last validation result
};