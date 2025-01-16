#pragma once
#include <Acquire.h>
#include <AppConfig.h>
#include <Camera.h>
#include <wrapper.h>

#include <jai_rosbridge/action/hdr_trigger.hpp>
#include <rclcpp_action/create_server.hpp>
#include <std_srvs/srv/trigger.hpp>

using HDRTrigger = jai_rosbridge::action::HDRTrigger;
using GoalHandleHDRTrigger = rclcpp_action::ServerGoalHandle<HDRTrigger>;

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

  void configureExposureAll(float exposure);

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
  void storeHDRSequence(std::vector<__uint64_t> timestamp,
                        std::vector<cv::Mat> images);
};

struct ThreadArgs {
  int device_idx;
  int stream_idx;
  JAIRGBNIRCamera* cameraObj;
  std::vector<cv::Mat>& dst;
  __uint64_t& timestamp;
  std::atomic<bool>& doneFlag;
  std::mutex& device_mutex;  // 해당 디바이스에 대한 mutex
                             // 기타 필요한 공유 자원 포인터
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
   * ROS2 node initialization
   *
   */
  void initNodeService();

  /**
   *  Acquisition of HDR images
   */

  void collectHdrImages(
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
};