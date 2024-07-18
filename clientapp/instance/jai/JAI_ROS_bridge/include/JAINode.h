#pragma once

#include <AppConfig.h>
#include <Camera.h>
#include <HdrFusion.h>
#include <HdrScenario.h>
#include <PvBuffer.h>
#include <wrapper.h>
class JAINode : public rclcpp::Node {
 public:
  JAINode();
  ~JAINode();
  void join_thread();

 private:
  long long systemTimeNano();

  void createPublishers();

  void enlistJAIDevice(int device_num, std::string device_name,
                       int channel_count);

  void createServiceClient();

  void createCameraConfigureService(int device_num, int channel_num);

  void openStream(int camera_num = -1);
  void closeStream(int camera_num = -1);

  bool validateBufferTimestamp(int device_num, int source_num,
                               int64_t buffer_time, int64_t current_time);

  void convertTo16Bit(const uint8_t* src, int width, int height, cv::Mat& dst,
                      int bitDepth, bool isRGB);

  std::vector<std::vector<
      rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> >
      imagePublishers;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr logPublisher;

  std::vector<
      std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> >
      cameraInfoPublishers;

  std::vector<std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> >
      cameraDeviceParamPublishers;

  std::vector<
      std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> >
      cameraDeviceParamSubscribers;

  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr>
      cameraStreamTriggerSubscribers;

  std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr>
      cameraAutoExposureHoldTriggerSubscribers;

  std::function<void(PvBuffer*)> getEbusRosCallback(const int device_num,
                                                    const int source_num);

  void initDevices();

  void initMultispectralCamera(int camera_num, std::string deviceName,
                               std::string macAddress);

  std::vector<MultiSpectralCamera*> cameras;

  void emitRosImageMsg(int device_num, int source_num, PvBuffer* buffer);

  void emitRosDeviceParamMsg(int device_num, int source_num, std::string param);

  ////// HDR Scenario

  bool hdr_capture_mode = true;

  std::vector<std::vector<
      rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> >
      imagePublishers_hdr;
  std::vector<std::vector<
      rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> >
      imagePublishers_fusion;

  HdrScenario hdr_scenario;

  std::vector<std::thread> subscription_thread;

  int64_t timestamp_begin_ros;

  HdrFusion hdr_fusion;
};
