#include <Camera.h>
#include <HdrFusion.h>
#include <PvBuffer.h>
#include <cv_bridge/cv_bridge.h>

#include <chrono>
#include <functional>
#include <map>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

#define DEVICE_LEFT_NAME "jai_1600_left"
#define DEVICE_RIGHT_NAME "jai_1600_right"
#define DEVICE_LEFT_ADDRESS "00:0c:df:0a:b9:62"
#define DEVICE_RIGHT_ADDRESS "00:0c:df:0a:c6:c8"

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

  std::vector<std::vector<
      rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> >
      imagePublishers_hdr;
  std::vector<std::vector<
      rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> >
      imagePublishers_fusion;

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

  bool hdr_capture_mode = false;
  std::vector<std::vector<std::vector<std::pair<int, uint8_t*> > > >
      hdr_exposure_image;

  int hdr_exposure_idx[2][2] = {{-1, -1}, {-1, -1}};
  int hdr_exposures[4] = {1000, 4000, 16000, 64000};

  void processHdrImage(int device_num, int source_num, PvBuffer* buffer,
                       unsigned long buffer_time);

  std::vector<std::thread> subscription_thread;

  int64_t timestamp_begin_ros;

  HdrFusion hdr_fusion;
};
