#include <Camera.h>
#include <PvBuffer.h>

#include <functional>
#include <map>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
class JAINode : public rclcpp::Node {
 public:
  JAINode();
  ~JAINode();
  void join_thread();

 private:
  void createPublishers();

  void enlistJAIDevice(int device_num, std::string device_name,
                       int channel_count);

  void createServiceClient();

  void createCameraConfigureService(int device_num, int channel_num);

  void openStream(int camera_num = -1);
  void closeStream(int camera_num = -1);

  std::vector<
      std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> >
      imagePublishers;

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

  std::function<void(PvBuffer*)> getEbusRosCallback(const int device_num,
                                                    const int source_num);

  void initMultispectralCamera(int camera_num);

  std::vector<MultiSpectralCamera*> cameras;

  void emitRosImageMsg(int device_num, int source_num, PvBuffer* buffer);

  void emitRosDeviceParamMsg(int device_num, int source_num);

  std::thread subscription_thread;

  int64_t timestamp_begin_ros;
};
