#include <Camera.h>
#include <PvBuffer.h>

#include <functional>
#include <map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class JAINode : public rclcpp::Node {
 public:
  JAINode();
  ~JAINode();

 private:
  void createPublishers();

  void enlistJAIDevice(int device_num, std::string device_name,
                       int channel_count);

  void openStream(int camera_num = -1);
  void closeStream(int camera_num = -1);

  std::vector<
      std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> >
      imagePublishers;

  std::function<void(PvBuffer*)> getEbusRosCallback(const int device_num,
                                                    const int source_num);

  void initMultispectralCamera(int camera_num);

  std::vector<MultiSpectralCamera*> cameras;

  void emitRosImageMsg(int device_num, int source_num, PvBuffer* buffer);

  std::thread subscription_thread;
};
