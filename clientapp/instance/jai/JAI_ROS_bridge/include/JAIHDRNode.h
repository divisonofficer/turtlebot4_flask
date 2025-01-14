#pragma once
#include <Acquire.h>
#include <AppConfig.h>
#include <Camera.h>
#include <wrapper.h>

#include <std_srvs/srv/trigger.hpp>

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

  void configureExposure(int dn, int sn, float exposure);

  void configureExposureAll(float exposure);

  int readImage(int dn, cv::Mat& dst, cv::Mat& dst_2, __uint64_t& timestamp);
  double ts_cam_bs;
  double ts_exp_;

 private:
  std::vector<MultiSpectralCamera*> cameras;
};

class HDRStorage {
 public:
  HDRStorage();
  void storeHDRSequence(std::vector<__uint64_t> timestamp,
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
   * ROS2 node initialization
   *
   */
  void initNodeService();

  /**
   *  Acquisition of HDR images
   */

  void collectHdrImages();

  void collectHdrImagesFor(int dn, int sn);

 private:
  JAIRGBNIRCamera camera;
  HDRStorage storage;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_hdr_trigger;
};