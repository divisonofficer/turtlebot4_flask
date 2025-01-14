#pragma once
#include <HdrFusion.h>
#include <Logger.h>
#include <PvBuffer.h>
#include <wrapper.h>
class HdrScenario {
 public:
  HdrScenario();

  void processHdrImage(int dn, int sn, PvBuffer* buffer,
                       unsigned long buffer_time);

  std::function<void(sensor_msgs::msg::CompressedImage)>
      publishFusionImageCallback[2][2];
  std::function<void(sensor_msgs::msg::CompressedImage)>
      publishHdrImageCallback[2][2];

  std::function<float()> getCameraExposureCallback[2][2];
  std::function<void(float)> setCameraExposureCallback[2][2];

 private:
  bool validateImageIntensities(int dn, int sn);
  bool validateImageExposure(int dn, int sn, cv::Mat& img);

  void configureExposure(int device, int source, float exposure);
  float getCameraExposure(int device, int source);
  void clearImageVector(int dn, int sn);
  void publishFusionImage(int dn, int sn, cv::Mat& fusion,
                          unsigned long buffer_time);
  void publishHdrImage(int dn, int sn, cv::Mat& hdr, unsigned long buffer_time);
  uint64_t systemTimeNano();

  std::vector<std::vector<std::vector<std::pair<int, cv::Mat>>>>
      hdr_exposure_image;
  HdrFusion hdr_fusion;

  int hdr_exposure_idx[2];
  int hdr_exposure_count[2][2];
  std::vector<double> hdr_exp;
  uint64_t hdr_config_timestamp[2][2];
  float hdr_intensity_history[2][2];
  int hdr_intensity_error_count[2][2];
};
