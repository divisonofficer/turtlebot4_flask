#ifndef CONFIG_H
#define CONFIG_H
#include <PvPixelType.h>

#include <memory>
#include <vector>

// #define DEVICE_LEFT_NAME "jai_1600_left"
// #define DEVICE_RIGHT_NAME "jai_1600_right"
// #define DEVICE_LEFT_ADDRESS "00:0c:df:0a:b9:62"
// #define DEVICE_RIGHT_ADDRESS "00:0c:df:0a:c6:c8"
// #define BUFFER_COUNT (64)
// #define BUFFER_SIZE (1440 * 1080)
// #define ROS_MSG_BUFFER_SIZE (1440 * 1080)  // half size : 720 540
// #define ROS_SCALE_DOWN false
// #define VIZ_PIXEL_ACQUIRE_FORMAT PvPixelBayerRG8
// #define NIR_PIXEL_ACQUIRE_FORMAT PvPixelMono8

// #define FRAME_RATE 4.0f

// #define MULTIFRAME_COUNT 50
// #define HDR_CAPTURE_MODE true
// #define STEREO_EXPOSURE_SYNC false
// #define TRIGGER_SYNC false
// #define MIN_DIFFS 1
// #define HDR_EXPOSURE_DELAY 1250

class AppConfig {
 public:
  AppConfig() = default;
  std::string DEVICE_LEFT_NAME;
  std::string DEVICE_RIGHT_NAME;
  std::string DEVICE_LEFT_ADDRESS;
  std::string DEVICE_RIGHT_ADDRESS;
  int BUFFER_COUNT;
  int BUFFER_SIZE;
  int ROS_MSG_BUFFER_SIZE;
  bool ROS_SCALE_DOWN;
  PvPixelType VIZ_PIXEL_ACQUIRE_FORMAT;  // 17301513
  PvPixelType NIR_PIXEL_ACQUIRE_FORMAT;  // 17301505
  float FRAME_RATE;
  int MULTIFRAME_COUNT;
  bool HDR_CAPTURE_MODE;
  bool HDR_CAPTURE_SINGLE;
  bool HDR_PARALLEL_MODE;  // 병렬 처리 모드 추가
  bool STEREO_EXPOSURE_SYNC;
  float STEREO_EXPOSURE_SYNC_RATIO;
  bool TRIGGER_SYNC;
  int MIN_DIFFS;
  int HDR_EXPOSURE_DELAY;

  int HDR_TIMEOUT_CNT = 4;
  int NODE_MODE = 0;
  bool STREAM_BUFFER = true;  // buffer list 생성 여부, false면 직접 buffer
                              // 만들어서 queue해줘야됨
  std::vector<double> HDR_EXPOSURE;
  std::vector<double> HDR_EXPOSURE_NIR;

  // HDR Burst Capture Performance Optimization Settings
  int REQUEST_TIMEOUT = 500;       // GigE packet retrieval timeout (ms)
  int RESEND_DELAY = 100;          // Delay before resending lost packets (ms)
  int TIMESTAMP_TOLERANCE_MS = 200;  // Frame acceptance window (ms)
  int GEV_SCPD_RGB = 0;            // Stream Channel Packet Delay for RGB (us)
  int GEV_SCPD_NIR = 5000;         // Stream Channel Packet Delay for NIR (us)
  int RETRY_BASE_DELAY_MS = 50;    // Base delay for retry backoff (ms)
  bool ENABLE_SEQUENCER_MODE = false;  // Enable camera sequencer for HDR
  int SEQUENCER_LIGHT_ON_COUNT = 2;  // Number of exposures with light ON (first N)
  void load_from_json(const std::string& file_path);

 private:                                           // 생성자를 private으로
  AppConfig(const AppConfig&) = delete;             // 복사 금지
  AppConfig& operator=(const AppConfig&) = delete;  // 할당 금지
};

extern std::shared_ptr<AppConfig> config;

#endif  // CONFIG_H