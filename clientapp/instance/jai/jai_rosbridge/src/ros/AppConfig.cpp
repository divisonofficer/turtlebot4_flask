#include <AppConfig.h>

#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>

std::shared_ptr<AppConfig> config = std::make_shared<AppConfig>();
void AppConfig::load_from_json(const std::string& file_path) {
  std::ifstream file(file_path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open config file: " + file_path);
  }

  nlohmann::json j;
  file >> j;
  DEVICE_LEFT_NAME = j["DEVICE_LEFT_NAME"];
  DEVICE_RIGHT_NAME = j["DEVICE_RIGHT_NAME"];
  DEVICE_LEFT_ADDRESS = j["DEVICE_LEFT_ADDRESS"];
  DEVICE_RIGHT_ADDRESS = j["DEVICE_RIGHT_ADDRESS"];
  BUFFER_COUNT = j["BUFFER_COUNT"];
  BUFFER_SIZE = j["BUFFER_SIZE"];
  ROS_MSG_BUFFER_SIZE = j["ROS_MSG_BUFFER_SIZE"];
  ROS_SCALE_DOWN = j["ROS_SCALE_DOWN"];
  VIZ_PIXEL_ACQUIRE_FORMAT = j["VIZ_PIXEL_ACQUIRE_FORMAT"];
  NIR_PIXEL_ACQUIRE_FORMAT = j["NIR_PIXEL_ACQUIRE_FORMAT"];
  FRAME_RATE = j["FRAME_RATE"];
  MULTIFRAME_COUNT = j["MULTIFRAME_COUNT"];
  HDR_CAPTURE_MODE = j["HDR_CAPTURE_MODE"];
  HDR_CAPTURE_SINGLE = j["HDR_CAPTURE_SINGLE"];
  STEREO_EXPOSURE_SYNC = j["STEREO_EXPOSURE_SYNC"];
  STEREO_EXPOSURE_SYNC_RATIO = j["STEREO_EXPOSURE_SYNC_RATIO"];
  TRIGGER_SYNC = j["TRIGGER_SYNC"];
  MIN_DIFFS = j["MIN_DIFFS"];
  HDR_EXPOSURE_DELAY = j["HDR_EXPOSURE_DELAY"];
  HDR_EXPOSURE = j["HDR_EXPOSURE"].get<std::vector<double>>();
  HDR_EXPOSURE_NIR = j["HDR_EXPOSURE_NIR"].get<std::vector<double>>();
  NODE_MODE = j["NODE_MODE"];
  STREAM_BUFFER = j["STREAM_BUFFER"];
  if (j.find("HDR_TIMEOUT_CNT") != j.end()) {
    HDR_TIMEOUT_CNT = j["HDR_TIMEOUT_CNT"];
  }
  if (j.find("HDR_PARALLEL_MODE") != j.end()) {
    HDR_PARALLEL_MODE = j["HDR_PARALLEL_MODE"];
  }

  // HDR Burst Capture Performance Optimization Settings
  if (j.find("REQUEST_TIMEOUT") != j.end()) {
    REQUEST_TIMEOUT = j["REQUEST_TIMEOUT"];
  }
  if (j.find("RESEND_DELAY") != j.end()) {
    RESEND_DELAY = j["RESEND_DELAY"];
  }
  if (j.find("TIMESTAMP_TOLERANCE_MS") != j.end()) {
    TIMESTAMP_TOLERANCE_MS = j["TIMESTAMP_TOLERANCE_MS"];
  }
  if (j.find("GEV_SCPD_RGB") != j.end()) {
    GEV_SCPD_RGB = j["GEV_SCPD_RGB"];
  }
  if (j.find("GEV_SCPD_NIR") != j.end()) {
    GEV_SCPD_NIR = j["GEV_SCPD_NIR"];
  }
  if (j.find("RETRY_BASE_DELAY_MS") != j.end()) {
    RETRY_BASE_DELAY_MS = j["RETRY_BASE_DELAY_MS"];
  }
  if (j.find("ENABLE_SEQUENCER_MODE") != j.end()) {
    ENABLE_SEQUENCER_MODE = j["ENABLE_SEQUENCER_MODE"];
  }
  if (j.find("SEQUENCER_LIGHT_ON_COUNT") != j.end()) {
    SEQUENCER_LIGHT_ON_COUNT = j["SEQUENCER_LIGHT_ON_COUNT"];
  }
}
