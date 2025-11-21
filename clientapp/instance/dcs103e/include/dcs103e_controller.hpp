#ifndef DCS103E_CONTROLLER_HPP_
#define DCS103E_CONTROLLER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "DCS100.h"
#include "DCS_Info.h"

namespace dcs103e_controller {

class DCS103eController {
 public:
  DCS103eController();
  ~DCS103eController();

  // Initialize connection to DCS device at specified IP
  bool connect(const std::string& ip_address);

  // Disconnect from DCS device
  void disconnect();

  // Check if connected
  bool isConnected() const;

  // Get number of channels
  size_t getChannelCount() const;

  // Channel control functions
  bool enableChannel(int channel, double current = 0.0);
  bool disableChannel(int channel);
  bool setChannelCurrent(int channel, double current);
  bool setChannelMode(int channel, const std::string& mode,
                      double current = 0.0, double pulse_width = 0.0,
                      double pulse_delay = 0.0);

  // Get channel status
  bool isChannelEnabled(int channel) const;
  double getChannelCurrent(int channel) const;
  double getMaxContinuousCurrent(int channel) const;

  // Trigger functions
  bool triggerChannel(int channel);
  bool setTriggerInput(int channel, int trigger_channel);

 private:
  std::unique_ptr<AdvancedIllumination::DCS_100> device_;
  mutable std::mutex device_mutex_;
  bool connected_;
  std::string current_ip_;

  // Validate channel number
  bool isValidChannel(int channel) const;
};

}  // namespace dcs103e_controller

#endif  // DCS103E_CONTROLLER_HPP_
