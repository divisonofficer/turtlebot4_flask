#include "dcs103e_controller.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

namespace dcs103e_controller {

DCS103eController::DCS103eController()
    : device_(std::make_unique<AdvancedIllumination::DCS_100>()),
      connected_(false) {}

DCS103eController::~DCS103eController() { disconnect(); }

bool DCS103eController::connect(const std::string& ip_address) {
  std::lock_guard<std::mutex> lock(device_mutex_);

  try {
    if (connected_) {
      disconnect();
    }

    device_->connect(ip_address);
    connected_ = true;
    current_ip_ = ip_address;

    // Small delay to ensure connection is stable
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    return true;
  } catch (const AdvancedIllumination::DeviceError& e) {
    connected_ = false;
    return false;
  } catch (const std::exception& e) {
    connected_ = false;
    return false;
  }
}

void DCS103eController::disconnect() {
  std::lock_guard<std::mutex> lock(device_mutex_);

  if (connected_ && device_) {
    try {
      // Turn off all channels before disconnecting
      size_t channel_count = device_->ChannelCount();
      for (size_t i = 0; i < channel_count; ++i) {
        device_->operator[](i).current(0.0);
      }

      device_->disconnect();
    } catch (...) {
      // Ignore errors during disconnect
    }
    connected_ = false;
    current_ip_.clear();
  }
}

bool DCS103eController::isConnected() const {
  std::lock_guard<std::mutex> lock(device_mutex_);
  return connected_;
}

size_t DCS103eController::getChannelCount() const {
  std::lock_guard<std::mutex> lock(device_mutex_);
  if (!connected_ || !device_) {
    return 0;
  }

  try {
    return device_->ChannelCount();
  } catch (...) {
    return 0;
  }
}

bool DCS103eController::enableChannel(int channel, double current) {
  std::lock_guard<std::mutex> lock(device_mutex_);

  if (!connected_ || !device_ || !isValidChannel(channel)) {
    return false;
  }

  try {
    auto& ch = device_->operator[](channel);

    // Set to continuous mode if not already set
    ch.mode(AdvancedIllumination::Mode::Continuous);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Set current
    if (current > 0.0) {
      double max_current = ch.maxContinuous();
      if (current > max_current) {
        current = max_current;
      }
      ch.current(current);
    }

    return true;
  } catch (const AdvancedIllumination::DeviceError& e) {
    return false;
  } catch (const std::exception& e) {
    return false;
  }
}

bool DCS103eController::disableChannel(int channel) {
  std::lock_guard<std::mutex> lock(device_mutex_);

  if (!connected_ || !device_ || !isValidChannel(channel)) {
    return false;
  }

  try {
    auto& ch = device_->operator[](channel);
    ch.current(0.0);
    return true;
  } catch (const AdvancedIllumination::DeviceError& e) {
    return false;
  } catch (const std::exception& e) {
    return false;
  }
}

bool DCS103eController::setChannelCurrent(int channel, double current) {
  std::lock_guard<std::mutex> lock(device_mutex_);

  if (!connected_ || !device_ || !isValidChannel(channel)) {
    return false;
  }

  try {
    auto& ch = device_->operator[](channel);

    // Clamp current to max value
    double max_current = ch.maxContinuous();
    if (current > max_current) {
      current = max_current;
    }
    if (current < 0.0) {
      current = 0.0;
    }

    ch.current(current);
    return true;
  } catch (const AdvancedIllumination::DeviceError& e) {
    return false;
  } catch (const std::exception& e) {
    return false;
  }
}

bool DCS103eController::setChannelMode(int channel, const std::string& mode,
                                       double current, double pulse_width,
                                       double pulse_delay) {
  std::lock_guard<std::mutex> lock(device_mutex_);

  if (!connected_ || !device_ || !isValidChannel(channel)) {
    return false;
  }

  try {
    auto& ch = device_->operator[](channel);

    if (mode == "continuous") {
      ch.mode(AdvancedIllumination::Mode::Continuous);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      if (current > 0.0) {
        double max_current = ch.maxContinuous();
        if (current > max_current) {
          current = max_current;
        }
        ch.current(current);
      }
    } else if (mode == "pulsed") {
      ch.mode(AdvancedIllumination::Mode::Pulsed);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));

      if (current > 0.0) {
        ch.current(current);
      }
      if (pulse_width > 0.0) {
        ch.pulseWidth(pulse_width);
      }
      if (pulse_delay > 0.0) {
        ch.pulseDelay(pulse_delay);
      }
    } else {
      return false;  // Invalid mode
    }

    return true;
  } catch (const AdvancedIllumination::DeviceError& e) {
    return false;
  } catch (const std::exception& e) {
    return false;
  }
}

bool DCS103eController::isChannelEnabled(int channel) const {
  std::lock_guard<std::mutex> lock(device_mutex_);

  if (!connected_ || !device_ || !isValidChannel(channel)) {
    return false;
  }

  try {
    auto& ch = device_->operator[](channel);
    return ch.current() > 0.0;
  } catch (...) {
    return false;
  }
}

double DCS103eController::getChannelCurrent(int channel) const {
  std::lock_guard<std::mutex> lock(device_mutex_);

  if (!connected_ || !device_ || !isValidChannel(channel)) {
    return 0.0;
  }

  try {
    auto& ch = device_->operator[](channel);
    return ch.current();
  } catch (...) {
    return 0.0;
  }
}

double DCS103eController::getMaxContinuousCurrent(int channel) const {
  std::lock_guard<std::mutex> lock(device_mutex_);

  if (!connected_ || !device_ || !isValidChannel(channel)) {
    return 0.0;
  }

  try {
    auto& ch = device_->operator[](channel);
    return ch.maxContinuous();
  } catch (...) {
    return 0.0;
  }
}

bool DCS103eController::triggerChannel(int channel) {
  std::lock_guard<std::mutex> lock(device_mutex_);

  if (!connected_ || !device_ || !isValidChannel(channel)) {
    return false;
  }

  try {
    auto& ch = device_->operator[](channel);
    ch.trigger();
    return true;
  } catch (const AdvancedIllumination::DeviceError& e) {
    return false;
  } catch (const std::exception& e) {
    return false;
  }
}

bool DCS103eController::setTriggerInput(int channel, int trigger_channel) {
  std::lock_guard<std::mutex> lock(device_mutex_);

  if (!connected_ || !device_ || !isValidChannel(channel)) {
    return false;
  }

  try {
    auto& ch = device_->operator[](channel);

    AdvancedIllumination::Channel trigger_ch;
    switch (trigger_channel) {
      case 0:
        trigger_ch = AdvancedIllumination::Channel::One;
        break;
      case 1:
        trigger_ch = AdvancedIllumination::Channel::Two;
        break;
      case 2:
        trigger_ch = AdvancedIllumination::Channel::Three;
        break;
      default:
        return false;
    }

    ch.triggerInput(trigger_ch);
    return true;
  } catch (const AdvancedIllumination::DeviceError& e) {
    return false;
  } catch (const std::exception& e) {
    return false;
  }
}

bool DCS103eController::isValidChannel(int channel) const {
  if (!connected_ || !device_) {
    return false;
  }

  try {
    size_t channel_count = device_->ChannelCount();
    return (channel >= 0 && static_cast<size_t>(channel) < channel_count);
  } catch (...) {
    return false;
  }
}

}  // namespace dcs103e_controller
