#include <Acquire.h>
#include <Camera.h>
#include <Device.h>
#include <Logger.h>
#include <ParamManager.h>

#include <chrono>

MultiSpectralCamera::MultiSpectralCamera() {
  this->macAddressInit = "";
  initDevice();
}

MultiSpectralCamera::MultiSpectralCamera(std::string deviceName,
                                         std::string macAddress) {
  this->deviceName = deviceName;
  this->macAddressInit = macAddress;
  initDevice();
}

void MultiSpectralCamera::initDevice() {
  streamCallback[0] = nullptr;
  streamCallback[1] = nullptr;
  dualDevice = DeviceManager::getInstance()->connectDualDevice(macAddressInit);
  flagInterrupted = false;
}

void MultiSpectralCamera::initCameraTimestamp() {
  dualDevice->getDevice(0)
      ->GetParameters()
      ->GetInteger("Timestamp")
      ->GetValue(timestamp_begin);
}

void MultiSpectralCamera::addStreamCallback(
    int source, std::function<void(PvBuffer*)> callback) {
  Info << "Registering Callback from " << source;
  streamCallback[source] = callback;
  Info << "Callback Registeration Successful";
}

void MultiSpectralCamera::openStream() {
  auto systemNano = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::time_point_cast<std::chrono::nanoseconds>(
                            std::chrono::high_resolution_clock::now())
                            .time_since_epoch())
                        .count();

  dualDevice->getDevice(0)->GetParameters()->ExecuteCommand("TimestampReset");

  //   __int64_t timestampLatchValue = rand() % 10000000;

  // dualDevice->getDevice(0)->GetParameters()->SetIntegerValue(
  //     "TimestampLatchValue", timestampLatchValue);
  // dualDevice->getDevice(0)->GetParameters()->ExecuteCommand("TimestampLatch");

  // this->timestamp_begin = systemNano - timestampLatchValue;

  this->timestamp_begin = systemNano;
  dualDevice->getDevice(0)->GetParameters()->ExecuteCommand("AcquisitionStart");
}

void MultiSpectralCamera::closeStream() {
  dualDevice->getDevice(0)->GetParameters()->ExecuteCommand("AcquisitionStop");
}

void MultiSpectralCamera::configureExposure(int source, float exposure) {
  // ParamManager::setParamEnum(dualDevice->getDevice(0)->GetParameters(),
  //                            "SourceSelector", source);
  // // double value;
  // // dualDevice->getDevice(0)->GetParameters()->GetFloatValue("ExposureTime",
  // //                                                          value);
  configureSourceRuntime(source, [&](PvGenParameterArray* params) {
    ParamManager::setParam(params, "ExposureTime", exposure);
  });
}

void MultiSpectralCamera::configureGain(int source, float gain) {
  ParamManager::setParamEnum(dualDevice->getDevice(0)->GetParameters(),
                             "SourceSelector", source);
  double value;
  dualDevice->getDevice(0)->GetParameters()->GetFloatValue("Gain", value);

  configureSourceRuntime(source, [&](PvGenParameterArray* params) {
    ParamManager::setParam(params, "Gain", gain + float(value));
  });
}

double MultiSpectralCamera::getExposure(int source) {
  ParamManager::setParamEnum(dualDevice->getDevice(0)->GetParameters(),
                             "SourceSelector", source);
  double value;
  dualDevice->getDevice(0)->GetParameters()->GetFloatValue("ExposureTime",
                                                           value);
  return value;
}

double MultiSpectralCamera::getGain(int source) {
  ParamManager::setParamEnum(dualDevice->getDevice(0)->GetParameters(),
                             "SourceSelector", source);
  double value;
  dualDevice->getDevice(0)->GetParameters()->GetFloatValue("Gain", value);
  return value;
}

void MultiSpectralCamera::configureDevice(int source, std::string command,
                                          std::string type, std::string value) {
  ParamManager::setParamEnum(dualDevice->getDevice(0)->GetParameters(),
                             "SourceSelector", source);
  if (type == "int") {
    configureSourceRuntime(source, [&](PvGenParameterArray* params) {
      ParamManager::setParam(params, command.c_str(), std::stoi(value));
    });
  } else if (type == "float") {
    configureSourceRuntime(source, [&](PvGenParameterArray* params) {
      ParamManager::setParam(params, command.c_str(), std::stof(value));
    });
  } else if (type == "bool") {
    configureSourceRuntime(source, [&](PvGenParameterArray* params) {
      ParamManager::setParamEnum(params, command.c_str(),
                                 value == "True" ? 1 : 0);
    });
  } else if (type == "boolean") {
    configureSourceRuntime(source, [&](PvGenParameterArray* params) {
      ParamManager::setParam(params, command.c_str(), value == "True");
    });
  } else if (type == "enum") {
    configureSourceRuntime(source, [&](PvGenParameterArray* params) {
      ParamManager::setParamEnum(params, command.c_str(), std::stoi(value));
    });
  } else {
    configureSourceRuntime(source, [&](PvGenParameterArray* params) {
      ParamManager::setParam(params, command.c_str(), value.c_str());
    });
  }
}

void MultiSpectralCamera::configureSourceRuntime(
    int source, std::function<void(PvGenParameterArray*)> runBlock) {
  dualDevice->getDevice(0)->GetParameters()->ExecuteCommand("AcquisitionStop");
  ParamManager::setParamEnum(dualDevice->getDevice(0)->GetParameters(),
                             "SourceSelector", source);
  runBlock(dualDevice->getDevice(0)->GetParameters());
  dualDevice->getDevice(0)->GetParameters()->ExecuteCommand("AcquisitionStart");
}

void MultiSpectralCamera::runUntilInterrupted(int streamIndex) {
  while (!flagInterrupted) {
    auto buffer = AcquireManager::getInstance()->AcquireBuffer(
        dualDevice->getStream(streamIndex));
    if (buffer) {
      if (streamCallback[streamIndex]) {
        (streamCallback[streamIndex])(buffer);
      }
      AcquireManager::getInstance()->queueBuffer(
          dualDevice->getStream(streamIndex), buffer);
    }
  }
  flagInterrupted = false;
}

void MultiSpectralCamera::printCameraConfig() {
  ParamManager::setParam(dualDevice->getDevice(0)->GetParameters(),
                         "GevStreamChannelSelector", 0);
  int64_t port;
  dualDevice->getDevice(0)
      ->GetParameters()
      ->GetInteger("GevSCPHostPort")
      ->GetValue(port);
  Info << "Source 0 Port: " << port;
  ParamManager::setParam(dualDevice->getDevice(0)->GetParameters(),
                         "GevStreamChannelSelector", 1);

  dualDevice->getDevice(0)
      ->GetParameters()
      ->GetInteger("GevSCPHostPort")
      ->GetValue(port);
  Info << "Source 1 Port: " << port;

  Info << "Exposure 0: " << getExposure(0);
  Info << "Exposure 1: " << getExposure(1);
  Info << "Gain 0: " << getGain(0);
  Info << "Gain 1: " << getGain(1);
}

void MultiSpectralCamera::interrupt() { flagInterrupted = true; }

std::string MultiSpectralCamera::getParameter(int source,
                                              std::string parameter) {
  ParamManager::setParamEnum(dualDevice->getDevice(0)->GetParameters(),
                             "SourceSelector", source);
  auto value = ParamManager::getParameterAsString(
      dualDevice->getDevice(0)->GetParameters(), parameter.c_str());
  return value;
}

double MultiSpectralCamera::holdAutoExposureAndGetValue(bool hold) {
  auto exposureStatus = ParamManager::getParameterAsString(
      dualDevice->getDevice(0)->GetParameters(), "ExposureAuto");
  for (int i = 0; i < 2; i++) {
    ParamManager::setParamEnum(dualDevice->getDevice(0)->GetParameters(),
                               "SourceSelector", i);
    if (hold) {
      if (exposureStatus == "0") {
        ErrorLog << "Exposure Auto is already off";
        return -1;
      }
      double value;
      dualDevice->getDevice(0)->GetParameters()->GetFloatValue("ExposureTime",
                                                               value);
      ParamManager::setParamEnum(dualDevice->getDevice(0)->GetParameters(),
                                 "ExposureAuto", 0);
      value = value * 0.85;
      ParamManager::setParam(dualDevice->getDevice(0)->GetParameters(),
                             "ExposureTime", float(value));
    } else {
      if (exposureStatus == "2") {
        ErrorLog << "Exposure Auto is already on";
        return -1;
      }

      ParamManager::setParamEnum(dualDevice->getDevice(0)->GetParameters(),
                                 "ExposureAuto", 2);
    }
  }
  return 1;
}