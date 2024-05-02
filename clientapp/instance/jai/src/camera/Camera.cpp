#include <Acquire.h>
#include <Camera.h>
#include <Device.h>
#include <Logger.h>
#include <ParamManager.h>

MultiSpectralCamera::MultiSpectralCamera() {
  streamCallback[0] = nullptr;
  streamCallback[1] = nullptr;
  dualDevice = DeviceManager::getInstance()->connectDualDevice();
  flagInterrupted = false;
}

void MultiSpectralCamera::addStreamCallback(
    int source, std::function<void(PvBuffer*)> callback) {
  Info << "Registering Callback from " << source;
  streamCallback[source] = callback;
  Info << "Callback Registeration Successful";
}

void MultiSpectralCamera::openStream() {
  dualDevice->getDevice(0)->GetParameters()->ExecuteCommand("AcquisitionStart");
}

void MultiSpectralCamera::closeStream() {
  dualDevice->getDevice(0)->GetParameters()->ExecuteCommand("AcquisitionStop");
}

void MultiSpectralCamera::configureExposure(int source, float exposure) {
  ParamManager::setParamEnum(dualDevice->getDevice(0)->GetParameters(),
                             "SourceSelector", source);
  double value;
  dualDevice->getDevice(0)->GetParameters()->GetFloatValue("ExposureTime",
                                                           value);
  configureSourceRuntime(source, [&](PvGenParameterArray* params) {
    ParamManager::setParam(params, "ExposureTime", exposure + (float)value);
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

void MultiSpectralCamera::configureSourceRuntime(
    int source, std::function<void(PvGenParameterArray*)> runBlock) {
  dualDevice->getDevice(0)->GetParameters()->ExecuteCommand("AcquisitionStop");
  ParamManager::setParamEnum(dualDevice->getDevice(0)->GetParameters(),
                             "SourceSelector", source);
  runBlock(dualDevice->getDevice(0)->GetParameters());
  dualDevice->getDevice(0)->GetParameters()->ExecuteCommand("AcquisitionStart");
}

void MultiSpectralCamera::runUntilInterrupted() {
  while (!flagInterrupted) {
    for (int i = 0; i < 2; i++) {
      auto buffer = AcquireManager::getInstance()->AcquireBuffer(
          dualDevice->getStream(i));
      if (buffer) {
        if (streamCallback[i]) {
          (streamCallback[i])(buffer);
        }
        AcquireManager::getInstance()->queueBuffer(dualDevice->getStream(i),
                                                   buffer);
      }
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