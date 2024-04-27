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
  streamCallback[source] = callback;
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
  ParamManager::setParam(dualDevice->getDevice(0)->GetParameters(),
                         "ExposureTime", exposure);
}

void MultiSpectralCamera::configureGain(int source, float gain) {
  ParamManager::setParamEnum(dualDevice->getDevice(0)->GetParameters(),
                             "SourceSelector", source);
  ParamManager::setParam(dualDevice->getDevice(0)->GetParameters(), "Gain",
                         gain);
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

void MultiSpectralCamera::interrupt() { flagInterrupted = true; }