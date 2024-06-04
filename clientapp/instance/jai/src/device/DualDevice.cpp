#include <Device.h>
#include <DualDevice.h>
#include <ParamManager.h>
#include <Stream.h>

DualDevice::DualDevice(PvString &connection_ID) {
  rgb_device = static_cast<PvDeviceGEV *>(
      DeviceManager::getInstance()->DeviceConnectToDevice(connection_ID));

  auto streamManager = StreamManager::getInstance();
  rgb_stream =
      static_cast<PvStreamGEV *>(streamManager->OpenStream(connection_ID));
  // nir_stream = rgb_stream;
  nir_stream =
      static_cast<PvStreamGEV *>(streamManager->OpenStream(connection_ID));
  StreamManager::getInstance()->ConfigureStream(rgb_device, rgb_stream, 0);
  StreamManager::getInstance()->ConfigureStream(rgb_device, nir_stream, 1);

  ParamManager::setParam(rgb_device->GetParameters(), "AcquisitionFrameRate",
                         4.0f);

  ParamManager::setParamEnum(rgb_device->GetParameters(), "SourceSelector", 0);
  ParamManager::setParam(rgb_device->GetParameters(), "ExposureTime", 14000.0f);
  ParamManager::setParam(rgb_device->GetParameters(), "Gamma", 1.0f);
  ParamManager::setParam(rgb_device->GetParameters(), "Gain", 7.0f);
  ParamManager::setParam(rgb_device->GetParameters(), "ALCReference", 30);
  ParamManager::setParamEnum(rgb_device->GetParameters(), "BalanceWhiteAuto",
                             2);
  ParamManager::setParamEnum(rgb_device->GetParameters(), "PixelFormat",
                             PvPixelBayerRG10);

  ParamManager::setParamEnum(rgb_device->GetParameters(), "SourceSelector", 1);
  ParamManager::setParam(rgb_device->GetParameters(), "ExposureTime", 80000.0f);
  ParamManager::setParam(rgb_device->GetParameters(), "ALCReference", 30);
  ParamManager::setParam(rgb_device->GetParameters(), "Gamma", 1.0f);
  ParamManager::setParam(rgb_device->GetParameters(), "Gain", 7.0f);
  ParamManager::setParamEnum(rgb_device->GetParameters(), "BalanceWhiteAuto",
                             2);
  ParamManager::setParamEnum(rgb_device->GetParameters(), "PixelFormat",
                             PvPixelMono10);
  ParamManager::setParamEnum(rgb_device->GetParameters(), "AcquisitionSyncMode",
                             1);

  /**
   *
   * Packet Dealy in Microseconds (not miliseconds!)
   */

  ParamManager::setParam(rgb_device->GetParameters(),
                         "GevStreamChannelSelector", 0);

  ParamManager::setParam(rgb_device->GetParameters(), "GevSCPD",
                         rand() % 2 + 1000);

  ParamManager::setParam(rgb_device->GetParameters(),
                         "GevStreamChannelSelector", 1);

  ParamManager::setParam(rgb_device->GetParameters(), "GevSCPD",
                         rand() % 10 * 5000 + 30000);

  streamManager->CreateStreamBuffers(rgb_device, rgb_stream, &rgb_buffer_list);
  streamManager->CreateStreamBuffers(rgb_device, nir_stream, &nir_buffer_list);
}

PvDevice *DualDevice::getDevice(int source) { return rgb_device; }

PvStream *DualDevice::getStream(int source) {
  if (source == 0) {
    return rgb_stream;
  } else {
    return nir_stream;
  }
}

std::vector<PvBuffer *> *DualDevice::getBufferList(int source) {
  if (source == 0) {
    return &rgb_buffer_list;
  } else {
    return &nir_buffer_list;
  }
}