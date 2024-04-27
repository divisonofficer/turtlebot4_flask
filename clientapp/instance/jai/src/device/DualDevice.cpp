#include <Device.h>
#include <DualDevice.h>
#include <Stream.h>

DualDevice::DualDevice(PvString &connection_ID) {
  rgb_device = static_cast<PvDeviceGEV *>(
      DeviceManager::getInstance()->DeviceConnectToDevice(connection_ID));
  nir_device = rgb_device;

  auto streamManager = StreamManager::getInstance();
  rgb_stream =
      static_cast<PvStreamGEV *>(streamManager->OpenStream(connection_ID));
  // nir_stream = rgb_stream;
  nir_stream =
      static_cast<PvStreamGEV *>(streamManager->OpenStream(connection_ID));

  rgb_buffer_list.reserve(128);
  nir_buffer_list.reserve(128);
  StreamManager::getInstance()->ConfigureStream(nir_device, nir_stream, 1);
  StreamManager::getInstance()->ConfigureStream(rgb_device, rgb_stream, 0);

  streamManager->CreateStreamBuffers(rgb_device, rgb_stream, &rgb_buffer_list);
  streamManager->CreateStreamBuffers(nir_device, nir_stream, &nir_buffer_list);
}

PvDevice *DualDevice::getDevice(int source) {
  if (source == 0) {
    return rgb_device;
  } else {
    return nir_device;
  }
}

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
    return &rgb_buffer_list;
  }
}