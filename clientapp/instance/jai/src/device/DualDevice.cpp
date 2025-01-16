#include <AppConfig.h>
#include <Device.h>
#include <DualDevice.h>
#include <Logger.h>
#include <ParamManager.h>
#include <Stream.h>

DualDevice::DualDevice(PvString &connection_ID) {
  rgb_device = static_cast<PvDeviceGEV *>(
      DeviceManager::getInstance()->DeviceConnectToDevice(connection_ID));

  auto streamManager = StreamManager::getInstance();
  rgb_stream =
      static_cast<PvStreamGEV *>(streamManager->OpenStream(connection_ID));
  nir_stream =
      static_cast<PvStreamGEV *>(streamManager->OpenStream(connection_ID));
  StreamManager::getInstance()->ConfigureStream(rgb_device, rgb_stream, 0);
  StreamManager::getInstance()->ConfigureStream(rgb_device, nir_stream, 1);

  rgb_device->GetParameters()->ExecuteCommand("AcquisitionStop");

  if (config->HDR_CAPTURE_MODE) {
    ParamManager::setParamEnum(rgb_device->GetParameters(), "AcquisitionMode",
                               2);
  } else {
    if (config->TRIGGER_SYNC) {
      ParamManager::setParamEnum(rgb_device->GetParameters(), "AcquisitionMode",
                                 2);
      ParamManager::setParam(rgb_device->GetParameters(),
                             "AcquisitionFrameCount", config->MULTIFRAME_COUNT);
    } else {
      ParamManager::setParamEnum(rgb_device->GetParameters(), "AcquisitionMode",
                                 2);
    }
  }

  ParamManager::setParamEnum(rgb_device->GetParameters(),
                             "PulseGeneratorSelector", 0);
  ParamManager::setParam(rgb_device->GetParameters(), "PulseGeneratorFrequency",
                         config->FRAME_RATE);
  ParamManager::setParamEnum(rgb_device->GetParameters(), "TriggerSelector", 3);

  ParamManager::setParamEnum(rgb_device->GetParameters(), "TriggerSource", 19);
  ParamManager::setParamEnum(rgb_device->GetParameters(), "TriggerMode", 1);
  // ParamManager::setParamEnum(rgb_device->GetParameters(), "TriggerSource",
  // 7);
  ParamManager::setParamEnum(rgb_device->GetParameters(),
                             "PulseGeneratorClearSource", 1);
  Info << "Setting Channel 0 Parameters";

  ParamManager::setParamEnum(rgb_device->GetParameters(), "SourceSelector", 0);
  ParamManager::setParamEnum(rgb_device->GetParameters(), "ExposureAuto", 0);
  ParamManager::setParam(rgb_device->GetParameters(), "ExposureAutoControlMax",
                         40000.0f);
  ParamManager::setParam(rgb_device->GetParameters(), "ExposureAutoControlMin",
                         100.0f);
  ParamManager::setParam(rgb_device->GetParameters(), "Gamma", 0.45);
  ParamManager::setParam(rgb_device->GetParameters(), "Gain", 1.0f);
  ParamManager::setParamEnum(rgb_device->GetParameters(), "LUTMode", 0);

  ParamManager::setParam(rgb_device->GetParameters(), "GainAutoControlMax",
                         10.0f);
  ParamManager::setParam(rgb_device->GetParameters(), "ALCReference", 50);
  // ParamManager::setParam(rgb_device->GetParameters(), "ALCReference", 30);
  // ParamManager::setParamEnum(rgb_device->GetParameters(), "BalanceWhiteAuto",
  //                            2);
  ParamManager::setParamEnum(rgb_device->GetParameters(), "PixelFormat",
                             config->VIZ_PIXEL_ACQUIRE_FORMAT);
  // ParamManager::setParamEnum(rgb_device->GetParameters(), "PixelFormat",
  //                            PvPixelBayerRG10);

  Info << "Setting Channel 1 Parameters";

  ParamManager::setParamEnum(rgb_device->GetParameters(), "SourceSelector", 1);
  ParamManager::setParamEnum(rgb_device->GetParameters(), "ExposureAuto", 0);
  ParamManager::setParam(rgb_device->GetParameters(), "ExposureAutoControlMax",
                         40000.0f);
  ParamManager::setParam(rgb_device->GetParameters(), "ExposureAutoControlMin",
                         1.0f);

  ParamManager::setParam(rgb_device->GetParameters(), "BinningHorizontal",
                         config->ROS_SCALE_DOWN ? 2 : 1);
  ParamManager::setParam(rgb_device->GetParameters(), "BinningVertical",
                         config->ROS_SCALE_DOWN ? 2 : 1);
  ParamManager::setParam(rgb_device->GetParameters(), "ALCReference", 50);
  ParamManager::setParam(rgb_device->GetParameters(), "Gamma", 0.45);
  ParamManager::setParam(rgb_device->GetParameters(), "Gain", 1.0f);
  ParamManager::setParamEnum(rgb_device->GetParameters(), "LUTMode", 0);
  ParamManager::setParam(rgb_device->GetParameters(), "GainAutoControlMax",
                         10.0f);
  // ParamManager::setParamEnum(rgb_device->GetParameters(), "PixelFormat",
  //                            PvPixelMono10);
  ParamManager::setParamEnum(rgb_device->GetParameters(), "PixelFormat",
                             config->NIR_PIXEL_ACQUIRE_FORMAT);
  ParamManager::setParamEnum(rgb_device->GetParameters(), "AcquisitionSyncMode",
                             1);

  /**
   *
   * Packet Dealy in Microseconds (not miliseconds!)
   */
  Info << "Setting GevStream Parameters";
  ParamManager::setParam(rgb_device->GetParameters(),
                         "GevStreamChannelSelector", 0);

  ParamManager::setParam(rgb_device->GetParameters(), "GevSCPD",
                         (rand() % 1000));

  ParamManager::setParam(rgb_device->GetParameters(),
                         "GevStreamChannelSelector", 1);

  ParamManager::setParam(rgb_device->GetParameters(), "GevSCPD",
                         (rand() % 10 + 1) * 20000);

  if (config->STREAM_BUFFER) {
    streamManager->CreateStreamBuffers(rgb_device, rgb_stream,
                                       &rgb_buffer_list);
    streamManager->CreateStreamBuffers(rgb_device, nir_stream,
                                       &nir_buffer_list);
  }

  rgb_device->GetParameters()->ExecuteCommand("TimestampReset");

  ParamManager::setParam(rgb_device->GetParameters(), "InterPacketGap", 100);
  ParamManager::setParam(rgb_device->GetParameters(),
                         "NetworkThroughputSafetyMargin", 50);
  ParamManager::setParamEnum(rgb_device->GetParameters(),
                             "MultiStreamPacketCollisionAvoidMode", 1);
}

// PvBuffer *DualDevice::popAndCreateNewBuffer(int source) {
//   auto buffer_vector = source == 0 ? rgb_buffer_list : nir_buffer_list;
//   auto buffer = buffer_vector.back();
//   buffer->Free();
//   buffer->Alloc(static_cast<uint32_t>(config->BUFFER_SIZE));
//   return buffer;
// }

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