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
  ParamManager::setParamEnum(rgb_device->GetParameters(), "TriggerMode", 1);
  if (config->TRIGGER_SYNC) {
    ParamManager::setParamEnum(rgb_device->GetParameters(), "TriggerSource", 7);
  } else {
    ParamManager::setParamEnum(rgb_device->GetParameters(), "TriggerSource",
                               19);
  }

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
  if (config->HDR_CAPTURE_MODE) {
    ParamManager::setParamEnum(rgb_device->GetParameters(), "PixelFormat",
                               PvPixelMono12);
  } else {
    ParamManager::setParamEnum(rgb_device->GetParameters(), "PixelFormat",
                               config->NIR_PIXEL_ACQUIRE_FORMAT);
  }
  ParamManager::setParamEnum(rgb_device->GetParameters(), "AcquisitionSyncMode",
                             1);

  /**
   * Packet Delay in Microseconds (not milliseconds!)
   * GevSCPD: Stream Channel Packet Delay
   * Using config values instead of random for consistent HDR burst performance
   */
  Info << "Setting GevStream Parameters";
  ParamManager::setParam(rgb_device->GetParameters(),
                         "GevStreamChannelSelector", 0);
  ParamManager::setParam(rgb_device->GetParameters(), "GevSCPD",
                         config->GEV_SCPD_RGB);  // 0 for minimal delay

  ParamManager::setParam(rgb_device->GetParameters(),
                         "GevStreamChannelSelector", 1);
  ParamManager::setParam(rgb_device->GetParameters(), "GevSCPD",
                         config->GEV_SCPD_NIR);  // Small delay to avoid collision

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

  // Configure Sequencer Mode if enabled
  configureSequencer();
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

void DualDevice::configureSequencer() {
  if (!config->ENABLE_SEQUENCER_MODE) {
    return;
  }

  auto params = rgb_device->GetParameters();
  Info << "Configuring Sequencer Mode for HDR burst capture (2-burst mode)";

  // 1. Disable sequencer and enter configuration mode
  ParamManager::setParamEnum(params, "SequencerMode", 0);              // Off
  ParamManager::setParamEnum(params, "SequencerConfigurationMode", 1); // On

  // 2. Enable ExposureTime feature for sequencer control
  ParamManager::setParamEnum(params, "SequencerFeatureSelector", 0); // ExposureTime
  ParamManager::setParam(params, "SequencerFeatureEnable", true);

  // 3. Configure sequencer sets for 2-burst HDR capture
  // Burst 1 (Light ON):  Set 0,1 -> RGB[0,1], NIR[0,1]
  // Burst 2 (Light OFF): Set 2,3 -> RGB[2,3], NIR[0,1]
  int numSets = std::min(static_cast<int>(config->HDR_EXPOSURE.size()), 8);
  int lightOnCount = config->SEQUENCER_LIGHT_ON_COUNT;

  for (int i = 0; i < numSets; i++) {
    ParamManager::setParam(params, "SequencerSetSelector", i);

    // Configure RGB channel (Source 0) exposure
    ParamManager::setParamEnum(params, "SourceSelector", 0);
    ParamManager::setParam(params, "ExposureTime",
                           static_cast<float>(config->HDR_EXPOSURE[i]));

    // Configure NIR channel (Source 1) exposure
    // NIR cycles through its array within each burst
    ParamManager::setParamEnum(params, "SourceSelector", 1);
    int nirIdx = i % static_cast<int>(config->HDR_EXPOSURE_NIR.size());
    ParamManager::setParam(params, "ExposureTime",
                           static_cast<float>(config->HDR_EXPOSURE_NIR[nirIdx]));

    // Set path to next set within the same burst
    // Burst 1: 0->1->0 (stop at 1, controlled by AcquisitionFrameCount)
    // Burst 2: 2->3->2 (stop at 3, controlled by AcquisitionFrameCount)
    ParamManager::setParam(params, "SequencerPathSelector", 0);
    if (i < lightOnCount - 1) {
      // Within burst 1: point to next set
      ParamManager::setParam(params, "SequencerSetNext", i + 1);
    } else if (i == lightOnCount - 1) {
      // End of burst 1: loop back (will be stopped by AcquisitionFrameCount)
      ParamManager::setParam(params, "SequencerSetNext", 0);
    } else if (i < numSets - 1) {
      // Within burst 2: point to next set
      ParamManager::setParam(params, "SequencerSetNext", i + 1);
    } else {
      // End of burst 2: loop back to burst 2 start
      ParamManager::setParam(params, "SequencerSetNext", lightOnCount);
    }

    // Trigger on FrameStart
    ParamManager::setParamEnum(params, "SequencerTriggerSource", 0); // FrameStart

    // Save this set configuration
    params->ExecuteCommand("SequencerSetSave");

    const char* burstLabel = (i < lightOnCount) ? "LIGHT_ON" : "LIGHT_OFF";
    Info << "Sequencer Set " << i << " [" << burstLabel << "]: RGB="
         << config->HDR_EXPOSURE[i] << "us, NIR="
         << config->HDR_EXPOSURE_NIR[nirIdx] << "us";
  }

  // 4. Set start set to 0 (burst 1) and enable sequencer
  // Note: SequencerSetStart will be changed dynamically before each burst
  ParamManager::setParam(params, "SequencerSetStart", 0);
  ParamManager::setParamEnum(params, "SequencerConfigurationMode", 0); // Off
  ParamManager::setParamEnum(params, "SequencerMode", 1);              // On

  Info << "Sequencer Mode enabled: " << lightOnCount << " sets for LIGHT_ON, "
       << (numSets - lightOnCount) << " sets for LIGHT_OFF";
}