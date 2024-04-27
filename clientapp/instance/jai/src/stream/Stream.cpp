#include <Logger.h>
#include <ParamManager.h>
#include <PvBuffer.h>
#include <PvDeviceGEV.h>
#include <PvStreamGEV.h>
#include <Stream.h>

#include <vector>

PvStream* StreamManager::OpenStream(const PvString& aConnectionID) {
  PvStream* lStream;
  PvResult lResult;

  // Open stream to the GigE Vision or USB3 Vision device
  Debug << "Opening stream from device."
        << "\n";
  lStream = PvStream::CreateAndOpen(aConnectionID, &lResult);
  if (lStream == NULL) {
    Debug << "Unable to stream from device. "
          << lResult.GetCodeString().GetAscii() << " ("
          << lResult.GetDescription().GetAscii() << ")"
          << "\n";
  }

  return lStream;
}

void StreamManager::ConfigureStream(PvDevice* aDevice, PvStream* aStream,
                                    u_int32_t channel = 0) {
  // If this is a GigE Vision device, configure GigE Vision specific streaming
  // parameters
  PvDeviceGEV* lDeviceGEV = dynamic_cast<PvDeviceGEV*>(aDevice);
  if (lDeviceGEV != NULL) {
    PvStreamGEV* lStreamGEV = static_cast<PvStreamGEV*>(aStream);

    // Negotiate packet size
    lDeviceGEV->NegotiatePacketSize();

    // Configure device streaming destination

    Debug << "Setting Device Stream Destination"
          << "\n"
          << "IP: " << lStreamGEV->GetLocalIPAddress().GetAscii() << "\n"
          << "Port: " << lStreamGEV->GetLocalPort() << "\n"
          << "Channel: " << channel << "\n";

    auto localIpAddress = lStreamGEV->GetLocalIPAddress().GetAscii();
    int64_t localIpDecimal = 0;
    int64_t localIpDecimalParts = 0;
    for (; *localIpAddress != '\0'; localIpAddress++) {
      if (*localIpAddress != '.') {
        localIpDecimalParts =
            localIpDecimalParts * 10 + (*localIpAddress - '0');
      }

      if (*localIpAddress == '.' || *(localIpAddress + 1) == '\0') {
        localIpDecimal = (localIpDecimal << 8) + localIpDecimalParts;
        localIpDecimalParts = 0;
      }
    }

    if (channel == 1) {
      ParamManager::setParam(lDeviceGEV->GetParameters(),
                             "GevStreamChannelSelector", channel);
      ParamManager::setParam(lDeviceGEV->GetParameters(), "GevSCDA",
                             localIpDecimal);
      ParamManager::setParam(lDeviceGEV->GetParameters(), "GevSCPHostPort",
                             lStreamGEV->GetLocalPort());

    }

    else
      lDeviceGEV->SetStreamDestination(lStreamGEV->GetLocalIPAddress(),
                                       lStreamGEV->GetLocalPort(), channel);

    ParamManager::setParamEnum(lDeviceGEV->GetParameters(), "SourceSelector",
                               channel);
    ParamManager::setParamEnum(lDeviceGEV->GetParameters(), "ExposureAuto",
                               true);
    ParamManager::setParam(lDeviceGEV->GetParameters(),
                           "ExposureAutoControlMax", 150000.0f);
    ParamManager::setParam(lDeviceGEV->GetParameters(), "GainAutoControlMax",
                           15.0f);
    ParamManager::setParamEnum(lDeviceGEV->GetParameters(), "GainAuto", true);

    // int param_count = lDeviceGEV->GetParameters()->GetCount();
    // for (int i = 0; i < param_count; i++) {
    //   PvGenParameter* param = lDeviceGEV->GetParameters()->Get(i);
    //   if (param->IsAvailable()) {
    //     Debug << "Parameter: " << param->GetName().GetAscii() << " "
    //           << param->GetUpdatesEnabled() << "\n";
    //   }
    // }
  }
}

void StreamManager::CreateStreamBuffers(PvDevice* aDevice, PvStream* aStream,
                                        std::vector<PvBuffer*>* aBufferList) {
  // Reading payload size from device
  uint32_t lSize = aDevice->GetPayloadSize();

  // Use BUFFER_COUNT or the maximum number of buffers, whichever is smaller
  uint32_t lBufferCount = (aStream->GetQueuedBufferMaximum() < BUFFER_COUNT)
                              ? aStream->GetQueuedBufferMaximum()
                              : BUFFER_COUNT;

  // Allocate buffers
  for (uint32_t i = 0; i < lBufferCount; i++) {
    // Create new buffer object
    PvBuffer* lBuffer = new PvBuffer;

    // Have the new buffer object allocate payload memory
    lBuffer->Alloc(static_cast<uint32_t>(lSize));

    // Add to external list - used to eventually release the buffers
    aBufferList->push_back(lBuffer);
  }

  // Queue all buffers in the stream
  for (auto buffer : *aBufferList) {
    aStream->QueueBuffer(buffer);
  }
}

void StreamManager::FreeStreamBuffers(std::vector<PvBuffer*>* aBufferList) {
  // Go through the buffer list
  for (auto buffer : *aBufferList) {
    // Release the buffer
    buffer->Free();
    delete buffer;
  }

  // Clear the buffer list
  aBufferList->clear();
}

StreamManager* StreamManager::getInstance() { return &staticInstance; }
StreamManager StreamManager::staticInstance;