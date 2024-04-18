#include <Logger.h>
#include <PvBuffer.h>
#include <PvDeviceGEV.h>
#include <PvStreamGEV.h>
#include <Stream.h>

#include <list>

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

void StreamManager::ConfigureStream(PvDevice* aDevice, PvStream* aStream) {
  // If this is a GigE Vision device, configure GigE Vision specific streaming
  // parameters
  PvDeviceGEV* lDeviceGEV = dynamic_cast<PvDeviceGEV*>(aDevice);
  if (lDeviceGEV != NULL) {
    PvStreamGEV* lStreamGEV = static_cast<PvStreamGEV*>(aStream);

    // Negotiate packet size
    lDeviceGEV->NegotiatePacketSize();

    // Configure device streaming destination
    lDeviceGEV->SetStreamDestination(lStreamGEV->GetLocalIPAddress(),
                                     lStreamGEV->GetLocalPort());
  }
}

void StreamManager::CreateStreamBuffers(PvDevice* aDevice, PvStream* aStream,
                                        std::list<PvBuffer*>* aBufferList) {
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
  std::list<PvBuffer*>::iterator lIt = aBufferList->begin();
  while (lIt != aBufferList->end()) {
    aStream->QueueBuffer(*lIt);
    lIt++;
  }
}

void StreamManager::FreeStreamBuffers(std::list<PvBuffer*>* aBufferList) {
  // Go through the buffer list
  std::list<PvBuffer*>::iterator lIt = aBufferList->begin();
  while (lIt != aBufferList->end()) {
    delete *lIt;
    lIt++;
  }

  // Clear the buffer list
  aBufferList->clear();
}

StreamManager& StreamManager::getInstance() { return staticInstance; }
StreamManager StreamManager::staticInstance;