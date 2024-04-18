// *****************************************************************************
//
//      Copyright (c) 2013, Pleora Technologies Inc., All rights reserved.
//
// *****************************************************************************

//
// Shows how to use a PvStream object to acquire images from a GigE Vision or
// USB3 Vision device.
//

#include <Acquire.h>
#include <Device.h>
#include <Logger.h>
#include <PvBuffer.h>
#include <PvDevice.h>
#include <PvSampleUtils.h>
#include <PvStream.h>
#include <Stream.h>

#include <list>

typedef std::list<PvBuffer*> BufferList;

PV_INIT_SIGNAL_HANDLER();

#define BUFFER_COUNT (16)

//
// Main function
//
int main() {
  PvDevice* lDevice = NULL;
  PvStream* lStream = NULL;
  BufferList lBufferList;

  PV_SAMPLE_INIT();

  Debug << "PvStreamSample:"
        << "\n\n";

  PvString lConnectionID;
  if (PvSelectDevice(&lConnectionID)) {
    lDevice = DeviceManager::getInstance().DeviceConnectToDevice(lConnectionID);
    if (NULL != lDevice) {
      lStream = StreamManager::getInstance().OpenStream(lConnectionID);
      if (NULL != lStream) {
        StreamManager::getInstance().ConfigureStream(lDevice, lStream);
        StreamManager::getInstance().CreateStreamBuffers(lDevice, lStream,
                                                         &lBufferList);
        AcquireManager::getInstance().AcquireImages(lDevice, lStream);
        StreamManager::getInstance().FreeStreamBuffers(&lBufferList);

        // Close the stream
        cout << "Closing stream" << endl;
        lStream->Close();
        PvStream::Free(lStream);
      }

      // Disconnect the device
      cout << "Disconnecting device" << endl;
      lDevice->Disconnect();
      PvDevice::Free(lDevice);
    }
  }

  cout << endl;
  cout << "<press a key to exit>" << endl;
  PvWaitForKeyPress();

  PV_SAMPLE_TERMINATE();

  return 0;
}