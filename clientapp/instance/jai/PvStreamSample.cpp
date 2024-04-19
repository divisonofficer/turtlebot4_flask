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

void print_help_run_stream() {
  std::cout << "Commands: " << std::endl;
  std::cout << "capture_single: Capture a single image" << std::endl;
  std::cout << "capture: Capture images" << std::endl;
  std::cout << "exit: Exit the program" << std::endl;
}
void run_stream(PvDevice* lDevice, PvStream* lStream, std::string displayName) {
  std::string command;
  while (command != "exit") {
    std::cout << "[" << displayName << "]" << std::endl;
    std::cout << "Enter a command: ";
    std::cin >> command;
    PvGetChar();
    PvGetChar();
    if (command == "help") {
      print_help_run_stream();
    }
    if (command == "capture_single") {
      AcquireManager::getInstance().AcquireSingleImage(lDevice, lStream);
    }
    if (command == "capture") {
      AcquireManager::getInstance().AcquireImages(lDevice, lStream);
    }
  }
}
int run() {
  PvDevice* lDevice = NULL;
  PvStream* lStream = NULL;
  BufferList lBufferList;

  PV_SAMPLE_INIT();

  Debug << "PvStreamSample:"
        << "\n\n";

  PvString lConnectionID;
  std::string displayName;
  // DeviceManager::getInstance().SelectDevice(lConnectionID, lDevice)
  if (DeviceManager::getInstance().findDevice(lConnectionID, lDevice,
                                              displayName)) {
    lStream = StreamManager::getInstance().OpenStream(lConnectionID);
    if (NULL != lStream) {
      StreamManager::getInstance().ConfigureStream(lDevice, lStream);
      StreamManager::getInstance().CreateStreamBuffers(lDevice, lStream,
                                                       &lBufferList);

      run_stream(lDevice, lStream, displayName);

      AcquireManager::getInstance().streamDestroy(lDevice, lStream);
      StreamManager::getInstance().FreeStreamBuffers(&lBufferList);

      // Close the stream
      std::cout << "Closing stream" << endl;
      lStream->Close();
      PvStream::Free(lStream);
    }

    // Disconnect the device
    std::cout << "Disconnecting device" << endl;
    lDevice->Disconnect();
    PvDevice::Free(lDevice);
  }

  std::cout << endl;
  std::cout << "<press a key to exit>" << endl;
  PvWaitForKeyPress();

  PV_SAMPLE_TERMINATE();

  return 0;
}

void print_help_main() {
  std::cout << "Commands: " << std::endl;
  std::cout << "run: Run the program" << std::endl;
  std::cout << "exit: Exit the program" << std::endl;
}

int main() {
  std::string command = "";
  while (command != "exit") {
    std::cout << "Enter a command: ";
    std::cin >> command;
    if (command == "help") {
      print_help_main();
    }
    if (command == "run") {
      run();
    }
  }
  return 0;
}