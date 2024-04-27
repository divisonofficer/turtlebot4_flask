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
#include <DualDevice.h>
#include <Logger.h>
#include <PvBuffer.h>
#include <PvDevice.h>
#include <PvSampleUtils.h>
#include <PvStream.h>
#include <Stream.h>

#include <vector>

typedef std::vector<PvBuffer*> BufferList;

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
  std::cout << "nir: Configure stream to NIR" << std::endl;
  std::cout << "rgb: Configure stream to RGB" << std::endl;
}
void run_stream(PvDevice* lDevice, PvStream* lStream, std::string displayName,
                BufferList* lBufferList) {
  std::string command;

  while (command != "exit") {
    auto streamManager = StreamManager::getInstance();
    auto acquireManager = AcquireManager::getInstance();
    std::cout << "[" << displayName << "]" << std::endl;
    std::cout << "Enter a command: ";
    std::cin >> command;

    if (command == "help") {
      print_help_run_stream();
    }
    if (command == "capture_single") {
      acquireManager->AcquireSingleImage(lDevice, lStream);
    }
    if (command == "capture") {
      acquireManager->AcquireImages(lDevice, lStream);
    }
  }
}

int dual_device_run() {
  DualDevice* dualDevice = nullptr;
  std::string command;
  auto dbm = DeviceManager::getInstance();
  if (dualDevice = dbm->connectDualDevice()) {
    while (true) {
      std::cout << "Enter a command: rgb / nir / exit ";
      std::cin >> command;
      if (command == "exit") {
        break;
      }
      if (command == "rgb") {
        auto stream = dualDevice->getStream(0);
        auto device = dualDevice->getDevice(0);
        run_stream(device, stream, "RGB", dualDevice->getBufferList(0));
      }
      if (command == "nir") {
        auto stream = dualDevice->getStream(1);
        auto device = dualDevice->getDevice(1);
        run_stream(device, stream, "NIR", dualDevice->getBufferList(1));
      }
      if (command == "both") {
        AcquireManager::getInstance()->AcquireSingleImageDual(dualDevice);
      }
    }
  }

  return 0;
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
  if (DeviceManager::getInstance()->findDevice(lConnectionID, lDevice,
                                               displayName)) {
    lStream = StreamManager::getInstance()->OpenStream(lConnectionID);
    if (NULL != lStream) {
      StreamManager::getInstance()->ConfigureStream(lDevice, lStream, 0);
      StreamManager::getInstance()->CreateStreamBuffers(lDevice, lStream,
                                                        &lBufferList);

      run_stream(lDevice, lStream, displayName, &lBufferList);
      StreamManager::getInstance()->FreeStreamBuffers(&lBufferList);
      AcquireManager::getInstance()->streamDestroy(lDevice, lStream);

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
  std::cout << "run_dual: Run the program with dual devices" << std::endl;
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
    if (command == "run_dual") {
      dual_device_run();
    }
  }
  return 0;
}