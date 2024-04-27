#include <Camera.h>
#include <Logger.h>
#include <PvBuffer.h>
#include <PvSampleUtils.h>
#include <termios.h>
#include <unistd.h>

#include <thread>
PV_INIT_SIGNAL_HANDLER();

#define BUFFER_COUNT (16)

// Function to detect keyboard event
char detectKeyboardEvent() {
  struct termios oldSettings, newSettings;
  tcgetattr(STDIN_FILENO, &oldSettings);
  newSettings = oldSettings;
  newSettings.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newSettings);

  char key = '\0';
  if (read(STDIN_FILENO, &key, 1) == -1) {
    key = '\0';
  }

  tcsetattr(STDIN_FILENO, TCSANOW, &oldSettings);
  return key;
}

void callback(int source, PvBuffer* buffer) {
  // Do something with the buffer
  if (buffer) {
    Debug << "Buffer received from source " << source;
    Debug << "Buffer size: " << buffer->GetPayloadSize();
    Debug << "Image resolution: " << buffer->GetImage()->GetWidth() << "x"
          << buffer->GetImage()->GetHeight();
  }
}

void print_help() {
  std::cout << "Commands: " << std::endl;
  std::cout << "o: Open stream" << std::endl;
  std::cout << "c: Close stream" << std::endl;
  std::cout << "e: Configure exposure" << std::endl;
  std::cout << "g: Configure gain" << std::endl;
  std::cout << "r: Get exposure" << std::endl;
  std::cout << "t: Get gain" << std::endl;
  std::cout << "q: Quit" << std::endl;
}

int main() {
  MultiSpectralCamera* camera = new MultiSpectralCamera();
  std::thread t([&camera]() { camera->runUntilInterrupted(); });
  camera->addStreamCallback(0, [](PvBuffer* buffer) { callback(0, buffer); });
  camera->addStreamCallback(1, [](PvBuffer* buffer) { callback(1, buffer); });
  int key = 1;
  while (key) {
    if (key == 1) {
      print_help();
    }
    key = 1;
    switch (detectKeyboardEvent()) {
      case 'q':

        key = 0;
        break;
      case 'o':
        camera->openStream();
        break;

      case 'c':
        camera->closeStream();
        break;
      case 'e':
        camera->configureExposure(0, 100000);
        camera->configureExposure(1, 100000);
        break;
      case 'g':
        camera->configureGain(0, 10);
        camera->configureGain(1, 10);
        break;
      case 'r':
        Debug << "Exposure 0: " << camera->getExposure(0);
        Debug << "Exposure 1: " << camera->getExposure(1);
        break;
      case 't':
        Debug << "Gain 0: " << camera->getGain(0);
        Debug << "Gain 1: " << camera->getGain(1);
        break;
      default:
        key = 2;
        break;
    }
  }
  camera->interrupt();
  t.join();
  t.detach();
  return 0;
}
