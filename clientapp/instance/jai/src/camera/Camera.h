#include <DualDevice.h>

#include <atomic>
#include <functional>
class MultiSpectralCamera {
 public:
  MultiSpectralCamera();
  MultiSpectralCamera(std::string deviceName, std::string macAddress);
  ~MultiSpectralCamera();

  void openStream();
  void closeStream();

  void initCameraTimestamp();

  void addStreamCallback(int source, std::function<void(PvBuffer*)> callback);

  void configureDevice(int source, std::string command, std::string type,
                       std::string value);

  std::vector<std::string> sourcePixelFormat;

  void configureExposure(int source, float exposure);
  void configureGain(int source, float gain);
  double getExposure(int source);
  double getGain(int source);

  void runUntilInterrupted(int streamIndex);

  void interrupt();

  void printCameraConfig();

  void configureSourceRuntime(
      int source, std::function<void(PvGenParameterArray*)> runBlock);

  std::string getParameter(int source, std::string parameter);

  int64_t timestamp_begin;

  std::string deviceName;

  double holdAutoExposureAndGetValue(bool hold);

 private:
  void initDevice();
  std::function<void(PvBuffer*)> streamCallback[2];
  DualDevice* dualDevice;
  std::atomic<bool> flagInterrupted;
  std::string macAddressInit;
};
;
