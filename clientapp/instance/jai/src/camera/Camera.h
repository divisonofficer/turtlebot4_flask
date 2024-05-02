#include <DualDevice.h>

#include <atomic>
#include <functional>
class MultiSpectralCamera {
 public:
  MultiSpectralCamera();
  ~MultiSpectralCamera();

  void openStream();
  void closeStream();

  void addStreamCallback(int source, std::function<void(PvBuffer*)> callback);

  void configureExposure(int source, float exposure);
  void configureGain(int source, float gain);
  double getExposure(int source);
  double getGain(int source);

  void runUntilInterrupted();

  void interrupt();

  void printCameraConfig();

  void configureSourceRuntime(
      int source, std::function<void(PvGenParameterArray*)> runBlock);

 private:
  std::function<void(PvBuffer*)> streamCallback[2];
  DualDevice* dualDevice;
  std::atomic<bool> flagInterrupted;
};
;
