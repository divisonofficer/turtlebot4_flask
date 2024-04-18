#include <PvDevice.h>

class DeviceManager {
 public:
  /**
   * @brief Run Device Connect Process
   */
  bool connectDevice();
  /**
   * @brief Run Device Connect Process
   * @param aConnectionID Connection ID of the device to connect
   */
  PvDevice *DeviceConnectToDevice(const PvString &aConnectionID);

  static DeviceManager &getInstance();

 private:
  static DeviceManager staticInstance;
};
