#include <DualDevice.h>
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

  bool SelectDevice(PvString &aConnectionID, PvDevice *&aDevice);

  /**
   * Get the first GEV Device
   * @param aConnectionID where the connection ID is stored
   * @param aDevice where the device is stored
   * @param displayName where the display name is stored
   */
  bool findDevice(PvString &aConnectionID, PvDevice *&aDevice,
                  std::string &displayName);

  PvDevice *DeviceConnectToDevice(const PvString &aConnectionID);

  DualDevice *connectDualDevice();

  static DeviceManager *getInstance();

 private:
  static DeviceManager staticInstance;
  bool findDeviceConnectionID(PvString &aConnectionID,
                              std::string &displayName);
};
