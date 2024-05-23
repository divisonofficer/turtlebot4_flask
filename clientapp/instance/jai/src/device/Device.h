#include <DualDevice.h>
#include <PvDevice.h>

#include <set>
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

  bool findDevice(PvString &aConnectionID, PvDevice *&aDevice,
                  std::string &displayName, std::string macAddress);

  PvDevice *DeviceConnectToDevice(const PvString &aConnectionID);

  DualDevice *connectDualDevice();
  DualDevice *connectDualDevice(std::string macAddress);

  static DeviceManager *getInstance();

 private:
  static DeviceManager staticInstance;

  bool findDeviceConnectionID(PvString &aConnectionID,
                              std::string &displayName);

  bool findDeviceConnectionID(PvString &aConnectionID, std::string &displayName,
                              std::string macAddress);

  int validateDeviceIP(const PvDeviceInfo *aDeviceInfo);

  std::set<std::string> aConnectionIdSet;
};
