#include <Device.h>
#include <Logger.h>
#include <PvSampleUtils.h>
#include <PvSystem.h>
DeviceManager *DeviceManager::getInstance() { return &staticInstance; }
DeviceManager DeviceManager::staticInstance;

bool DeviceManager::connectDevice() {
  // Connect to device
  return true;
}

bool DeviceManager::SelectDevice(PvString &aConnectionID, PvDevice *&aDevice) {
  if (!PvSelectDevice(&aConnectionID)) {
    return false;
  }
  aDevice = DeviceConnectToDevice(aConnectionID);
  if (aDevice == NULL) {
    return false;
  }
  return true;
}
bool DeviceManager::findDevice(PvString &aConnectionID, PvDevice *&aDevice,
                               std::string &displayName,
                               std::string macAddress) {
  if (!findDeviceConnectionID(aConnectionID, displayName, macAddress)) {
    return false;
  }

  aDevice = DeviceConnectToDevice(aConnectionID);
  if (aDevice == NULL) {
    ErrorLog << "Unable to connect to device.";
    return false;
  }
  return true;
}

bool DeviceManager::findDevice(PvString &aConnectionID, PvDevice *&aDevice,
                               std::string &displayName) {
  findDevice(aConnectionID, aDevice, displayName, 0);

  if (aConnectionID.GetAscii() == "") {
    return false;
  }
  return true;
}

DualDevice *DeviceManager::connectDualDevice() { return connectDualDevice(""); }
DualDevice *DeviceManager::connectDualDevice(std::string macAddress) {
  PvString lConnectionID;
  std::string displayName;
  if (!findDeviceConnectionID(lConnectionID, displayName, macAddress)) {
    return nullptr;
  }
  DualDevice *dualDevice = new DualDevice(lConnectionID);
  return dualDevice;
}

bool DeviceManager::findDeviceConnectionID(PvString &aConnectionID,
                                           std::string &displayName) {
  return findDeviceConnectionID(aConnectionID, displayName, "");
}

bool DeviceManager::findDeviceConnectionID(PvString &aConnectionID,
                                           std::string &displayName,
                                           std::string macAddress = "") {
  Info << "Finding Device Connection ID";
  PvSystem lSystem;
  PvDeviceInfo *aDeviceInfo = nullptr;
  lSystem.Find();
  int deviceCount = 0;

  // Find the first GEV Device
  for (int i = 0; i < lSystem.GetInterfaceCount(); i++) {
    const PvInterface *lInterface = lSystem.GetInterface(i);
    for (int j = 0; j < lInterface->GetDeviceCount(); j++) {
      const PvDeviceInfo *lDeviceInfo = lInterface->GetDeviceInfo(j);
      if (!lDeviceInfo) continue;
      if (lDeviceInfo->GetType() != PvDeviceInfoTypeGEV) continue;
      if (macAddress != "") {
        const PvDeviceInfoGEV *lDeviceGEV =
            dynamic_cast<const PvDeviceInfoGEV *>(lDeviceInfo);
        if (lDeviceGEV->GetMACAddress().GetAscii() != macAddress) {
          continue;
        }
      }

      if (aConnectionIdSet.find(lDeviceInfo->GetConnectionID().GetAscii()) !=
          aConnectionIdSet.end()) {
        continue;
      }

      aDeviceInfo = const_cast<PvDeviceInfo *>(lDeviceInfo);
      break;
    }
  }
  // Get the connection ID and Display Name
  aConnectionID = aDeviceInfo->GetConnectionID();
  displayName = aDeviceInfo->GetDisplayID().GetAscii();
  if (!aDeviceInfo) {
    ErrorLog << "No device found.";
    return false;
  }

  Info << "Device found: " << aDeviceInfo->GetDisplayID().GetAscii();

  // if IP Configuration is not valid, force new IP Address
  int ipValidation = validateDeviceIP(aDeviceInfo);
  if (ipValidation == -1) {
    return false;
  } else if (ipValidation == 1) {
    return findDeviceConnectionID(aConnectionID, displayName, macAddress);
  }
  return true;
}

int DeviceManager::validateDeviceIP(const PvDeviceInfo *aDeviceInfo) {
  PvResult lResult;
  if (aDeviceInfo->IsConfigurationValid() == false) {
    Info << "Device IP Configuration is not valid. Attempting to force new IP ";
    // Force New IP Address
    const PvDeviceInfoGEV *lDeviceGEV =
        dynamic_cast<const PvDeviceInfoGEV *>(aDeviceInfo);
    if (!lDeviceGEV) {
      ErrorLog << "Not a Valid GEV Device";
      return false;
    }
    // @todo : Get available IP address from somewhere
    lResult = PvDeviceGEV::SetIPConfiguration(
        lDeviceGEV->GetMACAddress().GetAscii(),
        ("192.168.185." + std::to_string(rand() % 200 + 10)).c_str(),
        lDeviceGEV->GetSubnetMask().GetAscii(),
        lDeviceGEV->GetDefaultGateway().GetAscii());
    if (!lResult.IsOK()) {
      ErrorLog << "Unable to force new IP address.";
      return -1;
    }
    return 1;
  }
  return 0;
}

PvDevice *DeviceManager::DeviceConnectToDevice(const PvString &aConnectionID) {
  PvDevice *lDevice;
  PvResult lResult;

  // Connect to the GigE Vision or USB3 Vision device
  Debug << "Connecting to device.\n";
  lDevice = PvDevice::CreateAndConnect(aConnectionID, &lResult);
  if (lDevice == NULL) {
    Debug << "Unable to connect to device: "
          << lResult.GetCodeString().GetAscii() << " "
          << lResult.GetDescription().GetAscii() << "\n";
  }

  /**
   * If the connection is successful, add the connection ID to the set
   * to avoid connecting to the same device again
   **/
  aConnectionIdSet.insert(aConnectionID.GetAscii());

  return lDevice;
}
