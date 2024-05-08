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
                               std::string &displayName) {
  if (!findDeviceConnectionID(aConnectionID, displayName)) {
    return false;
  }

  aDevice = DeviceConnectToDevice(aConnectionID);
  if (aDevice == NULL) {
    ErrorLog << "Unable to connect to device.";
    return false;
  }
  return true;
}

DualDevice *DeviceManager::connectDualDevice() {
  PvString lConnectionID;
  std::string displayName;
  if (!findDeviceConnectionID(lConnectionID, displayName)) {
    return nullptr;
  }
  DualDevice *dualDevice = new DualDevice(lConnectionID);
  return dualDevice;
}

bool DeviceManager::findDeviceConnectionID(PvString &aConnectionID,
                                           std::string &displayName) {
  PvSystem lSystem;
  PvDeviceInfo *aDeviceInfo = nullptr;
  PvResult lResult;
  lSystem.Find();

  // Find the first GEV Device
  for (int i = 0; i < lSystem.GetInterfaceCount(); i++) {
    const PvInterface *lInterface = lSystem.GetInterface(i);
    for (int j = 0; j < lInterface->GetDeviceCount(); j++) {
      const PvDeviceInfo *lDeviceInfo = lInterface->GetDeviceInfo(j);
      if (!lDeviceInfo) continue;
      if (lDeviceInfo->GetType() != PvDeviceInfoTypeGEV) continue;

      aDeviceInfo = const_cast<PvDeviceInfo *>(lDeviceInfo);
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
        lDeviceGEV->GetMACAddress().GetAscii(), "192.168.185.11",
        lDeviceGEV->GetSubnetMask().GetAscii(),
        lDeviceGEV->GetDefaultGateway().GetAscii());
    if (!lResult.IsOK()) {
      ErrorLog << "Unable to force new IP address.";
      return false;
    }
  }
  return true;
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

  return lDevice;
}
