#include <Device.h>
#include <Logger.h>

DeviceManager &DeviceManager::getInstance()
{
    return staticInstance;
}
DeviceManager DeviceManager::staticInstance;

bool DeviceManager::connectDevice()
{
    // Connect to device
    return true;
}

PvDevice *DeviceManager::DeviceConnectToDevice(const PvString &aConnectionID)
{
    PvDevice *lDevice;
    PvResult lResult;

    // Connect to the GigE Vision or USB3 Vision device
    Debug << "Connecting to device.\n";
    lDevice = PvDevice::CreateAndConnect(aConnectionID, &lResult);
    if (lDevice == NULL)
    {
        Debug << "Unable to connect to device: " << lResult.GetCodeString().GetAscii() << " " << lResult.GetDescription().GetAscii() << "\n";
    }

    return lDevice;
}
