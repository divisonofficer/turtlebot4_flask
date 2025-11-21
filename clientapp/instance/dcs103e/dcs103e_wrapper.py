"""
DCS103E Python Wrapper
Advanced Illumination DCS-103E Light Source Controller Python Interface

This module provides a Python interface for controlling DCS-103E light source devices.
It wraps the C++ SDK to provide easy-to-use Python functions for:
- Discovering connected light sources
- Controlling individual channels (turn on/off, set intensity)
- Managing device configurations and profiles
"""

import ctypes
import os
import platform
from typing import List, Dict, Optional
from enum import Enum
import json
from dcs_bindings import get_dcs_library, Mode as BindingMode, DCSType as BindingDCSType


class Mode(Enum):
    """Light source operation modes"""

    OFF = 0
    CONTINUOUS = 1
    PULSED = 2
    GATED = 3


class Trigger(Enum):
    """Trigger edge types"""

    FALLING = 0
    RISING = 1


class Channel(Enum):
    """Available channels"""

    ONE = 1
    TWO = 2
    THREE = 3


class DCSType(Enum):
    """DCS device types"""

    DCS_100E = 0  # Single output, three channel
    DCS_103E = 1  # Three output, single channel per output


class DCSException(Exception):
    """Base exception for DCS operations"""

    pass


class DeviceError(DCSException):
    """Device error exception"""

    pass


class DeviceWarning(DCSException):
    """Device warning exception"""

    pass


class DCSInfo:
    """Information about a discovered DCS device"""

    def __init__(
        self, name: str, firmware: str, lighthead: str, host: str, device_type: DCSType
    ):
        self.name = name
        self.firmware = firmware
        self.lighthead = lighthead
        self.host = host
        self.device_type = device_type

    def to_dict(self) -> Dict:
        """Convert to dictionary for JSON serialization"""
        return {
            "name": self.name,
            "firmware": self.firmware,
            "lighthead": self.lighthead,
            "host": self.host,
            "device_type": self.device_type.name,
        }

    def __str__(self) -> str:
        return f"DCS Device: {self.name} ({self.device_type.name}) at {self.host}"


class DCSChannel:
    """Represents a single DCS channel with its properties and controls"""

    def __init__(self, parent, channel_number: int):
        self._parent = parent
        self._channel_number = channel_number
        self._current = 0.0
        self._pulse_width = 0.01  # 10ms default
        self._pulse_delay = 5e-6  # 5 microseconds default
        self._mode = Mode.OFF
        self._trigger_mode = Trigger.RISING
        self._trigger_input = Channel.THREE  # Default from C++ example
        self._max_continuous = 0.0
        self._max_strobe = 0.0
        self._max_frequency = 0.0

    @property
    def current(self) -> float:
        """Get current intensity (0-100%)"""
        if self._parent._lib and self._parent._device_handle:
            try:
                # Get current from hardware and convert from mA to percentage
                current_ma = self._parent._lib.get_channel_current(
                    self._parent._device_handle, self._channel_number
                )
                max_current = self.max_continuous
                if max_current > 0:
                    self._current = (current_ma / max_current) * 100.0
                else:
                    self._current = 0.0
                return self._current
            except Exception as e:
                raise DeviceError(
                    f"Failed to get current for channel {self._channel_number}: {e}"
                )
        return self._current

    @current.setter
    def current(self, value: float):
        """Set current intensity (0-100%)"""
        if not 0 <= value <= 100:
            raise ValueError("Current must be between 0 and 100%")

        if self._parent._lib and self._parent._device_handle:
            try:
                # Convert percentage to mA based on max current
                max_current = self.max_continuous
                current_ma = (value / 100.0) * max_current

                success = self._parent._lib.set_channel_current(
                    self._parent._device_handle, self._channel_number, current_ma
                )
                if not success:
                    raise DeviceError(
                        f"Failed to set current for channel {self._channel_number}"
                    )

                self._current = value
            except Exception as e:
                raise DeviceError(
                    f"Failed to set current for channel {self._channel_number}: {e}"
                )
        else:
            self._current = value

    def set_current_ma(self, current_ma: float):
        """Set current directly in milliamps (as in C++ example)"""
        if self._parent._lib and self._parent._device_handle:
            try:
                success = self._parent._lib.set_channel_current(
                    self._parent._device_handle, self._channel_number, current_ma
                )
                if not success:
                    raise DeviceError(
                        f"Failed to set current for channel {self._channel_number}"
                    )

                # Update percentage based on max current
                max_current = self.max_continuous
                if max_current > 0:
                    self._current = (current_ma / max_current) * 100.0
                else:
                    self._current = 0.0
            except Exception as e:
                raise DeviceError(
                    f"Failed to set current for channel {self._channel_number}: {e}"
                )
        else:
            # Simulate for testing
            max_current = self._max_continuous or 1000.0  # Default 1A
            self._current = (current_ma / max_current) * 100.0

    @property
    def mode(self) -> Mode:
        """Get channel mode"""
        if self._parent._lib and self._parent._device_handle:
            try:
                mode_int = self._parent._lib.get_channel_mode(
                    self._parent._device_handle, self._channel_number
                )
                self._mode = Mode(mode_int)
                return self._mode
            except Exception as e:
                raise DeviceError(
                    f"Failed to get mode for channel {self._channel_number}: {e}"
                )
        return self._mode

    @mode.setter
    def mode(self, value: Mode):
        """Set channel mode"""
        if self._parent._lib and self._parent._device_handle:
            try:
                success = self._parent._lib.set_channel_mode(
                    self._parent._device_handle, self._channel_number, value.value
                )
                if not success:
                    raise DeviceError(
                        f"Failed to set mode for channel {self._channel_number}"
                    )

                self._mode = value
            except Exception as e:
                raise DeviceError(
                    f"Failed to set mode for channel {self._channel_number}: {e}"
                )
        else:
            self._mode = value

    @property
    def pulse_width(self) -> float:
        """Get pulse width in seconds"""
        if self._parent._lib and self._parent._device_handle:
            try:
                self._pulse_width = self._parent._lib.get_channel_pulse_width(
                    self._parent._device_handle, self._channel_number
                )
            except Exception as e:
                raise DeviceError(
                    f"Failed to get pulse width for channel {self._channel_number}: {e}"
                )
        return self._pulse_width

    @pulse_width.setter
    def pulse_width(self, value: float):
        """Set pulse width in seconds"""
        if value < 0:
            raise ValueError("Pulse width must be positive")

        if self._parent._lib and self._parent._device_handle:
            try:
                success = self._parent._lib.set_channel_pulse_width(
                    self._parent._device_handle, self._channel_number, value
                )
                if not success:
                    raise DeviceError(
                        f"Failed to set pulse width for channel {self._channel_number}"
                    )

                self._pulse_width = value
            except Exception as e:
                raise DeviceError(
                    f"Failed to set pulse width for channel {self._channel_number}: {e}"
                )
        else:
            self._pulse_width = value

    @property
    def pulse_delay(self) -> float:
        """Get pulse delay in seconds"""
        if self._parent._lib and self._parent._device_handle:
            try:
                self._pulse_delay = self._parent._lib.get_channel_pulse_delay(
                    self._parent._device_handle, self._channel_number
                )
            except Exception as e:
                raise DeviceError(
                    f"Failed to get pulse delay for channel {self._channel_number}: {e}"
                )
        return self._pulse_delay

    @pulse_delay.setter
    def pulse_delay(self, value: float):
        """Set pulse delay in seconds"""
        if value < 0:
            raise ValueError("Pulse delay must be positive")

        if self._parent._lib and self._parent._device_handle:
            try:
                success = self._parent._lib.set_channel_pulse_delay(
                    self._parent._device_handle, self._channel_number, value
                )
                if not success:
                    raise DeviceError(
                        f"Failed to set pulse delay for channel {self._channel_number}"
                    )

                self._pulse_delay = value
            except Exception as e:
                raise DeviceError(
                    f"Failed to set pulse delay for channel {self._channel_number}: {e}"
                )
        else:
            self._pulse_delay = value

    @property
    def trigger_input(self) -> Channel:
        """Get trigger input channel"""
        if self._parent._lib and self._parent._device_handle:
            try:
                trigger_input = self._parent._lib.get_channel_trigger_input(
                    self._parent._device_handle, self._channel_number
                )
                self._trigger_input = Channel(trigger_input)
            except Exception as e:
                raise DeviceError(
                    f"Failed to get trigger input for channel {self._channel_number}: {e}"
                )
        return self._trigger_input

    @trigger_input.setter
    def trigger_input(self, value: Channel):
        """Set trigger input channel"""
        if self._parent._lib and self._parent._device_handle:
            try:
                success = self._parent._lib.set_channel_trigger_input(
                    self._parent._device_handle, self._channel_number, value.value
                )
                if not success:
                    raise DeviceError(
                        f"Failed to set trigger input for channel {self._channel_number}"
                    )

                self._trigger_input = value
            except Exception as e:
                raise DeviceError(
                    f"Failed to set trigger input for channel {self._channel_number}: {e}"
                )
        else:
            self._trigger_input = value

    @property
    def max_continuous(self) -> float:
        """Get maximum continuous current"""
        if self._parent._lib and self._parent._device_handle:
            try:
                self._max_continuous = self._parent._lib.get_channel_max_continuous(
                    self._parent._device_handle, self._channel_number
                )
            except Exception as e:
                raise DeviceError(
                    f"Failed to get max continuous for channel {self._channel_number}: {e}"
                )
        return self._max_continuous

    @property
    def max_strobe(self) -> float:
        """Get maximum strobe current"""
        if self._parent._lib and self._parent._device_handle:
            try:
                self._max_strobe = self._parent._lib.get_channel_max_strobe(
                    self._parent._device_handle, self._channel_number
                )
            except Exception as e:
                raise DeviceError(
                    f"Failed to get max strobe for channel {self._channel_number}: {e}"
                )
        return self._max_strobe

    def turn_on(self, intensity: float = 100.0):
        """Turn on the channel with specified intensity"""
        self.mode = Mode.CONTINUOUS
        self.current = intensity

    def turn_off(self):
        """Turn off the channel"""
        self.mode = Mode.OFF
        self.current = 0.0

    def is_on(self) -> bool:
        """Check if channel is on"""
        return self.mode != Mode.OFF and self.current > 0

    def trigger(self):
        """Trigger the channel (for pulsed/gated modes)"""
        if self._parent._lib and self._parent._device_handle:
            try:
                success = self._parent._lib.trigger_channel(
                    self._parent._device_handle, self._channel_number
                )
                if not success:
                    raise DeviceError(
                        f"Failed to trigger channel {self._channel_number}"
                    )
            except Exception as e:
                raise DeviceError(
                    f"Failed to trigger channel {self._channel_number}: {e}"
                )

    def set_device_name(self, name: str):
        """Set device name (as shown in C++ example)"""
        if self._parent._lib and self._parent._device_handle:
            try:
                success = self._parent._lib.set_device_name(
                    self._parent._device_handle, name
                )
                if not success:
                    raise DeviceError(f"Failed to set device name")
                self._parent._name = name
            except Exception as e:
                raise DeviceError(f"Failed to set device name: {e}")
        else:
            self._parent._name = name

    def set_web_config_enabled(self, enabled: bool):
        """Enable/disable web configuration"""
        if self._parent._lib and self._parent._device_handle:
            try:
                success = self._parent._lib.set_web_config_enabled(
                    self._parent._device_handle, enabled
                )
                if not success:
                    raise DeviceError(f"Failed to set web config")
            except Exception as e:
                raise DeviceError(f"Failed to set web config: {e}")

    def run_cpp_example_test(self):
        """Run the test sequence from the C++ example"""
        print(f"Running C++ example test on channel {self._channel_number}")

        # Check if channel has valid max continuous current
        if self.max_continuous == 0:
            print(
                f"Channel {self._channel_number} has no max continuous current, skipping"
            )
            return

        import time

        try:
            # Set continuous mode
            print(f"Setting channel {self._channel_number} mode to Continuous")
            self.mode = Mode.CONTINUOUS
            time.sleep(0.2)

            # Set current to 50% of max continuous
            target_current = 0.5 * self.max_continuous
            print(
                f"Setting channel {self._channel_number} current to {target_current:.3f} mA"
            )
            self.set_current_ma(target_current)
            time.sleep(2)

            # Turn off current
            print("Setting 0 current")
            self.set_current_ma(0)

            # Set pulsed mode
            print("Setting pulsed mode")
            self.mode = Mode.PULSED

            # Set current to 1.2A (as in C++ example)
            print("Setting 1.2A")
            self.set_current_ma(1200.0)  # 1.2A = 1200mA

            # Set pulse width to 10ms
            print("Setting PW 10ms")
            self.pulse_width = 10e-3  # 10 milliseconds

            # Set pulse delay to 5us
            print("Setting PD 5us")
            self.pulse_delay = 5e-6  # 5 microseconds

            # Set trigger input to channel 3
            print("Setting trigger input CH3")
            self.trigger_input = Channel.THREE

            # Trigger 3 times with blinks
            for j in range(3):
                print("(blink)")
                self.trigger()
                time.sleep(0.5)

            print()

            # Turn off current
            self.set_current_ma(0)
            time.sleep(0.5)

            print(f"Channel {self._channel_number} test completed")

        except Exception as e:
            print(f"Error during test: {e}")
            # Ensure channel is turned off on error
            try:
                self.set_current_ma(0)
            except:
                pass


class DCS103E:
    """Main interface for DCS-103E light source controller"""

    def __init__(self, ip_address: Optional[str] = None):
        self._lib = get_dcs_library()
        self._device_handle = None
        self._ip_address = ip_address
        self._name = ""
        self._firmware_version = ""
        self._channels = {}
        self._connected = False

        # Initialize channels
        for i in range(1, 4):  # DCS-103E has 3 channels
            self._channels[i] = DCSChannel(self, i)

        # Auto-connect if IP provided
        if ip_address:
            self.connect(ip_address)

    @staticmethod
    def discover_devices() -> List[DCSInfo]:
        """Discover all DCS devices on the network"""
        devices = []

        try:
            lib = get_dcs_library()
            device_data = lib.discover_devices()

            for data in device_data:
                device_type = (
                    DCSType.DCS_103E
                    if data["device_type"] == BindingDCSType.DCS_103E
                    else DCSType.DCS_100E
                )
                devices.append(
                    DCSInfo(
                        name=data["name"],
                        firmware=data["firmware"],
                        lighthead=data["lighthead"],
                        host=data["host"],
                        device_type=device_type,
                    )
                )
        except Exception as e:
            raise DeviceError(f"Failed to discover devices: {e}")

        return devices

    def connect(self, ip_address: str) -> bool:
        """Connect to a DCS device at the specified IP address"""
        try:
            self._ip_address = ip_address

            # Create device handle
            self._device_handle = self._lib.create_device(ip_address)
            if not self._device_handle:
                raise DeviceError("Failed to create device handle")

            # Connect to device
            success = self._lib.connect_device(self._device_handle, ip_address)
            if not success:
                self._lib.destroy_device(self._device_handle)
                self._device_handle = None
                raise DeviceError("Failed to connect to device")

            self._connected = True

            # Get device information
            self._name = self._lib.get_device_name(self._device_handle)
            self._firmware_version = self._lib.get_firmware_version(self._device_handle)

            # Refresh device configuration
            self.refresh_config()

            return True

        except Exception as e:
            self._connected = False
            if self._device_handle:
                self._lib.destroy_device(self._device_handle)
                self._device_handle = None
            raise DeviceError(f"Failed to connect to device at {ip_address}: {e}")

    def disconnect(self):
        """Disconnect from the device"""
        try:
            if self._lib and self._device_handle and self._connected:
                self._lib.disconnect_device(self._device_handle)
                self._lib.destroy_device(self._device_handle)

            self._device_handle = None
            self._connected = False
            self._name = ""
            self._firmware_version = ""

        except Exception as e:
            raise DeviceError(f"Failed to disconnect: {e}")

    def refresh_config(self):
        """Refresh device configuration from hardware"""
        if not self._connected or not self._device_handle:
            raise DeviceError("Not connected to device")

        try:
            # Refresh channel configurations
            for channel in self._channels.values():
                # This will trigger property getters which read from hardware
                _ = channel.max_continuous
                _ = channel.max_strobe

        except Exception as e:
            raise DeviceError(f"Failed to refresh configuration: {e}")

    @property
    def connected(self) -> bool:
        """Check if connected to device"""
        return self._connected

    @property
    def name(self) -> str:
        """Get device name"""
        return self._name

    @property
    def firmware_version(self) -> str:
        """Get firmware version"""
        return self._firmware_version

    @property
    def ip_address(self) -> str:
        """Get IP address"""
        return self._ip_address or ""

    def get_channel(self, channel_number: int) -> DCSChannel:
        """Get a specific channel"""
        if channel_number not in self._channels:
            raise ValueError(f"Invalid channel number: {channel_number}")
        return self._channels[channel_number]

    def get_all_channels(self) -> Dict[int, DCSChannel]:
        """Get all channels"""
        return self._channels.copy()

    def turn_on_channel(self, channel_number: int, intensity: float = 100.0):
        """Turn on a specific channel with given intensity"""
        channel = self.get_channel(channel_number)
        channel.turn_on(intensity)

    def turn_off_channel(self, channel_number: int):
        """Turn off a specific channel"""
        channel = self.get_channel(channel_number)
        channel.turn_off()

    def turn_off_all_channels(self):
        """Turn off all channels"""
        for channel in self._channels.values():
            channel.turn_off()

    def set_channel_intensity(self, channel_number: int, intensity: float):
        """Set intensity for a specific channel"""
        channel = self.get_channel(channel_number)
        channel.current = intensity

    def get_channel_status(self, channel_number: int) -> Dict:
        """Get status information for a specific channel"""
        channel = self.get_channel(channel_number)
        return {
            "channel": channel_number,
            "current": channel.current,
            "mode": channel.mode.name,
            "is_on": channel.is_on(),
            "max_continuous": channel.max_continuous,
            "max_strobe": channel.max_strobe,
        }

    def set_device_name(self, name: str):
        """Set device name"""
        if self._lib and self._device_handle:
            try:
                success = self._lib.set_device_name(self._device_handle, name)
                if not success:
                    raise DeviceError("Failed to set device name")
                self._name = name
            except Exception as e:
                raise DeviceError(f"Failed to set device name: {e}")
        else:
            self._name = name

    def set_web_config_enabled(self, enabled: bool):
        """Enable/disable web configuration"""
        if self._lib and self._device_handle:
            try:
                success = self._lib.set_web_config_enabled(self._device_handle, enabled)
                if not success:
                    raise DeviceError("Failed to set web config")
            except Exception as e:
                raise DeviceError(f"Failed to set web config: {e}")

    def get_web_config_enabled(self) -> bool:
        """Check if web configuration is enabled"""
        if self._lib and self._device_handle:
            try:
                return self._lib.get_web_config_enabled(self._device_handle)
            except Exception as e:
                raise DeviceError(f"Failed to get web config status: {e}")
        return True  # Default simulation value

    def run_cpp_example_test(self):
        """Run the complete test sequence from the C++ example"""
        if not self.connected:
            raise DeviceError("Not connected to device")

        print(f"Running C++ example test on device: {self.name}")

        try:
            # Test each channel as in the C++ example
            for i in range(
                3
            ):  # 3 channels (0, 1, 2 in C++ which maps to 1, 2, 3 in Python)
                channel_num = i + 1
                channel = self.get_channel(channel_num)

                print(f"\nTesting channel {channel_num}...")
                channel.run_cpp_example_test()

            # Set device name to "Tested" as in C++ example
            print("\nSetting device name to 'Tested'")
            self.set_device_name("Tested")

            # Enable web config as in C++ example
            print("Enabling web configuration")
            self.set_web_config_enabled(True)

            print("\nC++ example test sequence completed successfully!")

        except Exception as e:
            print(f"Error during C++ example test: {e}")
            # Try to turn off all channels on error
            try:
                self.turn_off_all_channels()
            except:
                pass
            raise

    def get_device_status(self) -> Dict:
        """Get complete device status"""
        channels_status = {}
        for channel_num in self._channels:
            channels_status[channel_num] = self.get_channel_status(channel_num)

        status = {
            "connected": self.connected,
            "name": self.name,
            "firmware_version": self.firmware_version,
            "ip_address": self.ip_address,
            "web_config_enabled": (
                self.get_web_config_enabled() if self.connected else False
            ),
            "channels": channels_status,
        }

        return status
        """Get complete device status"""
        channels_status = {}
        for channel_num in self._channels:
            channels_status[channel_num] = self.get_channel_status(channel_num)

        status = {
            "connected": self.connected,
            "name": self.name,
            "firmware_version": self.firmware_version,
            "ip_address": self.ip_address,
            "channels": channels_status,
        }

        return status

    def __enter__(self):
        """Context manager entry"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit - ensure disconnection"""
        if self._connected:
            self.disconnect()


# Convenience functions
def discover_light_sources() -> List[DCSInfo]:
    """Discover all DCS light sources on the network"""
    return DCS103E.discover_devices()


def connect_to_light_source(ip_address: str) -> DCS103E:
    """Connect to a light source at the specified IP address"""
    device = DCS103E()
    device.connect(ip_address)
    return device


# Example usage and testing
if __name__ == "__main__":
    try:
        print("DCS-103E Python Wrapper Test")
        print("=" * 40)

        # Discover devices
        print("Discovering devices...")
        devices = discover_light_sources()
        print(f"Found {len(devices)} devices:")
        for device in devices:
            print(f"  - {device}")

        # Connect to first device (or use a specific IP)
        if devices:
            device_ip = devices[0].host
        else:
            device_ip = "192.168.1.100"  # Default IP for testing

        print(f"\nConnecting to device at {device_ip}...")

        with DCS103E(device_ip) as dcs:
            print(f"Connected to: {dcs.name}")
            print(f"Firmware: {dcs.firmware_version}")

            # Get device status
            status = dcs.get_device_status()
            print(f"\nDevice Status:")
            print(json.dumps(status, indent=2))

            # Test channel control
            print("\nTesting channel control...")

            # Turn on channel 1 at 50% intensity
            print("Turning on channel 1 at 50% intensity")
            dcs.turn_on_channel(1, 50.0)

            # Turn on channel 2 at 75% intensity
            print("Turning on channel 2 at 75% intensity")
            dcs.turn_on_channel(2, 75.0)

            # Get updated status
            print("\nUpdated channel status:")
            for i in range(1, 4):
                status = dcs.get_channel_status(i)
                print(f"Channel {i}: {status}")

            # Turn off all channels
            print("\nTurning off all channels...")
            dcs.turn_off_all_channels()

            print("Test completed successfully!")

    except Exception as e:
        print(f"Error: {e}")
