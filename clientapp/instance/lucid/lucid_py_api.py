from platform import node
import threading
from arena_api.system import system
from numpy import tri


TAB1 = "  "
TAB2 = "    "
import time
from typing import Callable, List, Optional
from arena_api._device import Device
from arena_api._node import NodeCommand
from arena_api.buffer import _Buffer
import numpy as np
from std_msgs.msg import Header

RESOLUTION_MAX = (2880, 1856)
RESOLUTION = (2880, 1856)


class LucidImage:
    def __init__(self, buffer_np: np.ndarray, timestamp_ns: int):
        self.buffer_np = buffer_np
        self.header = Header()
        timestamp_sec = timestamp_ns // 1_000_000_000
        timestamp_sec = timestamp_sec % 2147483647
        self.header.stamp.sec = timestamp_sec
        self.header.stamp.nanosec = timestamp_ns % 1_000_000_000

    def __repr__(self):
        return f"LucidImage(buffer_np_list={self.buffer_np})"

    def __del__(self):
        del self.buffer_np


class LucidPyAPI:
    def __init__(self):
        self.SERIAL = ["224201564", "224201585"]
        self.timestamp_base = [0, 0]
        self.FRAME_RATE = 4.0
        self.BUFFER_COUNT = 4
        self.buffers_device: List[List[_Buffer]] = [[], []]
        self.trigger_thread: Optional[threading.Thread] = None

    def create_devices_with_tries(self):
        """
        This function waits for the user to connect a device before raising
            an exception
        """

        tries = 0
        tries_max = 6
        sleep_time_secs = 10
        while tries < tries_max:  # Wait for device for 60 seconds
            devices = system.create_device()
            if not devices:
                print(
                    f"{TAB1}Try {tries+1} of {tries_max}: waiting for {sleep_time_secs} "
                    f"secs for a device to be connected!"
                )
                for sec_count in range(sleep_time_secs):
                    time.sleep(1)
                    print(
                        f"{TAB1}{sec_count + 1 } seconds passed ",
                        "." * sec_count,
                        end="\r",
                    )
                tries += 1
            else:
                print(f"{TAB1}Created {len(devices)} device(s)")
                return devices
        else:
            raise Exception(
                f"{TAB1}No device found! Please connect a device and run "
                f"the example again."
            )

    def device_collect_buffers(self, device: Device, idx: int, trigger=False):
        begin_time = time.time()
        device.nodemap.get_node("AcquisitionStart").execute()

        buffers = device.get_buffer(self.BUFFER_COUNT)

        self.buffers_device[idx] = buffers

    def trigger_loop(self):
        trigger_armed = False
        trigger_left = self.devices[0].nodemap.get_node("TriggerArmed")
        trigger_right = self.devices[1].nodemap.get_node("TriggerArmed")

        trigger_ex_left = self.devices[0].nodemap.get_node("TriggerSoftware")
        trigger_ex_right = self.devices[1].nodemap.get_node("TriggerSoftware")

        trigger_ac_left = self.devices[0].nodemap.get_node("AcquisitionStart")
        trigger_ac_right = self.devices[1].nodemap.get_node("AcquisitionStart")
        trigger_time = time.time()
        count = 0
        while True:
            try:
                trigger_armed = trigger_left.value and trigger_right.value
            except Exception as e:
                trigger_armed = False
            if trigger_armed:
                # print(f"Triggering took {time.time() - trigger_time} seconds")
                trigger_time = time.time()

                trigger_ex_left.execute()
                trigger_ex_right.execute()
                # if count == self.FRAME_RATE:
                #     trigger_ac_left.execute()
                #     trigger_ac_right.execute()
                #     count = 0
                count += 1
            # print(f"Armed {trigger_left.value} {trigger_right.value}")
            # print(f"OnAcquisitoin {self.devices[0].nodemap['AcquisitionControl']}")
            # time.sleep(0.05)

    def open_stream(self):
        if self.trigger_thread is not None:
            if self.trigger_thread.is_alive():
                return
        self.trigger_thread = threading.Thread(target=self.trigger_loop, daemon=True)
        self.trigger_thread.start()

        for idx, device in enumerate(self.devices):
            device.start_stream()

    def collect_images(self):
        threads = []

        for idx, device in enumerate(self.devices):
            thread = threading.Thread(
                target=self.device_collect_buffers, args=(device, idx), daemon=True
            )
            threads.append(thread)
            thread.start()
        for thread in threads:
            thread.join()
        buffer_np_list: List[List[LucidImage]] = [[], []]
        for idx, device in enumerate(self.devices):
            for buffer in self.buffers_device[idx]:
                time_begin = time.time()
                buffer_np = self.buffer_to_image(buffer)
                if buffer_np is not None:
                    timestamp_ns = buffer.timestamp_ns + self.timestamp_base[idx]
                    buffer_np_list[idx].append(LucidImage(buffer_np, timestamp_ns))
                device.requeue_buffer(buffer)

        return buffer_np_list

    def collect_image_loop(self, callback: Callable[[LucidImage, LucidImage], None]):
        while True:
            images = self.collect_images()
            for left, right in zip(images[0], images[1]):
                callback(left, right)

    def buffer_to_image(self, buffer: _Buffer) -> Optional[np.ndarray]:
        if buffer.is_incomplete:
            print(
                f"""
                  buffer.is_incomplete: {buffer.is_incomplete}
                  buffer.xbuffer.has_image_data: {buffer.xbuffer.xBufferHasImageData()}
                  buffer.xbuffer.has_chunk_data: {buffer.xbuffer.xBufferHasChunkData()}
                  {buffer.xbuffer.xBufferGetSizeFilled()} / {buffer.xbuffer.xBufferGetSizeOfBuffer()}
                  
                  """
            )
            return None
        pointer = buffer.xbuffer.xImageGetData()

        data_np: np.ndarray = np.ctypeslib.as_array(
            pointer, shape=(buffer.buffer_size,)
        ).copy()
        data_np = data_np.reshape(
            buffer.height, buffer.width, buffer.bits_per_pixel // 8
        )

        return data_np

    def connect_device(self):
        devices = self.create_devices_with_tries()
        print("Device List : ", devices)
        self.devices: List[Device] = []
        for serial in self.SERIAL:
            for device in devices:
                if device.nodemap.get_node("DeviceSerialNumber").value == serial:
                    self.devices.append(device)
                    break

        print(f"{TAB1}Connected to {len(self.devices)} device(s)")

        for idx, device in enumerate(self.devices):
            timestamp_ns = device.nodemap.get_node("PtpDataSet").value
            self.timestamp_base[idx] = time.time_ns() - timestamp_ns

            print(f"{TAB1}Timestamp: {timestamp_ns}")
            print(f"{TAB1}Timestamp base: {self.timestamp_base[idx]}")

        for idx, device in enumerate(self.devices):
            self.config_device(device, idx)
            print(
                f"{TAB1}Connected to device {device.nodemap.get_node('DeviceModelName').value}"
            )

            trigger_delay = device.nodemap.get_node("TriggerDelay").value
            print(f"{TAB1}Initial Trigger Delay: {trigger_delay}")
            device.nodemap.get_node("TriggerDelay").value = 0.0
        # self.devices[0].nodemap.get_node("TriggerDelay").value = 0.0

    def config_device(self, device: Device, idx: int):
        nodemap = device.nodemap
        device.stop_stream()
        resetTimestamp: NodeCommand = nodemap.get_node("TimestampReset")
        resetTimestamp.execute()
        print(
            nodemap.get_node("AcquisitionStartMode"),
            nodemap.get_node("TriggerLatency"),
            nodemap.get_node("TriggerActivation"),
            nodemap.get_node("TriggerSource"),
            nodemap.get_node("TriggerMode"),
            nodemap.get_node("TriggerSelector"),
            nodemap.get_node("AcquisitionFrameRate"),
            nodemap.get_node("Width"),
            nodemap.get_node("Height"),
            nodemap.get_node("TriggerOverlap"),
            nodemap["TriggerLatency"],
            nodemap["PayloadSize"],
        )

        nodemap.get_node("AcquisitionBurstFrameCount").value = 1
        nodemap.get_node("OffsetX").value = int(0)
        nodemap.get_node("OffsetY").value = int(0)
        nodemap.get_node("AcquisitionFrameRateEnable").value = True
        nodemap.get_node("Width").value = (
            RESOLUTION[0] // nodemap.get_node("BinningHorizontal").value
        )
        nodemap.get_node("Height").value = (
            RESOLUTION[1] // nodemap.get_node("BinningVertical").value
        )
        nodemap.get_node("BinningSelector").value = "Sensor"
        nodemap.get_node("BinningHorizontalMode").value = "Average"
        nodemap.get_node("BinningVerticalMode").value = "Average"

        nodemap.get_node("BinningHorizontal").value = int(2)
        nodemap.get_node("BinningVertical").value = int(2)

        nodemap.get_node("AcquisitionFrameRate").value = self.FRAME_RATE
        nodemap.get_node("AcquisitionFrameCount").value = self.BUFFER_COUNT
        nodemap.get_node("AcquisitionMode").value = "Continuous"

        nodemap.get_node("PixelFormat").value = "BayerRG24"

        nodemap["TriggerSelector"].value = "FrameStart"
        nodemap["TriggerOverlap"].value = "PreviousFrame"
        nodemap["TriggerMode"].value = "On"
        nodemap["TriggerSource"].value = "Software"

        tl_stream_nodemap = device.tl_stream_nodemap
        tl_stream_nodemap["StreamAutoNegotiatePacketSize"].value = True
        tl_stream_nodemap["StreamPacketResendEnable"].value = True

    def __del__(self):
        system.destroy_device()
        print(f"{TAB1}Destroyed device")


if __name__ == "__main__":
    lucid = LucidPyAPI()
    lucid.connect_device()
    lucid.open_stream()
    lucid.collect_images()
    lucid.collect_images()
    del lucid
