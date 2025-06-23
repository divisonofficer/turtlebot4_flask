from platform import node
import threading
from arena_api.system import system
from numpy import isin, tri


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
        self.timestamp_ns = timestamp_ns

    def __repr__(self):
        return f"LucidImage(buffer_np_list={self.buffer_np})"

    def __del__(self):
        del self.buffer_np


class LucidCamera:
    def __init__(self, serial: str = "224201564"):
        # self.SERIAL = ["224201564", "224201585"]
        self.SERIAL = serial
        self.timestamp_base = 0
        self.FRAME_RATE = 10.0
        self.BUFFER_COUNT = 1
        self.buffers_device: List[_Buffer] = []
        self.trigger_thread: Optional[threading.Thread] = None
        self.trigger_thread_stop = threading.Event()
        self.device: Optional[Device] = None

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

    def device_collect_buffers(self, device: Device, trigger=False):
        begin_time = time.time()
        device.nodemap.get_node("AcquisitionStart").execute()
        self.buffers_device = []
        try:
            buffers = device.get_buffer(1, timeout=300)
        except Exception as e:
            print(f"{TAB1}Error getting buffer: {e}")
            return

        if isinstance(buffers, _Buffer):
            self.buffers_device = [buffers]

    def trigger_loop(self):
        trigger_armed = False
        trigger_armed_node = self.device.nodemap.get_node("TriggerArmed")
        while True:
            if self.trigger_thread_stop.is_set():
                self.trigger_thread_stop.clear()
                break
            try:
                trigger_armed = trigger_armed_node.value
            except Exception as e:
                trigger_armed = False
            if trigger_armed:
                self.trigger_capture()
            time.sleep(0.03)

    def trigger_capture(self):
        trigger_sw_node = self.device.nodemap.get_node("TriggerSoftware")
        trigger_sw_node.execute()

    def open_stream(self):
        if self.device is None:
            self.connect_device()
            self.device.start_stream()

        if self.trigger_thread is not None:
            if self.trigger_thread.is_alive():
                print(f"{TAB1} {self.SERIAL} : Trigger thread already started")
                return
        self.trigger_thread = threading.Thread(target=self.trigger_loop, daemon=True)
        self.trigger_thread.start()

    def collect_images(self):
        if self.device is None:
            raise Exception("Device not connected")
        threads = []
        thread = threading.Thread(
            target=self.device_collect_buffers, args=(self.device,), daemon=True
        )
        threads.append(thread)
        thread.start()
        for thread in threads:
            thread.join()
        buffer_np_list: List[LucidImage] = []
        for buffer in self.buffers_device:
            time_begin = time.time()
            buffer_np = self.buffer_to_image(buffer)
            if buffer_np is not None:
                timestamp_ns = buffer.timestamp_ns + self.timestamp_base
                buffer_np_list.append(LucidImage(buffer_np, timestamp_ns))
            self.device.requeue_buffer(buffer)

        return buffer_np_list

    def collect_image_loop(self, callback: Callable[[LucidImage], None]):
        while True:
            images = self.collect_images()
            for raw_img in images:
                callback(raw_img)

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
        try:
            if self.device is not None:
                print(f"{TAB1}Device already connected")
                return True
            devices = self.create_devices_with_tries()
            print("Device List : ", devices)

            for device in devices:
                if device.nodemap.get_node("DeviceSerialNumber").value == self.SERIAL:
                    self.device = device
                    break

            print(
                f"{TAB1}Connected to device {self.device.nodemap.get_node('DeviceModelName').value}"
            )
            if self.device is None:
                print(
                    f"{TAB1}Device with serial {self.SERIAL} not found. "
                    f"Please check the connection."
                )
                return False
            device = self.device
            timestamp_ns = device.nodemap.get_node("PtpDataSet").value
            self.timestamp_base = time.time_ns() - timestamp_ns

            print(f"{TAB1}Timestamp: {timestamp_ns}")
            print(f"{TAB1}Timestamp base: {self.timestamp_base}")

            self.config_device(device)
            print(
                f"{TAB1}Connected to device {device.nodemap.get_node('DeviceModelName').value}"
            )

            trigger_delay = device.nodemap.get_node("TriggerDelay").value
            print(f"{TAB1}Initial Trigger Delay: {trigger_delay}")
            device.nodemap.get_node("TriggerDelay").value = 0.0

            return True
        except Exception as e:
            print(e)
            return False

    def config_device(self, device: Device):
        nodemap = device.nodemap
        device.stop_stream()
        resetTimestamp: NodeCommand = nodemap.get_node("TimestampReset")
        resetTimestamp.execute()

        # node_report_keys = [
        #     "AcquisitionStartMode",
        #     # "TriggerLatency",
        #     "TriggerActivation",
        #     "TriggerSource",
        #     "TriggerMode",
        #     "TriggerSelector",
        #     "AcquisitionFrameRate",
        #     "Width",
        #     "Height",
        #     "TriggerOverlap",
        #     "PayloadSize",
        #     "ExposureTime",
        #     "ExposureAuto",
        #     "ColorTransformationEnable",
        #     "BlackLevel",
        #     "BalanceWhiteEnable",
        #     "BalanceWhiteAuto",
        #     "HDROutput",
        #     "HDRTuningEnable",
        #     "LUTEnable",
        #     "LUTToneMapping",
        # ]

        # for node_key in node_report_keys:
        #     print(f"{TAB1}{node_key}: ")
        #     print({nodemap.get_node(node_key).value})

        # nodemap.get_node("Width").value = 2880
        # nodemap.get_node("Height").value = 1856

        nodemap.get_node("AcquisitionBurstFrameCount").value = 1
        nodemap.get_node("OffsetX").value = int(0)
        nodemap.get_node("OffsetY").value = int(0)
        nodemap.get_node("AcquisitionFrameRateEnable").value = True
        nodemap.get_node("Width").value = RESOLUTION[0] // 2
        nodemap.get_node("Height").value = RESOLUTION[1] // 2
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

        nodemap["ExposureTime"].value = 50000.0
        nodemap["ExposureAuto"].value = "Off"

        nodemap["LUTEnable"].value = False
        nodemap["HDRTuningEnable"].value = False
        nodemap["ColorTransformationEnable"].value = False

        nodemap["GainAuto"].value = "Off"
        nodemap["Gain"].value = 1.0
        nodemap["BalanceWhiteEnable"].value = False

        tl_stream_nodemap = device.tl_stream_nodemap
        tl_stream_nodemap["StreamAutoNegotiatePacketSize"].value = True
        tl_stream_nodemap["StreamPacketResendEnable"].value = True

    def __del__(self):
        system.destroy_device()
        print(f"{TAB1}Destroyed device")


if __name__ == "__main__":
    lucid = LucidCamera(serial="224201564")
    lucid_right = LucidCamera(serial="224201585")
    lucid.connect_device()
    lucid.open_stream()
    lucid_right.connect_device()
    lucid_right.open_stream()
    images = lucid.collect_images()
    print(f"Collected {len(images)} images")
    images = lucid_right.collect_images()
    print(f"Collected {len(images)} images")
    del lucid
