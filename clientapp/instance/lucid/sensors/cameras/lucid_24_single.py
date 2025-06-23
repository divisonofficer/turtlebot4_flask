import threading
from typing import Union, Optional

import cv2
import numpy as np
from lucid_cam import LucidCamera
import rclpy
from ..camera import Camera


class CameraLucid24Single(Camera):

    def __init__(self, name: str, serial: str):
        super().__init__(
            name,
            self.Const(
                srcs=1,
                raw_format="bayer",
                raw_bits=24,
                width=1440,
                height=928,
                preview_keys=["image"],
            ),
        )
        self.lucid_api = LucidCamera(serial)
        self.wb = np.asarray(
            [
                2.0841475322429894,
                1.0,
                1.9215893014341496,
            ]
        )
        self.buffer_resolve_thread: Optional[threading.Thread] = None
        self.config = self.Config(self)
        self.state.preview_interval = 0.5

    def start_stream(self):
        if self.state.stream_on:
            return
        self.lucid_api.open_stream()
        if self.lucid_api.device is not None:
            self.state.device_on = True

        self.state.stream_on = True
        if (
            self.buffer_resolve_thread is None
            or not self.buffer_resolve_thread.is_alive()
        ):
            self.buffer_resolve_loop_launch()

    def buffer_resolve_loop_launch(self):
        def buffer_resolve_loop():
            while rclpy.ok():
                buffers = self.lucid_api.collect_images()
                buffers.sort(key=lambda x: x.timestamp_ns)

                for buffer in buffers:
                    threading.Thread(
                        target=self.frame_callback,
                        args=(
                            self,
                            self.Frame(
                                buffer.timestamp_ns / 1e9,
                                {
                                    "image": buffer.buffer_np,
                                },
                                {},
                                file_format={"image": "npy"},
                            ),
                        ),
                    ).start()

        self.buffer_resolve_thread = threading.Thread(
            target=buffer_resolve_loop, daemon=True
        )
        self.buffer_resolve_thread.start()

    def stop_stream(self):
        if not self.state.stream_on:
            return
        if (
            self.lucid_api.trigger_thread is not None
            and self.lucid_api.trigger_thread.is_alive()
        ):
            self.lucid_api.trigger_thread_stop.set()
            print("Waiting for stream thread to finish...")
            self.lucid_api.trigger_thread.join()
            self.state.stream_on = False

    def post_process_thumbnail(self, frame, frame_id: str):
        image = frame.reshape(-1, 1440, 3)
        image0 = cv2.cvtColor(image[..., 0], cv2.COLOR_BAYER_RG2RGB)
        image1 = cv2.cvtColor(image[..., 1], cv2.COLOR_BAYER_RG2RGB)
        image2 = cv2.cvtColor(image[..., 2], cv2.COLOR_BAYER_RG2RGB)
        image = (
            image2.astype(np.float32) / 255
            + image1.astype(np.float32) / 255 / 256
            + image0.astype(np.float32) / 255 / 256 / 256
        )
        image *= self.wb.reshape(1, 1, 3)
        image = image ** (1 / 2.2)

        return (image * 255).astype(np.uint8)

    def refresh_config(self):
        self.config.refresh_config()

    def trigger_device(self):
        if not self.state.device_on:
            raise ValueError("Device not connected")
        self.lucid_api.trigger_capture()

    def launch_device(self):
        result = self.lucid_api.connect_device()
        self.state.device_on = result

    def validate_device(self):
        try:
            if self.lucid_api.device is None:
                raise ValueError("Device not connected")
            if not self.lucid_api.device.is_connected():
                raise ValueError("Device not open")
            self.state.device_on = True
            return
        except Exception as e:
            self.state.device_on = False
            raise ValueError(f"Device not connected: {e}")

    class Config(Camera.Config):
        def __init__(self, device):
            self.device: CameraLucid24Single = device
            super().__init__(
                configs={
                    "ExposureTime": self.Param(
                        name="ExposureTime",
                        value=50000,
                        range=[0.1, 100000],
                        type="float",
                        unit="us",
                    ),
                    "Gain": self.Param(
                        name="Gain",
                        value=1.0,
                        range=[1.0, 10],
                        type="float",
                        unit="dB",
                    ),
                }
            )

        def update_config(self, name: str, value: Union[float, bool]):
            param = self.configs[name]
            if param.type == "float":
                value = float(value)
            if param.type == "int":
                value = int(value)
            self.device.lucid_api.device.nodemap.get_node(name).value = value
            self.refresh_config(name)

        def refresh_config(self, name=None):
            if name is not None:
                config = self.configs[name]
                config.value = self.device.lucid_api.device.nodemap.get_node(name).value
            else:
                for name, config in self.configs.items():
                    config.value = self.device.lucid_api.device.nodemap.get_node(
                        name
                    ).value
                    if config.type == "float":
                        config.range = [
                            self.device.lucid_api.device.nodemap.get_node(name).min,
                            self.device.lucid_api.device.nodemap.get_node(name).max,
                        ]
