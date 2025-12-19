import sys
import os

sys.path.append("/home/cglab/project/turtlebot4_flask/clientapp/instance/lucid/sensors")


from ..camera import Camera

import threading
import cv2
import numpy as np
import pyrealsense2 as rs

import argparse
from typing import Callable, Optional, Union
from dataclasses import dataclass
from std_msgs.msg import Header
import time


@dataclass
class ImageRS(Camera.Frame):
    header: Header

    def __init__(
        self, frameRgb: Optional[np.ndarray], frameDisp: np.ndarray, timestamp_ns: int
    ):
        self.data = {"depth": frameDisp}
        if frameRgb is not None:
            self.data["rgb"] = frameRgb
        self.timestamp = timestamp_ns / 1_000_000_000
        self.attrs = {}
        self.file_format = {"rgb": "png", "depth": "npy"}

    def __repr__(self):
        return f"ImageRS(frameRgb={self.data.get('rgb', None)}, frameDisp={self.data['depth'].shape}, timestamp={self.timestamp})"


class CameraRS(Camera):
    class Config(Camera.Config):
        def __init__(self, device: "CameraRS"):
            self.device = device
            super().__init__(configs={})

        def update_config(self, name: str, value: Union[float, bool]):
            pass

        def refresh_config(self, name=None):
            pass

    def __init__(
        self,
        name: str,
        fps: int = 15,
        resolution: tuple = (1280, 720),
        alpha: Optional[float] = None,
    ):
        const = Camera.Const(
            srcs=1,
            raw_format="depth",
            raw_bits=16,
            width=resolution[0],
            height=resolution[1],
            preview_keys=["depth", "rgb"],
        )
        super().__init__(name, const)

        self.fps = fps
        self.resolution = resolution
        self.alpha = alpha
        self.config = self.Config(self)

        # RealSense pipeline and configuration
        self.pipeline = rs.pipeline()
        self.rsconfig = rs.config()
        self.rsconfig.enable_stream(
            rs.stream.depth, resolution[0], resolution[1], rs.format.z16, fps
        )
        self.rsconfig.enable_stream(
            rs.stream.color, resolution[0], resolution[1], rs.format.bgr8, fps
        )

        self.flag_kill = threading.Event()
        self.stream_thread: Optional[threading.Thread] = None

    def launch_device(self):
        if self.state.device_status not in ["disconnected", "error"]:
            return

        self.state.device_status = "connecting"
        try:
            self.pipeline.start(self.rsconfig)
            self.state.device_status = "connected"
            self.state.stream_on = True
            self.flag_kill.clear()
            self.stream_thread = threading.Thread(target=self._stream_loop)
            self.stream_thread.daemon = True
            self.stream_thread.start()
        except Exception as e:
            print(f"Error starting RealSense pipeline: {e}")
            self.state.device_status = "error"
            self.state.stream_on = False

    def start_stream(self) -> None:
        if not self.state.stream_on:
            self.launch_device()

    def _stream_loop(self) -> None:
        try:
            while not self.flag_kill.is_set():

                try:
                    frames = self.pipeline.wait_for_frames()
                except Exception as e:
                    print(f"Error waiting for frames: {e}")
                    self.state.device_status = "error"
                    self.state.stream_on = False
                    break
                depth_frame = frames.get_depth_frame()
                # intrinsics = (
                #     depth_frame.get_profile().as_video_stream_profile().get_intrinsics()
                # )
                # intrinsic_matrix = np.array(
                #     [
                #         [intrinsics.fx, 0, intrinsics.ppx],
                #         [0, intrinsics.fy, intrinsics.ppy],
                #         [0, 0, 1],
                #     ]
                # )
                # print(f"Intrinsics: {intrinsic_matrix.tolist()}")
                color_frame = frames.get_color_frame()
                # color_frame = None  # Disable color stream for now
                if not depth_frame and not color_frame:
                    continue
                # Convert frames to numpy arrays
                frameDisp = np.asanyarray(depth_frame.get_data()).copy()
                if color_frame is not None:
                    frameRgb = np.asanyarray(color_frame.get_data()).copy()
                else:
                    frameRgb = None

                # Get timestamps
                timestamp_ns = int(frames.get_timestamp() * 1e6)

                # print(timestamp_ns / 1e9, time.time())

                if self.frame_callback:
                    data = ImageRS(
                        frameRgb=frameRgb,
                        frameDisp=frameDisp,
                        timestamp_ns=timestamp_ns,
                    )
                    self.frame_callback(self, data)
        finally:
            self.pipeline.stop()

    def stop_stream(self) -> None:
        """
        Stop the RealSense camera stream.
        """
        if not self.state.stream_on:
            return
        self.flag_kill.set()
        if self.stream_thread is not None:
            self.stream_thread.join(timeout=2)
        self.state.stream_on = False
        self.state.device_status = "disconnected"

    def refresh_config(self):
        self.config.refresh_config()

    def post_process_thumbnail(self, frame, frame_id: str):
        max_depth = 16000
        if frame_id == "depth":
            frame = cv2.resize(frame, (320, 240))
            frame = np.clip(frame, 0, max_depth)
            frame = (frame / max_depth * 255).astype(np.uint8)
            frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
        return frame

    def validate_device(self):
        if self.state.device_status == "connected":
            if self.stream_thread is None or not self.stream_thread.is_alive():
                self.state.device_status = "error"
                self.state.stream_on = False

    def get_latest_frame(self):
        """Get the most recent frame from RealSense camera"""
        try:
            if not self.state.stream_on or self.state.device_status != "connected":
                return None

            # Capture a single frame
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if not depth_frame:
                return None

            # Convert frames to numpy arrays
            frameDisp = np.asanyarray(depth_frame.get_data()).copy()
            frameRgb = None
            if color_frame is not None:
                frameRgb = np.asanyarray(color_frame.get_data()).copy()

            # Get timestamp
            timestamp_ns = int(frames.get_timestamp() * 1e6)

            return ImageRS(
                frameRgb=frameRgb,
                frameDisp=frameDisp,
                timestamp_ns=timestamp_ns,
            )
        except Exception as e:
            print(f"Failed to get latest frame from RealSense: {e}")
            return None


if __name__ == "__main__":
    # Event to signal when a frame is captured
    frame_captured = threading.Event()

    def frame_handler(camera_instance, frame):
        """Callback to handle incoming frames."""
        # print(f"Received frame: {frame}")
        if "rgb" in frame.data:
            cv2.imwrite("sample.png", frame.data["rgb"])
            print("Saved sample.png")
        if "depth" in frame.data:
            disp_data = frame.data["depth"]
            """
            colorize disp map, using magma colormap
            """
            disp_colored = cv2.applyColorMap(
                cv2.convertScaleAbs(disp_data, alpha=0.03), cv2.COLORMAP_MAGMA
            )
            cv2.imwrite("sample_disp.png", disp_colored)

        # Signal that we've received a frame and can exit.
        frame_captured.set()

    # 1. Initialize the camera
    realsense_camera = CameraRS(name="realsense_test")

    # 2. Set the callback
    realsense_camera.register_callback(frame_handler)

    # 3. Start the stream in a background thread
    stream_thread = threading.Thread(target=realsense_camera.start_stream)
    stream_thread.daemon = True
    stream_thread.start()

    print("Camera stream started. Waiting for a frame...")

    # 4. Wait for the callback to signal that a frame has been saved
    captured = frame_captured.wait(timeout=15)  # Wait for 15 seconds

    # Wait for approximately 10 seconds before stopping the stream
    print("Continuing to capture frames for 10 seconds...")
    time.sleep(10)

    # 5. Stop the stream
    realsense_camera.stop_stream()

    if captured:
        print("Frame captured and saved. Stream ran for 10 seconds. Exiting.")
    else:
        print("Failed to capture a frame within the timeout. Exiting.")
