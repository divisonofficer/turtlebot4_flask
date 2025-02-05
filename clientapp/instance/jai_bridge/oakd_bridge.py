import threading
import cv2
import numpy as np
import depthai as dai
import argparse
from typing import Callable, Optional
from dataclasses import dataclass
from std_msgs.msg import Header
import time


@dataclass
class DepthAIData:
    frameRgb: Optional[np.ndarray]
    frameDisp: np.ndarray
    header: Header

    def __init__(self, frameRgb: np.ndarray, frameDisp: np.ndarray, timestamp_ns: int):
        self.frameRgb = frameRgb
        self.frameDisp = frameDisp
        self.header = Header()
        self.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
        self.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)

    def __repr__(self):
        return f"DepthAIData(frameRgb={self.frameRgb.shape if self.frameRgb is not None else None}, frameDisp={self.frameDisp.shape}, header={self.header} / {time.time()})"


class DepthAICamera:
    def __init__(
        self,
        fps: int = 30,
        resolution: dai.MonoCameraProperties.SensorResolution = dai.MonoCameraProperties.SensorResolution.THE_720_P,
        alpha: Optional[float] = None,
    ):
        self.fps = fps
        self.resolution = resolution
        self.alpha = alpha

        # Callback functions
        self.frame_callback: Callable[[DepthAIData], None]
        self.monoResolution = dai.MonoCameraProperties.SensorResolution.THE_720_P
        # Create pipeline
        self.pipeline = dai.Pipeline()
        self.device = dai.Device()
        self.queueNames = []
        self.time_base = 0

        # Set up the pipeline
        self._setup_pipeline()

        self.flag_kill = threading.Event()

    def _setup_pipeline(self) -> None:
        # Define sources and outputs
        camRgb = self.pipeline.create(dai.node.ColorCamera)
        left = self.pipeline.create(dai.node.MonoCamera)
        right = self.pipeline.create(dai.node.MonoCamera)
        stereo = self.pipeline.create(dai.node.StereoDepth)

        rgbOut = self.pipeline.create(dai.node.XLinkOut)
        disparityOut = self.pipeline.create(dai.node.XLinkOut)

        rgbOut.setStreamName("rgb")
        disparityOut.setStreamName("disp")
        self.queueNames.extend(["rgb", "disp"])

        # Properties
        rgbCamSocket = dai.CameraBoardSocket.CAM_A
        camRgb.setBoardSocket(rgbCamSocket)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
        camRgb.setFps(self.fps)

        stereo.setOutputSize(720, 540)

        left.setResolution(self.monoResolution)
        left.setCamera("left")
        left.setFps(self.fps)
        right.setResolution(self.monoResolution)
        right.setCamera("right")
        right.setFps(self.fps)

        # Set manual focus if available
        try:
            calibData = self.device.readCalibration2()
            lensPosition = calibData.getLensPosition(rgbCamSocket)
            if lensPosition:
                camRgb.initialControl.setManualFocus(lensPosition)
        except Exception as e:
            print(f"Error reading calibration data: {e}")
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setDepthAlign(rgbCamSocket)

        # Linking
        camRgb.video.link(rgbOut.input)
        left.out.link(stereo.left)
        right.out.link(stereo.right)
        stereo.disparity.link(disparityOut.input)

    def set_frame_callback(self, callback: Callable[[DepthAIData], None]) -> None:
        """
        Set a callback function to receive the frames.

        The callback should accept a DepthAIData object containing frameRgb, frameDisp, and timestamp.
        """
        self.frame_callback = callback

    def start(self) -> None:
        # Connect to the device and start the pipeline
        with self.device as device:
            calibration = device.readCalibration()
            print(calibration.getCameraIntrinsics(dai.CameraBoardSocket.RGB))
            print(calibration.getCameraIntrinsics(dai.CameraBoardSocket.LEFT))
            print(calibration.getCameraIntrinsics(dai.CameraBoardSocket.RIGHT))

            self.time_base = time.time() - dai.Clock.now().total_seconds()
            print(f"Time base: {self.time_base}")

            self.device.startPipeline(self.pipeline)

            while True:
                latestPacket = {queueName: None for queueName in self.queueNames}
                queueEvents = self.device.getQueueEvents(self.queueNames)
                for queueName in queueEvents:
                    packets = self.device.getOutputQueue(queueName).tryGetAll()
                    if packets:
                        latestPacket[queueName] = packets[-1]

                frameRgb: Optional[np.ndarray] = None
                frameDisp: Optional[np.ndarray] = None
                timestamp: Optional[int] = None
                timestamp_disp: Optional[int] = None

                if latestPacket["rgb"] is not None:
                    frameRgb = latestPacket["rgb"].getCvFrame()
                    timestamp = latestPacket["rgb"].getTimestamp().total_seconds()
                    timestamp = (timestamp + self.time_base) * 1e9

                if latestPacket["disp"] is not None:
                    frameDisp = latestPacket["disp"].getFrame()
                    timestamp_disp = (
                        latestPacket["disp"].getTimestamp().total_seconds()
                        + self.time_base
                    ) * 1e9

                if (
                    self.frame_callback is not None
                    and frameDisp is not None
                    # and frameRgb is not None
                ):
                    data = DepthAIData(
                        frameRgb=frameRgb,
                        frameDisp=frameDisp,
                        timestamp_ns=timestamp_disp,
                    )
                    frameRgb = None
                    frameDisp = None
                    self.frame_callback(data)

                if cv2.waitKey(1) == ord("q"):
                    break
                if self.flag_kill.is_set():
                    break

    def stop(self) -> None:
        self.flag_kill.set()
        self.device.close()
