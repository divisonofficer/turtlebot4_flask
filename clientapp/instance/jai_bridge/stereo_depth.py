from ast import Dict
import sys

sys.path.append("instance/jai_bridge/modules/RAFT_Stereo")

from typing import Literal, Optional
from modules.RAFT_Stereo.core.raft_stereo import RAFTStereo
from modules.RAFT_Stereo.core.utils.utils import InputPadder
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
import torch
import threading
from videostream import VideoStream
import time

import traceback


COLORMAP = cv2.COLORMAP_MAGMA
DISPARITY_MAX = 64
RATIO = (3, 4)


class CameraIntrinsic:
    mtx: np.ndarray
    dist: np.ndarray

    def __init__(self, mtx: np.ndarray, dist: np.ndarray):
        self.mtx = mtx
        self.dist = dist


class StereoCameraParameters:
    def __init__(self, parameter_npz: str):

        parameters = np.load(parameter_npz)
        self.left = CameraIntrinsic(parameters["mtx_left"], parameters["dist_left"])
        self.right = CameraIntrinsic(parameters["mtx_right"], parameters["dist_right"])
        self.R = parameters["R"]
        self.T = parameters["T"]
        self.resolution = (720, 540)
        self.scaled_map_dict: dict[float, tuple] = {}

        rectify_left, rectify_right, proj_left, proj_right, Q, roi_left, roi_right = (
            cv2.stereoRectify(
                self.left.mtx,
                self.left.dist,
                self.right.mtx,
                self.right.dist,
                self.resolution,
                self.R,
                self.T,
            )
        )
        self.map_left_x, self.map_left_y = cv2.initUndistortRectifyMap(
            self.left.mtx,
            self.left.dist,
            rectify_left,
            proj_left,
            self.resolution,
            cv2.CV_32FC1,
        )
        self.map_right_x, self.map_right_y = cv2.initUndistortRectifyMap(
            self.right.mtx,
            self.right.dist,
            rectify_right,
            proj_right,
            self.resolution,
            cv2.CV_32FC1,
        )

    def compute_scaled_map(self, scale):
        if scale in self.scaled_map_dict:
            return self.scaled_map_dict[scale]
        resolution_scaled = (
            int(self.resolution[0] * scale),
            int(self.resolution[1] * scale),
        )
        left_mtx_scaled = np.copy(self.left.mtx)
        left_mtx_scaled[0, 0] *= scale
        left_mtx_scaled[1, 1] *= scale
        left_mtx_scaled[0, 2] *= scale
        left_mtx_scaled[1, 2] *= scale
        right_mtx_scaled = np.copy(self.right.mtx)
        right_mtx_scaled[0, 0] *= scale
        right_mtx_scaled[1, 1] *= scale
        right_mtx_scaled[0, 2] *= scale
        right_mtx_scaled[1, 2] *= scale
        rectify_left, rectify_right, proj_left, proj_right, Q, roi_left, roi_right = (
            cv2.stereoRectify(
                left_mtx_scaled,
                self.left.dist,
                right_mtx_scaled,
                self.right.dist,
                resolution_scaled,
                self.R,
                self.T,
            )
        )
        map_left_x, map_left_y = cv2.initUndistortRectifyMap(
            left_mtx_scaled,
            self.left.dist,
            rectify_left,
            proj_left,
            resolution_scaled,
            cv2.CV_32FC1,
        )
        map_right_x, map_right_y = cv2.initUndistortRectifyMap(
            right_mtx_scaled,
            self.right.dist,
            rectify_right,
            proj_right,
            resolution_scaled,
            cv2.CV_32FC1,
        )
        self.scaled_map_dict[scale] = map_left_x, map_left_y, map_right_x, map_right_y
        return map_left_x, map_left_y, map_right_x, map_right_y


class StereoDepth:
    stream_disparity_viz: Optional[VideoStream]

    def __init__(
        self, model: RAFTStereo, signal_on: threading.Event, signal_off: threading.Event
    ):
        self.model = model
        self.parameter = StereoCameraParameters("instance/jai_bridge/calibration.npz")
        self.signal_on = signal_on
        self.signal_off = signal_off

    def raft_stereo(
        self,
        image_left: cv2.typing.MatLike,
        image_right: cv2.typing.MatLike,
        channel: Literal["rgb", "nir"],
        rectified=False,
    ):
        if self.signal_on.is_set():
            print("raft_stereo: processing is already running")
            return None
        result = None
        time_begin = time.time()
        try:
            self.signal_on.set()
            result = self.process_stereo(
                image_left, image_right, channel, rectified=rectified
            )
        except Exception as e:
            traceback.print_exc()
        finally:
            self.signal_off.clear()
        print("raft_stereo", channel, time.time() - time_begin)
        return result

    def compute_dimension(self, buffer_length):
        width = int(np.sqrt(buffer_length / (RATIO[0] * RATIO[1]))) * RATIO[1]
        height = int(np.sqrt(buffer_length / (RATIO[0] * RATIO[1]))) * RATIO[0]
        return width, height

    def decoding_msg(self, msg: CompressedImage, channel):
        msg_np = np.frombuffer(msg.data, np.uint8)
        width, height = self.compute_dimension(msg_np.shape[0])
        if width % 10 != 0 or height % 10 != 0:
            width, height = self.compute_dimension(msg_np.shape[0] / 3)
        if msg_np.shape[0] % (width * height) != 0:
            raise Exception("invalid buffer length")
        if channel == "nir":
            return msg_np.reshape(height, width)
        elif msg_np.shape[0] == height * width:
            img = msg_np.reshape(height, width)
            return cv2.cvtColor(img, cv2.COLOR_BayerRG2RGB)
        return msg_np.reshape(height, width, 3)

    def process_stereo(
        self,
        image_left: cv2.typing.MatLike,
        image_right: cv2.typing.MatLike,
        channel,
        scale=1,
        rectified=False,
    ):
        if channel == "nir" and len(image_left.shape) == 2:
            image_left = cv2.cvtColor(image_left, cv2.COLOR_GRAY2BGR)
            image_right = cv2.cvtColor(image_right, cv2.COLOR_GRAY2BGR)
        if not rectified:
            left_rect, right_rect = self.rectify_pair(image_left, image_right)
        else:
            left_rect, right_rect = image_left, image_right
        disparity, disparity_color = self.process_stereo_disparity(
            left_rect, right_rect, scale
        )
        return left_rect, right_rect, disparity, disparity_color

    def process_stereo_disparity(self, left_rect, right_rect, scale):
        left_rect_tensor = self.tensorfy(left_rect, scale)
        right_rect_tensor = self.tensorfy(right_rect, scale)

        disparity = self.raft_forward(left_rect_tensor, right_rect_tensor)
        if scale != 1:
            disparity = cv2.resize(
                disparity / scale,
                (left_rect.shape[1] / scale, left_rect.shape[0] / scale),
            )
        disparity_color = self.dispairty_visualization(disparity)
        return disparity, disparity_color

    def raft_forward(self, left_rect_tensor, right_rect_tensor):
        with torch.no_grad():
            padder = InputPadder(left_rect_tensor.shape, divis_by=32)
            left_rect_tensor, right_rect_tensor = padder.pad(
                left_rect_tensor, right_rect_tensor
            )
            _, flow_up = self.model(
                left_rect_tensor, right_rect_tensor, iters=7, test_mode=True
            )
            flow_up = -padder.unpad(flow_up).squeeze()
        return flow_up.cpu().numpy().squeeze()

    def disparity_videostream(self, left_rect, right_rect, disparity_color):
        concatenated_img = np.concatenate(
            (left_rect, right_rect, disparity_color), axis=1
        )
        if self.stream_disparity_viz is not None:
            self.stream_disparity_viz.cv_ndarray_callback(concatenated_img)

    def dispairty_visualization(self, disparity: np.ndarray):
        disparity = (
            np.clip(disparity, 0, DISPARITY_MAX).astype(np.float32)
            * 255.0
            / DISPARITY_MAX
        )
        disparity = cv2.applyColorMap(disparity.astype(np.uint8), COLORMAP)
        return disparity

    def rectify_pair(self, left: np.ndarray, right: np.ndarray):
        map_left_x, map_left_y, map_right_x, map_right_y = (
            self.parameter.compute_scaled_map(1)
        )
        left_rect = cv2.remap(left, map_left_x, map_left_y, cv2.INTER_LINEAR)
        right_rect = cv2.remap(right, map_right_x, map_right_y, cv2.INTER_LINEAR)
        return left_rect, right_rect

    def tensorfy(self, img: np.ndarray, scale=1.0):
        img = img.astype(np.float32)
        img = cv2.resize(img, (int(img.shape[1] * scale), int(img.shape[0] * scale)))
        tensor = torch.from_numpy(img).permute(2, 0, 1)
        return tensor[None].cuda()
