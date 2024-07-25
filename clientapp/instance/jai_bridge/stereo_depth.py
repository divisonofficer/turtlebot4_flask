import sys

sys.path.append("instance/jai_bridge/modules/RAFT_Stereo")

from typing import Optional
from rclpy.node import Node
from stereo_queue import StereoQueue
from modules.RAFT_Stereo.core.raft_stereo import RAFTStereo
from modules.RAFT_Stereo.core.utils.utils import InputPadder
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import torch
import threading
from videostream import VideoStream


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
        self.resolution = (1440, 1080)

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


import traceback


class StereoDepth:
    stream_disparity_viz: Optional[VideoStream]

    def __init__(self, model: RAFTStereo):
        self.model = model
        self.parameter = StereoCameraParameters("instance/jai_bridge/calibration.npz")
        self.signal = threading.Event()

    def raft_stereo(
        self, msg_left: CompressedImage, msg_right: CompressedImage, channel
    ):
        if self.signal.is_set():
            return

        try:
            self.signal.set()
            self.process_stereo(msg_left, msg_right, channel)
        except Exception as e:
            traceback.print_exc()
        finally:
            self.signal.clear()

    def decoding_msg(self, msg: CompressedImage, channel):
        msg_np = np.frombuffer(msg.data, np.uint8)
        if channel == "nir":
            return msg_np.reshape(1080, 1440)
        elif msg_np.shape[0] == 1080 * 1440:
            img = msg_np.reshape(1080, 1440)
            return cv2.cvtColor(img, cv2.COLOR_BayerRG2RGB)
        return msg_np.reshape(1080, 1440, 3)

    def process_stereo(
        self, msg_left: CompressedImage, msg_right: CompressedImage, channel
    ):
        image_left = self.decoding_msg(msg_left, channel)
        image_right = self.decoding_msg(msg_right, channel)
        if channel == "nir":
            image_left = cv2.cvtColor(image_left, cv2.COLOR_GRAY2BGR)
            image_right = cv2.cvtColor(image_right, cv2.COLOR_GRAY2BGR)
        else:
            image_left = cv2.cvtColor(image_left, cv2.COLOR_RGB2BGR)
            image_right = cv2.cvtColor(image_right, cv2.COLOR_RGB2BGR)

        left_rect, right_rect = self.rectify_pair(image_left, image_right)

        left_rect_tensor = self.tensorfy(left_rect, 0.5)
        right_rect_tensor = self.tensorfy(right_rect, 0.5)

        with torch.no_grad():
            padder = InputPadder(left_rect_tensor.shape, divis_by=32)
            left_rect_tensor, right_rect_tensor = padder.pad(
                left_rect_tensor, right_rect_tensor
            )
            _, flow_up = self.model(
                left_rect_tensor, right_rect_tensor, iters=7, test_mode=True
            )
            flow_up = -padder.unpad(flow_up).squeeze()

        disparity = flow_up.cpu().numpy().squeeze()
        disparity_color = self.dispairty_visualization(disparity)
        disparity_color = cv2.resize(disparity_color, (1440, 1080))
        self.signal.clear()

        self.disparity_videostream(left_rect, right_rect, disparity_color)

    def disparity_videostream(self, left_rect, right_rect, disparity_color):
        concatenated_img = np.concatenate(
            (left_rect, right_rect, disparity_color), axis=1
        )
        if self.stream_disparity_viz is not None:
            print(concatenated_img.shape)
            self.stream_disparity_viz.cv_ndarray_callback(concatenated_img)

    def dispairty_visualization(self, disparity: np.ndarray):
        disparity = np.clip(disparity, 0, 255).astype(np.uint8)
        disparity = cv2.applyColorMap(disparity, cv2.COLORMAP_JET)
        return disparity

    def rectify_pair(self, left: np.ndarray, right: np.ndarray):
        left_rect = cv2.remap(
            left, self.parameter.map_left_x, self.parameter.map_left_y, cv2.INTER_LINEAR
        )
        right_rect = cv2.remap(
            right,
            self.parameter.map_right_x,
            self.parameter.map_right_y,
            cv2.INTER_LINEAR,
        )
        return left_rect, right_rect

    def tensorfy(self, img: np.ndarray, scale=1.0):
        img = img.astype(np.float32)
        img = cv2.resize(img, (int(img.shape[1] * scale), int(img.shape[0] * scale)))
        tensor = torch.from_numpy(img).permute(2, 0, 1)
        return tensor[None].cuda()


class JaiStereoDepth(Node):

    def __init__(self):
        super().__init__("jai_stereo_depth")  # type: ignore

        self.stereo_queue_viz = StereoQueue(
            self,
            "/jai_1600_left/channel_0",
            "/jai_1600_right/channel_0",
            lambda l, r: self.stereo_callback(l, r, "viz"),
        )

        self.stereo_queue_nir = StereoQueue(
            self,
            "/jai_1600_left/channel_1",
            "/jai_1600_right/channel_1",
            lambda l, r: self.stereo_callback(l, r, "nir"),
        )

        self.__init_raft_stereo()

    def __init_raft_stereo(self):
        class Args:
            def __init__(self):
                self.model = "instance/jai_bridge/modules/RAFT_Stereo/models/raftstereo-realtime.pth"
                self.n_downsample = 3
                self.n_gru_layers = 2
                self.slow_fast_gru = False
                self.valid_iters = 7
                self.corr_imjplementation = "reg_cuda"
                self.mixed_precision = True
                self.hidden_dims = [128] * 3
                self.context_norm = "batch"
                self.corr_levels = 4
                self.corr_radius = 4
                self.shared_backbone = True
                self.corr_implementation = "reg_cuda"

        args = Args()
        print(torch.cuda.is_available())
        model = torch.nn.DataParallel(RAFTStereo(args), device_ids=[0])
        state = torch.load(args.model)
        model.load_state_dict(torch.load(args.model))
        self.raft_stereo = model.module
        self.raft_stereo.cuda()
        self.raft_stereo.eval()

        self.stereo_depth_viz = StereoDepth(self.raft_stereo)
        self.stereo_depth_nir = StereoDepth(self.raft_stereo)

        self.stream_disparity_viz = VideoStream()
        self.stream_disparity_nir = VideoStream()
        self.stereo_depth_viz.stream_disparity_viz = self.stream_disparity_viz
        self.stereo_depth_nir.stream_disparity_viz = self.stream_disparity_nir

    def stereo_callback(
        self, msg_left: CompressedImage, msg_right: CompressedImage, channel
    ):
        if channel == "nir":
            self.stereo_depth_nir.raft_stereo(msg_left, msg_right, channel)
        else:
            self.stereo_depth_viz.raft_stereo(msg_left, msg_right, channel)
