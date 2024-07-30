import sys

sys.path.append("instance/jai_bridge/modules/RAFT_Stereo")

from typing import Optional, Union
from rclpy.node import Node
from stereo_queue import StereoQueue, StereoItemMerged
from modules.RAFT_Stereo.core.raft_stereo import RAFTStereo
from modules.RAFT_Stereo.core.utils.utils import InputPadder
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import torch
import threading
from videostream import VideoStream
from stereo_storage import StereoCaptureItem, StereoMultiItem, StereoStorage
import time

COLORMAP = cv2.COLORMAP_VIRIDIS


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

    def __init__(
        self, model: RAFTStereo, signal_on: threading.Event, signal_off: threading.Event
    ):
        self.model = model
        self.parameter = StereoCameraParameters("instance/jai_bridge/calibration.npz")
        self.signal_on = signal_on
        self.signal_off = signal_off

    def raft_stereo(
        self, image_left: cv2.typing.MatLike, image_right: cv2.typing.MatLike, channel
    ):
        if self.signal_on.is_set():
            print("raft_stereo: processing is already running")
            return
        result = None
        time_begin = time.time()
        try:
            self.signal_on.set()
            result = self.process_stereo(image_left, image_right, channel)
        except Exception as e:
            traceback.print_exc()
        finally:
            self.signal_off.clear()
        print("raft_stereo", channel, time.time() - time_begin)
        return result

    def decoding_msg(self, msg: CompressedImage, channel):
        msg_np = np.frombuffer(msg.data, np.uint8)
        if channel == "nir":
            return msg_np.reshape(1080, 1440)
        elif msg_np.shape[0] == 1080 * 1440:
            img = msg_np.reshape(1080, 1440)
            return cv2.cvtColor(img, cv2.COLOR_BayerRG2RGB)
        return msg_np.reshape(1080, 1440, 3)

    def process_stereo(
        self,
        image_left: cv2.typing.MatLike,
        image_right: cv2.typing.MatLike,
        channel,
        scale=1,
    ):
        if channel == "nir":
            image_left = cv2.cvtColor(image_left, cv2.COLOR_GRAY2BGR)
            image_right = cv2.cvtColor(image_right, cv2.COLOR_GRAY2BGR)

        left_rect, right_rect = self.rectify_pair(image_left, image_right)
        disparity, disparity_color = self.process_stereo_disparity(
            left_rect, right_rect, scale
        )
        return left_rect, right_rect, disparity, disparity_color

    def process_stereo_disparity(self, left_rect, right_rect, scale):
        left_rect_tensor = self.tensorfy(left_rect, scale)
        right_rect_tensor = self.tensorfy(right_rect, scale)

        disparity = self.raft_forward(left_rect_tensor, right_rect_tensor)
        if scale != 1:
            disparity = cv2.resize(disparity / scale, (1440, 1080))
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
        disparity = np.clip(disparity, 0, 255).astype(np.uint8)
        disparity = cv2.applyColorMap(disparity, COLORMAP)
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

        # self.stereo_queue_merged = StereoQueue[StereoItemMerged](
        #     self,
        #     None,
        #     None,
        #     self.stereoMergedCallback,
        # )

        # self.stereo_queue_viz = StereoQueue[CompressedImage](
        #     self,
        #     "/jai_1600_left/channel_0",
        #     "/jai_1600_right/channel_0",
        #     lambda l, r: self.stereo_queue_merged.callback_left(StereoItemMerged(l, r)),
        # )

        # self.stereo_queue_nir = StereoQueue[CompressedImage](
        #     self,
        #     "/jai_1600_left/channel_1",
        #     "/jai_1600_right/channel_1",
        #     lambda l, r: self.stereo_queue_merged.callback_right(
        #         StereoItemMerged(l, r)
        #     ),
        # )

        self.stereo_merged_subscription = self.create_subscription(
            CompressedImage,
            "/jai_1600_stereo/merged",
            self.stereo_merged_callback,
            10,
        )

        self.__init_raft_stereo()

        self.stereo_storage_id: Optional[str] = None
        self.stereo_storage = StereoStorage()
        self.signal_merged = threading.Event()

        self.color_bar = cv2.applyColorMap(
            np.repeat(
                np.linspace(0, 255, 2160, dtype=np.uint8).reshape(2160, 1), 64, axis=1
            ),
            COLORMAP,
        )

        cv2.putText(
            self.color_bar,
            "0",
            (32, 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            self.color_bar,
            "255",
            (4, 2160 - 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

    def stereo_merged_callback(self, msg: CompressedImage):
        buffer_np = np.frombuffer(msg.data, np.uint8).reshape(4, 1080, 1440)
        self.stereoMergedCallback(
            StereoItemMerged(
                cv2.cvtColor(buffer_np[0], cv2.COLOR_BayerRG2RGB),
                cv2.cvtColor(buffer_np[1], cv2.COLOR_BayerRG2RGB),
            ),
            StereoItemMerged(buffer_np[2], buffer_np[3]),
        )

    def stereoMergedCallback(self, rgb: StereoItemMerged, nir: StereoItemMerged):
        try:
            if self.signal_merged.is_set():
                raise Exception("stereoMergedCallback is already running")
            self.signal_merged.set()
            timestamp = (
                rgb.header.stamp.sec + rgb.header.stamp.nanosec / 1e9
                if not isinstance(rgb.header, dict)
                else rgb.header["stamp"]["sec"]
            )
            result_viz = self.stereo_callback(rgb.left, rgb.right, "viz")
            result_nir = self.stereo_callback(nir.left, nir.right, "nir")

            stereo_multi_item = self.store_stereo_item(
                timestamp, result_viz, result_nir
            )
            if stereo_multi_item is not None:
                self.stream_disparity_viz.cv_ndarray_callback(stereo_multi_item.merged)
                if self.stereo_storage_id is not None:
                    self.stereo_storage.store_item(
                        self.stereo_storage_id, stereo_multi_item
                    )
        except Exception as e:
            traceback.print_exc()
        finally:
            self.signal_merged.clear()
            del rgb
            del nir

    def node_status(self):
        # todo : framerate, queue status, stored_count, left-right sync...
        return {
            "storage_id": self.stereo_storage_id,
            # "queue_status": {
            #     "viz": self.stereo_queue_viz.queue_status(),
            #     "nir": self.stereo_queue_nir.queue_status(),
            #     "merged": self.stereo_queue_merged.queue_status(),
            # },
        }

    def enable_stereo_storage(self, id: Optional[str] = None):
        self.stereo_storage_id = id
        if id is None:
            self.stereo_storage_id = time.strftime("%m-%d-%H-%M-%S")

    def disable_stereo_storage(self):
        self.stereo_storage_id = None

    def store_stereo_item(self, timestamp: float, result_viz, result_nir):
        if result_viz is None or result_nir is None:
            return None
        result_viz = StereoCaptureItem(*result_viz)
        result_nir = StereoCaptureItem(*result_nir)

        merged_viz = np.concatenate(
            (result_viz.left, result_viz.right, result_viz.disparity_color), axis=1
        )
        merged_nir = np.concatenate(
            (result_nir.left, result_nir.right, result_nir.disparity_color), axis=1
        )
        merged = np.concatenate((merged_viz, merged_nir), axis=0)
        merged = np.concatenate((merged, self.color_bar), axis=1)
        return StereoMultiItem(timestamp, result_viz, result_nir, merged)

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
        self.signal_viz = threading.Event()
        self.signal_nir = threading.Event()
        self.signal_nir.set()
        self.stereo_depth_viz = StereoDepth(
            self.raft_stereo, self.signal_viz, self.signal_nir
        )
        self.stereo_depth_nir = StereoDepth(
            self.raft_stereo, self.signal_nir, self.signal_viz
        )

        self.stream_disparity_viz = VideoStream()
        self.stream_disparity_nir = VideoStream()
        self.stereo_depth_viz.stream_disparity_viz = self.stream_disparity_viz
        self.stereo_depth_nir.stream_disparity_viz = self.stream_disparity_nir

    def decoding_msg(self, msg: CompressedImage, channel):
        msg_np = np.frombuffer(msg.data, np.uint8)
        if channel == "nir":
            return msg_np.reshape(1080, 1440)
        elif msg_np.shape[0] == 1080 * 1440:
            img = msg_np.reshape(1080, 1440)
            return cv2.cvtColor(img, cv2.COLOR_BayerRG2RGB)
        return msg_np.reshape(1080, 1440, 3)

    def stereo_callback(
        self,
        msg_left: Union[CompressedImage, cv2.typing.MatLike],
        msg_right: Union[CompressedImage, cv2.typing.MatLike],
        channel,
    ):
        if isinstance(msg_left, CompressedImage):
            msg_left = self.decoding_msg(msg_left, channel)
        if isinstance(msg_right, CompressedImage):
            msg_right = self.decoding_msg(msg_right, channel)
        if channel == "nir":
            return self.stereo_depth_nir.raft_stereo(msg_left, msg_right, channel)
        else:
            return self.stereo_depth_viz.raft_stereo(msg_left, msg_right, channel)
