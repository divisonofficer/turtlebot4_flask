import sys

sys.path.append("instance/jai_bridge/modules/RAFT_Stereo")

import traceback


from typing import Optional, Union
from rclpy.node import Node
from stereo_queue import StereoItemMerged
from modules.RAFT_Stereo.core.raft_stereo import RAFTStereo
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
import torch
import threading
from videostream import VideoStream
from stereo_storage import StereoCaptureItem, StereoMultiItem, StereoStorage
import time
from stereo_depth import DISPARITY_MAX, StereoDepth, COLORMAP


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
            str(DISPARITY_MAX),
            (4, 2160 - 32),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 0),
            2,
            cv2.LINE_AA,
        )

    def stereo_merged_callback(self, msg: CompressedImage):

        buffer_np = np.frombuffer(msg.data, np.uint8)
        width, height = self.stereo_depth_nir.compute_dimension(buffer_np.shape[0] / 8)
        buffer_np = buffer_np.reshape(8, height, width)
        self.stereoMergedCallback(
            StereoItemMerged(
                buffer_np[0:3].reshape(height, width, 3),
                buffer_np[3:6].reshape(height, width, 3),
                msg.header,
            ),
            StereoItemMerged(buffer_np[6], buffer_np[7], msg.header),
        )

    def stereoMergedCallback(self, rgb: StereoItemMerged, nir: StereoItemMerged):
        try:
            if self.signal_merged.is_set():
                raise Exception("stereoMergedCallback is already running")
            self.signal_merged.set()
            timestamp = rgb.header.stamp.sec + rgb.header.stamp.nanosec / 1e9

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

        size = result_viz.left.shape[:2]

        merged_viz = np.concatenate(
            (result_viz.left, result_viz.right, result_viz.disparity_color), axis=1
        )
        merged_nir = np.concatenate(
            (result_nir.left, result_nir.right, result_nir.disparity_color), axis=1
        )
        merged = np.concatenate((merged_viz, merged_nir), axis=0)
        color_bar_scaled = cv2.resize(
            self.color_bar,
            (int(self.color_bar.shape[1]), size[0] * 2),
            interpolation=cv2.INTER_NEAREST,
        )
        merged = np.concatenate((merged, color_bar_scaled), axis=1)
        return StereoMultiItem(timestamp, result_viz, result_nir, merged)

    def __init_raft_stereo(self):
        class Args:
            def __init__(self):
                self.model = "instance/jai_bridge/modules/RAFT_Stereo/models/raftstereo-realtime.pth"
                self.n_downsample = 3
                self.n_gru_layers = 2
                self.slow_fast_gru = False
                self.valid_iters = 7
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
