import os
from random import randint
import sys

from flask_socketio import SocketIO
import tqdm

from points2depth import Point2Depth

sys.path.append("instance/jai_bridge/modules/RAFT_Stereo")

import traceback


from typing import Optional, Tuple, Union
from rclpy.node import Node
from lucid.stereo_queue import StereoItemMerged, StereoQueue
from modules.RAFT_Stereo.core.raft_stereo import RAFTStereo
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage, PointCloud2
import torch
import threading
from videostream import VideoStream
from stereo_storage import StereoCaptureItem, StereoMultiItem, StereoStorage
import time
from stereo_depth import DISPARITY_MAX, StereoCameraParameters, StereoDepth, COLORMAP

from ouster_lidar.ouster_bridge import OusterBridge, OusterLidarData


class JaiStereoDepth(Node):

    def __init__(self, socket: SocketIO):
        super().__init__("jai_stereo_depth")  # type: ignore

        self.stereo_lidar_queue = StereoQueue[CompressedImage, OusterLidarData](
            self.stereo_lidar_callback,
            time_diff=0.1,
            max_queue_length=300,
            hold_right=True,
        )
        self.socket = socket
        try:
            self.lidar_bridge = OusterBridge()

            threading.Thread(
                target=self.lidar_bridge.collect_data,
                args=(self.ouster_lidar_callback,),
            ).start()

            self.stereo_merged_subscription = self.create_subscription(
                CompressedImage,
                "/jai_1600_stereo/merged",
                self.stereo_lidar_queue.callback_left,
                10,
            )
            self.stereo_merged_subscription

        except Exception as e:
            print(e)
        self.__init_raft_stereo()
        self.points2depth = Point2Depth()

        self.stereo_storage_id: Optional[str] = None
        self.stored_frame_cnt = 0
        self.prev_timestamp = time.time()
        self.frame_rate = 0.0
        self.stereo_storage = StereoStorage()
        threading.Thread(target=self.stereo_storage.queue_loop).start()
        self.signal_merged = threading.Event()

        self.timer = self.create_timer(5, self.node_status)

    def ouster_lidar_callback(self, msg: Union[OusterLidarData, Exception]):
        if isinstance(msg, Exception):
            print("Lidar error: ", msg)
            if self.stereo_storage_id is not None:
                self.enable_stereo_storage()
            return
        self.stereo_lidar_queue.callback_right(msg)

    def stereo_lidar_callback(self, msg: CompressedImage, lidar: OusterLidarData):
        rgb, nir = self.unpack_stereo_msg(msg)
        self.merged_frame_callback(rgb, nir, lidar)

    def unpack_stereo_msg(self, msg: CompressedImage):

        buffer_np = np.frombuffer(msg.data, np.uint8)
        width, height = self.stereo_depth_nir.compute_dimension(buffer_np.shape[0] / 8)
        buffer_np = buffer_np.reshape(8, height, width)
        exposure_times = [
            float(x) for x in msg.header.frame_id.split("stereo_")[-1].split("_")
        ]

        return (
            StereoItemMerged[Tuple[cv2.typing.MatLike, float]](
                (buffer_np[0:3].reshape(height, width, 3), exposure_times[0]),
                (buffer_np[3:6].reshape(height, width, 3), exposure_times[2]),
                msg.header,
            ),
            StereoItemMerged[Tuple[cv2.typing.MatLike, float]](
                (buffer_np[6], exposure_times[1]),
                (buffer_np[7], exposure_times[3]),
                msg.header,
            ),
        )

    def merged_frame_callback(
        self,
        rgb: StereoItemMerged[Tuple[cv2.typing.MatLike, float]],
        nir: StereoItemMerged[Tuple[cv2.typing.MatLike, float]],
        lidar: Optional[OusterLidarData] = None,
        disparity_matching=False,
    ):
        try:
            if self.signal_merged.is_set():
                raise Exception("stereoMergedCallback is already running")
            self.signal_merged.set()
            timestamp = rgb.header.stamp.sec + rgb.header.stamp.nanosec / 1e9
            if disparity_matching:
                result_viz = self.stereo_callback(rgb.left[0], rgb.right[0], "viz")
                result_nir = self.stereo_callback(nir.left[0], nir.right[0], "nir")
            else:
                result_viz = self.stereo_depth_viz.rectify_pair(
                    rgb.left[0], rgb.right[0]
                )
                result_nir = self.stereo_depth_nir.rectify_pair(
                    nir.left[0], nir.right[0]
                )
                # result_nir = (
                #     np.repeat(result_nir[0][:, :, np.newaxis], 3, axis=2),
                #     np.repeat(result_nir[1][:, :, np.newaxis], 3, axis=2),
                # )
            stereo_multi_item = self.wrap_stereo_multi_item(
                timestamp, result_viz, result_nir, lidar
            )
            if stereo_multi_item is not None:
                if randint(0, 10) == 0:
                    self.stream_disparity_viz.cv_ndarray_callback(
                        np.concatenate(
                            [
                                rgb.left[0],
                                np.repeat(
                                    nir.left[0][:, :, np.newaxis],
                                    3,
                                    axis=2,
                                ),
                            ],
                            axis=1,
                        )
                    )
                stereo_multi_item.rgb.exposure_left = rgb.left[1]
                stereo_multi_item.rgb.exposure_right = rgb.right[1]
                stereo_multi_item.nir.exposure_left = nir.left[1]
                stereo_multi_item.nir.exposure_right = nir.right[1]

                if self.stereo_storage_id is not None:
                    stereo_multi_item.id = self.stereo_storage_id
                    self.stereo_storage.enqueue(stereo_multi_item)
                    self.stored_frame_cnt += 1
            self.frame_rate = 1.0 / (time.time() - self.prev_timestamp)
            self.prev_timestamp = time.time()

        except Exception as e:
            traceback.print_exc()
        finally:
            self.signal_merged.clear()
            del stereo_multi_item

    def node_status(self):
        # todo : framerate, queue status, stored_count, left-right sync...
        status_dict = {
            "storage_id": self.stereo_storage_id,
            "queue_status": {
                "merged": self.stereo_lidar_queue.queue_status(),
            },
            "storage_status": {
                "stored_frame_cnt": self.stored_frame_cnt,
                "frame_rate": self.frame_rate,
                "queue_length": len(self.stereo_storage.storage_queue),
                "item_store_time": self.stereo_storage.item_store_time,
            },
        }
        self.socket.emit("stereo_status", status_dict)
        return status_dict

    def load_calibration(self, id: str):
        path = f"tmp/calibration/{id}/calibration.npz"
        self.stereo_depth_viz.parameter = StereoCameraParameters(path)
        self.stereo_depth_nir.parameter = StereoCameraParameters(path)

    def enable_stereo_storage(self, id: Optional[str] = None):
        self.stereo_storage_id = id
        self.prev_timestamp = time.time()
        self.stored_frame_cnt = 0
        if id is None:
            self.stereo_storage_id = time.strftime("%m-%d-%H-%M-%S")

    def disable_stereo_storage(self):
        self.stereo_storage_id = None

    def wrap_stereo_multi_item(
        self, timestamp: float, result_viz, result_nir, lidar: Optional[OusterLidarData]
    ):
        if result_viz is None or result_nir is None:
            return None
        result_viz = StereoCaptureItem(*result_viz)
        result_nir = StereoCaptureItem(*result_nir)
        return StereoMultiItem(
            timestamp,
            result_viz,
            result_nir,
            None,
            self.stereo_depth_nir.parameter.get_scaled_calibration_dict(
                result_viz.left.shape[0]
            ),
            lidar,
        )

    def npz_to_h5(self, scene_id: str, root: str):
        if hasattr(self, "npzh5_thread") and self.npzh5_thread.is_alive():
            raise Exception("npz_to_h5 is already running")

        if os.path.exists(os.path.join(root, scene_id, "0.hdf5")):
            raise Exception("HDF5 file already exists")

        def progress_callback(progress: float):
            self.socket.emit("npz_h5_progress", {"progress": progress})

        self.npzh5_thread = threading.Thread(
            target=self.stereo_storage.npz_to_h5,
            args=(scene_id, root, progress_callback),
        )
        self.npzh5_thread.start()

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

    def post_process_disparity_matching_scene(
        self, scene_folder: str, root="tmp/depth"
    ):
        frames = self.stereo_storage.read_storage_scene(scene_folder, root)
        for frame in tqdm.tqdm(frames):
            self.post_process_disparity_matching(
                os.path.join(root, scene_folder, frame)
            )

    def post_process_disparity_matching(self, frame_folder: str):
        img_left = cv2.imread(
            os.path.join(frame_folder, "rgb/left.png"), cv2.IMREAD_ANYCOLOR
        )
        img_right = cv2.imread(
            os.path.join(frame_folder, "rgb/right.png"), cv2.IMREAD_ANYCOLOR
        )
        img_nir_left = cv2.imread(
            os.path.join(frame_folder, "nir/left.png"), cv2.IMREAD_GRAYSCALE
        )
        img_nir_right = cv2.imread(
            os.path.join(frame_folder, "nir/right.png"), cv2.IMREAD_GRAYSCALE
        )
        result = self.stereo_depth_viz.raft_stereo(
            img_left, img_right, "rgb", rectified=True
        )
        if result is not None:
            _, _, disparity, disparity_color = result
            np.savez(os.path.join(frame_folder, "rgb/disparity.npz"), disparity)
            cv2.imwrite(
                os.path.join(frame_folder, "rgb/disparity_color.png"), disparity_color
            )
        result = self.stereo_depth_nir.raft_stereo(
            img_nir_left, img_nir_right, "nir", rectified=True
        )
        if result is not None:
            _, _, disparity, disparity_color = result
            np.savez(os.path.join(frame_folder, "nir/disparity.npz"), disparity)
            cv2.imwrite(
                os.path.join(frame_folder, "nir/disparity_color.png"), disparity_color
            )

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
