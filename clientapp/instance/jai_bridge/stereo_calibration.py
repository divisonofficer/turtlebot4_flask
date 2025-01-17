import imp
import threading
import time
from typing import List, Optional, Tuple, Union

from flask_socketio import SocketIO

from jai_pb2 import CameraMatrix, StereoMatrix
from sensor_msgs.msg import CompressedImage, PointCloud2
from rclpy.node import Node
from rclpy.subscription import Subscription
from videostream import VideoStream, decode_jai_compressedImage
import cv2
import numpy as np
from calibration_type import CalibrationOutput
from calibration import Calibration

from lucid.lucid_py_api import LucidPyAPI, LucidImage
from lucid.lucid_postprocess import LucidPostProcess

from stereo_queue import StereoQueue
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from points2depth import Point2Depth


class JaiStereoCalibration(Node):
    msg_right_queue: list[CompressedImage] = []
    msg_left_queue: list[CompressedImage] = []
    subscription_jai_left: Optional[Subscription] = None
    subscription_jai_right: Optional[Subscription] = None

    def __init__(self, socket: SocketIO):
        super().__init__("jai_stereo_calibration")  # type: ignore

        self.calibration = Calibration()

        self.video_stream_raw = VideoStream(create_subscriber=None)
        self.video_stream_depth = VideoStream(create_subscriber=None)
        self.socket = socket
        self.stop_signal = threading.Event()
        self.start_signal = threading.Event()
        self.start_signal.set()
        self.lidar_points: Optional[PointCloud2] = None

        self.create_timer(3, self.timer_signal_callback)

        self.calibrate_signal = threading.Event()

        self.point2depth = Point2Depth()

        self.max_depth = 20
        self.depth_colormap = cv2.applyColorMap(
            np.arange(256, dtype=np.uint8), cv2.COLORMAP_MAGMA
        )

    def enable_lucid_camera(self):
        if hasattr(self, "lucid_api"):
            return
        self.lucid_api = LucidPyAPI()
        self.lucid_post_process = LucidPostProcess()
        self.lucid_api.connect_device()
        self.lucid_api.open_stream()

        threading.Thread(
            target=self.lucid_api.collect_image_loop, args=(self.lucid_callback,)
        ).start()

    def lucid_callback(self, left: LucidImage, right: LucidImage):

        self.calibrate(
            self.lucid_post_process.rawUint8ToTonemappedBgr(left.buffer_np),
            self.lucid_post_process.rawUint8ToTonemappedBgr(right.buffer_np),
        )

    def init_calibration_by_loading(self, id: int):
        result = self.calibration.load_calibration(id)
        if result:
            self.emit_calibration_result(self.calibration.output, None, None)
        return result

    def timer_signal_callback(self):
        if self.stop_signal.is_set():
            self.deinit_calibration_task()
        if self.start_signal.is_set():
            self.init_calibration_task()

    def is_task_running(self):
        return self.subscription_jai_left is not None

    def init_calibration_task(self):
        if not self.start_signal.is_set():
            return
        self.start_signal.clear()

        if self.subscription_jai_left is not None:
            return

        self.msg_right_queue: list[CompressedImage] = []

        self.create_subscription(
            CompressedImage,
            "/jai_1600_stereo/merged",
            self.stereo_merged_callback,
            10,
        )

    def lidar_callback(self, msg: PointCloud2):
        print("Lidar callback")
        self.lidar_points = msg

    def compute_dimension(self, buffer_length):
        RATIO = (3, 4)
        width = int(np.sqrt(buffer_length / (RATIO[0] * RATIO[1]))) * RATIO[1]
        height = int(np.sqrt(buffer_length / (RATIO[0] * RATIO[1]))) * RATIO[0]
        return width, height

    def stereo_merged_callback(self, msg: CompressedImage):
        buffer_np = np.frombuffer(msg.data, np.uint8)
        width, height = self.compute_dimension(buffer_np.shape[0] / 8)
        buffer_np = buffer_np.reshape(8, height, width)
        left, right = buffer_np[0:3].reshape(height, width, 3), buffer_np[3:6].reshape(
            height, width, 3
        )

        self.calibrate(left, right)

    def update_chessboard_size(
        self, length: Optional[float], shape: Optional[Tuple[int, int]]
    ):
        if length is not None:
            self.calibration.CHESS_CELL_WIDTH = length
        if shape is not None:
            self.calibration.CHESS_SHAPE = shape
        self.calibration.update_objp()

    def update_lidar_RT(self, RT: List[float]):
        if self.calibration.output is None:
            return
        RT_np = np.array(RT)
        if RT_np.shape != (4, 4):
            RT_np = np.concatenate((RT_np, np.array([[0, 0, 0, 1]])), axis=0)

        self.calibration.output.Lidar_RT = RT_np.reshape(4, 4)

    def get_lidar_RT(self):
        if self.calibration.output is None:
            return None
        return self.calibration.output.Lidar_RT.tolist()

    def deinit_calibration_task(self):
        if not self.stop_signal.is_set():
            return
        self.stop_signal.clear()

        if self.subscription_jai_left is not None:
            self.subscription_jai_left.destroy()
            self.subscription_jai_left = None
        if self.subscription_jai_right is not None:
            self.subscription_jai_right.destroy()
            self.subscription_jai_right = None
        time.sleep(3)

    def calibrate(
        self,
        image_left: Union[CompressedImage, np.ndarray],
        image_right: Union[CompressedImage, np.ndarray],
    ):

        time_begin = time.time()
        if isinstance(image_left, CompressedImage):
            im_left = decode_jai_compressedImage(image_left)
        else:
            im_left = image_left
        if isinstance(image_right, CompressedImage):
            im_right = decode_jai_compressedImage(image_right)
        else:
            im_right = image_right
        if im_left.dtype == np.uint16:
            im_left = cv2.normalize(im_left, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)  # type: ignore
            im_right = cv2.normalize(im_right, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)  # type: ignore

        parameter, im_left_chess, im_right_chess = (
            self.calibration.calibrate_camera(im_left, im_right)
            if self.calibrate_signal.is_set()
            else (None, None, None)
        )
        depth = self.calibration.depth(im_left, im_right)
        if im_left_chess is not None and im_right_chess is not None:
            im_left = im_left_chess
            im_right = im_right_chess

        if self.lidar_points is not None:
            projected_left = self.project_lidar_points(self.lidar_points, im_left)
            if projected_left is not None:
                im_left = projected_left
            projected_right = self.project_lidar_points(
                self.lidar_points, im_right, isRight=True
            )
            if projected_right is not None:
                im_right = projected_right

        self.emit_calibration_result(parameter, im_left, im_right)

        im_concated = np.concatenate((im_left, im_right), axis=1)  # type: ignore
        self.video_stream_raw.cv_ndarray_callback(im_concated)
        if depth is not None:
            self.broadcast_depth_image(depth)
        if parameter is not None:
            print(f"Calibration time: {time.time() - time_begin}")

    def project_lidar_points(
        self, lidar: PointCloud2, image: np.ndarray, isRight=False
    ):
        if self.calibration.output is None:
            return
        RT = self.calibration.output.Lidar_RT
        print(RT)
        if isRight:
            self.point2depth.set_intrinsics(self.calibration.output.mtx_right)
            right_t = np.eye(4)
            right_t[:3, 3] = self.calibration.output.T.T
            right_t[:3, :3] = self.calibration.output.R
            RT = np.linalg.inv(RT) @ right_t
        else:
            self.point2depth.set_intrinsics(self.calibration.output.mtx_left)

        self.point2depth.set_transform(RT)
        points, depth = self.point2depth.pointcloud2depth(lidar)
        width, height = image.shape[1], image.shape[0]
        filtered_points = points[
            (points[:, 0] >= 0)
            & (points[:, 0] < width)
            & (points[:, 1] >= 0)
            & (points[:, 1] < height)
        ]
        filtered_depth = depth[
            (points[:, 0] >= 0)
            & (points[:, 0] < width)
            & (points[:, 1] >= 0)
            & (points[:, 1] < height)
        ]

        for i, point in enumerate(filtered_points):
            x, y = point[0], point[1]
            depth = filtered_depth[i] / 1000
            depth = int(min(depth, self.max_depth) / self.max_depth * 255)
            if depth >= 0:
                color = tuple(self.depth_colormap[depth][0].astype(int).tolist())
                cv2.circle(
                    image,
                    (int(x), int(y)),
                    radius=3,
                    color=color,
                    thickness=-1,
                )

        return image

    def emit_calibration_result(
        self,
        parameter: Optional[CalibrationOutput],
        im_left,
        im_right,
    ):
        self.calibrate_signal.clear()
        if parameter is not None and im_left is not None and im_right is not None:
            pass
        else:
            parameter = self.calibration.output
        if parameter is None:
            return
        reprojection_error_left = [x[0] for x in self.calibration.reprojection_errors]
        reprojection_error_right = [x[1] for x in self.calibration.reprojection_errors]
        parameter_proto = StereoMatrix(
            R=parameter.R.flatten().tolist(),
            T=parameter.T.flatten().tolist(),
            E=parameter.E.flatten().tolist(),
            F=parameter.F.flatten().tolist(),
            left=CameraMatrix(
                mtx=parameter.mtx_left.flatten().tolist(),
                dist=parameter.dist_left.flatten().tolist(),
                reproject_error=reprojection_error_left,
            ),
            right=CameraMatrix(
                mtx=parameter.mtx_right.flatten().tolist(),
                dist=parameter.dist_right.flatten().tolist(),
                reproject_error=reprojection_error_right,
            ),
        )
        self.socket.emit("calibration", parameter_proto.SerializeToString())

    def broadcast_depth_image(self, depth):
        if depth.min() == depth.max():
            depth = np.zeros_like(depth)
        else:
            depth_min = 0  # Assign a value to depth_min
            depth_min = np.min(depth[~np.isinf(depth)])
            depth_max = np.max(depth[~np.isinf(depth)])
            depth[np.isinf(depth) & (depth > 0)] = depth_max
            depth[np.isinf(depth) & (depth < 0)] = depth_min

        normalized_depth = cv2.normalize(
            depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U  # type: ignore
        )  # type: ignore
        normalized_depth = cv2.applyColorMap(normalized_depth, cv2.COLORMAP_JET)
        self.video_stream_depth.cv_ndarray_callback(normalized_depth)

    def get_chessboard_image(self, idx: int, side: int):
        if len(self.calibration.chessboard_images) > idx:
            img = self.calibration.chessboard_images[idx][side]
            img_encoded = cv2.imencode(".png", img)[1]
            return img_encoded
        return None

    def delete_chessboard_image(self, idx: int):
        output = self.calibration.delete_chessboard_image(idx)
        if output is not None:
            self.emit_calibration_result(output[0], output[1], output[2])
