import threading
import time
from typing import Optional

from flask_socketio import SocketIO
from jai_pb2 import CameraMatrix, StereoMatrix
from sensor_msgs.msg import CompressedImage
import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from cv_bridge import CvBridge
from videostream import VideoStream, decode_jai_compressedImage
import cv2
import numpy as np


class CalibrationOutput:
    def __init__(self, ret, mtx_left, dist_left, mtx_right, dist_right, R, T, E, F):
        self.ret = ret
        self.mtx_left = mtx_left
        self.dist_left = dist_left
        self.mtx_right = mtx_right
        self.dist_right = dist_right
        self.R = R
        self.T = T
        self.E = E
        self.F = F


CHESS_CELL_WIDTH = 21.9


class Calibration:
    def __init__(self):
        self.init_calibration_points()

        self.output = None

    def init_calibration_points(self):
        self.objpoints: list[np.ndarray] = []
        self.imgpoints_left: list[np.ndarray] = []
        self.imgpoints_right: list[np.ndarray] = []
        self.board_size = (12, 6)
        self.objp = np.zeros((self.board_size[0] * self.board_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[
            0 : self.board_size[0], 0 : self.board_size[1]
        ].T.reshape(-1, 2)
        self.objp *= CHESS_CELL_WIDTH

    def calibrate(self, im_left: np.ndarray, im_right: np.ndarray):

        gray_left = cv2.cvtColor(im_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(im_right, cv2.COLOR_BGR2GRAY)
        rect_left, corners_left = cv2.findChessboardCorners(gray_left, self.board_size)
        rect_right, corners_right = cv2.findChessboardCorners(
            gray_right, self.board_size
        )

        if rect_left and rect_right:

            self.objpoints.append(self.objp)
            self.imgpoints_left.append(corners_left)
            self.imgpoints_right.append(corners_right)

            if len(self.objpoints) > 24:
                self.objpoints.pop(0)
                self.imgpoints_left.pop(0)
                self.imgpoints_right.pop(0)

            print(f"Calibration points: {len(self.objpoints)}")

            ret_left, mtx_left, dist_left, rvecs_left, tvecs_left = cv2.calibrateCamera(
                self.objpoints, self.imgpoints_left, gray_left.shape[::-1], None, None  # type: ignore
            )  # type: ignore
            (
                ret_right,
                mtx_right,
                dist_right,
                rvecs_right,
                tvecs_right,
            ) = cv2.calibrateCamera(
                self.objpoints, self.imgpoints_right, gray_right.shape[::-1], None, None  # type: ignore
            )
            ret, mtx_left, dist_left, mtx_right, dist_right, R, T, E, F = (
                cv2.stereoCalibrate(
                    self.objpoints,
                    self.imgpoints_left,
                    self.imgpoints_right,
                    mtx_left,
                    dist_left,
                    mtx_right,
                    dist_right,
                    gray_left.shape[::-1],
                )
            )
            self.output = CalibrationOutput(
                ret, mtx_left, dist_left, mtx_right, dist_right, R, T, E, F
            )
        return self.output, corners_left, corners_right

    def depth(self, image_left, image_right):
        if not self.output:
            return None
        gray_left = cv2.cvtColor(image_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(image_right, cv2.COLOR_BGR2GRAY)
        rectify_left, rectify_right, proj_left, proj_right, Q, roi_left, roi_right = (
            cv2.stereoRectify(
                self.output.mtx_left,
                self.output.dist_left,
                self.output.mtx_right,
                self.output.dist_right,
                gray_left.shape[::-1],
                self.output.R,
                self.output.T,
            )
        )
        map_left_x, map_left_y = cv2.initUndistortRectifyMap(
            self.output.mtx_left,
            self.output.dist_left,
            rectify_left,
            proj_left,
            gray_left.shape[::-1],
            cv2.CV_32FC1,
        )
        map_right_x, map_right_y = cv2.initUndistortRectifyMap(
            self.output.mtx_right,
            self.output.dist_right,
            rectify_right,
            proj_right,
            gray_right.shape[::-1],
            cv2.CV_32FC1,
        )
        left_rectified = cv2.remap(gray_left, map_left_x, map_left_y, cv2.INTER_LINEAR)
        right_rectified = cv2.remap(
            gray_right, map_right_x, map_right_y, cv2.INTER_LINEAR
        )
        stereo = cv2.StereoBM.create(numDisparities=16, blockSize=15)
        disparity = stereo.compute(left_rectified, right_rectified)
        focal_length = proj_left[0, 0]
        baseline = np.linalg.norm(self.output.T)
        disparity[disparity <= 0] = 1e-6
        depth = focal_length * baseline / disparity
        return depth


class JaiStereoCalibration(Node):
    msg_right_queue: list[CompressedImage] = []
    msg_left_queue: list[CompressedImage] = []
    subscription_jai_left: Optional[Subscription] = None
    subscription_jai_right: Optional[Subscription] = None

    def __init__(self, socket: SocketIO):
        super().__init__("jai_stereo_calibration")  # type: ignore

        self.video_stream_raw = VideoStream(create_subscriber=None)
        self.video_stream_depth = VideoStream(create_subscriber=None)
        self.socket = socket
        self.stop_signal = threading.Event()
        self.start_signal = threading.Event()

        self.create_timer(3, self.timer_signal_callback)

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

        self.subscription_jai_left = self.create_subscription(
            CompressedImage,
            "/jai_1600_left/channel_0",
            self.jai_callback_left,
            1,
        )
        self.subscription_jai_right = self.create_subscription(
            CompressedImage,
            "/jai_1600_right/channel_0",
            self.jai_callback_right,
            1,
        )

        self.calibration = Calibration()

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

    def calibrate(self, image_left: CompressedImage, image_right: CompressedImage):
        print(f"Calibrate on {image_left.header.stamp.sec}")
        time_begin = time.time()
        im_left = decode_jai_compressedImage(image_left)
        im_right = decode_jai_compressedImage(image_right)
        if im_left.dtype == np.uint16:
            im_left = cv2.normalize(im_left, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)  # type: ignore
            im_right = cv2.normalize(im_right, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)  # type: ignore
        parameter, lCorner, rCorner = self.calibration.calibrate(im_left, im_right)
        depth = self.calibration.depth(im_left, im_right)
        if parameter is not None:
            cv2.drawChessboardCorners(im_left, self.calibration.board_size, lCorner, True)  # type: ignore
            cv2.drawChessboardCorners(im_right, self.calibration.board_size, rCorner, True)  # type: ignore

            im_concated = np.concatenate((im_left, im_right), axis=1)
            self.video_stream_raw.cv_ndarray_callback(im_concated)

            parameter_proto = StereoMatrix(
                R=parameter.R.flatten().tolist(),
                T=parameter.T.flatten().tolist(),
                E=parameter.E.flatten().tolist(),
                F=parameter.F.flatten().tolist(),
                left=CameraMatrix(
                    mtx=parameter.mtx_left.flatten().tolist(),
                    dist=parameter.dist_left.flatten().tolist(),
                ),
                right=CameraMatrix(
                    mtx=parameter.mtx_right.flatten().tolist(),
                    dist=parameter.dist_right.flatten().tolist(),
                ),
            )

            self.socket.emit("calibration", parameter_proto.SerializeToString())
        if depth is not None:
            print(depth.shape, depth.min(), depth.max(), depth.mean())
            if depth.min() == depth.max():
                depth = np.zeros_like(depth)
            else:
                depth_min = 0  # Assign a value to depth_min
                depth_min = np.min(depth[~np.isinf(depth)])
                depth_max = np.max(depth[~np.isinf(depth)])
                depth[np.isinf(depth) & (depth > 0)] = depth_max
                depth[np.isinf(depth) & (depth < 0)] = depth_min

            print(depth.shape, depth.min(), depth.max(), depth.mean())
            normalized_depth = cv2.normalize(
                depth, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U
            )
            normalized_depth = cv2.applyColorMap(normalized_depth, cv2.COLORMAP_JET)
            self.video_stream_depth.cv_ndarray_callback(normalized_depth)
        print(f"Calibration time: {time.time() - time_begin}")

    def jai_callback_left(self, msg: CompressedImage):
        if len(self.msg_left_queue) < 10:
            self.msg_left_queue.append(msg)
        self.pop_queue()

    def jai_callback_right(self, msg: CompressedImage):
        if len(self.msg_right_queue) < 10:
            self.msg_right_queue.append(msg)
        self.pop_queue()

    def pop_queue(self):
        while len(self.msg_left_queue) > 0 and len(self.msg_right_queue) > 0:
            msg_left = self.msg_left_queue[0]
            msg_right = self.msg_right_queue[0]
            time_left = msg_left.header.stamp.sec + msg_left.header.stamp.nanosec / 1e9
            time_right = (
                msg_right.header.stamp.sec + msg_right.header.stamp.nanosec / 1e9
            )
            print(
                f"Queue: {time_left - time_right} {len(self.msg_left_queue)} {len(self.msg_right_queue)}"
            )
            if abs(time_left - time_right) < 0.1:
                self.msg_left_queue.pop(0)
                self.msg_right_queue.pop(0)
                self.calibrate(msg_left, msg_right)
                break

            if time_left < time_right:
                self.msg_left_queue.pop(0)
            else:
                self.msg_right_queue.pop(0)
