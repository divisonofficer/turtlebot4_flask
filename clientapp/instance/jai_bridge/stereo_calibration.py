from os import error
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
    def __init__(
        self,
        ret: float,
        mtx_left: np.ndarray,
        dist_left: np.ndarray,
        rvecs_left: np.ndarray,
        tvecs_left: np.ndarray,
        mtx_right: np.ndarray,
        dist_right: np.ndarray,
        rvect_right: np.ndarray,
        tvect_right: np.ndarray,
        R: np.ndarray,
        T: np.ndarray,
        E: np.ndarray,
        F: np.ndarray,
    ):
        self.ret = ret
        self.mtx_left = mtx_left
        self.dist_left = dist_left
        self.mtx_right = mtx_right
        self.dist_right = dist_right
        self.R = R
        self.T = T
        self.E = E
        self.F = F
        self.rvecs_left = rvecs_left
        self.tvecs_left = tvecs_left
        self.rvecs_right = rvect_right
        self.tvecs_right = tvect_right


CHESS_CELL_WIDTH = 21.9

import os
import json


class CalibrationStorage:
    def __init__(self):
        self.ROOT_FOLDER = "tmp/calibration/"

    def get_colmap_parameter(self, mtx, dist):
        return [
            mtx[0, 0],
            mtx[1, 1],
            mtx[0, 2],
            mtx[1, 2],
            dist[0],
            dist[1],
            dist[2],
            dist[3],
        ]

    def save_calibration(
        self,
        id: int,
        output: CalibrationOutput,
        image_points_left,
        image_points_right,
        image_pairs,
    ):
        folder = f"{self.ROOT_FOLDER}{id}/"
        os.makedirs(folder, exist_ok=True)
        meta_dict = {
            "id": id,
            "timestamp": time.time(),
            "month": time.strftime("%m"),
            "day": time.strftime("%d"),
            "hour": time.strftime("%H"),
            "image_count": len(image_points_left),
            "colmap_left": self.get_colmap_parameter(
                output.mtx_left, output.dist_left[0]
            ),
            "colmap_right": self.get_colmap_parameter(
                output.mtx_right, output.dist_right[0]
            ),
        }
        np.savez(
            f"{folder}calibration.npz",
            ret=output.ret,
            mtx_left=output.mtx_left,
            dist_left=output.dist_left,
            mtx_right=output.mtx_right,
            dist_right=output.dist_right,
            R=output.R,
            T=output.T,
            E=output.E,
            F=output.F,
            left_rvecs=output.rvecs_left,
            left_tvecs=output.tvecs_left,
            right_rvecs=output.rvecs_right,
            right_tvecs=output.tvecs_right,
            image_points_left=image_points_left,
            image_points_right=image_points_right,
        )
        for idx, (img_left, img_right) in enumerate(image_pairs):
            print(f"Saving {folder}{idx}.png")
            cv2.imwrite(f"{folder}left_{idx}.png", img_left)
            cv2.imwrite(f"{folder}right_{idx}.png", img_right)
        json.dump(meta_dict, open(f"{folder}meta.json", "w"))

    def list_calibrations(self):
        calibrations = []
        for folder in os.listdir(self.ROOT_FOLDER):
            if os.path.isdir(f"{self.ROOT_FOLDER}{folder}"):
                meta = json.load(open(f"{self.ROOT_FOLDER}{folder}/meta.json"))
                calibrations.append(meta)
        return calibrations

    def load_calibration(self, id: int):
        folder = f"{self.ROOT_FOLDER}{id}/"
        if not os.path.exists(folder):
            return None
        data = np.load(f"{folder}calibration.npz")
        output = CalibrationOutput(
            ret=data["ret"],
            mtx_left=data["mtx_left"],
            dist_left=data["dist_left"],
            mtx_right=data["mtx_right"],
            dist_right=data["dist_right"],
            R=data["R"],
            T=data["T"],
            E=data["E"],
            F=data["F"],
            rvecs_left=data["left_rvecs"],
            tvecs_left=data["left_tvecs"],
            rvect_right=data["right_rvecs"],
            tvect_right=data["right_tvecs"],
        )

        image_points_left = [x for x in data["image_points_left"]]
        image_points_right = [x for x in data["image_points_right"]]
        image_pairs = []
        for i in range(len(image_points_left)):
            img_left = cv2.imread(f"{folder}left_{id}.png")
            img_right = cv2.imread(f"{folder}right_{id}.png")
            image_pairs.append((img_left, img_right))
        return output, image_points_left, image_points_right, image_pairs

    def get_new_id(self):
        id = 0
        while os.path.exists(f"{self.ROOT_FOLDER}{id}/"):
            id += 1
        return id


class Calibration:
    def __init__(self):
        self.init_calibration_points()

        self.output = None
        self.reprojection_errors_mean = 0.0
        self.reprojection_errors: list[tuple[float, float]] = []
        self.storage = CalibrationStorage()

        self.calibration_id = self.storage.get_new_id()

    def init_calibration_points(self):
        self.objpoints: list[np.ndarray] = []
        self.imgpoints_left: list[np.ndarray] = []
        self.imgpoints_right: list[np.ndarray] = []
        self.chessboard_images: list[tuple[np.ndarray, np.ndarray]] = []
        self.board_size = (12, 6)
        self.objp = np.zeros((self.board_size[0] * self.board_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[
            0 : self.board_size[0], 0 : self.board_size[1]
        ].T.reshape(-1, 2)
        self.objp *= CHESS_CELL_WIDTH

    def append_calibration_points(
        self,
        im_left: np.ndarray,
        corners_left: np.ndarray,
        im_right: np.ndarray,
        corners_right: np.ndarray,
    ):
        self.objpoints.append(self.objp)
        self.imgpoints_left.append(corners_left)
        self.imgpoints_right.append(corners_right)
        self.chessboard_images.append((im_left, im_right))
        if len(self.objpoints) > 24:
            self.objpoints.pop(0)
            self.imgpoints_left.pop(0)
            self.imgpoints_right.pop(0)
            self.chessboard_images.pop(0)

    def compute_reprojection_error(self, rvecs, tvecs, mtx, dist, imgpoints):
        imgpoints2, _ = cv2.projectPoints(self.objpoints[0], rvecs, tvecs, mtx, dist)
        return cv2.norm(imgpoints, imgpoints2, cv2.NORM_L2)

    def compute_reprojection_errors(self):
        mean_error = 0.0
        error_list: list[tuple[float, float]] = []

        if not self.output:
            return []

        while len(self.objpoints) < len(self.output.rvecs_left):
            self.objpoints.append(self.objp)

        for i in range(len(self.objpoints)):
            error_left = self.compute_reprojection_error(
                self.output.rvecs_left[i],
                self.output.tvecs_left[i],
                self.output.mtx_left,
                self.output.dist_left,
                self.imgpoints_left[i],
            )
            error_right = self.compute_reprojection_error(
                self.output.rvecs_right[i],
                self.output.tvecs_right[i],
                self.output.mtx_right,
                self.output.dist_right,
                self.imgpoints_right[i],
            )
            error_list.append((error_left, error_right))
        return mean_error / len(self.objpoints), error_list

    def delete_chessboard_image(self, idx: int):
        if len(self.chessboard_images) > idx:
            self.chessboard_images.pop(idx)
            self.objpoints.pop(idx)
            self.imgpoints_left.pop(idx)
            self.imgpoints_right.pop(idx)
        if len(self.chessboard_images) > 0:
            return self.calibrate(None, None)
        return None

    def calibrate_camera(self, im_left: np.ndarray, im_right: np.ndarray):
        gray_left = cv2.cvtColor(im_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(im_right, cv2.COLOR_BGR2GRAY)
        rect_left, corners_left = cv2.findChessboardCorners(gray_left, self.board_size)
        rect_right, corners_right = cv2.findChessboardCorners(
            gray_right, self.board_size
        )

        if rect_left and rect_right:
            im_left = cv2.drawChessboardCorners(
                im_left, self.board_size, corners_left, rect_left
            )
            im_right = cv2.drawChessboardCorners(
                im_right, self.board_size, corners_right, rect_right
            )
            self.append_calibration_points(
                im_left, corners_left, im_right, corners_right
            )
            return True
        return False

    def calibrate(self, im_left: Optional[np.ndarray], im_right: Optional[np.ndarray]):

        if im_left is not None and im_right is not None:
            if not self.calibrate_camera(im_left, im_right):
                return None, None, None

            print(f"Calibration points: {len(self.objpoints)}")
        image_shape = self.chessboard_images[0][0].shape[:2]
        ret_left, mtx_left, dist_left, rvecs_left, tvecs_left = cv2.calibrateCamera(
            self.objpoints, self.imgpoints_left, image_shape, None, None  # type: ignore
        )  # type: ignore
        (
            ret_right,
            mtx_right,
            dist_right,
            rvecs_right,
            tvecs_right,
        ) = cv2.calibrateCamera(
            self.objpoints, self.imgpoints_right, image_shape, None, None  # type: ignore
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
                image_shape,
            )
        )
        self.output = CalibrationOutput(
            ret,
            mtx_left,
            dist_left,
            rvecs_left,
            tvecs_left,
            mtx_right,
            dist_right,
            rvecs_right,
            tvecs_right,
            R,
            T,
            E,
            F,
        )
        self.reprojection_errors_mean, self.reprojection_errors = (
            self.compute_reprojection_errors()
        )
        return self.output, self.chessboard_images[-1][0], self.chessboard_images[-1][1]

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

    def save_calibration(self):
        if self.output is None:
            return -1
        self.storage.save_calibration(
            self.calibration_id,
            self.output,
            self.imgpoints_left,
            self.imgpoints_right,
            self.chessboard_images,
        )
        return self.calibration_id

    def load_calibration(self, id: int):
        output = self.storage.load_calibration(id)
        if output is None:
            return False
        (
            self.output,
            self.imgpoints_left,
            self.imgpoints_right,
            self.chessboard_images,
        ) = output
        self.calibration_id = id
        self.reprojection_errors_mean, self.reprojection_errors = (
            self.compute_reprojection_errors()
        )
        return True


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

        self.calibrate_signal = threading.Event()

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
        parameter, im_left_chess, im_right_chess = (
            self.calibration.calibrate(im_left, im_right)
            if self.calibrate_signal.is_set()
            else (None, None, None)
        )
        depth = self.calibration.depth(im_left, im_right)
        if im_left_chess is not None and im_right_chess is not None:
            im_left = im_left_chess
            im_right = im_right_chess
        self.emit_calibration_result(parameter, im_left, im_right)
        im_concated = np.concatenate((im_left, im_right), axis=1)
        self.video_stream_raw.cv_ndarray_callback(im_concated)
        if depth is not None:
            self.broadcast_depth_image(depth)
        if parameter is not None:
            print(f"Calibration time: {time.time() - time_begin}")

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

    def jai_callback_left(self, msg: CompressedImage):
        if len(self.msg_left_queue) < 10:
            self.msg_left_queue.append(msg)
        self.pop_queue()

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
            # print(
            #     f"Queue: {time_left - time_right} {len(self.msg_left_queue)} {len(self.msg_right_queue)}"
            # )
            if abs(time_left - time_right) < 0.1:
                self.msg_left_queue.pop(0)
                self.msg_right_queue.pop(0)
                self.calibrate(msg_left, msg_right)
                break

            if time_left < time_right:
                self.msg_left_queue.pop(0)
            else:
                self.msg_right_queue.pop(0)
