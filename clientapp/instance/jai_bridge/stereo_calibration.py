import threading
import time
from typing import Optional, Tuple

from flask_socketio import SocketIO
from jai_pb2 import CameraMatrix, StereoMatrix
from sensor_msgs.msg import CompressedImage
from rclpy.node import Node
from rclpy.subscription import Subscription
from videostream import VideoStream, decode_jai_compressedImage
import cv2
import numpy as np
from calibration_type import CalibrationOutput
from calibration import Calibration


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

    def update_chessboard_size(
        self, length: Optional[float], shape: Optional[Tuple[int, int]]
    ):
        if length is not None:
            self.calibration.CHESS_CELL_WIDTH = length
        if shape is not None:
            self.calibration.CHESS_SHAPE = shape
        self.calibration.update_objp()

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
            self.calibration.calibrate_camera(im_left, im_right)
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
