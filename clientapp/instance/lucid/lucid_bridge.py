import random
import sys
from typing import Optional, Union

import cv2
import numpy as np

sys.path.append("../")
sys.path.append("../../")

from sensors.sensor import Sensor
from sensors.lidar.OusterOS1 import SensorOuster
from sensors.camera import Camera
from sensors.cameras.lucid_24 import CameraLucid24
from thumb_stream import ThumbStream
from sensors.cameras.oakd_pro import CameraOAK_D
from sensors.sensor_manager import SensorManager
from videostream import VideoStream
from lucid_storage import StereoMultiItem, StereoCaptureItem
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import threading
from ouster_lidar.ouster_bridge import OusterBridge, OusterLidarData
from stereo_queue import StereoQueue, StereoItemMerged
from lucid_storage import StereoStorage
from lucid_py_api import LucidPyAPI, LucidImage
import time
from flask import Flask, Response, request
from flask_socketio import SocketIO
import json
from flask_cors import CORS
from synchronized_queue import SQueue
from jai_bridge.oakd_bridge import DepthAICamera


class OusterStatus:
    working: bool = True
    exception: Optional[Exception] = None

    def __dict__(self):
        return {
            "working": self.working,
            "exception": (
                self.exception.__repr__() if self.exception is not None else None
            ),
        }


class LucidStatus:
    ouster = OusterStatus()
    lucid_queue: dict = {}
    lidar_queue: dict = {}
    storage_enabled: bool = False
    storage_queued_cnt: int = 0
    single_storage_mode: bool = False

    def __dict__(self):
        return {
            "ouster": self.ouster.__dict__(),
            "lucid_queue": self.lucid_queue,
            "lidar_queue": self.lidar_queue,
            "storage_enabled": self.storage_enabled,
            "storage_queued_cnt": self.storage_queued_cnt,
            "single_storage_mode": self.single_storage_mode,
        }


class LucidStereoNode(Node):

    def __init__(self, socket: SocketIO):
        self.socket = socket
        super().__init__("lucid_stereo_node")
        self.lucid_api = LucidPyAPI()
        self.lucid_api.connect_device()
        # self.lucid_api.open_stream()

        self.ouster_bridge = OusterBridge(multi_signal_enhance=(240000, 360000))

        self.queue = SQueue()

        # self.stereo_queue = StereoQueue[LucidImage, LucidImage](
        #     self.stereo_callback, max_queue_length=20
        # )
        # self.image_lidar_queue = StereoQueue[StereoItemMerged, OusterLidarData](
        #     self.image_lidar_callback,
        #     max_queue_length=200,
        #     hold_right=True,
        # )

        # self.trigger_clients = [
        #     self.create_client(Trigger, f"/arena_camera_node_{0}/trigger_image"),
        #     self.create_client(Trigger, f"/arena_camera_node_{1}/trigger_image"),
        # ]
        self.storage = StereoStorage()
        # self.trigger_loop_thread = threading.Thread(target=self.trigger_loop)

        self.storage_loop_thread = threading.Thread(target=self.storage.queue_loop)
        self.queue.register_synchronized_queue("lucid_stereo", is_root=True)
        self.queue_loop_thread = threading.Thread(
            target=self.queue.loop_yield_synchronized_items,
            args=(self.queue_callback,),
            daemon=True,
        )

        # self.trigger_loop_thread.start()
        # self.ouster_loop_thread = threading.Thread(target=self.ouster_loop)
        # self.ouster_loop_thread.start()
        self.storage_loop_thread.start()
        self.queue_loop_thread.start()
        self.storage_id: Optional[str] = None

        self.status = LucidStatus()

        self.timer = self.create_timer(3, self.status_callback)

        self.stereo_stream = VideoStream()

        self.sensor_manager = SensorManager(
            [
                CameraOAK_D("oakd_pro"),
                CameraLucid24("lucid_stereo", self.lucid_api),
                SensorOuster("ouster", self.ouster_bridge),
            ]
        )

        self.thumb_stream = ThumbStream(self.socket)
        self.sensor_threads = []
        for sensor in self.sensor_manager.sensors:
            self.queue.register_synchronized_queue(sensor.name, is_root=False)

            def callback(sensor: Sensor, data: Sensor.Frame):
                # synchronized queue
                self.queue.enqueue(sensor.name, data.timestamp, data)
                sensor.state.fps = max(
                    1 / (data.timestamp - sensor.state.timestamp_last), 0.0001
                )
                sensor.state.timestamp_last = data.timestamp

                # todo : on/off 추가 필요?
                if (
                    sensor.state.preview_on
                    and (time.time() - sensor.state.preview_last)
                    >= sensor.state.preview_interval
                ):
                    sensor.state.preview_last = time.time()
                    for key in data.data:
                        if key in sensor.const.preview_keys:
                            print(sensor.name, key)
                            self.thumb_stream.yield_thumbnails(
                                data.data[key], sensor, key
                            )

            sensor.register_callback(callback)
            thread = threading.Thread(target=sensor.start_stream, daemon=True)
            self.sensor_threads.append(thread)
            thread.start()

    def queue_callback(self, items):
        print(items.keys())
        if self.storage_id is not None:
            try:
                timestamp = list(items.values())[0].timestamp
                self.storage.enqueue((self.storage_id, timestamp, items))
            except Exception as e:
                print(e)

    def status_callback(self):
        # self.status.lidar_queue = self.image_lidar_queue.queue_status()
        # self.status.lucid_queue = self.stereo_queue.queue_status()
        self.status.storage_enabled = self.storage_id is not None
        self.socket.emit("status", self.status.__dict__())

        if self.status.single_storage_mode:
            self.disable_storage()

    def enable_storage(self):
        self.storage_id = time.strftime("%m_%d_%H_%M", time.localtime(time.time()))
        self.status.storage_queued_cnt = 0

    def disable_storage(self):
        self.storage_id = None

    def lidar_callback(self, msg: Union[OusterLidarData, Exception]):
        if isinstance(msg, Exception):
            print("Lidar error: ", msg)
            self.status.ouster.working = False
            self.status.ouster.exception = msg
            return
        self.status.ouster.working = True
        self.image_lidar_queue.callback_right(msg)

    def stereo_callback(self, left: LucidImage, right: LucidImage):

        self.image_lidar_queue.callback_left(StereoItemMerged(left, right))

    def image_lidar_callback(
        self, stereo: StereoItemMerged, lidar: Optional[OusterLidarData]
    ):
        # print("image_lidar_callback, stereo: ", stereo.header.stamp.sec)

        if self.storage_id is not None:
            self.storage.enqueue(
                StereoMultiItem(
                    self.storage_id,
                    stereo.header.stamp.sec + stereo.header.stamp.nanosec / 1e9,
                    StereoCaptureItem(stereo.left, stereo.right),
                    lidar,
                ),
            )
            self.status.storage_queued_cnt += 1

            if self.status.single_storage_mode:
                self.disable_storage()
        if random.randint(0, 3) == 0:
            """
            Thumbnail stream

            todo : 너무길다 분리하자
            """
            image_left: LucidImage = stereo.left
            image_right: LucidImage = stereo.right
            img_left = image_left.buffer_np.copy().reshape(-1, 1440, 3)[..., 2].astype(
                np.float32
            ) * 255 + image_left.buffer_np.copy().reshape(-1, 1440, 3)[..., 1].astype(
                np.float32
            )
            img_right = image_right.buffer_np.copy().reshape(-1, 1440, 3)[
                ..., 2
            ].astype(np.float32) * 255 + image_right.buffer_np.copy().reshape(
                -1, 1440, 3
            )[
                ..., 1
            ].astype(
                np.float32
            )
            intensity_max = max(img_left.max(), img_right.max()) / 100
            img_left = (img_left / intensity_max * 255).astype(np.uint8)
            img_right = (img_right / intensity_max * 255).astype(np.uint8)

            img_left = cv2.cvtColor(img_left, cv2.COLOR_BAYER_RG2RGB)
            img_right = cv2.cvtColor(img_right, cv2.COLOR_BAYER_RG2RGB)

            img_stereo = np.concatenate((img_left, img_right), axis=1)
            # img_stereo = cv2.resize(img_stereo, (720, 224))
            if lidar:
                points = lidar.points[:].reshape(-1, 3) * 1000
                lidar2cam = self.cam2lidar

                points = (
                    lidar2cam
                    @ np.concatenate([points, np.ones((points.shape[0], 1))], axis=1).T
                ).T

                points = points[:, :3]

                fx = 1346
                cx = 720
                cy = 464

                points[..., 0] = points[..., 0] * fx / points[..., 2] + cx
                points[..., 1] = points[..., 1] * fx / points[..., 2] + cy
                depth_map = np.zeros_like(img_left)

                points = points[
                    (points[..., 0] > 0)
                    & (points[..., 0] < 1440 - 3)
                    & (points[..., 1] > 0)
                    & (points[..., 1] < 928 - 3)
                    & (points[..., 2] > 0)
                ]
                points[..., 2] /= 100

                colors = cv2.applyColorMap(
                    (points[..., 2]).astype(np.uint8), cv2.COLORMAP_JET
                )
                for i in range(9):
                    depth_map[
                        points[..., 1].astype(int) + i % 3,
                        points[..., 0].astype(int) + i // 3,
                    ] = colors.reshape(-1, 3)
                img_stereo = np.concatenate((img_stereo, depth_map), axis=1)

                img_stereo = cv2.resize(img_stereo, (1440 * 3 // 4, 928 // 4))

            self.stereo_stream.cv_ndarray_callback(img_stereo)

    def trigger_camera_capture(
        self,
    ):
        idxs = [0, 1] if random.randint(0, 1) == 0 else [1, 0]
        self.trigger_clients[0].wait_for_service()
        self.trigger_clients[1].wait_for_service()
        for idx in idxs:

            request = Trigger.Request()

            future = self.trigger_clients[idx].call_async(request)

    def buffer_resolve(self, buffers):
        for idx, (buffer_left, buffer_right) in enumerate(zip(buffers[0], buffers[1])):
            timestamp = (
                buffer_left.header.stamp.sec + buffer_left.header.stamp.nanosec / 1e9
            )
            self.queue.enqueue("lucid_left", timestamp, buffer_left)
            self.queue.enqueue("lucid_right", timestamp, buffer_right)
            # self.stereo_queue.callback_left(buffer_left)
            # self.stereo_queue.callback_right(buffer_right)

    def trigger_loop(self):
        while rclpy.ok():
            buffers = self.lucid_api.collect_images()
            threading.Thread(
                target=self.buffer_resolve, args=(buffers,), daemon=True
            ).start()

    def ouster_loop(self):
        self.ouster_bridge.collect_data(self.lidar_callback)

    def __del__(self):
        self.trigger_loop_thread.join()
        self.ouster_loop_thread.join()
        del self.lucid_api
        del self.ouster_bridge


app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

node: LucidStereoNode


@app.route("/trigger", methods=["GET"])
def trigger_capture():
    node.trigger_camera_capture()
    return Response(status=200)


@app.route("/queue_status", methods=["GET"])
def queue_status():
    output = {
        "stereo_queue": node.stereo_queue.queue_status(),
        "lidar_queue": node.image_lidar_queue.queue_status(),
    }
    return Response(
        status=200,
        response=json.dumps(output),
        content_type="application/json",
    )


@app.route("/stream/preview/<timestamp>")
def stereo_disparity_videostream(timestamp):
    return Response(
        node.stereo_stream.generate_preview(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/storage/enable", methods=["POST"])
def enable_storage():
    node.enable_storage()
    return Response(status=200)


@app.route("/storage/disable", methods=["POST"])
def disable_storage():
    node.disable_storage()
    return Response(status=200)


@app.route("/status/update", methods=["POST"])
def update_status():

    attr_dict = request.json if request.json is not None else {}

    for key, value in attr_dict.items():
        if hasattr(node.status, key):
            setattr(node.status, key, value)

    return Response(status=200)


@app.route("/sensors", methods=["GET"])
def sensor_manger_get_sensor_list():
    sensors = node.sensor_manager.get_all_sensor_info()
    return Response(status=200, response=json.dumps(sensors))


@app.route("/sensors/<sensor_name>/preview/on", methods=["POST"])
def sensor_manager_set_preview_on(sensor_name):
    node.sensor_manager.preview_on(sensor_name, True)
    return node.sensor_manager.get_all_sensor_info()


@app.route("/sensors/<sensor_name>/preview/off", methods=["POST"])
def sensor_manager_set_preview_off(sensor_name):
    node.sensor_manager.preview_on(sensor_name, False)
    return node.sensor_manager.get_all_sensor_info()


def spin_node():
    rclpy.spin(node)


with app.app_context():
    rclpy.init()
    node = LucidStereoNode(socketio)
    threading.Thread(target=spin_node, daemon=True).start()


if __name__ == "__main__":
    # spin_node()
    socketio.run(app, port=5021, host="0.0.0.0")
