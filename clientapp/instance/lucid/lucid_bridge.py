import random
import sys
from typing import Optional, Union

import cv2
import numpy as np

sys.path.append("../")
sys.path.append("../../")

from sensors.cameras.lucid_24_single import CameraLucid24Single
from sensors.sensor import Sensor
from sensors.lidar.OusterOS1 import SensorOuster
from sensors.camera import Camera
from sensors.cameras.lucid_24 import CameraLucid24
from sensors.sensorgroup import SensorGroup
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

        try:
            self.ouster_bridge = OusterBridge(multi_signal_enhance=(240000, 360000))
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Ouster: {e}")
            self.ouster_bridge = None

        self.queue = SQueue()

        self.storage = StereoStorage()

        self.storage_loop_thread = threading.Thread(target=self.storage.queue_loop)
        # self.lucid_api = LucidPyAPI()
        # self.lucid_api.connect_device()
        # self.queue.register_synchronized_queue("lucid_stereo", is_root=True)
        self.queue.register_synchronized_queue("lucid_left", is_root=True)

        self.queue_loop_thread = threading.Thread(
            target=self.queue.loop_yield_synchronized_items,
            args=(self.queue_callback,),
            daemon=True,
        )

        self.storage_loop_thread.start()
        self.queue_loop_thread.start()
        self.storage_id: Optional[str] = None

        self.status = LucidStatus()

        self.timer = self.create_timer(3, self.status_callback)

        self.stereo_stream = VideoStream()
        # CameraOAK_D("oakd_pro"),
        # CameraLucid24("lucid_stereo", self.lucid_api),

        camera_lucid = [
            CameraLucid24Single("lucid_left", "224201564"),
            CameraLucid24Single("lucid_right", "224201585"),
        ]

        self.sensor_manager = SensorManager(
            [*camera_lucid],
            # + (
            #     [SensorOuster("ouster", self.ouster_bridge)]
            #     if self.ouster_bridge
            #     else []
            # )
            [
                SensorGroup(
                    "lucid_stereo",
                    [
                        SensorGroup.Entity(camera_lucid[0], trigger_sync=True),
                        SensorGroup.Entity(camera_lucid[1], trigger_sync=True),
                    ],
                    5,
                )
            ],
        )

        self.thumb_stream = ThumbStream(self.socket)
        self.sensor_threads = []
        for sensor in self.sensor_manager.sensors:
            self.queue.register_synchronized_queue(sensor.name, is_root=False)

            def callback(sensor: Sensor, data: Sensor.Frame):
                # synchronized queue
                self.queue.enqueue(sensor.name, data.timestamp, data)
                sensor.state.fps = round(
                    max(1 / (data.timestamp - sensor.state.timestamp_last), 0.0001), 3
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
        self.status.storage_enabled = self.storage_id is not None
        self.socket.emit("status", self.status.__dict__())

        if self.status.single_storage_mode:
            self.disable_storage()

    def enable_storage(self):
        self.storage_id = time.strftime("%m_%d_%H_%M", time.localtime(time.time()))
        self.status.storage_queued_cnt = 0

    def disable_storage(self):
        self.storage_id = None

    def __del__(self):
        # del self.lucid_api
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


@app.route("/sensors/groups", methods=["GET"])
def sensor_manger_get_group_list():
    groups = node.sensor_manager.get_all_group_info()
    return Response(status=200, response=json.dumps(groups))


@app.route("/sensors/group/<group_name>/trigger", methods=["POST"])
def sensor_manager_trigger_group(group_name):
    node.sensor_manager.trigger_group(group_name)
    return node.sensor_manager.get_all_group_info()


@app.route("/sensors/group/<group_name>/trigger/loop/start", methods=["POST"])
def sensor_manager_group_trigger_loop_on(group_name):
    node.sensor_manager.group_trigger_loop_on(group_name, True)
    return node.sensor_manager.get_all_group_info()


@app.route("/sensors/group/<group_name>/trigger/loop/stop", methods=["POST"])
def sensor_manager_group_trigger_loop_off(group_name):
    node.sensor_manager.group_trigger_loop_on(group_name, False)
    return node.sensor_manager.get_all_group_info()


@app.route("/sensors/<sensor_name>/stream/start", methods=["POST"])
def sensor_manager_set_stream_on(sensor_name):
    node.sensor_manager.stream_on(sensor_name, True)
    return node.sensor_manager.get_all_sensor_info()


@app.route("/sensors/<sensor_name>/config/<config_name>", methods=["POST"])
def sensor_manager_set_config(sensor_name, config_name):
    value = request.json["value"]
    node.sensor_manager.search_sensor_by_name(sensor_name).update_config(
        config_name, value
    )
    return node.sensor_manager.get_all_sensor_info()


@app.route("/sensors/<sensor_name>/stream/stop", methods=["POST"])
def sensor_manager_set_stream_off(sensor_name):
    node.sensor_manager.stream_on(sensor_name, False)
    return node.sensor_manager.get_all_sensor_info()


@app.route("/sensors/<sensor_name>/preview/on", methods=["POST"])
def sensor_manager_set_preview_on(sensor_name):
    node.sensor_manager.preview_on(sensor_name, True)
    return node.sensor_manager.get_all_sensor_info()


@app.route("/sensors/<sensor_name>/preview/off", methods=["POST"])
def sensor_manager_set_preview_off(sensor_name):
    node.sensor_manager.preview_on(sensor_name, False)
    return node.sensor_manager.get_all_sensor_info()


@app.route("/sensors/all/refresh", methods=["POST"])
def sensor_manager_refresh_all():
    node.sensor_manager.refresh_all()
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
