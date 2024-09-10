import random
import sys
from typing import Optional, Union

sys.path.append("../")


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
        self.lucid_api.open_stream()

        self.ouster_bridge = OusterBridge()

        self.trigger_sig = threading.Event()

        self.stereo_queue = StereoQueue[LucidImage, LucidImage](
            self.stereo_callback, max_queue_length=20
        )
        self.image_lidar_queue = StereoQueue[StereoItemMerged, OusterLidarData](
            self.image_lidar_callback,
            max_queue_length=200,
            hold_right=True,
        )

        self.trigger_clients = [
            self.create_client(Trigger, f"/arena_camera_node_{0}/trigger_image"),
            self.create_client(Trigger, f"/arena_camera_node_{1}/trigger_image"),
        ]
        self.storage = StereoStorage()
        self.trigger_loop_thread = threading.Thread(target=self.trigger_loop)
        self.ouster_loop_thread = threading.Thread(target=self.ouster_loop)
        self.storage_loop_thread = threading.Thread(target=self.storage.queue_loop)
        self.trigger_loop_thread.start()
        self.ouster_loop_thread.start()
        self.storage_loop_thread.start()

        self.storage_id: Optional[str] = None

        self.status = LucidStatus()

        self.timer = self.create_timer(3, self.status_callback)

    def status_callback(self):
        self.status.lidar_queue = self.image_lidar_queue.queue_status()
        self.status.lucid_queue = self.stereo_queue.queue_status()
        self.status.storage_enabled = self.storage_id is not None
        self.socket.emit("status", self.status.__dict__())

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

    def image_lidar_callback(self, stereo: StereoItemMerged, lidar: OusterLidarData):
        print("image_lidar_callback, stereo: ", stereo.header.stamp.sec)
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
            self.stereo_queue.callback_left(buffer_left)
            self.stereo_queue.callback_right(buffer_right)

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


def spin_node():
    rclpy.spin(node)


with app.app_context():
    rclpy.init()
    node = LucidStereoNode(socketio)
    threading.Thread(target=spin_node, daemon=True).start()


if __name__ == "__main__":
    # spin_node()
    socketio.run(app, port=5021, host="0.0.0.0")
