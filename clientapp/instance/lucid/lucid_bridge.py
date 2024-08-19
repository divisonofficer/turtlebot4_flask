CAMERA_SERIAL = [224201564, 224201585]
PIXEL_FORMAT = "rgb8"
RESOLUTION = (720, 480)


from crypt import methods
import random
from time import sleep

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import subprocess
import threading
from sensor_msgs.msg import Image, PointCloud2

from stereo_queue import StereoQueue, StereoItemMerged
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from lucid_py_api import LucidPyAPI, LucidImage


class LucidStereoNode(Node):

    def __init__(self):

        super().__init__("lucid_stereo_node")

        self.lucid_api = LucidPyAPI()
        self.lucid_api.connect_device()
        self.lucid_api.open_stream()

        self.trigger_sig = threading.Event()

        self.stereo_queue = StereoQueue[LucidImage, LucidImage](self.stereo_callback)
        self.image_lidar_queue = StereoQueue[StereoItemMerged, PointCloud2](
            self.image_lidar_callback
        )

        self.trigger_clients = [
            self.create_client(Trigger, f"/arena_camera_node_{0}/trigger_image"),
            self.create_client(Trigger, f"/arena_camera_node_{1}/trigger_image"),
        ]

        self.lidar_subscriptions = self.create_subscription(
            PointCloud2,
            "/ouster/points",
            self.lidar_callback,
            QoSProfile(
                depth=2,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
            ),
        )

        threading.Thread(target=self.trigger_loop, daemon=True).start()
        # threading.Thread(target=self.trigger_sig_consumer, daemon=True).start()

    def lidar_callback(self, msg):
        print("lidar_callback")
        self.image_lidar_queue.callback_right(msg)

    def stereo_callback(self, left, right):
        print(
            "stereo_callback, left: ",
            left.header.stamp.sec,
            "right: ",
            right.header.stamp.sec,
        )
        self.image_lidar_queue.callback_left(StereoItemMerged(left, right))

    def image_lidar_callback(self, stereo, lidar):
        print("image_lidar_callback, stereo: ", stereo.header.stamp.sec)

    def trigger_camera_capture(
        self,
    ):
        idxs = [0, 1] if random.randint(0, 1) == 0 else [1, 0]
        self.trigger_clients[0].wait_for_service()
        self.trigger_clients[1].wait_for_service()
        for idx in idxs:

            request = Trigger.Request()

            future = self.trigger_clients[idx].call_async(request)

    def trigger_loop(self):
        while rclpy.ok():
            buffers = self.lucid_api.collect_images()
            for idx, (buffer_left, buffer_right) in enumerate(
                zip(buffers[0], buffers[1])
            ):
                self.stereo_queue.callback_left(buffer_left)
                self.stereo_queue.callback_right(buffer_right)

    def trigger_sig_consumer(self):
        while rclpy.ok():
            self.trigger_sig.wait()
            self.trigger_camera_capture()
            self.trigger_sig.clear()


from flask import Flask, Response
import json

app = Flask(__name__)
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
    print(output)
    return Response(
        status=200,
        response=json.dumps(output),
        content_type="application/json",
    )


def spin_node():
    rclpy.spin(node)


with app.app_context():
    rclpy.init()
    node = LucidStereoNode()
    threading.Thread(target=spin_node, daemon=True).start()


if __name__ == "__main__":
    # spin_node()
    app.run(port=5021)
