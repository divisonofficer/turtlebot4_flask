from flask import Flask, Response, request, send_file
from flask_cors import CORS
from flask_socketio import SocketIO

import os
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
import sys

sys.path.append("../..")
from videostream import VideoStream
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from typing import List, Dict, Any, Optional


app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")


DEVICE_INFO = [
    {
        "name": "jai_1600",
        "source_count": 2,
        "source_types": ["bgr8", "mono8"],
        "fps": 4,
        "configurable": [
            {
                "name": "ExposureTime",
                "type": "float",
                "min": 100,
                "max": 50000,
            },
            {
                "name": "Gain",
                "type": "float",
                "min": 1,
                "max": 16,
            },
        ],
    }
]


class JaiBridgeNode(Node):

    service_clients: Dict[str, List[Publisher]] = {}
    videoStreams: Dict[str, List[VideoStream]] = {}
    videoStreamSubscriptions: Dict[str, List[Subscription]] = {}
    configureSubscriptions: Dict[str, List[Subscription]] = {}

    configure_dict: Dict[str, List[Dict[str, Any]]] = {}

    def __init__(self):
        super().__init__("jai_bridge_node")
        self.register_clients()
        self.create_timer(30, self.get_camera_params)

    def configure_callback_gen(self, device_name: str, channel_id: int):
        def configure_callback(msg: String):
            data_line = msg.data.split(";")
            if device_name not in self.configure_dict:
                self.configure_dict[device_name] = []
            while channel_id >= len(self.configure_dict[device_name]):
                self.configure_dict[device_name].append({})
            for line in data_line:
                config_name, config_value = line.split("=")

                self.configure_dict[device_name][channel_id][config_name] = {
                    "value": config_value,
                    "type": None,
                }

        return configure_callback

    def register_clients(self):
        for device in DEVICE_INFO:
            self.service_clients[str(device["name"])] = []
            self.videoStreams[str(device["name"])] = []
            self.videoStreamSubscriptions[str(device["name"])] = []
            self.configureSubscriptions[str(device["name"])] = []
            for i in range(int(device["source_count"])):
                self.service_clients[str(device["name"])].append(
                    self.create_publisher(
                        String,
                        f"/{device['name']}/channel_{i}/manual_configure",
                        10,
                    )
                )
                self.videoStreams[str(device["name"])].append(
                    VideoStream(preview_compress=True, timestampWatermark=True)
                )
                self.videoStreamSubscriptions[str(device["name"])].append(
                    self.create_subscription(
                        Image,
                        f"/{device['name']}/channel_{i}",
                        self.videoStreams[str(device["name"])][i].cv_raw_callback,
                        10,
                    )
                )

                self.configureSubscriptions[str(device["name"])].append(
                    self.create_subscription(
                        String,
                        f"/{device['name']}/channel_{i}/device_param",
                        self.configure_callback_gen(str(device["name"]), i),
                        10,
                    )
                )

    def publish_configure(
        self,
        device_name: str,
        channel_id: int,
        config_name: str,
        config_type: str,
        config_value: str,
    ):
        msg = String()
        msg.data = f"{config_name}=<{config_type}>{config_value}"
        self.service_clients[device_name][channel_id].publish(msg)

    def get_camera_params(self):
        for device_name, device_config in self.configure_dict.items():
            for channel_id, channel_config in enumerate(device_config):
                msg = String()
                self.service_clients[device_name][channel_id].publish(msg)
        return self.configure_dict


node: JaiBridgeNode

import threading


def spin_node():
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()


@app.route("/preview/<device_name>/<channel_id>/<timestamp>")
def camera_preview(device_name, channel_id, timestamp):
    channel_id = channel_id.split("_")[1] if "_" in channel_id else channel_id
    return Response(
        node.videoStreams[device_name][int(channel_id)].generate_preview(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/device/<device_name>/<channel_id>/configure", methods=["POST"])
def configure_camera(device_name, channel_id):
    data = request.json
    if not data or type(data) != dict:
        return {"status": "error", "message": "Invalid data"}
    key = data.get("key")
    t_type = data.get("type")
    value = data.get("value")
    node.publish_configure(
        device_name, int(channel_id), str(key), str(t_type), str(value)
    )
    return {"status": "success"}


@app.route("/device")
def get_camera_device_infos():
    return DEVICE_INFO


@app.route("/status", methods=["GET"])
def get_camera_params():
    return node.get_camera_params()


with app.app_context():
    rclpy.init()
    node = JaiBridgeNode()
    spin_node()

if __name__ == "__main__":
    socketio.run(app, port="5015")