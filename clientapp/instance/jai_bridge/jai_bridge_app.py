import json
from flask import Flask, Response, request, send_file
from flask_cors import CORS
from flask_socketio import SocketIO

import os
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
import sys
from google.protobuf.json_format import MessageToDict
from time import time, sleep

sys.path.append("../..")
sys.path.append("../../../public/proto/python")
sys.path.append("../public/proto/python")

import logging

logging.basicConfig(
    format="JaiBridgeApp %(message)s",
)
from videostream import VideoStream
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String, Bool
from typing import List, Dict, Any, Optional
import cv2

from jai_pb2 import (
    DeviceInfo,
    ParameterEnum,
    ParameterInfo,
    ParameterValue,
    ParameterUpdate,
    SourceInfo,
)

app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")


from google.protobuf.json_format import MessageToJson

DEVICE_INFO: List[DeviceInfo] = [
    DeviceInfo(
        name="jai_1600_left",
        source_count=2,
        source_types=[
            SourceInfo(
                name="rgb",
                type="bayer_rg8",
                parameters=ParameterUpdate(
                    parameters=[
                        ParameterValue(
                            name="ExposureTime", value="40000", type="float"
                        ),
                        ParameterValue(name="Gain", value="2.0", type="float"),
                        ParameterValue(
                            name="AcquisitionFrameRate", value="2.0", type="float"
                        ),
                        ParameterValue(
                            name="ExposureAutoControlMax", value="100000", type="float"
                        ),
                        ParameterValue(
                            name="ExposureAutoControlMin", value="1000", type="float"
                        ),
                        ParameterValue(name="ExposureAuto", value="0", type="enum"),
                        ParameterValue(name="GainAuto", value="0", type="enum"),
                    ]
                ),
            ),
            SourceInfo(
                name="nir",
                type="mono8",
                parameters=ParameterUpdate(
                    parameters=[
                        ParameterValue(
                            name="ExposureTime", value="60000", type="float"
                        ),
                        ParameterValue(name="Gain", value="3.0", type="float"),
                        ParameterValue(
                            name="ExposureAutoControlMax", value="100000", type="float"
                        ),
                        ParameterValue(
                            name="ExposureAutoControlMin", value="1000", type="float"
                        ),
                        ParameterValue(name="ExposureAuto", value="0", type="enum"),
                        ParameterValue(name="GainAuto", value="0", type="enum"),
                    ]
                ),
            ),
        ],
        fps=4,
        configurable=[
            ParameterInfo(
                name="ExposureTime",
                type="float",
                min=100,
                max=100000,
                source=ParameterInfo.Source.SOURCE,
            ),
            ParameterInfo(
                name="Gain",
                type="float",
                min=1,
                max=16,
                source=ParameterInfo.Source.SOURCE,
            ),
            ParameterInfo(
                name="AcquisitionFrameRate",
                type="float",
                min=0.1,
                max=8,
                source=ParameterInfo.Source.DEVICE,
            ),
            ParameterInfo(
                name="ExposureAutoControlMax",
                type="float",
                min=1000,
                max=100000,
                source=ParameterInfo.Source.SOURCE,
            ),
            ParameterInfo(
                name="ExposureAutoControlMin",
                type="float",
                min=100,
                max=5000,
                source=ParameterInfo.Source.SOURCE,
            ),
            ParameterInfo(
                name="ExposureAuto",
                type="enum",
                source=ParameterInfo.Source.SOURCE,
                enumDefs=[
                    ParameterEnum(
                        index=0,
                        value="Off",
                    ),
                    ParameterEnum(
                        index=2,
                        value="Continuous",
                    ),
                ],
            ),
            ParameterInfo(
                name="GainAuto",
                type="enum",
                source=ParameterInfo.Source.SOURCE,
                enumDefs=[
                    ParameterEnum(
                        index=0,
                        value="Off",
                    ),
                    ParameterEnum(
                        index=2,
                        value="Continuous",
                    ),
                ],
            ),
        ],
    ),
    DeviceInfo(
        name="jai_1600_right",
        source_count=2,
        source_types=[
            SourceInfo(
                name="rgb",
                type="bayer_rg8",
                parameters=ParameterUpdate(
                    parameters=[
                        ParameterValue(
                            name="ExposureTime", value="30000", type="float"
                        ),
                        ParameterValue(name="Gain", value="2.0", type="float"),
                        ParameterValue(
                            name="AcquisitionFrameRate", value="2.0", type="float"
                        ),
                        ParameterValue(
                            name="ExposureAutoControlMax", value="100000", type="float"
                        ),
                        ParameterValue(
                            name="ExposureAutoControlMin", value="1000", type="float"
                        ),
                        ParameterValue(name="ExposureAuto", value="0", type="enum"),
                        ParameterValue(name="GainAuto", value="0", type="enum"),
                    ]
                ),
            ),
            SourceInfo(
                name="nir",
                type="mono8",
                parameters=ParameterUpdate(
                    parameters=[
                        ParameterValue(
                            name="ExposureTime", value="30000", type="float"
                        ),
                        ParameterValue(name="Gain", value="3.0", type="float"),
                        ParameterValue(
                            name="ExposureAutoControlMax", value="100000", type="float"
                        ),
                        ParameterValue(
                            name="ExposureAutoControlMin", value="1000", type="float"
                        ),
                        ParameterValue(name="ExposureAuto", value="0", type="enum"),
                        ParameterValue(name="GainAuto", value="0", type="enum"),
                    ]
                ),
            ),
        ],
        fps=4,
        configurable=[
            ParameterInfo(
                name="ExposureTime",
                type="float",
                min=100,
                max=100000,
                source=ParameterInfo.Source.SOURCE,
            ),
            ParameterInfo(
                name="Gain",
                type="float",
                min=1,
                max=16,
                source=ParameterInfo.Source.SOURCE,
            ),
            ParameterInfo(
                name="AcquisitionFrameRate",
                type="float",
                min=0.1,
                max=8,
                source=ParameterInfo.Source.DEVICE,
            ),
            ParameterInfo(
                name="ExposureAutoControlMax",
                type="float",
                min=1000,
                max=100000,
                source=ParameterInfo.Source.SOURCE,
            ),
            ParameterInfo(
                name="ExposureAutoControlMin",
                type="float",
                min=100,
                max=5000,
                source=ParameterInfo.Source.SOURCE,
            ),
            ParameterInfo(
                name="ExposureAuto",
                type="enum",
                source=ParameterInfo.Source.SOURCE,
                enumDefs=[
                    ParameterEnum(
                        index=0,
                        value="Off",
                    ),
                    ParameterEnum(
                        index=2,
                        value="Continuous",
                    ),
                ],
            ),
            ParameterInfo(
                name="GainAuto",
                type="enum",
                source=ParameterInfo.Source.SOURCE,
                enumDefs=[
                    ParameterEnum(
                        index=0,
                        value="Off",
                    ),
                    ParameterEnum(
                        index=2,
                        value="Continuous",
                    ),
                ],
            ),
        ],
    ),
]


class JaiBridgeNode(Node):

    service_clients: Dict[str, List[Publisher]] = {}
    device_service_clients: Dict[str, Dict[str, Publisher]] = {}
    videoStreams: Dict[str, List[VideoStream]] = {}
    videoStreamSubscriptions: Dict[str, List[Subscription]] = {}
    configureSubscriptions: Dict[str, List[Subscription]] = {}

    configure_dict: Dict[str, List[Dict[str, Any]]] = {}

    def __init__(self):
        super().__init__("jai_bridge_node")  # type: ignore
        self.register_clients()
        # self.create_timer(30, self.get_camera_params)

    def configure_callback_gen(self, device_name: str, channel_id: int):
        def configure_callback(msg: String):
            parameters = ParameterUpdate.FromString(msg.data.encode("utf-8"))

            if device_name not in self.configure_dict:
                self.configure_dict[device_name] = []
            while channel_id >= len(self.configure_dict[device_name]):
                self.configure_dict[device_name].append({})

            device = [x for x in DEVICE_INFO if x.name == device_name][0]

            self.configure_dict[device_name][channel_id]["timestamp"] = time()

            for param in parameters.parameters:

                self.configure_dict[device_name][channel_id][param.name] = {
                    "value": param.value,
                    "type": [x for x in device.configurable if x.name == param.name][
                        0
                    ].type,
                    "name": param.name,
                }

        return configure_callback

    def initialize_camera_config(self):
        self.load_device_info_from_json_file()
        for device in DEVICE_INFO:
            for channel_id, source in enumerate(device.source_types):
                msg = String()
                msg.data = source.parameters.SerializePartialToString().decode("utf-8")
                self.service_clients[device.name][channel_id].publish(msg)
                sleep(2)

    def register_clients(self):
        for device in DEVICE_INFO:
            device_name = device.name
            self.service_clients[device_name] = []
            self.videoStreams[device_name] = []
            self.videoStreamSubscriptions[device_name] = []
            self.configureSubscriptions[device_name] = []
            for i in range(int(device.source_count)):
                self.service_clients[device_name].append(
                    self.create_publisher(
                        String,
                        f"/{device_name}/channel_{i}/manual_configure",
                        10,
                    )
                )

                self.device_service_clients[device_name] = {
                    "open_stream": self.create_publisher(
                        Bool, f"/{device_name}/stream_trigger", 10
                    ),
                    "auto_exposure_hold_trigger": self.create_publisher(
                        Bool, f"/{device_name}/auto_exposure_hold_trigger", 10
                    ),
                }

                self.videoStreams[device_name].append(
                    VideoStream(preview_compress=True, timestampWatermark=True)
                )
                self.videoStreamSubscriptions[device_name].append(
                    self.create_subscription(
                        CompressedImage,
                        f"/{device_name}/channel_{i}",
                        self.videoStreams[device_name][i].cv_raw_callback,
                        1,
                    )
                )

                self.configureSubscriptions[device_name].append(
                    self.create_subscription(
                        String,
                        f"/{device_name}/channel_{i}/device_param",
                        self.configure_callback_gen(device_name, i),
                        10,
                    )
                )
        self.videoStreams["oakd"] = [
            VideoStream(preview_compress=True, timestampWatermark=True)
        ]

        self.videoStreamSubscriptions["oakd"] = [
            self.create_subscription(
                CompressedImage,
                "/oakd/rgb/image_raw/compressed",
                self.videoStreams["oakd"][0].cv_raw_callback,
                1,
            )
        ]

    def publish_configure(
        self,
        device_name: str,
        channel_id: int,
        config_name: str,
        config_type: str,
        config_value: str,
    ):
        msg = String()

        msg_proto = ParameterUpdate()
        msg_proto_parameter = ParameterValue()
        msg_proto_parameter.name = config_name
        msg_proto_parameter.value = config_value
        msg_proto_parameter.type = config_type
        msg_proto.parameters.extend([msg_proto_parameter])
        msg.data = msg_proto.SerializeToString().decode("utf-8")
        self.service_clients[device_name][channel_id].publish(msg)

    def get_camera_params(self):
        for device_name, device_config in self.configure_dict.items():
            for channel_id, channel_config in enumerate(device_config):
                msg = String()
                params = ParameterUpdate()

                for key, value in channel_config.items():
                    if key == "timestamp":
                        continue
                    param = ParameterValue()
                    param.name = key
                    param.value = ""
                    params.parameters.extend([param])
                msg.data = params.SerializeToString().decode("utf-8")
                self.service_clients[device_name][channel_id].publish(msg)
        time_stand = time()
        while self.configure_dict[DEVICE_INFO[0].name][0].get("timestamp") is None or self.configure_dict[DEVICE_INFO[0].name][0].get("timestamp") < time_stand:  # type: ignore
            sleep(1)

            if time() - time_stand > 10:
                return {"status": "error", "message": "Timeout"}
        return self.configure_dict

    def call_service(
        self, device_name: str, service_name: str, request: Optional[Any] = None
    ):
        if service_name == "open_stream":
            ros_request = Bool()
            ros_request.data = request
            self.device_service_clients[device_name][service_name].publish(ros_request)

        if service_name == "auto_exposure_hold_trigger":
            ros_request = Bool()
            request = (request == "true") if type(request) == str else request
            ros_request.data = request
            self.device_service_clients[device_name][service_name].publish(ros_request)

    def device_info_to_json_file(self):
        json.dump(
            [MessageToDict(device) for device in DEVICE_INFO],
            open("device_info.json", "w"),
        )

    def load_device_info_from_json_file(self):
        global DEVICE_INFO
        DEVICE_INFO = []
        with open("device_info.json") as json_file:
            device_info_json = json.load(json_file)
            for device_json in device_info_json:
                device_proto = DeviceInfo()
                ParseDict(device_json, device_proto, ignore_unknown_fields=True)
                DEVICE_INFO.append(device_proto)

        for device in DEVICE_INFO:
            self.configure_dict[device.name] = [
                {
                    param.name: {"value": param.value, "type": param.type}
                    for param in source.parameters.parameters
                }
                for source in device.source_types
            ]

    def update_device_info_state(self):
        global DEVICE_INFO
        for device_name, device_config in self.configure_dict.items():
            device = [x for x in DEVICE_INFO if x.name == device_name][0]

            for channel_id, channel_config in enumerate(device_config):
                for key, value in channel_config.items():
                    source = device.source_types[channel_id]
                    for param in source.parameters.parameters:
                        if param.name == key:
                            param.value = value["value"]
                            break
        self.device_info_to_json_file()

    def get_device_params(self, param_request: list[tuple[str, int, str]]):
        time_begin = time()
        result = []
        for device_name, channel_id, param_name in param_request:
            timestamp = self.configure_dict[device_name][channel_id].get("timestamp")
            while time_begin > timestamp if timestamp else time_begin - 1:
                sleep(1)
                if time() - time_begin > 10:
                    return {"status": "error", "message": "Timeout"}
            result.append(self.configure_dict[device_name][channel_id][param_name])
        return result


node: JaiBridgeNode

import threading


def spin_node():
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()


@app.route("/snapshot/<device_name>/<channel_id>")
def get_camera_snapshot(device_name, channel_id):
    channel_id = channel_id.split("_")[1] if "_" in channel_id else channel_id

    result_image_bytes = cv2.imencode(".png", node.videoStreams[device_name][int(channel_id)].cv_image)[1].tobytes()  # type: ignore

    return Response(
        result_image_bytes,
        mimetype="image/jpeg",
    )


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


from json import JSONDecoder
from google.protobuf.json_format import ParseDict


@app.route("/device")
def get_camera_device_infos():
    serialized = [JSONDecoder().decode(MessageToJson(device)) for device in DEVICE_INFO]

    return serialized


@app.route("/status", methods=["GET"])
def get_camera_params():
    return node.get_camera_params()


@app.route("/device/init/all", methods=["POST"])
def initialize_camera_config():
    node.initialize_camera_config()
    return {"status": "success"}


@app.route("/device/update/all", methods=["POST"])
def update_device_info_state():
    node.update_device_info_state()
    return {"status": "success"}


@app.route("/device/close/all", methods=["POST"])
def close_camera_stream_all():
    for device in DEVICE_INFO:
        node.call_service(device.name, "open_stream", False)
    return {"status": "success"}


@app.route("/device/<device_name>/open_stream", methods=["POST"])
def open_camera_stream(device_name):
    node.call_service(device_name, "open_stream", True)
    return {"status": "success"}


@app.route("/device/<device_name>/close_stream", methods=["POST"])
def close_camera_stream(device_name):
    node.call_service(device_name, "open_stream", False)
    return {"status": "success"}


@app.route("/device/<device_name>/auto_exposure_hold", methods=["POST"])
def auto_exposure_hold(device_name):
    node.call_service(
        device_name,
        "auto_exposure_hold_trigger",
        request.json["hold"] if request.json else False,
    )

    return node.get_camera_params()


@app.route("/device/all/auto_exposure_hold", methods=["POST"])
def auto_exposure_hold_all():
    for device in DEVICE_INFO:
        node.call_service(
            device.name,
            "auto_exposure_hold_trigger",
            request.json["hold"] if request.json else False,
        )

    return node.get_camera_params()


with app.app_context():
    rclpy.init()
    node = JaiBridgeNode()
    spin_node()

    node.initialize_camera_config()


if __name__ == "__main__":
    socketio.run(app, port="5015")  # type: ignore
