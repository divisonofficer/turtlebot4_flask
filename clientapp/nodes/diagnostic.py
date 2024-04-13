from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Image
import json
import base64
from videostream import VideoStream


def encode_bytes_to_base64(data):
    if isinstance(data, bytes):
        return base64.b64encode(data).decode("utf-8")
    elif isinstance(data, dict):
        return {key: encode_bytes_to_base64(value) for key, value in data.items()}
    elif isinstance(data, list):
        return [encode_bytes_to_base64(element) for element in data]
    else:
        return data


from time import time
from typing import Optional, Callable
import subprocess


class DiagnosticNode(Node):
    __counter: int = 0
    callback: Optional[Callable[[str], None]] = None
    callback_monitor: Optional[Callable[[str], None]] = None

    def __init__(self):
        super().__init__("client_diagnostic_node")
        self.create_timer(8, self.timer_callback)
        self.subscription = self.create_subscription(
            DiagnosticArray, "/diagnostics", self.diagnostic_callback, 10
        )

        self.diagnostic_dict = {}
        self.monitor_dict = {
            "topic_time": time(),
            "topic_interval": 100000,
        }

        self.oakdPreview = VideoStream()
        self.topic_interval = None
        self.oakdSubscription = self.create_subscription(
            Image,
            "/oakd/rgb/preview/image_raw",
            self.oakdPreview.cv_raw_callback,
            10,
        )
        self.oakdPreview.interval_callback = self.oakd_preview_interval_callback

    def diagnostic_callback(self, msg: DiagnosticArray):

        self.monitor_dict["topic_interval"] = time() - self.monitor_dict["topic_time"]
        self.monitor_dict["topic_time"] = time()

        if self.__counter < 1:
            return
        self.counter = 0
        for status in msg.status:
            if type(status) != DiagnosticStatus:
                continue

            values = {}
            for value in status.values:
                if type(value) == KeyValue:
                    values[value.key] = value.value

            self.diagnostic_dict[status.name] = {
                "name": status.name,
                "level": status.level,
                "message": status.message,
                "hardware_id": status.hardware_id,
                "values": values,
            }

        if self.topic_interval:
            self.diagnostic_dict["oakd_preview_interval:Oakd Preview Interval"] = {
                "name": "oakd_preview_interval: Oakd Preview Interval",
                "values": {
                    "topic_interval": self.topic_interval,
                    "yield_interval": self.yield_interval,
                },
            }

    def timer_callback(self):
        self.__counter = 1
        self.get_logger().info("Diagnostic node is running")
        if self.callback:
            self.callback(json.dumps(encode_bytes_to_base64(self.diagnostic_dict)))

        self.monitor_dict["robot_ros_status"] = self.monitor_dict["topic_interval"] < 10
        self.monitor_dict["robot_node_status"] = self.diagnostic_turtlebot_node()

        self.monitor_dict["robot_network_status"] = (
            self.diagnostic_turtlebot_ping()
            if not self.monitor_dict["robot_ros_status"]
            else True
        )

        self.monitor_dict["robot_joy_status"] = self.diagnostic_joy()
        self.monitor_dict["robot_battery"] = self.diagnostic_turtlebot_battery()
        self.monitor_dict["robot_lidar_status"] = self.diagnostic_lidar_running()
        self.monitor_dict["robot_oakd_status"] = self.diagnostic_oakd_running()
        self.monitor_dict["create3_status"] = self.diagnostic_create3_node()

        if self.callback_monitor:
            self.callback_monitor(json.dumps(encode_bytes_to_base64(self.monitor_dict)))

    def diagnostic_turtlebot_node(self):
        nodes = self.get_node_names()
        if "turtlebot4_node" in nodes:
            return True
        return False

    def diagnostic_joy(self):
        if "joy_linux_node: Joystick Driver Status" not in self.diagnostic_dict:
            return False
        return (
            self.diagnostic_dict["joy_linux_node: Joystick Driver Status"]["message"]
            != "Joystick not open."
        )

    def diagnostic_turtlebot_battery(self):
        if "turtlebot4_diagnostics: Battery Percentage" not in self.diagnostic_dict:
            return 0
        return (
            float(
                self.diagnostic_dict["turtlebot4_diagnostics: Battery Percentage"][
                    "values"
                ]["Battery Percentage"]
            )
            * 100
        )

    def diagnostic_lidar_running(self):
        if "turtlebot4_diagnostics: /scan topic status" not in self.diagnostic_dict:
            return False
        return (
            self.diagnostic_dict["turtlebot4_diagnostics: /scan topic status"][
                "message"
            ]
            == "Desired frequency met"
        )

    def diagnostic_oakd_running(self):
        if (
            "turtlebot4_diagnostics: /color/preview/image topic status"
            not in self.diagnostic_dict
        ):
            return False
        return (
            self.diagnostic_dict[
                "turtlebot4_diagnostics: /color/preview/image topic status"
            ]["message"]
            == "Desired frequency met"
        )

    def diagnostic_create3_node(self):
        nodes = self.get_node_names()
        if "motion_control" in nodes:
            return True
        return False

    def diagnostic_turtlebot_ping(self):

        try:
            output = subprocess.check_output(
                ["ping", "-c", "1", "192.168.185.3"], timeout=0.1
            )
            return True
        except subprocess.TimeoutExpired:
            return False

    def oakd_preview_interval_callback(self, topic_interval, yield_interval):
        self.topic_interval = topic_interval
        self.yield_interval = yield_interval
