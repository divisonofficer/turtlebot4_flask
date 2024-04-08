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


class DiagnosticNode(Node):
    __counter = 0

    def __init__(self):
        super().__init__("client_diagnostic_node")
        self.create_timer(20, self.timer_callback)
        self.subscription = self.create_subscription(
            DiagnosticArray, "/diagnostics", self.diagnostic_callback, 10
        )
        self.callback = None
        self.diagnostic_dict = {}

        self.oakdPreview = VideoStream()
        self.oakdSubscription = self.create_subscription(
            Image,
            "/stereo/depth",
            # "/oakd/rgb/preview/image_raw",
            self.oakdPreview.cv_raw_callback,
            10,
        )

    def diagnostic_callback(self, msg: DiagnosticArray):
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

        if self.callback:
            self.callback(json.dumps(encode_bytes_to_base64(self.diagnostic_dict)))

    def timer_callback(self):
        self.__counter = 1
        self.get_logger().info("Diagnostic node is running")
