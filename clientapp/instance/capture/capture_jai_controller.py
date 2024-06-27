from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from std_msgs.msg import Bool
import requests


class CaptureJaiController:

    publisher_jai_trigger: Publisher

    def __init__(self):
        pass

    def open_stream(self):
        self.publisher_jai_trigger.publish(Bool(data=True))

    def close_stream(self):
        self.publisher_jai_trigger.publish(Bool(data=False))

    def freeze_auto_exposure(self):
        self.__hold_jai_autoExposure(True)

    def unfreeze_auto_exposure(self):
        self.__hold_jai_autoExposure(False)

    def __hold_jai_autoExposure(self, hold: bool):
        request = requests.post(
            f"http://localhost/jai/device/all/auto_exposure_hold",
            json={"hold": hold},
        )

        if not request.ok:
            raise Exception("Failed to hold auto exposure")

        if hold:
            return request.json()

    def set_exposure(self, exposure_viz: float, exposure_ir: float):
        for dv_idx in ["jai_1600_left", "jai_1600_right"]:
            for src, exposure in enumerate([exposure_viz, exposure_ir]):
                request = requests.post(
                    f"http://localhost/jai/device/{dv_idx}/{src}/configure",
                    json={"key": "ExposureTime", "value": exposure, "type": "float"},
                )

                if not request.ok:
                    raise Exception("Failed to set exposure")
