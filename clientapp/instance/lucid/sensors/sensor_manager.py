from dataclasses import asdict
from .sensor import Sensor
from .camera import Camera
from typing import List


class SensorManager:

    def __init__(self, sensors: List[Sensor]):
        self.sensors = sensors

    def get_sensor_info(self, sensor: Sensor):
        info = {
            "name": sensor.name,
            "type": "sensor",
            "const": asdict(sensor.const),
            "state": asdict(sensor.state),
        }
        if isinstance(sensor, Camera):
            info["camera_info"] = self.get_camera_info(sensor)
            info["type"] = "camera"
        return info

    def get_camera_info(self, camera: Camera):
        return {
            "config": camera.get_config_status(),
        }

    def get_all_sensor_info(self):
        return [self.get_sensor_info(sensor) for sensor in self.sensors]

    def preview_on(self, sensor_name, on: bool):
        sensor = self.search_sensor_by_name(sensor_name)
        if sensor is not None:
            sensor.state.preview_on = on

    def search_sensor_by_name(self, name: str):
        sensors = [x for x in self.sensors if x.name == name]
        if len(sensors) < 1:
            return None
        return sensors[0]
