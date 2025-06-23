from dataclasses import asdict
from .sensor import Sensor
from .sensorgroup import SensorGroup
from .camera import Camera
from typing import List


class SensorManager:

    def __init__(self, sensors: List[Sensor], groups: List[SensorGroup] = []):
        self.sensors = sensors
        self.groups = groups

    def get_sensor_info(self, sensor: Sensor):
        info = {
            "name": sensor.name,
            "type": "sensor",
            "const": asdict(sensor.const),
            "state": asdict(sensor.state),
            "config": sensor.config.get_dict(),
        }
        if isinstance(sensor, Camera):
            info["camera_info"] = self.get_camera_info(sensor)
            info["type"] = "camera"
        return info

    def get_group_info(self, group: SensorGroup):
        info = {
            "name": group.name,
            "type": "group",
            "state": asdict(group.state),
            "sensors": [entity.sensor.name for entity in group.sensors],
        }
        return info

    def get_camera_info(self, camera: Camera):
        return {
            "config": camera.get_config_status(),
        }

    def get_all_sensor_info(self):
        return [self.get_sensor_info(sensor) for sensor in self.sensors]

    def get_all_group_info(self):
        return [self.get_group_info(group) for group in self.groups]

    def refresh_all(self):
        for sensor in self.sensors:
            sensor.refresh_config()

    def preview_on(self, sensor_name, on: bool):
        sensor = self.search_sensor_by_name(sensor_name)
        if sensor is not None:
            sensor.state.preview_on = on

    def stream_on(self, sensor_name, on: bool):
        sensor = self.search_sensor_by_name(sensor_name)
        if sensor is not None:
            if on:
                sensor.start_stream()
            else:
                sensor.stop_stream()

    def trigger_group(self, group_name: str):
        group = self.search_group_by_name(group_name)
        if group is not None:
            group.trigger_sync()

    def group_trigger_loop_on(self, group_name: str, on: bool):
        group = self.search_group_by_name(group_name)
        if group is not None:
            if on:
                group.start_trigger_loop()
            else:
                group.stop_trigger_loop()

    def search_sensor_by_name(self, name: str):
        sensors = [x for x in self.sensors if x.name == name]
        if len(sensors) < 1:
            return None
        return sensors[0]

    def search_group_by_name(self, name: str):
        groups = [x for x in self.groups if x.name == name]
        if len(groups) < 1:
            return None
        return groups[0]
