import time
from typing import Callable, Union
from ouster.sdk import client
from ouster.sdk.client import (
    SensorInfo,
    Sensor,
    SensorConfig,
    LidarMode,
    OperatingMode,
    PacketFormat,
    LidarScan,
)

from contextlib import closing
from std_msgs.msg import Header

import numpy as np

HOSTNAME = "os-122107000458.local"
LIDAR_MODE = LidarMode.MODE_1024x20


class OusterLidarData:
    def __init__(
        self,
        metadata: SensorInfo,
        timestamp_ns: int,
        reflectivity: np.ndarray,
        ranges: np.ndarray,
        points: np.ndarray,
    ):
        self.metadata = metadata
        self.header = Header()
        self.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
        self.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)
        self.timestamp_ns = timestamp_ns
        self.reflectivity = reflectivity
        self.ranges = ranges
        self.points = points

    def __repr__(self):
        return f"""
    OusterLidarData(metadata={self.metadata}, 
    timestamp_ns={self.timestamp_ns}, 
    currentTime = {time.time_ns()},
    reflectivity={self.reflectivity.shape}, 
    ranges={self.ranges.shape})"""

    def __dict__(self):
        return {
            "reflectivity": self.reflectivity,
            "ranges": self.ranges,
            "points": self.points,
            "lidar_timestamp_ns": self.timestamp_ns,
            "beam_altitude_angles": self.metadata.beam_altitude_angles,
            "beam_azimuth_angles": self.metadata.beam_azimuth_angles,
            "imu_to_sensor_transform": self.metadata.imu_to_sensor_transform,
            "lidar_to_sensor_transform": self.metadata.lidar_to_sensor_transform,
            "lidar_origin_to_beam_origin_mm": self.metadata.lidar_origin_to_beam_origin_mm,
            "beam_to_lidar_transform": self.metadata.beam_to_lidar_transform,
        }

    def __del__(self):
        del self.reflectivity
        del self.ranges
        del self.points


class OusterBridge:
    def __init__(self):
        self.base_time = 0
        config = SensorConfig()
        config.udp_port_lidar = 7502
        config.udp_port_imu = 7503
        config.operating_mode = OperatingMode.OPERATING_NORMAL
        config.lidar_mode = LIDAR_MODE
        client.set_config(HOSTNAME, config, persist=True, udp_dest_auto=True)
        self.sensor = client.Scans.stream(HOSTNAME, 7502, complete=False)

        self.packet_format = PacketFormat(self.sensor.metadata)
        self.xyzlut = client.XYZLut(self.sensor.metadata)

    def collect_data(
        self, callback: Callable[[Union[OusterLidarData, Exception]], None]
    ):
        with closing(self.sensor) as stream:
            show = True

            while show:
                try:
                    for packet in stream:
                        if isinstance(packet, LidarScan):
                            if self.base_time == 0:
                                self.base_time = time.time_ns() - packet.timestamp[-1]

                            if not packet.complete():
                                print("Incomplete packet!")
                                continue

                            reflectivity = client.destagger(
                                stream.metadata,
                                packet.field(client.ChanField.REFLECTIVITY),
                            )
                            ranges = client.destagger(
                                stream.metadata, packet.field(client.ChanField.RANGE)
                            )
                            xyz = self.xyzlut(packet)
                            timestamp = packet.timestamp[-1]
                            callback(
                                OusterLidarData(
                                    self.sensor.metadata,
                                    timestamp + self.base_time,
                                    reflectivity,
                                    ranges,
                                    xyz,
                                )
                            )
                except client.ClientTimeout as e:
                    print("Lidar Timeout!")
                    callback(e)

    def __del__(self):
        if hasattr(self, "sensor"):
            self.sensor.close()
            del self.sensor
            del self.packet_format


import cv2

if __name__ == "__main__":
    ouster_bridge = OusterBridge()
    print("Ouster bridge initialized")
    ouster_bridge.collect_data(
        lambda data: (
            print(data),
            cv2.imwrite("Reflecitivity.png", data.reflectivity.astype(np.uint8)),
        )
    )
