import time
from typing import Callable, Iterator, List, Union, cast
from ouster.sdk import client
from ouster.sdk.client import (
    SensorInfo,
    Sensor,
    SensorConfig,
    LidarMode,
    OperatingMode,
    PacketFormat,
    LidarScan,
    ImuPacket,
    LidarPacket,
)
from ouster.sdk.client._client import ScanBatcher

from contextlib import closing
from std_msgs.msg import Header

import numpy as np

HOSTNAME = "os-122107000458.local"
LIDAR_MODE = LidarMode.MODE_1024x20


class OusterImuData:
    def __init__(
        self,
        timestamp_ns: int,
        la: np.ndarray,
        av: np.ndarray,
        av_cov: np.ndarray,
        la_cov: np.ndarray,
    ):
        self.timestamp_ns = timestamp_ns
        self.la = la
        self.av = av
        self.av_cov = av_cov
        self.la_cov = la_cov

    def __repr__(self):
        return f"""
    OusterImuData(timestamp_ns={self.timestamp_ns},
    la={self.la},
    av={self.av},
    av_cov={self.av_cov},
    la_cov={self.la_cov})
    """

    def dict(self):
        return {
            "timestamp_ns": self.timestamp_ns,
            "la": self.la,
            "av": self.av,
            "av_cov": self.av_cov,
            "la_cov": self.la_cov,
        }


class OusterLidarData:
    def __init__(
        self,
        metadata: SensorInfo,
        timestamp_ns: int,
        reflectivity: np.ndarray,
        ranges: np.ndarray,
        points: np.ndarray,
        pose: np.ndarray,
        imu: OusterImuData,
    ):
        self.metadata = metadata
        self.header = Header()
        self.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
        self.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)
        self.timestamp_ns = timestamp_ns
        self.reflectivity = reflectivity
        self.ranges = ranges
        self.points = points
        self.pose = pose
        self.imu = imu

    def __repr__(self):
        return f"""
    OusterLidarData(metadata={self.metadata}, 
    timestamp_ns={self.timestamp_ns}, 
    currentTime = {time.time_ns()},
    reflectivity={self.reflectivity.shape}, 
    ranges={self.ranges.shape})
    imu={self.imu},

    """

    def meta_dict(self):
        return {
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


class ScanWithImu(client.Scans):
    def __iter__(self) -> Iterator[LidarScan]:
        """Get an iterator."""

        w = self._source.metadata.format.columns_per_frame
        h = self._source.metadata.format.pixels_per_column
        columns_per_packet = self._source.metadata.format.columns_per_packet
        packets_per_frame = w // columns_per_packet
        column_window = self._source.metadata.format.column_window

        # If source is a sensor, make a type-specialized reference available
        sensor = (
            cast(Sensor, self._source) if isinstance(self._source, Sensor) else None
        )

        ls_write = None
        pf = PacketFormat.from_info(self._source.metadata)
        batch = ScanBatcher(w, pf)

        # Time from which to measure timeout
        start_ts = time.monotonic()

        it = iter(self._source)
        self._packets_consumed = 0
        self._scans_produced = 0
        while True:
            try:
                packet = next(it)
                self._packets_consumed += 1
            except StopIteration:
                if ls_write is not None:
                    if not self._complete or ls_write.complete(column_window):
                        yield ls_write
                return

            if self._timeout is not None and (
                time.monotonic() >= start_ts + self._timeout
            ):
                raise client.ClientTimeout(
                    f"No valid frames received within {self._timeout}s"
                )

            if isinstance(packet, LidarPacket):
                ls_write = ls_write or LidarScan(
                    h, w, self._field_types, columns_per_packet
                )

                if batch(packet, ls_write):
                    # Got a new frame, return it and start another
                    if not self._complete or ls_write.complete(column_window):
                        yield ls_write
                        self._scans_produced += 1
                        start_ts = time.monotonic()
                    ls_write = None

                    # Drop data along frame boundaries to maintain _max_latency and
                    # clear out already-batched first packet of next frame
                    if self._max_latency and sensor is not None:
                        buf_frames = sensor.buf_use // packets_per_frame
                        drop_frames = buf_frames - self._max_latency + 1

                        if drop_frames > 0:
                            sensor.flush(drop_frames)
                            batch = ScanBatcher(w, pf)
            if isinstance(packet, ImuPacket):
                yield packet  # type: ignore


class OusterBridge:
    def __init__(self):
        self.base_time = 0
        config = SensorConfig()
        config.udp_port_lidar = 7502
        config.udp_port_imu = 7503
        config.operating_mode = OperatingMode.OPERATING_NORMAL
        config.lidar_mode = LIDAR_MODE
        client.set_config(HOSTNAME, config, persist=True, udp_dest_auto=True)

        self.imu_sensor = client.Sensor(HOSTNAME, 7502, 7503, buf_size=640)

        # self.sensor = client.Scans.stream(HOSTNAME, 7502, complete=False)
        self.sensor = ScanWithImu(self.imu_sensor, complete=False, _max_latency=2)
        self.packet_format = PacketFormat(self.sensor.metadata)
        self.xyzlut = client.XYZLut(self.sensor.metadata)

        self.imu_packet_queue: list[ImuPacket] = []

    def calculate_covariance(self, data: List) -> np.ndarray:
        """
        주어진 데이터의 공분산 행렬을 계산합니다.
        :param data: deque 형태의 데이터 (N x 3)
        :return: 9개의 요소를 가지는 리스트 (3x3 행렬)
        """
        if len(data) < 2:
            # 데이터가 충분하지 않으면 기본값 반환
            return np.zeros((3, 3))

        data_array = np.array(data)  # N x 3
        covariance_matrix = np.cov(data_array, rowvar=False)  # 3x3

        # 리스트 형태로 변환
        return covariance_matrix

    def get_imu_from_packet(self):
        """
        imu_packet_queue의 모든 패킷들을 처리하여 Imu 메시지를 생성합니다.
        """

        # 모든 패킷을 처리하여 데이터 버퍼에 추가 (이미 add_imu_packet에서 추가됨)
        # 여기서는 이미 add_imu_packet을 통해 데이터 버퍼가 업데이트 되었다고 가정

        # 최근 패킷의 타임스탬프를 사용 (또는 필요한 다른 타임스탬프 기준으로 설정)
        latest_packet = self.imu_packet_queue[-1]
        timestamp = self.packet_format.imu_sys_ts(latest_packet.buf) + self.base_time

        # 가속도 평균값 계산

        accel_data = [
            [
                self.packet_format.imu_av_x(packet.buf),
                self.packet_format.imu_av_y(packet.buf),
                self.packet_format.imu_av_z(packet.buf),
            ]
            for packet in self.imu_packet_queue
        ]

        gyro_data = [
            [
                self.packet_format.imu_la_x(packet.buf),
                self.packet_format.imu_la_y(packet.buf),
                self.packet_format.imu_la_z(packet.buf),
            ]
            for packet in self.imu_packet_queue
        ]

        accel_array = np.array(accel_data)
        accel_mean = np.mean(accel_array, axis=0)

        # 자이로 평균값 계산
        gyro_array = np.array(gyro_data)
        gyro_mean = np.mean(gyro_array, axis=0)

        # 공분산 계산
        accel_covariance = self.calculate_covariance(accel_data)
        gyro_covariance = self.calculate_covariance(gyro_data)

        imu = OusterImuData(
            timestamp,
            accel_mean,
            gyro_mean,
            accel_covariance,
            gyro_covariance,
        )
        while self.imu_packet_queue:
            self.imu_packet_queue.pop(0)
        return imu

    def collect_data(
        self, callback: Callable[[Union[OusterLidarData, Exception]], None]
    ):
        # self.imu_thread = threading.Thread(target=self.collect_imu_data)
        # self.imu_thread.start()

        with closing(self.sensor) as stream:
            show = True

            while show:
                try:
                    for packet in stream:

                        if isinstance(packet, ImuPacket):
                            if self.base_time == 0:
                                self.base_time = (
                                    time.time_ns()
                                    - self.packet_format.imu_sys_ts(packet.buf)
                                )
                            if len(self.imu_packet_queue) > 100:
                                self.imu_packet_queue.pop(0)
                            self.imu_packet_queue.append(packet)

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
                            pose = packet.pose
                            callback(
                                OusterLidarData(
                                    self.sensor.metadata,
                                    timestamp + self.base_time,
                                    reflectivity,
                                    ranges,
                                    xyz,
                                    pose,
                                    self.get_imu_from_packet(),
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
    ouster_bridge.collect_data(lambda data: (print(data),))
