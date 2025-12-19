import os
import time
from typing import Callable, Iterator, List, Tuple, Union, cast
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

import threading

HOSTNAME = "os-122107000458.local"
LIDAR_MODE = LidarMode.MODE_2048x10


class OusterImuData:
    def __init__(
        self,
        timestamp_ns: np.ndarray,
        av: np.ndarray,
        la: np.ndarray,
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

    def __dict__(self):
        return {
            "points": self.points,
        }

    def meta_dict(self):
        return {
            "lidar_timestamp_ns": self.timestamp_ns,
            # "beam_altitude_angles": self.metadata.beam_altitude_angles,
            # "beam_azimuth_angles": self.metadata.beam_azimuth_angles,
            # "imu_to_sensor_transform": self.metadata.imu_to_sensor_transform,
            # "lidar_to_sensor_transform": self.metadata.lidar_to_sensor_transform,
            # "lidar_origin_to_beam_origin_mm": self.metadata.lidar_origin_to_beam_origin_mm,
            # "beam_to_lidar_transform": self.metadata.beam_to_lidar_transform,
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


IMU_STORAGE = "tmp/ouster_imu"


class OusterBridge:
    def __init__(self, multi_signal_enhance=None):
        self.base_time = 0
        # lock to protect sensor re-init from multiple threads
        self._sensor_lock = threading.Lock()
        # indicate whether a manual reconnect was requested
        self._reconnect_requested = threading.Event()
        # connected flag (best-effort)
        self.connected = False
        config = SensorConfig()
        config.udp_port_lidar = 7502
        config.udp_port_imu = 7503
        config.operating_mode = OperatingMode.OPERATING_NORMAL
        config.lidar_mode = LIDAR_MODE
        config.timestamp_mode = client.TimestampMode.TIME_FROM_PTP_1588

        if multi_signal_enhance is not None:
            config.signal_multiplier = 3.0
            config.azimuth_window = multi_signal_enhance
        else:
            config.signal_multiplier = 1.0
            config.azimuth_window = (0, 360000)



        client.set_config(HOSTNAME, config, persist=True, udp_dest_auto=True)

        # initialize sensor and helpers through a method so we can re-create them
        # at runtime when needed
        self._init_sensor()

        self.imu_packet_queue: list[ImuPacket] = []
        self.imu_value_queue: list[Tuple[int, np.ndarray, np.ndarray]] = []
        self.imu_value_queue_storage: list[Tuple[int, np.ndarray, np.ndarray]] = []

        self.flag_kill = threading.Event()

    def _init_sensor(self):
        """(Re)initialize the underlying Sensor and helper objects.

        This is safe to call multiple times but not concurrently. Caller should
        hold external synchronization if needed; this method uses an internal
        lock to protect concurrent inits.
        """
        with getattr(self, "_sensor_lock", threading.Lock()):
            try:
                # set_config is cheap and idempotent; keep it so settings persist
                # but calling it repeatedly is not harmful
                # client.set_config(HOSTNAME, config, persist=True, udp_dest_auto=True)

                # close previous sensor if present
                if hasattr(self, "sensor"):
                    try:
                        self.sensor.close()
                    except Exception:
                        pass
                    try:
                        del self.sensor
                    except Exception:
                        pass

                if hasattr(self, "imu_sensor"):
                    try:
                        self.imu_sensor.close()
                    except Exception:
                        pass
                    try:
                        del self.imu_sensor
                    except Exception:
                        pass

                # create new sensor
                self.imu_sensor = client.Sensor(HOSTNAME, 7502, 7503, buf_size=640)
                self.sensor = ScanWithImu(self.imu_sensor, complete=False, _max_latency=2)
                self.packet_format = PacketFormat(self.sensor.metadata)
                self.xyzlut = client.XYZLut(self.sensor.metadata)
                self.connected = True
                # clear reconnect request flag
                self._reconnect_requested.clear()
            except Exception as e:
                # mark as disconnected and surface the exception to caller if needed
                self.connected = False
                raise

    def reconnect(self, max_attempts: int = 5, base_delay: float = 1.0) -> bool:
        """Attempt to reinitialize the connection to the lidar with retries.

        Returns True if reconnection succeeded, False otherwise.
        """
        # simple exponential backoff retry
        attempt = 0
        while attempt < max_attempts and not self.flag_kill.is_set():
            attempt += 1
            try:
                self._init_sensor()
                print(f"OusterBridge: reconnected on attempt {attempt}")
                return True
            except Exception as e:
                wait = base_delay * (2 ** (attempt - 1))
                print(f"OusterBridge: reconnect attempt {attempt} failed: {e}; retrying in {wait}s")
                time.sleep(wait)

        print("OusterBridge: failed to reconnect after attempts")
        return False

    def force_reconnect(self, wait_for_success: bool = False, **kwargs) -> bool:
        """Request a reconnect. If wait_for_success is True, block until
        reconnect returns or attempts are exhausted.
        """
        # allow external callers to trigger reconnect safely
        self._reconnect_requested.set()
        if wait_for_success:
            return self.reconnect(**kwargs)
        else:
            # spawn a background thread to reconnect so caller isn't blocked
            t = threading.Thread(target=self.reconnect, kwargs=kwargs, daemon=True)
            t.start()
            return True

    def store_imu_value_queue_storage(self):
        """
        imu_value: Tuple[timestamp, la, av]
        store as numpy array on npz file
        """
        timestamp_np = np.array([x[0] for x in self.imu_value_queue_storage])
        la_np = np.array([x[1] for x in self.imu_value_queue_storage])
        av_np = np.array([x[2] for x in self.imu_value_queue_storage])
        filename = str(self.imu_value_queue[0][0]) + ".npz"
        np.savez(
            os.path.join(IMU_STORAGE, filename),
            timestamp=timestamp_np,
            la=la_np,
            av=av_np,
        )
        self.imu_value_queue_storage = []

    def stop(self):
        self.flag_kill.set()
        self.store_imu_value_queue_storage()

    def get_imu_value_queue_until_time(self, time_ns: int):
        """
        time, av, la
        """
        timestamp = []
        av = []
        la = []
        while self.imu_value_queue and self.imu_value_queue[0][0] <= time_ns:
            imu_value = self.imu_value_queue.pop(0)
            timestamp.append(imu_value[0])
            av.append(imu_value[1])
            la.append(imu_value[2])

        av_cov = self.calculate_covariance(av)
        la_cov = self.calculate_covariance(la)
        return OusterImuData(
            np.array(timestamp), np.array(av), np.array(la), av_cov, la_cov
        )

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

        timestamp_data = [
            self.packet_format.imu_sys_ts(packet.buf) + self.base_time
            for packet in self.imu_packet_queue
        ]
        # 가속도 평균값 계산

        av_data = [
            [
                self.packet_format.imu_av_x(packet.buf),
                self.packet_format.imu_av_y(packet.buf),
                self.packet_format.imu_av_z(packet.buf),
            ]
            for packet in self.imu_packet_queue
        ]

        la_data = [
            [
                self.packet_format.imu_la_x(packet.buf),
                self.packet_format.imu_la_y(packet.buf),
                self.packet_format.imu_la_z(packet.buf),
            ]
            for packet in self.imu_packet_queue
        ]

        av_array = np.array(av_data)
        av_mean = np.mean(av_array, axis=0)

        # 자이로 평균값 계산
        la_data = np.array(la_data)
        la_mean = np.mean(la_data, axis=0)

        # 공분산 계산
        av_covariance = self.calculate_covariance(av_data)
        la_covariance = self.calculate_covariance(la_data)

        for i in range(len(self.imu_packet_queue)):
            self.imu_value_queue.append((timestamp_data[i], av_array[i], la_data[i]))

        imu = OusterImuData(
            np.asarray(timestamp_data),
            av_mean,
            la_mean,
            av_covariance,
            la_covariance,
        )
        while len(self.imu_packet_queue) > 500:
            self.imu_packet_queue.pop(0)
        return imu

    def collect_data(
        self, callback: Callable[[Union[OusterLidarData, Exception]], None]
    ):
        # self.imu_thread = threading.Thread(target=self.collect_imu_data)
        # self.imu_thread.start()

        # We will repeatedly open a stream from the current sensor instance.
        # If a timeout or other error occurs, attempt to reconnect and then
        # continue streaming without requiring an application restart.
        while not self.flag_kill.is_set():
            # if a reconnect was requested externally, try it first
            if self._reconnect_requested.is_set():
                print("OusterBridge: external reconnect requested")
                self.reconnect()

            # create a local reference so we can safely close/replace self.sensor
            try:
                with closing(self.sensor) as stream:
                    for packet in stream:

                        if self.flag_kill.is_set():
                            print("LiDAR kill flag is set")
                            break

                        if isinstance(packet, ImuPacket):
                            if self.base_time == 0:
                                try:
                                    self.base_time = (
                                        time.time_ns()
                                        - self.packet_format.imu_sys_ts(packet.buf)
                                    )
                                except Exception:
                                    # packet_format might be stale; ignore and continue
                                    pass
                            if len(self.imu_packet_queue) > 3000:
                                self.imu_packet_queue.pop(0)
                            self.imu_packet_queue.append(packet)

                        if isinstance(packet, LidarScan):
                            if self.base_time == 0:
                                try:
                                    self.base_time = time.time_ns() - packet.timestamp[-1]
                                except Exception:
                                    pass
                            if not packet.complete():
                                timestamp = time.time_ns() - self.base_time
                            else:
                                timestamp = packet.timestamp[-1]
                            try:
                                reflectivity = client.destagger(
                                    stream.metadata,
                                    packet.field(client.ChanField.REFLECTIVITY),
                                )
                                ranges = client.destagger(
                                    stream.metadata, packet.field(client.ChanField.RANGE)
                                )
                                xyz = self.xyzlut(packet)
                            except Exception as e:
                                # something wrong with current sensor state; request reconnect
                                print(f"OusterBridge: error while processing packet: {e}")
                                callback(e)
                                # attempt reconnect and break to restart loop
                                self.reconnect()
                                break

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
                # try to reconnect and continue
                ok = self.reconnect()
                if not ok:
                    # wait a bit before next outer attempt
                    time.sleep(1.0)
            except Exception as e:
                # generic exception from iterating stream or sensor; attempt reconnect
                print(f"OusterBridge: unexpected error reading stream: {e}")
                callback(e)
                ok = self.reconnect()
                if not ok:
                    time.sleep(1.0)
            # small pause to avoid a busy loop when reconnecting fails
            time.sleep(0.01)

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
