import threading
import time
from typing import Union, Optional

import cv2
import numpy as np
from ouster_lidar.ouster_bridge import OusterBridge, OusterLidarData
from ..sensor import Sensor


class SensorOuster(Sensor):

    def __init__(self, name, ouster_bridge: OusterBridge):
        self.ouster_bridge = ouster_bridge
        const = Sensor.Const(preview_enabled=True, preview_keys=["points"])

        super().__init__(name, const, self.State())

        # 센서 alive 상태 추적을 위한 변수들
        self.last_frame_time: float = 0.0
        self.prev_frame_time: float = 0.0
        self.connection_timeout = 5.0  # 5초 후 연결 끊어진 것으로 판단
        self.thread: Optional[threading.Thread] = None

        # 최신 데이터 저장을 위한 변수 추가
        self.latest_lidar_data: Optional[OusterLidarData] = None
        self.latest_frame: Optional[Sensor.Frame] = None
        self.data_lock = threading.Lock()  # 스레드 안전성을 위한 락

        self.viz_max_depth = 20000
        # self.cam2lidar = np.array(
        #     [
        #         [
        #             0.8643344180119186,
        #             0.5027809366545345,
        #             -0.011719367593503036,
        #             -158.7326471248788,
        #         ],
        #         [
        #             -0.012185371637328744,
        #             -0.0023593352234670107,
        #             -0.9999229721610432,
        #             -7.9801565543126065,
        #         ],
        #         [
        #             -0.5027698584422295,
        #             0.864410645049017,
        #             0.004087317928107839,
        #             -777.6752812183358,
        #         ],
        #         [0, 0, 0, 1],
        #     ]
        # )
        self.cam2lidar = np.load("lidar2rear12.npy")

    def start_stream(
        self,
    ):
        if self.thread is not None and self.thread.is_alive():
            return
        # 스트림 시작 시 상태를 connecting으로 설정
        self.state.device_status = "connecting"
        self.state.stream_on = True
        self.thread = threading.Thread(
            target=self.ouster_bridge.collect_data, args=(self.__on_lidar_frame,)
        )
        self.thread.start()

    def __on_lidar_frame(self, msg: Union[OusterLidarData, Exception]):
        current_time = time.time()

        if isinstance(msg, OusterLidarData):
            # 정상적인 데이터 수신 시 상태 업데이트
            self.last_frame_time = current_time
            self.state.device_status = "connected"

            frame = self.Frame(
                msg.timestamp_ns / 1e9,
                {
                    "points": msg.points,
                    "imu_av": msg.imu.av,
                    "imu_la": msg.imu.la,
                    "imu_ts": msg.imu.timestamp_ns,
                    "lidar_ts": msg.timestamp_ns,
                },
                {},
                {
                    "points": "npy",
                    "imu_av": "npy",
                    "imu_la": "npy",
                    "imu_ts": "npy",
                    "lidar_ts": "npy",
                },
            )

            # 최신 데이터를 메모리에 저장 (스레드 안전성 보장)
            with self.data_lock:
                self.latest_lidar_data = msg
                self.latest_frame = frame

            self.frame_callback(self, frame)
        elif isinstance(msg, Exception):
            # 예외 발생 시 에러 상태로 설정
            self.state.device_status = "error"
            print(f"Ouster sensor error: {msg}")

        # 연결 상태 모니터링
        self._check_connection_status()

    def _check_connection_status(self):
        """센서의 연결 상태를 확인하고 업데이트"""
        current_time = time.time()

        # 마지막 프레임으로부터 일정 시간이 지나면 연결 끊어진 것으로 판단
        if (current_time - self.last_frame_time) > self.connection_timeout:
            if self.state.device_status == "connected":
                self.state.device_status = "disconnected"
                self.state.stream_on = False
                print("Ouster sensor connection timeout - marking as disconnected")

    def stop_stream(self):
        """스트림 중지 및 상태 업데이트"""
        if hasattr(self, "ouster_bridge"):
            self.ouster_bridge.stop()
        self.state.stream_on = False
        self.state.device_status = "disconnected"

        # 캐시된 데이터 정리
        self.clear_cached_data()

    def get_connection_status(self):
        """현재 연결 상태 반환"""
        self._check_connection_status()
        return {
            "device_status": self.state.device_status,
            "stream_on": self.state.stream_on,
            "last_frame_time": self.last_frame_time,
            "fps": self.state.fps,
            "timestamp_last": self.state.timestamp_last,
        }

    def launch_device(self):
        """디바이스 실행 및 상태 업데이트"""
        try:
            self.state.device_status = "connecting"
            # 실제 디바이스 초기화 로직이 있다면 여기에 추가
            self.start_stream()
        except Exception as e:
            self.state.device_status = "error"
            print(f"Failed to launch Ouster device: {e}")

    def validate_device(self):
        """디바이스 유효성 검사 및 상태 확인"""
        try:
            # 연결 상태 확인
            self._check_connection_status()

            # 추가적인 유효성 검사 로직
            if hasattr(self, "ouster_bridge") and self.ouster_bridge:
                return self.state.device_status == "connected"
            else:
                self.state.device_status = "disconnected"
                return False
        except Exception as e:
            self.state.device_status = "error"
            print(f"Device validation failed: {e}")
            return False

    def trigger_device(self):
        pass

    def register_callback(self, callback):
        self.frame_callback = callback

    def get_latest_frame(self):
        """Get the most recent frame from Ouster LIDAR"""
        try:
            if self.state.device_status != "connected":
                return None

            # 메모리에 저장된 최신 프레임 반환 (스레드 안전성 보장)
            with self.data_lock:
                if self.latest_frame is not None:
                    # 최신 프레임의 복사본 반환
                    return self.Frame(
                        self.latest_frame.timestamp,
                        self.latest_frame.data.copy(),
                        self.latest_frame.attrs.copy(),
                        self.latest_frame.file_format.copy(),
                    )

            return None
        except Exception as e:
            print(f"Failed to get latest frame from Ouster: {e}")
            return None

    def get_latest_lidar_data(self):
        """Get the most recent raw OusterLidarData"""
        try:
            if self.state.device_status != "connected":
                return None

            # 메모리에 저장된 최신 원시 LIDAR 데이터 반환
            with self.data_lock:
                return self.latest_lidar_data

        except Exception as e:
            print(f"Failed to get latest LIDAR data from Ouster: {e}")
            return None

    def clear_cached_data(self):
        """Clear cached data (useful for memory management)"""
        with self.data_lock:
            self.latest_lidar_data = None
            self.latest_frame = None

    def is_data_recent(self, max_age_seconds: float = 1.0):
        """Check if the cached data is recent enough"""
        try:
            if self.latest_frame is None:
                return False

            current_time = time.time()
            data_age = current_time - self.latest_frame.timestamp
            return data_age <= max_age_seconds

        except Exception as e:
            print(f"Failed to check data recency: {e}")
            return False

    def get_data_age(self):
        """Get the age of the cached data in seconds"""
        try:
            if self.latest_frame is None:
                return None

            current_time = time.time()
            return current_time - self.latest_frame.timestamp

        except Exception as e:
            print(f"Failed to get data age: {e}")
            return None

    def post_process_thumbnail(self, frame, frame_id: str):

        if frame_id == "points":
            points = frame
            points = points.reshape(-1, 3) * 1000
            lidar2cam = self.cam2lidar

            points = (
                lidar2cam
                @ np.concatenate([points, np.ones((points.shape[0], 1))], axis=1).T
            ).T

            points = points[:, :3]

            fx = 1346
            cx = 720
            cy = 464

            points[..., 0] = points[..., 0] * fx / points[..., 2] + cx
            points[..., 1] = points[..., 1] * fx / points[..., 2] + cy
            depth_map = np.zeros((928, 1440, 3), dtype=np.uint8)

            points = points[
                (points[..., 0] > 0)
                & (points[..., 0] < 1440 - 3)
                & (points[..., 1] > 0)
                & (points[..., 1] < 928 - 3)
                & (points[..., 2] > 0)
            ]
            points[..., 2] = points[..., 2] * 255 / self.viz_max_depth

            colors = cv2.applyColorMap(
                (points[..., 2]).astype(np.uint8), cv2.COLORMAP_JET
            )
            for i in range(9):
                depth_map[
                    points[..., 1].astype(int) + i % 3,
                    points[..., 0].astype(int) + i // 3,
                ] = colors.reshape(-1, 3)
            frame = depth_map
        return frame

    def get_cache_info(self):
        """Get information about the cached data"""
        try:
            with self.data_lock:
                info = {
                    "has_cached_data": self.latest_frame is not None,
                    "has_raw_data": self.latest_lidar_data is not None,
                    "data_age": self.get_data_age(),
                    "is_recent": self.is_data_recent(),
                    "device_status": self.state.device_status,
                    "last_frame_time": self.last_frame_time,
                }

                if self.latest_frame is not None:
                    info["cached_timestamp"] = self.latest_frame.timestamp
                    info["data_keys"] = list(self.latest_frame.data.keys())

                return info

        except Exception as e:
            print(f"Failed to get cache info: {e}")
            return {"error": str(e)}
