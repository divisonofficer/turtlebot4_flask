import random
import sys
import traceback
from typing import Optional, Union

import cv2
import numpy as np

sys.path.append("../")
sys.path.append("../../")

from sensors.cameras.lucid_24_single import CameraLucid24Single
from sensors.sensor import Sensor
from sensors.lidar.OusterOS1 import SensorOuster
from sensors.camera import Camera
from sensors.cameras.lucid_24 import CameraLucid24
from sensors.sensorgroup import SensorGroup
from thumb_stream import ThumbStream
from sensors.cameras.oakd_pro import CameraOAK_D
from sensors.cameras.realsense import CameraRS
from sensors.sensor_manager import SensorManager
from videostream import VideoStream
from lucid_storage import StereoMultiItem, StereoCaptureItem
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import threading
from ouster_lidar.ouster_bridge import OusterBridge, OusterLidarData
from stereo_queue import StereoQueue, StereoItemMerged
from lucid_storage import StereoStorage
from lucid_cam import get_global_device_manager, set_node_value_safely
import time
from flask import Flask, Response, request
from flask_socketio import SocketIO
import json
from flask_cors import CORS
from synchronized_queue import SQueue
from jai_bridge.oakd_bridge import DepthAICamera
from lucid_cam import _device_manager, LucidImage  # 전역 디바이스 매니저 import
from hdr_image_processor import process_high_gain_frames


class OusterStatus:
    working: bool = True
    exception: Optional[Exception] = None

    def __dict__(self):
        return {
            "working": self.working,
            "exception": (
                self.exception.__repr__() if self.exception is not None else None
            ),
        }


class LucidStatus:
    ouster = OusterStatus()
    lucid_queue: dict = {}
    lidar_queue: dict = {}
    storage_enabled: bool = False
    storage_queued_cnt: int = 0
    single_storage_mode: bool = False
    storage_id: Optional[str] = None

    # HDR Burst Imaging Status
    hdr_burst_mode: bool = False
    hdr_burst_in_progress: bool = False
    hdr_burst_progress: dict = (
        {}
    )  # {"left": {"current_exposure": 0, "total_exposures": 4}, "right": {...}}
    hdr_burst_current_step: str = (
        ""  # "preparing", "capturing_88us", "capturing_880us", etc.
    )

    # High Gain Burst Option
    hdr_high_gain_mode: bool = False  # 고gain 추가 촬영 모드
    hdr_high_gain_burst_count: int = 8  # 고gain에서 촬영할 버스트 이미지 수
    hdr_high_gain_processing: bool = False  # 고gain 이미지 처리 중 상태
    hdr_high_gain_processing_progress: dict = {}  # 카메라별 고gain 처리 진행상황

    def __dict__(self):
        return {
            "ouster": self.ouster.__dict__(),
            "lucid_queue": self.lucid_queue,
            "lidar_queue": self.lidar_queue,
            "storage_enabled": self.storage_enabled,
            "storage_queued_cnt": self.storage_queued_cnt,
            "single_storage_mode": self.single_storage_mode,
            "storage_id": self.storage_id,
            "hdr_burst_mode": self.hdr_burst_mode,
            "hdr_burst_in_progress": self.hdr_burst_in_progress,
            "hdr_burst_progress": self.hdr_burst_progress,
            "hdr_burst_current_step": self.hdr_burst_current_step,
            "hdr_high_gain_mode": self.hdr_high_gain_mode,
            "hdr_high_gain_burst_count": self.hdr_high_gain_burst_count,
            "hdr_high_gain_processing": self.hdr_high_gain_processing,
            "hdr_high_gain_processing_progress": self.hdr_high_gain_processing_progress,
        }


class LucidStereoNode(Node):

    def __init__(self, socket: SocketIO):
        self.socket = socket
        super().__init__("lucid_stereo_node")

        # HDR 노출 시간 설정 (마이크로초 단위)
        self.hdr_exposure_times = [0,160, 2400, -1]
        self.hdr_exposure_times_12 = [0, 100, 400, 2000, 8000, 32000, -1]

        try:
            self.ouster_bridge = OusterBridge(
                multi_signal_enhance=[60000, 180000]
            )
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Ouster: {e}")
            self.ouster_bridge = None

        self.queue = SQueue()

        self.storage = StereoStorage()

        self.storage_loop_thread = threading.Thread(target=self.storage.queue_loop)
        # self.lucid_api = LucidPyAPI()
        # self.lucid_api.connect_device()
        # self.queue.register_synchronized_queue("lucid_stereo", is_root=True)
        self.queue.register_synchronized_queue("lucid_left", is_root=True)

        self.queue_loop_thread = threading.Thread(
            target=self.queue.loop_yield_synchronized_items,
            args=(self.queue_callback,),
            daemon=True,
        )

        self.storage_loop_thread.start()
        self.queue_loop_thread.start()
        self.storage_id: Optional[str] = None
        self.single_mode_storage_id: Optional[str] = None
        self.storage_enabled_timestamp: Optional[float] = None  # 저장 활성화 시점 기록

        self.status = LucidStatus()

        self.timer = self.create_timer(5, self.status_callback)

        self.stereo_stream = VideoStream()
        # CameraOAK_D("oakd_pro"),
        # CameraLucid24("lucid_stereo", self.lucid_api),

        camera_lucid = [
            CameraLucid24Single(
                "lucid_left",
                "224201564",
                ptpmode=True,
                ptpmaster=True,
                exposure_auto_target=0,
            ),
            CameraLucid24Single("lucid_right", "224201585", ptpmode=True),
            CameraLucid24Single(
                "lucid_12_left",
                "253200234",
                ptpmode=True,
                type="TRI032S-C",
                exposure_auto_target=96,
            ),
            CameraLucid24Single(
                "lucid_12_rear",
                "253300072",
                ptpmode=True,
                type="TRI032S-C",
                ptpmaster=False,
                exposure_auto_target=24,
            ),
            CameraLucid24Single(
                "lucid_12_right",
                "253300071",
                ptpmode=True,
                type="TRI032S-C",
                exposure_auto_target=32,
            ),
            CameraLucid24Single(
                "lucid_12_left_sub",
                "253200221",
                ptpmode=True,
                type="TRI032S-C",
                exposure_auto_target=1,
            ),
            CameraLucid24Single(
                "lucid_12_rear_sub",
                "253300069",
                ptpmode=True,
                type="TRI032S-C",
                exposure_auto_target=4,
            ),
            CameraLucid24Single(
                "lucid_12_right_sub",
                "253300070",
                ptpmode=True,
                type="TRI032S-C",
                exposure_auto_target=8,
            ),
        ]

        camera_rs = CameraRS("realsense", fps=15, resolution=(1280, 720))
        tof_sensor = CameraLucid24Single(
            "helios_tof",
            serial="252902574",
            type="HELIOS2",
            ptpmode=True,
        )
        ouster = SensorOuster("ouster", self.ouster_bridge)
        self.sensor_manager = SensorManager(
            [*camera_lucid, camera_rs, tof_sensor, ouster],
            [
                # SensorGroup(
                #     "lucid_stereo",
                #     [
                #         SensorGroup.Entity(camera_lucid[0], trigger_sync=True),
                #         SensorGroup.Entity(camera_lucid[1], trigger_sync=True),
                #     ],
                #     5,
                # )
            ],
        )

        self.thumb_stream = ThumbStream(self.socket)
        self.sensor_threads = []
        essential_sensors = [
            camera_lucid[0],
            camera_lucid[2],
            camera_lucid[3],
            camera_lucid[4],
            ouster,
        ]
        none_essential_sensors = [
            camera_lucid[1],
            camera_lucid[5],
            camera_lucid[6],
            camera_lucid[7],
            camera_rs,
            tof_sensor,
        ]
        for s in essential_sensors:
            self.queue.register_synchronized_queue(s.name, is_essential=True)
        for s in none_essential_sensors:
            self.queue.register_synchronized_queue(s.name, is_essential=False)
        for sensor in self.sensor_manager.sensors:

            def callback(sensor: Sensor, data: Sensor.Frame):
                # synchronized queue

                if sensor.state.timestamp_last >= data.timestamp:
                    return
                self.queue.enqueue(sensor.name, data.timestamp, data)
                sensor.state.fps = round(
                    max(
                        1 / (data.timestamp - sensor.state.timestamp_last + 1e-8),
                        0.0001,
                    ),
                    3,
                )
                sensor.state.timestamp_last = data.timestamp

                self.socket.emit(
                    "sensor_update",
                    {
                        "name": sensor.name,
                        "timestamp_last": sensor.state.timestamp_last,
                        "fps": sensor.state.fps,
                    },
                )

                # todo : on/off 추가 필요?
                if (
                    sensor.state.preview_on
                    and (time.time() - sensor.state.preview_last)
                    >= sensor.state.preview_interval
                ):
                    sensor.state.preview_last = time.time()

                    for key in data.data:
                        if key in sensor.const.preview_keys:
                            self.thumb_stream.yield_thumbnails(
                                data.timestamp, data.data[key], sensor, key
                            )

            sensor.register_callback(callback)
            thread = threading.Thread(target=sensor.start_stream, daemon=True)
            self.sensor_threads.append(thread)
            thread.start()

        def lucid_sync_timestamp():
            while True:

                time.sleep(30)
                if (
                    camera_lucid[0].lucid_api is None
                    or camera_lucid[0].lucid_api.device is None
                ):
                    continue
                camera_lucid[0].start_stream()
                camera_lucid[0].lucid_api.device_config_timestamp_base()
                for i in range(1, len(camera_lucid)):
                    camera_lucid[i].lucid_api.device_config_timestamp_base()
                # for i in range(1, len(camera_lucid)):
                #     camera_lucid[i].lucid_api.timestamp_base = camera_lucid[
                #         0
                #     ].lucid_api.timestamp_base
                tof_sensor.lucid_api.device_config_timestamp_base()
                # tof_sensor.lucid_api.timestamp_base = camera_lucid[
                #     0
                # ].lucid_api.timestamp_base
                #break

        threading.Thread(target=lucid_sync_timestamp, daemon=True).start()

    def queue_callback(self, items):
        #print(items.keys())

        # HDR 모드가 활성화된 경우 일반 프레임 저장을 건너뛰기
        if self.status.hdr_burst_mode and self.storage_id is not None:
            print("HDR burst mode active - skipping regular frame storage")
            return

        if self.storage_id is not None:
            try:
                timestamp = list(items.values())[0].timestamp
                
                # storage_enabled_timestamp 이후의 프레임만 저장
                if self.storage_enabled_timestamp is not None and timestamp < self.storage_enabled_timestamp:
                    print(f"Skipping old frame: frame_ts={timestamp:.3f}, enabled_ts={self.storage_enabled_timestamp:.3f}, diff={self.storage_enabled_timestamp - timestamp:.3f}s")
                    return
                
                self.storage.enqueue((self.storage_id, timestamp, items))
                self.status.storage_queued_cnt += 1
                if self.status.single_storage_mode:
                    self.disable_storage()
            except Exception as e:
                print(e)

    def status_callback(self):
        self.status.storage_enabled = self.storage_id is not None
        self.status.storage_id = self.storage_id
        for sensor in self.sensor_manager.sensors:
            if hasattr(sensor, "validate_device"):
                sensor.validate_device()
        # Merge storage internal status into emitted status to allow UI to show
        # actual queue size, memory usage and saved frame counts.
        try:
            status_payload = self.status.__dict__()
            # attach storage status under `storage` key
            try:
                status_payload["storage"] = self.storage.get_status()
            except Exception as e:
                status_payload["storage"] = {"error": str(e)}
        except Exception:
            # fallback - emit minimal status
            status_payload = {
                "storage_enabled": self.storage_id is not None,
                "storage_id": self.storage_id,
            }

        self.socket.emit("status", status_payload)
        self.socket.emit("sensors", self.sensor_manager.get_all_sensor_info())
        
        

    def enable_storage(self):
        if self.status.single_storage_mode:
            if self.single_mode_storage_id is None:
                self.single_mode_storage_id = time.strftime(
                    "%m_%d_%H_%M", time.localtime(time.time())
                )
            self.storage_id = self.single_mode_storage_id
        else:
            self.storage_id = time.strftime("%m_%d_%H_%M", time.localtime(time.time()))
            self.single_mode_storage_id = None

        # 저장 활성화 시점 기록 (이 시점 이후의 프레임만 저장됨)
        self.storage_enabled_timestamp = time.time()
        print(f"Storage enabled at timestamp: {self.storage_enabled_timestamp:.3f}")
        
        self.status.storage_queued_cnt = 0

        # HDR 모드가 활성화된 경우 알림 메시지
        if self.status.hdr_burst_mode:
            print(
                "Storage enabled in HDR burst mode - only HDR burst frames will be saved"
            )
        else:
            print("Storage enabled - regular frame collection will be saved")

    def disable_storage(self):
        self.storage_id = None
        self.storage_enabled_timestamp = None  # 타임스탬프도 초기화
        print("Storage disabled")

    def _set_cameras_to_minimum_exposure(self):
        """Set all Lucid cameras to minimum exposure for HDR preparation"""

        # Find lucid cameras
        lucid_cameras = []
        for sensor in self.sensor_manager.sensors:
            if "lucid" in sensor.name.lower():
                lucid_cameras.append(sensor)

        print(
            f"Setting {len(lucid_cameras)} Lucid cameras to their respective minimum exposures"
        )

        for camera in lucid_cameras:
            try:
                # 카메라별로 적절한 minimum exposure 결정
                if "lucid_12" in camera.name.lower():
                    minimum_exposure_us = self.hdr_exposure_times_12[0]
                    camera_type = "12-bit"
                else:
                    minimum_exposure_us = self.hdr_exposure_times[0]
                    camera_type = "24-bit"

                if hasattr(camera, "set_exposure_time"):
                    success = camera.set_exposure_time(minimum_exposure_us)
                    if success:
                        print(
                            f"✓ Set minimum exposure for {camera.name} ({camera_type}): {minimum_exposure_us}us"
                        )
                    else:
                        print(
                            f"✗ Failed to set minimum exposure for {camera.name} ({camera_type}): {minimum_exposure_us}us"
                        )
                else:
                    print(f"✗ Camera {camera.name} does not support exposure setting")
            except Exception as e:
                print(f"✗ Error setting minimum exposure for {camera.name}: {e}")

        # Wait for exposure setting to take effect
        time.sleep(1)
        print("Minimum exposure setting completed")

    def enable_hdr_burst_mode(self):
        """Enable HDR burst imaging mode and set exposure to minimum"""
        self.status.hdr_burst_mode = True
        self.status.hdr_burst_in_progress = False

        # 연결된 Lucid 카메라들을 찾아서 각각에 맞는 exposure count 설정
        lucid_cameras = [
            sensor
            for sensor in self.sensor_manager.sensors
            if "lucid" in sensor.name.lower()
        ]

        # HDR 모드 활성화 시 모든 Lucid 카메라의 Auto Exposure(ExposureAuto, GainAuto)를 끕니다.
        # 디바이스가 아직 연결되어 있지 않은 경우에는 상태만 변경해 두고,
        # 디바이스가 연결되면 `config_device`에서 반영됩니다.
        for camera in lucid_cameras:
            try:
                # 상태 객체에 이전 Auto 값이 필요하면 저장(복원을 원할 경우 사용 가능)
                if hasattr(camera, "state"):
                    try:
                        camera.state.prev_exposure_auto = getattr(
                            camera.state, "exposure_auto", True
                        )
                    except Exception:
                        pass
                    camera.state.exposure_auto = False

                # lucid_api 쪽 플래그 갱신
                if hasattr(camera, "lucid_api"):
                    try:
                        camera.lucid_api.exposure_auto = False
                    except Exception:
                        pass

                    # 디바이스가 연결되어 있으면 nodemap에 직접 적용
                    dev = getattr(camera.lucid_api, "device", None)
                    if dev is not None:
                        try:
                            # ExposureAuto와 GainAuto를 Off로 설정
                            set_node_value_safely(dev, "ExposureAuto", "Off")
                            set_node_value_safely(dev, "GainAuto", "Off")
                            print(f"Auto exposure disabled on device for {camera.name}")
                        except Exception as e:
                            print(f"Failed to disable auto exposure for {camera.name}: {e}")
            except Exception as e:
                print(f"Error while disabling auto exposure for {camera.name}: {e}")

        # 진행상황을 카메라별로 맞는 노출 수에 따라 동적 초기화
        self.status.hdr_burst_progress = {}

        for camera in lucid_cameras:
            # 12비트 카메라인지 확인
            if "lucid_12" in camera.name.lower():
                total_exposures = len(self.hdr_exposure_times_12)
                print(
                    f"Camera {camera.name}: 12-bit camera, using {total_exposures} exposure times"
                )
            else:
                total_exposures = len(self.hdr_exposure_times)
                print(
                    f"Camera {camera.name}: 24-bit camera, using {total_exposures} exposure times"
                )

            self.status.hdr_burst_progress[camera.name] = {
                "current_exposure": 0,
                "total_exposures": total_exposures,
            }
        self.status.hdr_burst_current_step = "ready"

        # HDR 모드 활성화 시 모든 Lucid 카메라의 exposure를 최저값(88us)으로 설정
        self._set_cameras_to_minimum_exposure()

        self.socket.emit("status", self.status.__dict__())

    def enable_hdr_high_gain_mode(self):
        """Enable high gain mode for additional dark scene capture"""
        self.status.hdr_high_gain_mode = True
        print(
            "HDR high gain mode enabled - will capture 8 additional frames at gain 4.0"
        )
        self.socket.emit("status", self.status.__dict__())

    def disable_hdr_high_gain_mode(self):
        """Disable high gain mode"""
        self.status.hdr_high_gain_mode = False
        print("HDR high gain mode disabled")
        self.socket.emit("status", self.status.__dict__())

    def set_hdr_exposure_times(self, exposure_times: list):
        """
        HDR 노출 시간 설정

        Args:
            exposure_times: 노출 시간 리스트 (마이크로초 단위)
                예: [88, 880, 8800] - 3단계
                예: [88, 880] - 2단계
                예: [88, 880, 8800, 88000] - 4단계
        """
        if not exposure_times or len(exposure_times) == 0:
            print("Error: exposure_times cannot be empty")
            return False

        # 노출 시간 검증 (최소 88us, 최대 100ms)
        valid_exposures = []
        for exp in exposure_times:
            if isinstance(exp, (int, float)) and 80 <= exp <= 80000:
                valid_exposures.append(int(exp))
            else:
                print(f"Warning: Invalid exposure time {exp}us, skipping")

        if len(valid_exposures) == 0:
            print("Error: No valid exposure times provided")
            return False

        self.hdr_exposure_times = valid_exposures
        print(f"HDR exposure times set to: {self.hdr_exposure_times} microseconds")

        # HDR 모드가 활성화된 경우 진행상황 업데이트
        if hasattr(self.status, "hdr_burst_mode") and self.status.hdr_burst_mode:
            # 연결된 Lucid 카메라들을 찾아서 각각에 맞는 exposure count 설정
            lucid_cameras = [
                sensor
                for sensor in self.sensor_manager.sensors
                if "lucid" in sensor.name.lower()
            ]

            # 진행상황을 카메라별로 맞는 노출 수에 따라 동적 초기화
            self.status.hdr_burst_progress = {}

            for camera in lucid_cameras:
                # 12비트 카메라인지 확인
                if "lucid_12" in camera.name.lower():
                    total_exposures = len(self.hdr_exposure_times_12)
                    print(
                        f"Camera {camera.name}: 12-bit camera, using {total_exposures} exposure times"
                    )
                else:
                    total_exposures = len(self.hdr_exposure_times)
                    print(
                        f"Camera {camera.name}: Updated to 24-bit camera, using {total_exposures} exposure times"
                    )

                self.status.hdr_burst_progress[camera.name] = {
                    "current_exposure": 0,
                    "total_exposures": total_exposures,
                }

            self.socket.emit("status", self.status.__dict__())

        return True

    def get_hdr_exposure_times(self):
        """현재 설정된 HDR 노출 시간 반환"""
        return self.hdr_exposure_times.copy()

    def set_hdr_exposure_times_12(self, exposure_times: list):
        """
        12bit 카메라용 HDR 노출 시간 설정

        Args:
            exposure_times: 노출 시간 리스트 (마이크로초 단위)

        Returns:
            bool: 설정 성공 여부
        """
        if not exposure_times or len(exposure_times) == 0:
            print("Error: exposure_times cannot be empty")
            return False

        # Validate exposure times (must be positive)
        valid_exposures = []
        for exp in exposure_times:
            if isinstance(exp, (int, float)) and exp > 0:
                # Ensure it's within reasonable range (1us to 1 second)
                if 1 <= exp <= 1_000_000:
                    valid_exposures.append(float(exp))
                else:
                    print(
                        f"Warning: Exposure time {exp}us out of range (1-1000000us), skipping"
                    )
            else:
                print(f"Warning: Invalid exposure time {exp}us, skipping")

        if len(valid_exposures) == 0:
            print("Error: No valid exposure times provided")
            return False

        self.hdr_exposure_times_12 = valid_exposures
        print(
            f"HDR 12-bit exposure times set to: {self.hdr_exposure_times_12} microseconds"
        )

        # HDR 모드가 활성화된 경우 진행상황 업데이트
        if hasattr(self.status, "hdr_burst_mode") and self.status.hdr_burst_mode:
            # 연결된 Lucid 카메라들을 찾아서 각각에 맞는 exposure count 설정
            lucid_cameras = [
                sensor
                for sensor in self.sensor_manager.sensors
                if "lucid" in sensor.name.lower()
            ]

            # 진행상황을 카메라별로 맞는 노출 수에 따라 동적 초기화
            self.status.hdr_burst_progress = {}

            for camera in lucid_cameras:
                # 12비트 카메라인지 확인
                if "lucid_12" in camera.name.lower():
                    total_exposures = len(self.hdr_exposure_times_12)
                    print(
                        f"Camera {camera.name}: Updated to 12-bit camera, using {total_exposures} exposure times"
                    )
                else:
                    total_exposures = len(self.hdr_exposure_times)
                    print(
                        f"Camera {camera.name}: 24-bit camera, using {total_exposures} exposure times"
                    )

                self.status.hdr_burst_progress[camera.name] = {
                    "current_exposure": 0,
                    "total_exposures": total_exposures,
                }

            self.socket.emit("status", self.status.__dict__())

        return True

    def get_hdr_exposure_times_12(self):
        """현재 설정된 12bit HDR 노출 시간 반환"""
        return self.hdr_exposure_times_12.copy()

    def disable_hdr_burst_mode(self):
        """Disable HDR burst imaging mode and reset exposure to minimum"""
        self.status.hdr_burst_mode = False
        self.status.hdr_burst_in_progress = False
        self.status.hdr_burst_current_step = ""

        # HDR 모드 비활성화 시 알림
        if self.storage_id is not None:
            print("HDR burst mode disabled - resuming regular frame storage")

        # HDR 모드 비활성화 시에도 exposure를 최소값으로 재설정
        print("HDR burst mode disabled - resetting cameras to minimum exposure")
        self._set_cameras_to_minimum_exposure()

        self.socket.emit("status", self.status.__dict__())

    def capture_hdr_burst_frame(self):
        """Capture a single HDR burst frame with 4 different exposures"""
        if self.status.hdr_burst_in_progress:
            return {
                "status": "error",
                "message": "HDR burst capture already in progress",
            }

        def hdr_burst_worker():
            try:
                # HDR 촬영 시작 시점 기록
                hdr_capture_start_time = time.time()
                
                self.status.hdr_burst_in_progress = True
                self.status.hdr_burst_current_step = "preparing"
                self.socket.emit("status", self.status.__dict__())

                # Find lucid cameras and categorize by type
                lucid_cameras = []
                lucid_12_cameras = []
                lucid_24_cameras = []

                for sensor in self.sensor_manager.sensors:
                    if "lucid" in sensor.name.lower():
                        lucid_cameras.append(sensor)
                        if "lucid_12" in sensor.name.lower():
                            lucid_12_cameras.append(sensor)
                        else:
                            lucid_24_cameras.append(sensor)

                # 카메라별 노출 시간 매핑 생성
                camera_exposure_map = {}
                for camera in lucid_cameras:
                    if "lucid_12" in camera.name.lower():
                        camera_exposure_map[camera.name] = (
                            self.hdr_exposure_times_12.copy()
                        )
                        print(
                            f"Camera {camera.name}: Using 12-bit exposure times {camera_exposure_map[camera.name]}"
                        )
                    else:
                        camera_exposure_map[camera.name] = (
                            self.hdr_exposure_times.copy()
                        )
                        print(
                            f"Camera {camera.name}: Using 24-bit exposure times {camera_exposure_map[camera.name]}"
                        )

                if len(lucid_cameras) == 0:
                    raise Exception("No Lucid cameras found")

                # 가장 긴 exposure 시퀀스 길이 찾기 (전체 캡처 진행을 위해)
                max_exposure_count = (
                    max(len(exposures) for exposures in camera_exposure_map.values())
                    if camera_exposure_map
                    else 0
                )

                # Collect HDR data
                hdr_frame_data: dict = {}
                timestamp = time.time()

                # HDR 캡처 시작 전 모든 카메라를 최소 exposure로 초기화
                print("Starting HDR capture - ensuring all cameras at minimum exposure")
                self._set_cameras_to_minimum_exposure()

                # 카메라별로 각각의 exposure sequence를 처리
                for exposure_step in range(max_exposure_count):
                    self.status.hdr_burst_current_step = (
                        f"capturing_step_{exposure_step + 1}"
                    )

                    # 이번 스텝에서 촬영할 카메라와 exposure 설정
                    cameras_to_capture = []
                    for camera in lucid_cameras:
                        camera_exposures = camera_exposure_map[camera.name]
                        if exposure_step < len(camera_exposures):
                            exposure_us = camera_exposures[exposure_step]
                            cameras_to_capture.append((camera, exposure_us))

                    if not cameras_to_capture:
                        continue

                    # exposure 설정
                    exposure_update_retries = 3
                    successful_cameras = []  # 성공적으로 exposure가 설정된 카메라들

                    while exposure_update_retries > 0:
                        all_set = True
                        temp_successful_cameras = []

                        for camera, exposure_us in cameras_to_capture:
                            # 마지막 exposure인지 확인
                            camera_exposures = camera_exposure_map[camera.name]
                            is_last_exposure = (
                                exposure_step == len(camera_exposures) - 1
                            )

                            try:
                                success = camera.set_exposure_time(exposure_us)
                                if not success:
                                    if is_last_exposure:
                                        print(
                                            f"⚠ Last exposure {exposure_us}us failed for {camera.name} - skipping this exposure"
                                        )
                                        # 마지막 exposure 실패시 해당 카메라는 제외하지만 전체 진행은 계속
                                    else:
                                        print(
                                            f"✗ Failed to set exposure for {camera.name} to {exposure_us}us"
                                        )
                                        all_set = False
                                else:
                                    print(
                                        f"✓ Set exposure for {camera.name} to {exposure_us}us"
                                    )
                                    temp_successful_cameras.append(
                                        (camera, exposure_us)
                                    )

                            except Exception as e:
                                if is_last_exposure:
                                    print(
                                        f"⚠ Last exposure {exposure_us}us error for {camera.name}: {e} - skipping this exposure"
                                    )
                                else:
                                    print(
                                        f"✗ Error setting exposure for {camera.name}: {e}"
                                    )
                                    all_set = False

                        if all_set or exposure_update_retries == 1:
                            successful_cameras = temp_successful_cameras
                            break
                        else:
                            exposure_update_retries -= 1
                            time.sleep(1)

                    if not successful_cameras:
                        print(
                            f"⚠ No cameras successfully set for exposure step {exposure_step + 1}, skipping frame collection"
                        )
                        continue

                    # 성공한 카메라들의 진행상황 업데이트
                    for camera, exposure_us in successful_cameras:
                        self.status.hdr_burst_progress[camera.name][
                            "current_exposure"
                        ] = (exposure_step + 1)

                    self.socket.emit("status", self.status.__dict__())

                    time.sleep(3.0)

                    # 프레임 수집 (성공한 카메라들만)
                    for camera, exposure_us in successful_cameras:
                        try:
                            # Lucid 카메라에서 LucidImage 객체를 직접 가져옴 (메타데이터 포함)
                            if (
                                hasattr(camera, "get_latest_frame")
                                and "lucid" in camera.name.lower()
                            ):
                                lucid_image = camera.get_latest_frame()
                                if lucid_image:
                                    if camera.name not in hdr_frame_data:
                                        hdr_frame_data[camera.name] = []

                                    hdr_frame_data[camera.name].append(lucid_image)
                                    print(
                                        f"✓ Collected HDR LucidImage from {camera.name} at {exposure_us}us"
                                    )
                                else:
                                    print(
                                        f"✗ No valid HDR LucidImage collected from {camera.name} at {exposure_us}us"
                                    )
                            else:
                                # 다른 카메라는 기존 방식
                                frame = camera.get_latest_frame()
                                if frame and "image" in frame.data:
                                    if camera.name not in hdr_frame_data:
                                        hdr_frame_data[camera.name] = []
                                    hdr_frame_data[camera.name].append(frame)
                                    print(
                                        f"✓ Collected HDR frame from {camera.name} at {exposure_us}us"
                                    )
                                else:
                                    print(
                                        f"✗ No valid HDR frame collected from {camera.name} at {exposure_us}us"
                                    )
                        except Exception as e:
                            print(
                                f"✗ Failed to capture HDR frame from {camera.name}: {e}"
                            )

                # High-gain burst mode (optional)
                if self.status.hdr_high_gain_mode:
                    try:
                        self.status.hdr_burst_current_step = "high_gain_burst"
                        self.socket.emit("status", self.status.__dict__())

                        print("Starting high-gain burst mode...")
                        high_gain_value = 20.0  # 고정 고득 값
                        high_gain_frames_count = 8  # 8장 추가 촬영

                        # Set high gain for all cameras
                        for camera in lucid_cameras:
                            if hasattr(camera, "set_gain"):
                                if camera.set_gain(high_gain_value):
                                    print(
                                        f"Set high gain {high_gain_value} for {camera.name}"
                                    )
                                else:
                                    print(f"Failed to set high gain for {camera.name}")

                        # Capture additional LucidImage frames with high gain
                        high_gain_lucid_frames = {}
                        for frame_idx in range(high_gain_frames_count):
                            self.status.hdr_high_gain_burst_count = frame_idx + 1
                            self.socket.emit("status", self.status.__dict__())

                            print(
                                f"Capturing high-gain frame {frame_idx + 1}/{high_gain_frames_count}"
                            )

                            # Collect LucidImage frames from cameras
                            for camera in lucid_cameras:
                                try:
                                    time.sleep(0.2)  # 프레임 간격 대기

                                    # Collect LucidImage with retries
                                    lucid_image = None
                                    max_retries = 3
                                    for retry in range(max_retries):
                                        try:
                                            if (
                                                hasattr(camera, "get_latest_frame")
                                                and "lucid" in camera.name.lower()
                                            ):
                                                lucid_image = camera.get_latest_frame()
                                                if lucid_image and hasattr(
                                                    lucid_image, "buffer_np"
                                                ):
                                                    break
                                            time.sleep(0.05)
                                        except Exception as frame_e:
                                            print(
                                                f"High-gain frame collection attempt {retry + 1} failed for {camera.name}: {frame_e}"
                                            )
                                            if retry < max_retries - 1:
                                                time.sleep(0.2)

                                    if lucid_image:
                                        if camera.name not in high_gain_lucid_frames:
                                            high_gain_lucid_frames[camera.name] = []
                                        high_gain_lucid_frames[camera.name].append(
                                            lucid_image
                                        )
                                        print(
                                            f"✓ Collected high-gain LucidImage from {camera.name}"
                                        )
                                    else:
                                        print(
                                            f"No high-gain LucidImage collected from {camera.name}"
                                        )

                                except Exception as e:
                                    print(
                                        f"Failed to capture high-gain LucidImage from {camera.name}: {e}"
                                    )

                        # Reset gain to default for all cameras
                        for camera in lucid_cameras:
                            if hasattr(camera, "set_gain"):
                                if camera.set_gain(1.0):  # 기본 게인으로 복원
                                    print(f"Reset gain to 1.0 for {camera.name}")
                                else:
                                    print(f"Failed to reset gain for {camera.name}")

                        # High-gain 완료 후 exposure도 최소값으로 재설정
                        print(
                            "High-gain burst completed - resetting exposure to minimum"
                        )
                        self._set_cameras_to_minimum_exposure()

                        print(
                            f"High-gain burst completed: captured {high_gain_frames_count} additional frames"
                        )

                        # Process high-gain LucidImage frames using the new module
                        for sensor_name, lucid_frames in high_gain_lucid_frames.items():
                            if len(lucid_frames) > 0:
                                # 고gain 처리 시작 알림
                                self.status.hdr_high_gain_processing = True
                                self.status.hdr_high_gain_processing_progress[
                                    sensor_name
                                ] = {
                                    "total_frames": len(lucid_frames),
                                    "current_step": "starting",
                                    "progress_percent": 0,
                                }
                                self.socket.emit("status", self.status.__dict__())

                                print(
                                    f"Processing high-gain LucidImage frames for {sensor_name}: {len(lucid_frames)} frames"
                                )

                                # 진행상황 콜백 함수 정의
                                def progress_callback(step, percent):
                                    self.status.hdr_high_gain_processing_progress[
                                        sensor_name
                                    ]["current_step"] = step
                                    self.status.hdr_high_gain_processing_progress[
                                        sensor_name
                                    ]["progress_percent"] = percent
                                    self.socket.emit("status", self.status.__dict__())

                                try:
                                    # 새로운 모듈을 사용하여 LucidImage 처리
                                    merged_image, processing_stats = (
                                        process_high_gain_frames(
                                            lucid_frames, progress_callback
                                        )
                                    )

                                    # 통계 정보 업데이트
                                    self.status.hdr_high_gain_processing_progress[
                                        sensor_name
                                    ].update(
                                        {
                                            "kept_frames": processing_stats.get(
                                                "kept_frames", len(lucid_frames)
                                            ),
                                            "dropped_frames": processing_stats.get(
                                                "dropped_frames", 0
                                            ),
                                        }
                                    )

                                    # 첫 번째 LucidImage를 기반으로 새로운 LucidImage 생성
                                    merged_lucid_image = lucid_frames[
                                        0
                                    ]  # 메타데이터 유지를 위해 첫 번째 이미지 사용

                                    # 합성된 이미지를 LucidImage의 buffer_np에 저장
                                    # float32 -> 원래 포맷으로 변환 (24비트의 경우)
                                    if (
                                        len(merged_image.shape) == 2
                                    ):  # 단일 채널 float32
                                        # 24비트로 다시 변환
                                        merged_24bit = (
                                            merged_image * (2**24 - 1)
                                        ).astype(np.uint32)
                                        # 3채널 uint8로 분할
                                        channel0 = (merged_24bit & 0xFF).astype(
                                            np.uint8
                                        )
                                        channel1 = ((merged_24bit >> 8) & 0xFF).astype(
                                            np.uint8
                                        )
                                        channel2 = ((merged_24bit >> 16) & 0xFF).astype(
                                            np.uint8
                                        )
                                        merged_lucid_image.buffer_np = np.stack(
                                            [channel0, channel1, channel2], axis=2
                                        )
                                    else:
                                        # 이미 적절한 형식인 경우
                                        merged_lucid_image.buffer_np = (
                                            merged_image.astype(np.uint8)
                                        )

                                    # HDR 프레임 데이터에 처리된 LucidImage 추가
                                    hdr_frame_data[sensor_name].append(
                                        merged_lucid_image
                                    )

                                    print(
                                        f"✓ Processed high-gain LucidImage frames for {sensor_name}"
                                    )
                                    print(
                                        f"  - Total frames: {processing_stats.get('total_frames', 0)}"
                                    )
                                    print(
                                        f"  - Kept frames: {processing_stats.get('kept_frames', 0)}"
                                    )
                                    print(
                                        f"  - Dropped frames: {processing_stats.get('dropped_frames', 0)}"
                                    )

                                except Exception as process_e:
                                    print(
                                        f"Error processing high-gain frames for {sensor_name}: {process_e}"
                                    )
                                    # 오류 발생시 첫 번째 프레임만 사용
                                    if lucid_frames:
                                        hdr_frame_data[sensor_name].append(
                                            lucid_frames[0]
                                        )

                                # Progress: 100% - 완료
                                self.status.hdr_high_gain_processing_progress[
                                    sensor_name
                                ]["progress_percent"] = 100
                                self.status.hdr_high_gain_processing_progress[
                                    sensor_name
                                ]["current_step"] = "completed"
                                self.socket.emit("status", self.status.__dict__())

                        # 모든 카메라 처리 완료
                        self.status.hdr_high_gain_processing = False
                        self.socket.emit("status", self.status.__dict__())

                    except Exception as hg_e:
                        print(f"High-gain burst error: {hg_e}")
                        # Reset gain on error
                        for camera in lucid_cameras:
                            try:
                                if hasattr(camera, "set_gain"):
                                    camera.set_gain(1.0)
                            except:
                                pass
                        # 오류 발생시 처리 상태 초기화
                        self.status.hdr_high_gain_processing = False
                        self.socket.emit("status", self.status.__dict__())

                # Collect data from other sensors (Lidar, RealSense) if available
                self.status.hdr_burst_current_step = "collecting_other_sensors"
                self.socket.emit("status", self.status.__dict__())

                other_sensor_data = {}
                for sensor in self.sensor_manager.sensors:
                    if "lucid" not in sensor.name.lower():
                        try:
                            if hasattr(sensor, "get_latest_frame"):
                                frame = sensor.get_latest_frame()
                                if frame:
                                    other_sensor_data[sensor.name] = frame
                            elif hasattr(sensor, "state") and hasattr(
                                sensor.state, "timestamp_last"
                            ):
                                # Try to get the most recent data from the synchronized queue
                                if (
                                    sensor.name in self.queue.queue_dict
                                    and len(self.queue.queue_dict[sensor.name]) > 0
                                ):
                                    latest_item = self.queue.queue_dict[sensor.name][-1]
                                    other_sensor_data[sensor.name] = latest_item
                        except Exception as e:
                            print(f"Failed to collect data from {sensor.name}: {e}")

                # Combine all data into a single frame
                combined_frame_data = {**hdr_frame_data, **other_sensor_data}

                print(
                    f"HDR DEBUG: Combined frame data keys: {list(combined_frame_data.keys())}"
                )
                for key, value in combined_frame_data.items():
                    if isinstance(value, list):
                        print(f"HDR DEBUG: {key} has {len(value)} items")
                    else:
                        print(f"HDR DEBUG: {key} type: {type(value)}")
                # Store the HDR burst frame
                if self.storage_id is not None:
                    # storage_enabled_timestamp 체크 (HDR 촬영 시작 시점 기준)
                    hdr_capture_start_time = time.time()
                    if self.storage_enabled_timestamp is not None and hdr_capture_start_time < self.storage_enabled_timestamp:
                        print(f"Skipping HDR frame: capture started before storage was enabled")
                        print(f"  Capture time: {hdr_capture_start_time:.3f}, Enabled time: {self.storage_enabled_timestamp:.3f}")
                    else:
                        self.status.hdr_burst_current_step = "saving"
                        self.socket.emit("status", self.status.__dict__())

                        # Create a special HDR frame identifier
                        hdr_timestamp = f"{timestamp}_hdr_burst"

                        print(f"HDR DEBUG: About to store HDR data")
                        print(f"HDR DEBUG: Storage ID: {self.storage_id}")
                        print(f"HDR DEBUG: HDR timestamp: {hdr_timestamp}")

                        # HDR 데이터 저장을 위한 전용 메서드 사용
                        self.storage.enqueue_hdr_data(
                            self.storage_id, hdr_timestamp, combined_frame_data
                        )
                        self.status.storage_queued_cnt += 1
                        print(
                            f"HDR burst data queued for storage: {self.storage_id}/{hdr_timestamp}"
                        )
                else:
                    print("Storage not enabled - HDR burst data not saved")

                # Reset progress
                for camera_name in self.status.hdr_burst_progress:
                    self.status.hdr_burst_progress[camera_name]["current_exposure"] = 0

                self.status.hdr_burst_current_step = "completed"
                self.socket.emit("status", self.status.__dict__())

            except Exception as e:
                self.status.hdr_burst_current_step = f"error: {str(e)}"
                self.socket.emit("status", self.status.__dict__())
                traceback.print_exc()
                print(f"HDR burst capture error: {e}")
            finally:
                self.status.hdr_burst_in_progress = False
                self.status.hdr_high_gain_burst_count = 0  # Reset high-gain burst count
                # Reset progress
                for camera_name in self.status.hdr_burst_progress:
                    self.status.hdr_burst_progress[camera_name]["current_exposure"] = 0

                # HDR 캡처 완료 후 모든 카메라의 exposure를 최소값으로 재설정
                print(
                    "HDR burst capture completed - resetting cameras to minimum exposure"
                )
                self._set_cameras_to_minimum_exposure()

                self.socket.emit("status", self.status.__dict__())

        # Run HDR capture in separate thread to avoid blocking
        threading.Thread(target=hdr_burst_worker, daemon=True).start()

        return {"status": "success", "message": "HDR burst capture started"}

    def __del__(self):
        # del self.lucid_api
        del self.ouster_bridge


app = Flask(__name__)
CORS(app)
socketio = SocketIO(
    app,
    cors_allowed_origins="*",
    async_mode="threading",
    logger=False,
    engineio_logger=False,
)

node: LucidStereoNode


@app.route("/trigger", methods=["GET"])
def trigger_capture():
    node.trigger_camera_capture()
    return Response(status=200)


@app.route("/queue_status", methods=["GET"])
def queue_status():
    output = {
        "stereo_queue": node.stereo_queue.queue_status(),
        "lidar_queue": node.image_lidar_queue.queue_status(),
    }
    return Response(
        status=200,
        response=json.dumps(output),
        content_type="application/json",
    )


@app.route("/stream/preview/<timestamp>")
def stereo_disparity_videostream(timestamp):
    return Response(
        node.stereo_stream.generate_preview(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/storage/enable", methods=["POST"])
def enable_storage():
    node.enable_storage()
    return Response(status=200)


@app.route("/storage/disable", methods=["POST"])
def disable_storage():
    node.disable_storage()
    return Response(status=200)


@app.route("/hdr/enable", methods=["POST"])
def enable_hdr_burst():
    """Enable HDR burst imaging mode"""
    node.enable_hdr_burst_mode()
    return Response(
        status=200,
        response=json.dumps({"status": "success", "message": "HDR burst mode enabled"}),
        content_type="application/json",
    )


@app.route("/hdr/disable", methods=["POST"])
def disable_hdr_burst():
    """Disable HDR burst imaging mode"""
    node.disable_hdr_burst_mode()
    return Response(
        status=200,
        response=json.dumps(
            {"status": "success", "message": "HDR burst mode disabled"}
        ),
        content_type="application/json",
    )


@app.route("/hdr/capture", methods=["POST"])
def capture_hdr_burst():
    """Trigger HDR burst capture"""
    result = node.capture_hdr_burst_frame()
    return Response(
        status=200 if result["status"] == "success" else 400,
        response=json.dumps(result),
        content_type="application/json",
    )


@app.route("/hdr/high_gain/enable", methods=["POST"])
def enable_hdr_high_gain():
    """Enable HDR high-gain mode"""
    node.enable_hdr_high_gain_mode()
    return Response(
        status=200,
        response=json.dumps(
            {"status": "success", "message": "HDR high-gain mode enabled"}
        ),
        content_type="application/json",
    )


@app.route("/hdr/high_gain/disable", methods=["POST"])
def disable_hdr_high_gain():
    """Disable HDR high-gain mode"""
    node.disable_hdr_high_gain_mode()
    return Response(
        status=200,
        response=json.dumps(
            {"status": "success", "message": "HDR high-gain mode disabled"}
        ),
        content_type="application/json",
    )


@app.route("/status/update", methods=["POST"])
def update_status():

    attr_dict = request.json if request.json is not None else {}

    for key, value in attr_dict.items():
        if hasattr(node.status, key):
            setattr(node.status, key, value)

    return Response(status=200)


@app.route("/sensors", methods=["GET"])
def sensor_manger_get_sensor_list():
    sensors = node.sensor_manager.get_all_sensor_info()
    return Response(status=200, response=json.dumps(sensors))


@app.route("/sensors/groups", methods=["GET"])
def sensor_manger_get_group_list():
    groups = node.sensor_manager.get_all_group_info()
    return Response(status=200, response=json.dumps(groups))


@app.route("/sensors/group/<group_name>/trigger", methods=["POST"])
def sensor_manager_trigger_group(group_name):
    node.sensor_manager.trigger_group(group_name)
    return node.sensor_manager.get_all_group_info()


@app.route("/sensors/group/<group_name>/trigger/loop/start", methods=["POST"])
def sensor_manager_group_trigger_loop_on(group_name):
    node.sensor_manager.group_trigger_loop_on(group_name, True)
    return node.sensor_manager.get_all_group_info()


@app.route("/sensors/group/<group_name>/trigger/loop/stop", methods=["POST"])
def sensor_manager_group_trigger_loop_off(group_name):
    node.sensor_manager.group_trigger_loop_on(group_name, False)
    return node.sensor_manager.get_all_group_info()


@app.route("/sensors/<sensor_name>/stream/start", methods=["POST"])
def sensor_manager_set_stream_on(sensor_name):
    node.sensor_manager.stream_on(sensor_name, True)
    return node.sensor_manager.get_all_sensor_info()


@app.route("/sensors/<sensor_name>/config/<config_name>", methods=["POST"])
def sensor_manager_set_config(sensor_name, config_name):
    value = request.json["value"]
    node.sensor_manager.search_sensor_by_name(sensor_name).update_config(
        config_name, value
    )
    return node.sensor_manager.get_all_sensor_info()


@app.route("/sensors/<sensor_name>/stream/stop", methods=["POST"])
def sensor_manager_set_stream_off(sensor_name):
    node.sensor_manager.stream_on(sensor_name, False)
    return node.sensor_manager.get_all_sensor_info()


@app.route("/sensors/<sensor_name>/preview/on", methods=["POST"])
def sensor_manager_set_preview_on(sensor_name):
    node.sensor_manager.preview_on(sensor_name, True)
    return node.sensor_manager.get_all_sensor_info()


@app.route("/sensors/<sensor_name>/preview/off", methods=["POST"])
def sensor_manager_set_preview_off(sensor_name):
    node.sensor_manager.preview_on(sensor_name, False)
    return node.sensor_manager.get_all_sensor_info()


@app.route("/sensors/<sensor_name>/launch", methods=["POST"])
def sensor_manager_launch_device(sensor_name):
    node.sensor_manager.launch_device(sensor_name)
    return node.sensor_manager.get_all_sensor_info()


@app.route("/sensors/all/refresh", methods=["POST"])
def sensor_manager_refresh_all():
    node.sensor_manager.refresh_all()
    return node.sensor_manager.get_all_sensor_info()


@app.route("/hdr/exposure_times", methods=["GET"])
def get_hdr_exposure_times():
    """현재 설정된 HDR 노출 시간 반환"""
    return {
        "status": "success",
        "exposure_times": node.get_hdr_exposure_times(),
        "count": len(node.get_hdr_exposure_times()),
    }


@app.route("/hdr/exposure_times", methods=["POST"])
def set_hdr_exposure_times():
    """HDR 노출 시간 설정"""
    try:
        data = request.get_json()
        if not data or "exposure_times" not in data:
            return {
                "status": "error",
                "message": "exposure_times required in request body",
            }, 400

        exposure_times = data["exposure_times"]
        if not isinstance(exposure_times, list):
            return {"status": "error", "message": "exposure_times must be a list"}, 400

        success = node.set_hdr_exposure_times(exposure_times)
        if success:
            return {
                "status": "success",
                "message": f"HDR exposure times set to {node.get_hdr_exposure_times()}",
                "exposure_times": node.get_hdr_exposure_times(),
                "count": len(node.get_hdr_exposure_times()),
            }
        else:
            return {
                "status": "error",
                "message": "Failed to set HDR exposure times",
            }, 400

    except Exception as e:
        return {
            "status": "error",
            "message": f"Error setting HDR exposure times: {str(e)}",
        }, 500


@app.route("/hdr/exposure_times_12", methods=["GET"])
def get_hdr_exposure_times_12():
    """현재 설정된 12bit HDR 노출 시간 반환"""
    return {
        "status": "success",
        "exposure_times_12": node.get_hdr_exposure_times_12(),
        "count": len(node.get_hdr_exposure_times_12()),
    }


@app.route("/hdr/exposure_times_12", methods=["POST"])
def set_hdr_exposure_times_12():
    """12bit 카메라용 HDR 노출 시간 설정"""
    try:
        data = request.get_json()
        if not data or "exposure_times" not in data:
            return {
                "status": "error",
                "message": "exposure_times required in request body",
            }, 400

        exposure_times = data["exposure_times"]
        if not isinstance(exposure_times, list):
            return {"status": "error", "message": "exposure_times must be a list"}, 400

        success = node.set_hdr_exposure_times_12(exposure_times)
        if success:
            return {
                "status": "success",
                "message": f"HDR 12-bit exposure times set to {node.get_hdr_exposure_times_12()}",
                "exposure_times_12": node.get_hdr_exposure_times_12(),
                "count": len(node.get_hdr_exposure_times_12()),
            }
        else:
            return {
                "status": "error",
                "message": "Failed to set HDR 12-bit exposure times",
            }, 400

    except Exception as e:
        return {
            "status": "error",
            "message": f"Error setting HDR 12-bit exposure times: {str(e)}",
        }, 500


@app.route("/lucid/devices", methods=["GET"])
def get_lucid_devices():
    """연결된 모든 Lucid 디바이스 정보 반환"""
    try:
        device_info = _device_manager.get_device_info()
        return {
            "status": "success",
            "devices": [
                {"serial": info["serial"], "model": info["model"]}
                for info in device_info
            ],
            "count": len(device_info),
        }
    except Exception as e:
        return {
            "status": "error",
            "message": f"Error getting Lucid devices: {str(e)}",
        }, 500


@app.route("/lucid/devices/refresh", methods=["POST"])
def refresh_lucid_devices():
    """Lucid 디바이스 목록을 강제로 새로고침"""
    try:
        device_info = _device_manager.refresh_devices()
        # Get device information safely from global device manager
        device_manager = get_global_device_manager()
        device_info_list = device_manager.get_device_info()

        return {
            "status": "success",
            "message": "Device list refreshed",
            "devices": device_info_list,
            "count": len(device_info_list),
        }
    except Exception as e:
        return {
            "status": "error",
            "message": f"Error refreshing Lucid devices: {str(e)}",
        }, 500


@app.route("/sensors/<sensor_name>/stream/status", methods=["GET"])
def get_sensor_stream_status(sensor_name):
    """특정 센서의 스트림 상태 정보를 반환"""
    try:
        sensor = node.sensor_manager.search_sensor_by_name(sensor_name)
        if sensor is None:
            return {
                "status": "error",
                "message": f"Sensor '{sensor_name}' not found",
            }, 404

        # Lucid 카메라인지 확인하고 스트림 상태 반환
        if hasattr(sensor, "lucid_api") and hasattr(
            sensor.lucid_api, "get_stream_status"
        ):
            stream_status = sensor.lucid_api.get_stream_status()
            device_status = sensor.lucid_api.get_device_status()

            return {
                "status": "success",
                "sensor_name": sensor_name,
                "stream_status": stream_status,
                "device_status": device_status,
            }
        else:
            return {
                "status": "error",
                "message": f"Sensor '{sensor_name}' does not support stream status monitoring",
            }, 400

    except Exception as e:
        return {
            "status": "error",
            "message": f"Error getting stream status: {str(e)}",
        }, 500


@app.route("/sensors/<sensor_name>/stream/recover", methods=["POST"])
def recover_sensor_stream(sensor_name):
    """특정 센서의 스트림을 수동으로 복구"""
    try:
        sensor = node.sensor_manager.search_sensor_by_name(sensor_name)
        if sensor is None:
            return {
                "status": "error",
                "message": f"Sensor '{sensor_name}' not found",
            }, 404

        # Lucid 카메라인지 확인하고 스트림 복구 시도
        if hasattr(sensor, "lucid_api") and hasattr(
            sensor.lucid_api, "_attempt_stream_recovery"
        ):
            recovery_result = sensor.lucid_api._attempt_stream_recovery()

            return {
                "status": "success" if recovery_result else "failed",
                "sensor_name": sensor_name,
                "message": f"Stream recovery {'successful' if recovery_result else 'failed'} for {sensor_name}",
                "recovery_result": recovery_result,
            }
        else:
            return {
                "status": "error",
                "message": f"Sensor '{sensor_name}' does not support stream recovery",
            }, 400

    except Exception as e:
        return {
            "status": "error",
            "message": f"Error during stream recovery: {str(e)}",
        }, 500


@app.route("/sensors/stream/status", methods=["GET"])
def get_all_sensors_stream_status():
    """모든 센서의 스트림 상태 정보를 반환"""
    try:
        all_status = {}

        for sensor in node.sensor_manager.sensors:
            if hasattr(sensor, "lucid_api") and hasattr(
                sensor.lucid_api, "get_stream_status"
            ):
                stream_status = sensor.lucid_api.get_stream_status()
                device_status = sensor.lucid_api.get_device_status()

                all_status[sensor.name] = {
                    "stream_status": stream_status,
                    "device_status": device_status,
                }

        return {
            "status": "success",
            "sensors": all_status,
            "total_sensors": len(all_status),
        }

    except Exception as e:
        return {
            "status": "error",
            "message": f"Error getting sensors stream status: {str(e)}",
        }, 500


@app.route("/sensors/stream/health_check", methods=["GET"])
def sensors_health_check():
    """모든 센서의 스트림 건강성을 체크하고 문제가 있는 센서 반환"""
    try:
        unhealthy_sensors = []
        healthy_sensors = []

        for sensor in node.sensor_manager.sensors:
            if hasattr(sensor, "lucid_api") and hasattr(
                sensor.lucid_api, "_is_stream_healthy"
            ):
                is_healthy = sensor.lucid_api._is_stream_healthy()
                stream_status = sensor.lucid_api.get_stream_status()

                sensor_info = {
                    "name": sensor.name,
                    "serial": getattr(sensor.lucid_api, "SERIAL", "Unknown"),
                    "stream_status": stream_status,
                }

                if is_healthy:
                    healthy_sensors.append(sensor_info)
                else:
                    unhealthy_sensors.append(sensor_info)

        return {
            "status": "success",
            "healthy_sensors": healthy_sensors,
            "unhealthy_sensors": unhealthy_sensors,
            "total_healthy": len(healthy_sensors),
            "total_unhealthy": len(unhealthy_sensors),
        }

    except Exception as e:
        return {
            "status": "error",
            "message": f"Error during health check: {str(e)}",
        }, 500


def spin_node():
    rclpy.spin(node)


with app.app_context():
    rclpy.init()
    node = LucidStereoNode(socketio)
    threading.Thread(target=spin_node, daemon=True).start()


if __name__ == "__main__":
    # spin_node()
    import logging

    logging.getLogger("werkzeug").disabled = True
    socketio.run(app, port=5021, host="0.0.0.0")
