from concurrent.futures import thread
from platform import node
from sys import thread_info
import threading
from arena_api.system import system
from arena_api import enums
from numpy import isin, tri
import json
import os


TAB1 = "  "
TAB2 = "    "
import time
from typing import Callable, List, Literal, Optional, Dict, Any
from arena_api._device import Device
from arena_api._node import NodeCommand
from arena_api.buffer import _Buffer
import numpy as np
from std_msgs.msg import Header

RESOLUTION_12 = (2048, 1536)
RESOLUTION = (2880, 1856)
BINNING = (2, 2)
RESDOWN = (2, 2)


def update_node_safely(device, node_name: str, value=None, execute=False):
    """
    안전하게 nodemap 노드를 업데이트하거나 값을 가져옵니다.

    Args:
        device: Device 객체
        node_name: 노드 이름
        value: 설정할 값 (None이면 값을 가져옴)
        execute: True이면 노드를 실행 (command 노드용)

    Returns:
        성공 시: (True, value_or_result)
        실패 시: (False, error_message)
    """
    node = None
    try:
        if device is None or not hasattr(device, "nodemap"):
            return False, "Device is None or has no nodemap"

        node = device.nodemap.get_node(node_name)
        if node is None:
            return False, f"Node '{node_name}' not found"

        if execute:
            # Command 노드 실행
            node.execute()
            return True, "Command executed"
        elif value is not None:
            # 값 설정
            node.value = value
            return True, f"Set {node_name} = {value}"
        else:
            # 값 가져오기
            return True, node.value

    except Exception as e:
        error_msg = (
            f"Error accessing node '{node_name}' while setting {value}: {str(e)}"
        )
        print(node)
        print(f"{TAB1}{error_msg}")

        return False, error_msg


def get_node_value_safely(device, node_name: str, default_value=None):
    """
    안전하게 노드 값을 가져옵니다.

    Args:
        device: Device 객체
        node_name: 노드 이름
        default_value: 실패 시 반환할 기본값

    Returns:
        노드 값 또는 기본값
    """
    success, result = update_node_safely(device, node_name)
    return result if success else default_value


def set_node_value_safely(device, node_name: str, value):
    """
    안전하게 노드 값을 설정합니다.

    Args:
        device: Device 객체
        node_name: 노드 이름
        value: 설정할 값

    Returns:
        성공 여부 (bool)
    """
    success, _ = update_node_safely(device, node_name, value)
    return success


def execute_node_safely(device, node_name: str):
    """
    안전하게 노드를 실행합니다 (command 노드용).

    Args:
        device: Device 객체
        node_name: 노드 이름

    Returns:
        성공 여부 (bool)
    """
    success, _ = update_node_safely(device, node_name, execute=True)
    return success


class LucidDeviceManager:
    """전역 Lucid 디바이스 관리자 - 모든 LucidCamera 인스턴스가 공유"""

    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
                    cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return
        self._devices = None
        self._devices_lock = threading.Lock()
        self._last_scan_time = 0.0  # float으로 초기화
        self._scan_interval = 30  # 30초마다 재스캔
        self._initialized = True

    def get_devices(self, force_rescan=False):
        """디바이스 목록을 가져옵니다. 필요시에만 스캔을 수행합니다."""
        current_time = time.time()
        print(f"{TAB1}get_devices called (force_rescan={force_rescan})")
        with self._devices_lock:
            # 강제 재스캔이거나, 아직 스캔하지 않았거나, 스캔 간격이 지났으면 재스캔
            if (
                force_rescan
                or self._devices is None
                or (current_time - self._last_scan_time) > self._scan_interval
            ):

                print(f"{TAB1}Scanning for Lucid devices...")
                self._devices = self._create_devices_with_tries()
                self._last_scan_time = current_time

                if self._devices:
                    print(f"{TAB1}Found {len(self._devices)} Lucid devices:")
                    for device in self._devices:
                        serial = device.nodemap.get_node("DeviceSerialNumber").value
                        model = device.nodemap.get_node("DeviceModelName").value
                        print(f"{TAB2}Device: {model} (Serial: {serial})")
                else:
                    print(f"{TAB1}No Lucid devices found")

            return self._devices

    def _create_devices_with_tries(self):
        """
        디바이스 연결을 시도합니다. 기존 create_devices_with_tries와 동일한 로직
        """
        tries = 0
        tries_max = 6
        sleep_time_secs = 10

        while tries < tries_max:  # Wait for device for 60 seconds
            print(f"{TAB1}Attempt {tries + 1} of {tries_max} to create devices...")
            devices = system.create_device()
            if not devices:
                print(
                    f"{TAB1}Try {tries+1} of {tries_max}: waiting for {sleep_time_secs} "
                    f"secs for a device to be connected!"
                )
                for sec_count in range(sleep_time_secs):
                    time.sleep(1)
                    print(
                        f"{TAB1}{sec_count + 1 } seconds passed ",
                        end="\r",
                        flush=True,
                    )
                print()
                tries += 1
            else:
                print(f"{TAB1}Created {len(devices)} device(s)")
                return devices

        # 최대 시도 횟수 도달 시 빈 리스트 반환
        print(f"{TAB1}Failed to find devices after {tries_max} attempts")
        return []

    def find_device_by_serial(self, serial: str):
        """시리얼 번호로 특정 디바이스를 찾습니다."""
        devices = self.get_devices()
        for device in devices:
            device_serial = get_node_value_safely(device, "DeviceSerialNumber")
            if device_serial == serial:
                return device
        return None

    def refresh_devices(self):
        """디바이스 목록을 강제로 새로고침합니다."""
        return self.get_devices(force_rescan=True)

    def cleanup_devices(self):
        """모든 디바이스를 정리합니다."""
        with self._devices_lock:
            if self._devices:
                try:
                    print(f"{TAB1}Cleaning up {len(self._devices)} devices...")
                    system.destroy_device(self._devices)
                    print(f"{TAB1}Devices cleaned up successfully")
                except Exception as e:
                    print(f"{TAB1}Error cleaning up devices: {e}")
                finally:
                    self._devices = None
                    self._last_scan_time = 0.0

    def get_device_info(self):
        """연결된 모든 디바이스의 정보를 반환합니다."""
        devices = self.get_devices()
        info = []
        for device in devices:
            try:
                serial = get_node_value_safely(device, "DeviceSerialNumber", "Unknown")
                model = get_node_value_safely(device, "DeviceModelName", "Unknown")
                if serial != "Unknown" and model != "Unknown":
                    info.append({"serial": serial, "model": model, "device": device})
                else:
                    print(f"{TAB1}Warning: Could not get device info for a device")
            except Exception as e:
                print(f"{TAB1}Error getting device info: {e}")
        return info


# 전역 디바이스 매니저 인스턴스
_device_manager = LucidDeviceManager()


class LucidImage:
    def __init__(
        self,
        buffer_np: np.ndarray,
        timestamp_ns: int,
        metadata: Optional[Dict[str, Any]] = None,
    ):
        self.buffer_np = buffer_np
        self.header = Header()
        timestamp_sec = timestamp_ns // 1_000_000_000
        timestamp_sec = timestamp_sec % 2147483647
        self.header.stamp.sec = timestamp_sec
        self.header.stamp.nanosec = timestamp_ns % 1_000_000_000
        self.timestamp_ns = timestamp_ns

        # 메타데이터 저장
        self.metadata = metadata or {}
        self.exposure_time = self.metadata.get("exposure_time")
        self.gain = self.metadata.get("gain")

    def __repr__(self):
        metadata_info = f", metadata={self.metadata}" if self.metadata else ""
        return f"LucidImage(buffer_np_list={self.buffer_np}{metadata_info})"

    def __del__(self):
        del self.buffer_np


class LucidCamera:
    def __init__(
        self,
        serial: str = "224201564",
        ptpmode=True,
        master: bool = True,
        type: Literal["TRI054S-C", "TRI032S-C", "HELIOS2"] = "TRI054S-C",
    ):
        # self.SERIAL = ["224201564", "224201585"]
        self.SERIAL = serial
        self.timestamp_base = 0
        self.FRAME_RATE = 5.0
        self.BUFFER_COUNT = 5
        self.buffers_device: List[_Buffer] = []
        self.trigger_thread: Optional[threading.Thread] = None
        self.trigger_thread_stop = threading.Event()
        self.device: Optional[Device] = None
        self.master = master
        self.ptpmode = ptpmode
        self.type = type
        self._latest_frame = None  # 최신 프레임 저장용
        self.lock = threading.Lock()
        self._is_tof = False  # TOF 카메라 여부 저장
        self.exposure_auto = True
        self.exposure_auto_target = 128

        # 스트림 상태 추적 변수들
        self._stream_active = False  # 실제 스트림 활성 상태
        self._last_frame_time = 0.0  # 마지막 프레임 수신 시간
        self._frame_timeout = 10.0  # 프레임 타임아웃 (초)
        self._stream_recovery_attempts = 0  # 스트림 복구 시도 횟수
        self._max_recovery_attempts = 3  # 최대 복구 시도 횟수
        self._last_recovery_time = 0.0  # 마지막 복구 시도 시간
        self._recovery_cooldown = 30.0  # 복구 시도 간격 (초)

    def _update_frame_timestamp(self):
        """프레임 수신 시간을 업데이트합니다."""
        self._last_frame_time = time.time()
        self._stream_recovery_attempts = 0  # 프레임을 받았으므로 복구 시도 횟수 리셋

    def _is_stream_healthy(self) -> bool:
        """스트림이 정상적으로 작동하는지 확인합니다."""
        if not self._stream_active:
            return False

        current_time = time.time()
        time_since_last_frame = current_time - self._last_frame_time

        # 프레임 타임아웃을 초과했으면 스트림이 비정상
        if time_since_last_frame > self._frame_timeout:
            print(
                f"{TAB1}Stream health check failed for {self.SERIAL}: "
                f"No frames for {time_since_last_frame:.1f}s (timeout: {self._frame_timeout}s)"
            )
            return False

        return True

    def _should_attempt_recovery(self) -> bool:
        """스트림 복구를 시도해야 하는지 확인합니다."""
        current_time = time.time()

        # 최대 복구 시도 횟수를 초과했으면 복구 시도 안함
        if self._stream_recovery_attempts >= self._max_recovery_attempts:
            print(
                f"{TAB1}Maximum recovery attempts ({self._max_recovery_attempts}) reached for {self.SERIAL}"
            )
            return False

        # 복구 시도 간격이 아직 안 되었으면 복구 시도 안함
        if (current_time - self._last_recovery_time) < self._recovery_cooldown:
            remaining_cooldown = self._recovery_cooldown - (
                current_time - self._last_recovery_time
            )
            print(
                f"{TAB1}Recovery cooldown for {self.SERIAL}: {remaining_cooldown:.1f}s remaining"
            )
            return False

        return True

    def _attempt_stream_recovery(self) -> bool:
        """스트림 복구를 시도합니다."""
        if not self._should_attempt_recovery():
            return False

        self._stream_recovery_attempts += 1
        self._last_recovery_time = time.time()

        print(
            f"{TAB1}Attempting stream recovery for {self.SERIAL} (attempt {self._stream_recovery_attempts}/{self._max_recovery_attempts})"
        )

        try:
            # 기존 스트림 정리
            self._stream_active = False
            if self.device is not None:
                try:
                    self.device.stop_stream()
                    print(f"{TAB1}Stopped existing stream for {self.SERIAL}")
                except Exception as e:
                    print(f"{TAB1}Error stopping stream: {e}")

            # 디바이스 연결 상태 확인 및 재연결
            if not self._is_device_connected():
                print(
                    f"{TAB1}Device disconnected, attempting reconnection for {self.SERIAL}"
                )
                self.disconnect_device()
                if not self.connect_device():
                    print(f"{TAB1}Failed to reconnect device {self.SERIAL}")
                    return False

            # 스트림 재시작
            self.open_stream()

            # 복구 성공 여부 확인 (짧은 시간 대기 후)
            time.sleep(2)
            if self._stream_active:
                print(f"{TAB1}Stream recovery successful for {self.SERIAL}")
                return True
            else:
                print(f"{TAB1}Stream recovery failed for {self.SERIAL}")
                return False

        except Exception as e:
            print(f"{TAB1}Exception during stream recovery for {self.SERIAL}: {e}")
            return False

    def get_stream_status(self) -> Dict[str, Any]:
        """현재 스트림 상태 정보를 반환합니다."""
        current_time = time.time()
        time_since_last_frame = (
            current_time - self._last_frame_time
            if self._last_frame_time > 0
            else float("inf")
        )

        return {
            "stream_active": self._stream_active,
            "last_frame_time": self._last_frame_time,
            "time_since_last_frame": time_since_last_frame,
            "is_healthy": self._is_stream_healthy(),
            "recovery_attempts": self._stream_recovery_attempts,
            "max_recovery_attempts": self._max_recovery_attempts,
            "can_attempt_recovery": self._should_attempt_recovery(),
            "frame_timeout": self._frame_timeout,
        }

    def is_tof_camera(self) -> bool:
        """현재 연결된 카메라가 TOF 카메라인지 확인합니다."""
        if self.device is None:
            return False

        try:
            # 1. 타입으로 먼저 확인
            if self.type == "HELIOS2":
                return True

            # 2. 픽셀 포맷으로 확인
            pixel_format = get_node_value_safely(self.device, "PixelFormat", "")
            if "Coord3D" in str(pixel_format):
                self._is_tof = True
                return True

            # 3. 모델명으로 확인 (HELIOS 시리즈)
            model_name = get_node_value_safely(self.device, "DeviceModelName", "")
            if "HELIOS" in str(model_name).upper():
                self._is_tof = True
                return True

            return False
        except Exception as e:
            print(f"{TAB1}Error checking if TOF camera: {e}")
            return False

    def create_devices_with_tries(self):
        """
        DEPRECATED: 이 메서드는 더 이상 사용되지 않습니다.
        대신 전역 LucidDeviceManager를 사용하세요.
        """
        print(
            f"{TAB1}WARNING: create_devices_with_tries is deprecated. Using global device manager instead."
        )
        return _device_manager.get_devices()

    def drain_buffers_nonblocking(self, max_iters=256, per_call_timeout_ms=1):
        """큐에 남아있는 버퍼를 타임아웃이 뜰 때까지 빨아들임"""
        collected = []
        for _ in range(max_iters):
            try:
                buf = self.device.get_buffer(1, timeout=per_call_timeout_ms)
            except Exception as e:
                # timeout이면 멈춤 (다른 에러면 로깅/정리)
                if "TIMEOUT" in str(e) or "timeout" in str(e):
                    break
                print(f"{TAB1}get_buffer error: {e}")
                self.device.start_stream()
                break

            if buf.is_incomplete:
                # print(f"[WARN] Incomplete buffer "
                #     f"{buf.xbuffer.xBufferGetSizeFilled()}/"
                #     f"{buf.xbuffer.xBufferGetSizeOfBuffer()}")
                try:
                    self.device.requeue_buffer(buf)  # 바로 재큐
                    time.sleep(0.05)
                except Exception as re_e:
                    print(f"Requeue failed: {re_e}")
                continue  # incomplete는 skip
            collected.append(buf)

        return collected

    def device_collect_buffers(self, device: Device, trigger=False):
        begin_time = time.time()
        # try:
        #     # todo : trigger의 역할이 대신할 수 있는?
        # device.nodemap.get_node("AcquisitionStart").execute()
        # except OSError as e:
        #     print(
        #         f"{TAB1}OSError on AcquisitionStart: {e}. Device disconnected. Stopping stream."
        #     )
        #     if self.trigger_thread and self.trigger_thread.is_alive():
        #         self.trigger_thread_stop.set()
        #     self.device = None
        #     return

        self.buffers_device = []
        try:
            # buffers = device.get_buffer(1, timeout=int(1000 / self.FRAME_RATE * 3))
            buffers = self.drain_buffers_nonblocking()

        except Exception as e:
            print(f"{TAB1}Error getting buffer: {e}")
            # 디바이스 연결 오류인 경우 device를 정리하고 None으로 설정
            if "disconnect" in str(e).lower() or "not connected" in str(e).lower():
                print(f"{TAB1}Device appears to be disconnected, cleaning up device")
                self.disconnect_device()
            elif e is BaseException:
                try:
                    if self.device is not None:
                        self.device.start_stream()
                except Exception as stream_error:
                    print(f"{TAB1}Failed to restart stream: {stream_error}")
                    self.disconnect_device()
            return

        if isinstance(buffers, _Buffer):
            buffers = [buffers]
        for buffer in buffers:
            if buffer.is_incomplete:
                print(
                    f"{TAB1}Incomplete buffer received. Requeuing... {buffer.xbuffer.xBufferGetSizeFilled()} / {buffer.xbuffer.xBufferGetSizeOfBuffer()}"
                )
                try:
                    if self.device is not None:
                        self.device.requeue_buffer(buffer)
                except Exception as requeue_error:
                    print(f"{TAB1}Failed to requeue buffer: {requeue_error}")
                    # 버퍼 재큐잉 실패는 디바이스 연결 문제일 가능성이 높음
                    self.disconnect_device()
                    break
            else:

                self.buffers_device.append(buffer)

    def trigger_loop(self):
        trigger_armed = False
        while True:
            if self.trigger_thread_stop.is_set():
                self.trigger_thread_stop.clear()
                break
            try:
                trigger_armed = get_node_value_safely(
                    self.device, "TriggerArmed", False
                )
            except Exception as e:
                trigger_armed = False
            if trigger_armed:
                self.trigger_capture()
            time.sleep(0.03)

    def trigger_capture(self):
        if self.trigger_sw_node is not None:
            try:
                self.trigger_sw_node.execute()
            except Exception as e:
                print(f"Failed to execute trigger software node: {e}")
        else:
            # Fallback to safe method
            if not self.execute_node_safely("TriggerSoftware"):
                print("Failed to trigger capture using fallback method")

    def open_stream(self):
        if self.device is None:
            self.connect_device()
            if self.device is None:
                print(f"{TAB1}Device not connected")
                self._stream_active = False
                return False
        try:
            self.device.start_stream()
            self._stream_active = True
            self._last_frame_time = time.time()  # 스트림 시작 시간 기록
            print(f"{TAB1}Stream started successfully for {self.SERIAL}")
            return True
        except Exception as e:
            print(f"{TAB1}Error starting stream: {e}")
            self._stream_active = False
            # 스트림 시작 실패는 디바이스 연결 문제일 가능성이 높음
            if "disconnect" in str(e).lower() or "not connected" in str(e).lower():
                print(f"{TAB1}Device appears to be disconnected, cleaning up device")
                self.disconnect_device()
            return False

        # if self.trigger_thread is not None:
        #     if self.trigger_thread.is_alive():
        #         print(f"{TAB1} {self.SERIAL} : Trigger thread already started")
        #         return
        # self.trigger_thread = threading.Thread(target=self.trigger_loop, daemon=True)
        # self.trigger_thread.start()

    def _is_device_connected(self) -> bool:
        """디바이스가 실제로 연결되어 있는지 확인합니다."""
        if self.device is None:
            return False

        try:
            # 여러 노드를 확인하여 연결 상태를 더 정확하게 판단
            serial = get_node_value_safely(self.device, "DeviceSerialNumber")
            if serial is None or serial == "Unknown":
                return False

            # 추가 연결 상태 확인 - 디바이스 온도나 다른 실시간 값으로 확인
            try:
                # DeviceTemperature나 다른 실시간 값을 읽어서 연결 상태 확인
                temp = get_node_value_safely(self.device, "DeviceTemperature", None)
                # 온도 값을 읽을 수 있으면 연결된 상태로 판단
                if temp is not None:
                    return True
            except:
                pass

            # 마지막으로 타임스탬프를 확인
            timestamp = get_node_value_safely(self.device, "PtpDataSet", None)
            return timestamp is not None

        except Exception as e:
            print(f"{TAB1}Device connection check failed: {e}")
            return False

    def get_device_status(self) -> Dict[str, Any]:
        """디바이스 상태 정보를 반환합니다."""
        status: Dict[str, Any] = {
            "connected": False,
            "serial": None,
            "model": None,
            "error": None,
            "stream_status": self.get_stream_status(),  # 스트림 상태 정보 추가
        }

        try:
            if self.device is None:
                status["error"] = "Device reference is None"
                return status

            # 연결 상태 확인
            if not self._is_device_connected():
                status["error"] = "Device connection check failed"
                # 연결이 끊어진 경우 device를 안전하게 정리
                self.disconnect_device()
                self._stream_active = (
                    False  # 디바이스가 연결되지 않았으면 스트림도 비활성
                )
                return status

            # 디바이스 정보 수집
            status["connected"] = True
            status["serial"] = get_node_value_safely(
                self.device, "DeviceSerialNumber", "Unknown"
            )
            status["model"] = get_node_value_safely(
                self.device, "DeviceModelName", "Unknown"
            )

        except Exception as e:
            status["error"] = f"Error getting device status: {str(e)}"
            # 오류 발생 시 device를 안전하게 정리
            self.disconnect_device()
            self._stream_active = False

        return status

    def collect_images(self):
        # 스트림 상태 건강성 확인 및 자동 복구
        if not self._is_stream_healthy():
            print(f"{TAB1}Stream unhealthy for {self.SERIAL}, attempting recovery...")
            if self._attempt_stream_recovery():
                print(f"{TAB1}Stream recovery successful for {self.SERIAL}")
            else:
                print(f"{TAB1}Stream recovery failed for {self.SERIAL}")
                return []  # 빈 리스트 반환

        if self.device is None:
            print(f"{TAB1}Device not connected, attempting to connect...")
            if not self.connect_device():
                self._stream_active = False
                raise Exception("Device not connected and connection failed")

        # 디바이스 연결 상태를 확인
        if not self._is_device_connected():
            print(f"{TAB1}Device disconnected, attempting to reconnect...")
            self.disconnect_device()
            if not self.connect_device():
                self._stream_active = False
                raise Exception("Device disconnected and reconnection failed")

            # 재연결 후 스트림 시작
            try:
                if self.device is not None:
                    self.device.start_stream()
                    self._stream_active = True
                    self._last_frame_time = time.time()
                    print(f"{TAB1}Stream restarted after reconnection")
            except Exception as e:
                print(f"{TAB1}Error restarting stream after reconnection: {e}")
                self._stream_active = False
                raise Exception("Failed to restart stream after reconnection")

        # threads = []
        # thread = threading.Thread(
        #     target=self.device_collect_buffers, args=(self.device,), daemon=True
        # )
        # threads.append(thread)
        # thread.start()
        # for thread in threads:
        #     thread.join()
        self.device_collect_buffers(self.device)
        buffer_np_list: List[LucidImage] = []

        for buffer in self.buffers_device:
            time_begin = time.time()
            # 이미지와 메타데이터를 함께 추출
            buffer_np, metadata = self.extract_image_and_metadata(buffer)
            if buffer_np is not None:
                timestamp_ns = buffer.timestamp_ns + self.timestamp_base
                buffer_np_list.append(
                    LucidImage(buffer_np.copy(), timestamp_ns, metadata)
                )
                # 프레임을 성공적으로 받았으므로 프레임 타임스탬프 업데이트
                self._update_frame_timestamp()
            if (
                self._latest_frame is None
                or self._latest_frame.timestamp_ns < timestamp_ns
            ):
                self._latest_frame = LucidImage(
                    buffer_np.copy(), timestamp_ns, metadata
                )
            if self.device is not None:
                self.device.requeue_buffer(buffer)

        return buffer_np_list

    def collect_image_loop(self, callback: Callable[[LucidImage], None]):
        while True:
            images = self.collect_images()
            for raw_img in images:
                callback(raw_img)

    def extract_image_and_metadata(self, buf):

        def _bytes_per_pixel(
            pixel_format: enums.PixelFormat, bits_per_pixel: int
        ) -> int:
            # PFNC에서 bpp로 바로 계산
            # Bayer/RGB/Mono 등 포맷을 모두 포괄; bpp는 Arena가 제공
            if bits_per_pixel % 8 != 0:
                raise ValueError(
                    f"Unsupported non-byte-aligned pixel format: {pixel_format} (bpp={bits_per_pixel})"
                )
            return bits_per_pixel // 8

        def _compute_stride(width: int, bpp: int, padding_x_bytes: int) -> int:
            # stride(bytes) = (width * bpp) + padding_x
            return width * bpp + int(padding_x_bytes)

        def _image_size_bytes(stride: int, height: int, padding_y_bytes: int) -> int:
            # 대부분 카메라는 Y padding을 0으로 주지만, API가 제공하므로 반영
            return stride * height + int(padding_y_bytes)

        def _numpy_view_from_buffer(ptr, size_in_bytes, dtype=np.uint8):
            """
            Arena의 buffer.pdata(pointer)로부터 size만큼을 NumPy 1D view로 만듭니다.
            Arena는 메모리를 소유하므로, 이 배열은 buffer의 생존/재큐 상태에 종속적입니다.
            """
            import ctypes

            buf_t = (ctypes.c_uint8 * size_in_bytes).from_address(
                ctypes.addressof(ptr.contents)
            )
            return np.ctypeslib.as_array(buf_t)

        """
        입력: Arena _Buffer (IMAGE 또는 IMAGE_EXTENDED_CHUNK)
        출력: (img, meta)
        - img: np.ndarray (H, W) 또는 (H, W, C)  [raw 인터리브 그대로]
        - meta: dict (exposure_us, gain_db, crc_ok, frame_id, ts_ns, pixel_format, payload_type 등)
        """
        # -------- 이미지 기하/포맷 ----------
        W = buf.width
        H = buf.height
        bits_per_pixel = buf.bits_per_pixel
        pf = buf.pixel_format  # enums.PixelFormat
        bpp = _bytes_per_pixel(pf, bits_per_pixel)
        pad_x = buf.padding_x  # Arena 문서상 "line end bytes"
        pad_y = buf.padding_y  # 보통 0
        stride = _compute_stride(W, bpp, pad_x)
        img_nbytes = _image_size_bytes(stride, H, pad_y)

        # -------- 전체 payload에서 "이미지 부분"만 슬라이스 ----------
        # Arena의 xImageGetData()는 이미지 시작 포인터를 가리킵니다.
        # 뒤에 chunk가 붙어 있어도 이미지 시작은 동일하므로, 계산한 img_nbytes만큼만 읽으면 안전.
        p = buf.pdata  # ctypes.POINTER(ctypes.c_uint8) to image start
        arr_1d = _numpy_view_from_buffer(p, img_nbytes, dtype=np.uint8)

        # -------- stride 고려 reshape ----------
        # stride == W*bpp 이면 padding 없음 → 간단 reshape
        # stride > W*bpp 이면 line padding 존재 → (H, stride)로 reshape 후 앞쪽 W*bpp만 슬라이스
        line_bytes = W * bpp
        if stride == line_bytes:
            # padding 없음
            img_2d = arr_1d.reshape(H, line_bytes)
        else:
            # padding 있음
            img_2d = arr_1d.reshape(H, stride)[:, :line_bytes]

        # -------- 채널 차원 구성 ----------
        # PFNC가 Mono 계열이면 (H, W), RGB/BGR 등 인터리브면 (H, W, C)로 뷰
        # (여기서는 raw 인터리브 기준으로 C 도출: bpp 기준 단순화)
        img = img_2d.reshape(H, W, bpp)

        # -------- Chunk metadata 추출 ----------
        # 사용자가 ExposureTime, Gain, CRC를 enable 했다고 가정
        meta = {}
        meta["frame_id"] = buf.frame_id
        meta["timestamp_ns"] = buf.timestamp_ns
        meta["payload_type"] = str(buf.payload_type.name)
        meta["pixel_format"] = str(pf.name)
        meta["bits_per_pixel"] = bits_per_pixel
        meta["stride_bytes"] = stride
        meta["padding_x"] = pad_x
        meta["padding_y"] = pad_y
        meta["has_chunkdata"] = buf.has_chunkdata
        meta["buffer_size"] = buf.xbuffer.xBufferGetSizeOfBuffer()

        # 각 chunk는 'Chunk<Name>' 문자열로 접근
        def _read_chunk(name, key, cast=float):
            try:
                node = buf.get_chunk(name)  # 예: 'ChunkExposureTime'
                if cast == str:
                    meta[key] = str(node.value)
                else:
                    meta[key] = cast(node.value)
            except Exception as e:
                meta[key] = None
                meta[f"{key}_error"] = str(e)

        if buf.has_chunkdata:
            # TOF 카메라인지 확인
            is_tof = self.is_tof_camera()

            if not is_tof:
                # RGB 카메라의 경우 기존 방식 사용
                _read_chunk("ChunkExposureTime", "ExposureTime", float)
                _read_chunk("ChunkGain", "Gain", float)
            else:

                # 타임스탬프 청크 시도
                _read_chunk("ChunkTimestamp", "Timestamp", int)

                # 픽셀 포맷 청크 시도 (str 타입으로 캐스팅)
                try:
                    node = buf.get_chunk("ChunkPixelFormat")
                    meta["ChunkPixelFormat"] = str(node.value)
                except Exception as e:
                    meta["ChunkPixelFormat"] = None
                    meta["ChunkPixelFormat_error"] = str(e)

                # 이미지 크기 정보 청크들 시도
                _read_chunk("ChunkWidth", "ChunkWidth", int)
                _read_chunk("ChunkHeight", "ChunkHeight", int)
                _read_chunk("ChunkOffsetX", "ChunkOffsetX", int)
                _read_chunk("ChunkOffsetY", "ChunkOffsetY", int)

                # TOF 특화 청크들 (있다면)
                _read_chunk("ChunkPixelDynamicRangeMin", "PixelDynamicRangeMin", float)
                _read_chunk("ChunkPixelDynamicRangeMax", "PixelDynamicRangeMax", float)

                # ExposureTime과 Gain은 TOF에서 지원되지 않으므로 None으로 설정
                meta["ExposureTime"] = None
                meta["ExposureTime_error"] = (
                    "'ChunkExposureTime' Chunk is not found in the buffer"
                )
                meta["Gain"] = None
                meta["Gain_error"] = "'ChunkGain' Chunk is not found in the buffer"
        else:
            print(f"{TAB1}Warning: Buffer has no chunk data")

        # CRC는 전용 property 사용 (CRC chunk가 enable돼 있어야 함)
        try:
            meta["crc_ok"] = bool(buf.is_valid_crc)
        except Exception as e:
            meta["crc_ok"] = None
            meta["crc_error"] = str(e)

        return img, meta

    def buffer_to_image(self, buffer: _Buffer) -> Optional[np.ndarray]:
        try:
            if buffer.is_incomplete:
                print(
                    f"""
                    buffer.is_incomplete: {buffer.is_incomplete}
                    buffer.xbuffer.has_image_data: {buffer.xbuffer.xBufferHasImageData()}
                    buffer.xbuffer.has_chunk_data: {buffer.xbuffer.xBufferHasChunkData()}
                    {buffer.xbuffer.xBufferGetSizeFilled()} / {buffer.xbuffer.xBufferGetSizeOfBuffer()}
                    
                    """
                )
                # return None
        except Exception as e:
            print(e)
            return None
        pointer = buffer.xbuffer.xImageGetData()

        data_np, meta = self.extract_image_and_metadata(buffer)
        print(meta)
        return data_np

    def set_exposure_time(self, exposure_us: int) -> bool:
        """노출 시간을 마이크로초 단위로 설정"""
        if self.device is None:
            print("Device not connected")
            return False

        try:
            if exposure_us <= 0:
                max_exposure_manual = 1e6
                min_exposure_manual = 40
                exposure_manual = max_exposure_manual
                if exposure_us == 0:

                    exposure_manual = min_exposure_manual
                print(f"DEBUG: exposure_us <= 0, attempting to get max exposure")
                # get max exposure time from node safely
                success, max_exposure = update_node_safely(self.device, "ExposureTime")
                if success:
                    try:
                        node = self.device.nodemap.get_node("ExposureTime")
                        if node and exposure_us == -1 and hasattr(node, "max"):
                            max_exposure = node.max
                            exposure_us = int(max_exposure)
                        elif node and exposure_us == 0 and hasattr(node, "min"):
                            min_exposure = node.min
                            exposure_us = int(min_exposure)
                        else:
                            exposure_us = exposure_manual # Default maximum 100ms
                    except Exception as e:
                        exposure_us = exposure_manual  # Default maximum 100ms
                else:
                    exposure_us = exposure_manual  # Default maximum 100ms

                print(f"DEBUG: Final exposure_us after conversion: {exposure_us}")

            print(
                f"DEBUG: About to call set_node_value_safely with exposure_us={exposure_us}"
            )
            # Arena SDK의 ExposureTime 노드 사용 (안전한 함수 사용)
            success = set_node_value_safely(
                self.device, "ExposureTime", float(exposure_us)
            )
            if success:
                print(f"Set exposure time to {exposure_us}us")
                return True
            else:
                print("Failed to set exposure time - ExposureTime node not accessible")
                return False
        except Exception as e:
            print(f"Error setting exposure time: {e}")
            return False

    def get_latest_frame(self) -> Optional[LucidImage]:
        """최신 프레임을 가져오기"""
        with self.lock:
            latest_frame = self._latest_frame
            # if latest frame is not None and too old frame, erase latest frame and trigger device.open_stream()
            if latest_frame is not None:
                age_sec = (time.time_ns() - latest_frame.timestamp_ns) / 1_000_000_000
                if age_sec > 5.0:
                    print(f"{TAB1}Latest frame is too old ({age_sec:.2f}s), clearing")
                    self._latest_frame = None
                    latest_frame = None
                    if self.device is not None:
                        try:
                            self.device.start_stream()
                        except Exception as e:
                            print(f"{TAB1}Failed to restart stream: {e}")
                            self.disconnect_device()
        return latest_frame

    def trigger_device(self) -> bool:
        """카메라 트리거 실행"""
        if self.device is None:
            print("Device not connected")
            return False

        try:
            # 소프트웨어 트리거 실행 (안전한 함수 사용)
            success = execute_node_safely(self.device, "TriggerSoftware")
            if success:
                print("Software trigger executed")
                return True
            else:
                print(
                    "Failed to execute software trigger - TriggerSoftware node not accessible"
                )
                return False

        except Exception as e:
            print(f"Failed to trigger device: {e}")
            # 대안 방법 시도
            try:
                # 다른 트리거 방식 시도
                if hasattr(self.device.nodemap, "TriggerMode"):
                    # 트리거 모드 확인 후 실행
                    self.device.nodemap.TriggerMode.value = "On"
                    time.sleep(0.01)  # 10ms 대기
                    if hasattr(self.device.nodemap, "TriggerSoftware"):
                        self.device.nodemap.TriggerSoftware.execute()
                        return True
            except Exception as e2:
                print(f"Alternative trigger method also failed: {e2}")
            return False

    def device_config_timestamp_base(self):
        device = self.device
        timestamp_ns_raw = get_node_value_safely(device, "PtpDataSet", 0)

        try:
            timestamp_ns = int(timestamp_ns_raw) if timestamp_ns_raw != 0 else 0
        except (ValueError, TypeError):
            print(f"{TAB1}Warning: Invalid PtpDataSet value: {timestamp_ns_raw}")
            timestamp_ns = 0

        if timestamp_ns == 0:
            print(
                f"{TAB1}Warning: Could not get valid PtpDataSet, using current time as base"
            )
            self.timestamp_base = 0
        else:
            self.timestamp_base = time.time_ns() - timestamp_ns

        print(f"{TAB1}Timestamp: {timestamp_ns}")
        print(f"{TAB1}Timestamp base: {self.timestamp_base}")

    def connect_device(self):
        try:
            # 디바이스 참조가 있어도 실제 연결 상태를 확인
            if self.device is not None:
                if self._is_device_connected():
                    print(f"{TAB1}Device already connected and valid")
                    return True
                else:
                    print(
                        f"{TAB1}Device reference exists but connection is invalid, reconnecting..."
                    )
                    self.disconnect_device()  # 기존 참조 정리

            # 전역 디바이스 매니저를 사용하여 디바이스 찾기
            print(f"{TAB1}Searching for device with serial: {self.SERIAL}")
            device = _device_manager.find_device_by_serial(self.SERIAL)

            if device is None:
                print(f"{TAB1}Device with serial {self.SERIAL} not found")
                return False

            self.device = device

            device_model = get_node_value_safely(
                self.device, "DeviceModelName", "Unknown Model"
            )
            print(f"{TAB1}Connected to device {device_model}")
            if self.device is None:
                print(
                    f"{TAB1}Device with serial {self.SERIAL} not found. "
                    f"Please check the connection."
                )
                return False
            device = self.device

            # Get device model name safely for confirmation
            device_model_confirm = self.get_node_value_safely(
                "DeviceModelName", "Unknown"
            )
            print(f"{TAB1}Connected to device {device_model_confirm}")

            # Get trigger software node safely
            try:
                self.trigger_sw_node = device.nodemap.get_node("TriggerSoftware")
                print(f"{TAB1}TriggerSoftware node: {self.trigger_sw_node}")
            except Exception as e:
                print(f"{TAB1}Warning: Could not get TriggerSoftware node: {e}")
                self.trigger_sw_node = None

            if self.ptpmode:
                self.set_node_value_safely("PtpEnable", True)
                print("PtpEnable set to True")
                self.set_node_value_safely("PtpSlaveOnly", not self.master)
                print(f"{TAB1}PtpSlaveOnly set to {not self.master}")
                self.set_node_value_safely("AcquisitionStartMode", "PTPSync")
                print(f"{TAB1}AcquisitionStartMode set to PTPSync")
                self.set_node_value_safely("PTPSyncFrameRate", self.FRAME_RATE)
                print(f"{TAB1}PTPSyncFrameRate set to {self.FRAME_RATE}")

            self.config_device(device)

            return True
        except Exception as e:
            print(f"{TAB1}Error connecting device {self.SERIAL}: {e}")
            # 연결 실패 시 안전하게 정리
            if self.device is not None:
                try:
                    self.disconnect_device()
                except Exception as cleanup_error:
                    print(f"{TAB1}Error during cleanup: {cleanup_error}")
                    self.device = None
            return False

    def config_device(self, device: Device):
        device.stop_stream()

        # Reset timestamp with safe wrapper
        if not self.execute_node_safely("TimestampReset"):
            print(f"{TAB1}Warning: Could not reset timestamp")
        else:
            print(f"{TAB1}Timestamp reset")

        # device의 gateway : 0.0.0.0
        node_report_keys = [
            # "AcquisitionStartMode",
            # # "TriggerLatency",
            # "TriggerActivation",
            # "TriggerSource",
            # "TriggerMode",
            # "TriggerSelector",
            # "AcquisitionFrameRate",
            # "Width",
            # "Height",
            # "TriggerOverlap",
            # "PayloadSize",
            # "ExposureTime",
            # "ExposureAuto",
            # "ColorTransformationEnable",
            # "BlackLevel",
            # "BalanceWhiteEnable",
            # "BalanceWhiteAuto",
            # "HDROutput",
            # "HDRTuningEnable",
            # "LUTEnable",
            # "LUTToneMapping",
            # "ExposureAutoLowerLimit",
            # "ExposureAutoUpperLimit",
            "TargetBrightness",
            "ExposureAutoAlgorithm",
            "AutoExposureAOI",
            "ShortExposureEnable",
        ]
        nodemap = device.nodemap
        # for node_key in node_report_keys:
        #     print(f"{TAB1}{node_key}: ")
        #     print(nodemap[node_key])

        # nodemap.get_node("Width").value = 2880
        # nodemap.get_node("Height").value = 1856

        # Set acquisition burst frame count
        if not self.set_node_value_safely("AcquisitionBurstFrameCount", 1):
            print(f"{TAB1}Warning: Could not set AcquisitionBurstFrameCount")
        else:
            print(f"{TAB1}Setting AcquisitionBurstFrameCount to 1")

        print(f"{TAB1}Enabling AcquisitionFrameRate")

        # Get current resolution
        current_width = self.get_node_value_safely("Width", 0)
        current_height = self.get_node_value_safely("Height", 0)
        print(
            f"{TAB1} {self.type} Current resolution : {current_width}x{current_height}"
        )
        if self.type == "TRI054S-C":
            print(
                f"{TAB1}Setting resolution to {RESOLUTION[0]//BINNING[0]}x{RESOLUTION[1]//BINNING[1]}"
            )
            self.set_node_value_safely("OffsetX", int(0))
            self.set_node_value_safely("OffsetY", int(0))

            print(f"{TAB1}Setting PixelFormat to BayerRG24")
            self.set_node_value_safely("BinningSelector", "Sensor")
            self.set_node_value_safely("BinningHorizontalMode", "Average")
            self.set_node_value_safely("BinningVerticalMode", "Average")

            self.set_node_value_safely("BinningHorizontal", int(BINNING[1]))
            self.set_node_value_safely("BinningVertical", int(BINNING[0]))
            self.set_node_value_safely("PixelFormat", "BayerRG24")

            self.set_node_value_safely("Width", RESOLUTION[0] // RESDOWN[0] // 4 * 4)
            self.set_node_value_safely("Height", RESOLUTION[1] // RESDOWN[1] // 4 * 4)
        elif self.type == "TRI032S-C":

            self.set_node_value_safely("PixelFormat", "BayerRG12")

            self.set_node_value_safely("BinningSelector", "Digital")
            self.set_node_value_safely("BinningHorizontalMode", "Sum")
            self.set_node_value_safely("BinningVerticalMode", "Sum")

            self.set_node_value_safely("BinningHorizontal", int(BINNING[1]))
            self.set_node_value_safely("BinningVertical", int(BINNING[0]))

            self.set_node_value_safely("Width", RESOLUTION_12[0] // RESDOWN[0] // 4 * 4)
            self.set_node_value_safely(
                "Height", RESOLUTION_12[1] // RESDOWN[1] // 4 * 4
            )
        elif self.type == "HELIOS2":
            # print(
            #     f"{TAB1}Setting resolution to {RESOLUTION_HELIOS2[0]//BINNING[0]}x{RESOLUTION_HELIOS2[1]//BINNING[1]}"
            # )
            print(nodemap["PixelFormat"])
            print(nodemap["Width"])
            print(nodemap["Height"])
            # self.set_node_value_safely("OffsetX", int(0))
            # self.set_node_value_safely("OffsetY", int(0))

            # print(f"{TAB1}Setting PixelFormat to Mono8")
            # self.set_node_value_safely("BinningSelector", "Sensor")
            # self.set_node_value_safely("BinningHorizontalMode", "Average")
            # self.set_node_value_safely("BinningVerticalMode", "Average")

            # self.set_node_value_safely("BinningHorizontal", int(BINNING[1]))
            # self.set_node_value_safely("BinningVertical", int(BINNING[0]))
            self.set_node_value_safely("PixelFormat", "Coord3D_CY16")

            # self.set_node_value_safely(
            #     "Width", RESOLUTION_HELIOS2[0] // RESDOWN[0] // 4 * 4
            # )
            # self.set_node_value_safely(
            #     "Height", RESOLUTION_HELIOS2[1] // RESDOWN[1] // 4 * 4
            # )

        # Set acquisition mode
        self.set_node_value_safely("AcquisitionMode", "Continuous")

        # Check if using PTP sync mode
        acquisition_start_mode = self.get_node_value_safely("AcquisitionStartMode", "")
        if acquisition_start_mode != "PTPSync":
            self.set_node_value_safely("TriggerSelector", "FrameStart")
            self.set_node_value_safely("TriggerOverlap", "PreviousFrame")
            self.set_node_value_safely("TriggerMode", "On")
            self.set_node_value_safely("TriggerSource", "Software")
        else:
            # self.set_node_value_safely("DeviceLinkThroughputLimitMode", "On")
            # self.set_node_value_safely("DeviceLinkThroughputLimit", 125_000_000)
            # self.set_node_value_safely("TLStreamBufferHandlingMode", "NewestOnly")
            # self.set_node_value_safely("TLStreamPacketResendEnable", "On")
            # self.set_node_value_safely("TriggerMode", "Off")
            pass

        print(f"{TAB1}TriggerActivation set to RisingEdge")

        # TOF 카메라인지 확인
        is_tof = self.is_tof_camera()
        print(f"{TAB1}Camera is TOF: {is_tof}")

        if not is_tof:
            # RGB 카메라에만 적용되는 설정들
            if self.exposure_auto:
                print(f"{TAB1}Setting ExposureAuto to Continuous")
                self.set_node_value_safely("ExposureAuto", "Continuous")
                self.set_node_value_safely("GainAuto", "Continuous")
                
                self.set_node_value_safely(
                    "TargetBrightness", self.exposure_auto_target
                )
            else:
                self.set_node_value_safely("ExposureAuto", "Off")
                self.set_node_value_safely("ExposureTime", 10000.0)
                self.set_node_value_safely("GainAuto", "Off")
                self.set_node_value_safely("Gain", 0.0)

            self.set_node_value_safely("LUTEnable", False)
            if self.type == "TRI054S-C":
                self.set_node_value_safely("HDRTuningEnable", False)
            self.set_node_value_safely("ColorTransformationEnable", False)
            self.set_node_value_safely("BalanceWhiteEnable", False)
        else:
            # TOF 카메라의 경우 exposure/gain 설정을 하지 않음
            print(f"{TAB1}Skipping exposure and gain settings for TOF camera")

        print(f"{TAB1}Setting BlackLevel to 0")

        # Configure metadata chunk data
        if True:
            self.device.stop_stream()
            # nodemap["AcquisitionStop"].execute()
            self.set_node_value_safely("ChunkModeActive", True)
            # CRC는 모든 카메라에 공통으로 적용
            self.set_node_value_safely("ChunkSelector", "CRC")
            self.set_node_value_safely("ChunkEnable", True)

            # TOF 카메라가 아닌 경우만 ExposureTime, Gain 청크 활성화
            if not is_tof:
                self.set_node_value_safely("ChunkSelector", "ExposureTime")
                self.set_node_value_safely("ChunkEnable", True)
                self.set_node_value_safely("ChunkSelector", "Gain")
                self.set_node_value_safely("ChunkEnable", True)
            else:
                # TOF 카메라의 경우 사용 가능한 청크들을 활성화
                # 기본적으로 지원되는 청크들: Timestamp, PixelFormat, Width, Height 등
                try:
                    self.set_node_value_safely("ChunkSelector", "Timestamp")
                    self.set_node_value_safely("ChunkEnable", True)
                    print(f"{TAB1}Enabled Timestamp chunk for TOF camera")
                except:
                    print(f"{TAB1}Warning: Could not enable Timestamp chunk")

                try:
                    self.set_node_value_safely("ChunkSelector", "PixelFormat")
                    self.set_node_value_safely("ChunkEnable", True)
                    print(f"{TAB1}Enabled PixelFormat chunk for TOF camera")
                except:
                    print(f"{TAB1}Warning: Could not enable PixelFormat chunk")

            # nodemap["AcquisitionStart"].execute()
        # Configure TL stream settings safely

        try:
            tl_stream_nodemap = device.tl_stream_nodemap
            tl_stream_nodemap["StreamAutoNegotiatePacketSize"].value = True
            tl_stream_nodemap["StreamPacketResendEnable"].value = True
            tl_stream_nodemap["StreamBufferHandlingMode"].value = "OldestFirst"

            print(f"{TAB1}TL stream settings configured")

        except Exception as e:
            print(f"{TAB1}Warning: Could not configure TL stream settings: {e}")

    def set_node_value_safely(self, node_name: str, value):
        """노드 값을 안전하게 설정하는 헬퍼 메서드"""
        if self.device is None:
            return False

        return set_node_value_safely(self.device, node_name, value)

    def get_node_value_safely(self, node_name: str, default_value=None):
        """노드 값을 안전하게 가져오는 헬퍼 메서드"""
        if self.device is None:
            return default_value

        return get_node_value_safely(self.device, node_name, default_value)

    def execute_node_safely(self, node_name: str):
        """노드 명령을 안전하게 실행하는 헬퍼 메서드"""
        if self.device is None:
            return False

        return execute_node_safely(self.device, node_name)

    def validate_device(self):
        """디바이스 연결 상태를 검증하고 필요시 재연결을 시도합니다."""
        if self.device is None:
            print(f"{TAB1}Device {self.SERIAL} is None, attempting to reconnect...")
            return self.connect_device()

        # 실제 연결 상태 확인
        if not self._is_device_connected():
            print(
                f"{TAB1}Device {self.SERIAL} connection lost, attempting to reconnect..."
            )
            self.disconnect_device()  # 기존 참조 정리
            return self.connect_device()  # 재연결 시도

        return True

    def set_gain(self, gain_value: float) -> bool:
        """게인 값을 설정합니다."""
        if self.device is None:
            print("Device not connected")
            return False

        try:
            success = set_node_value_safely(self.device, "Gain", float(gain_value))
            if success:
                print(f"Set gain to {gain_value}")
                return True
            else:
                print("Failed to set gain - Gain node not accessible")
                return False
        except Exception as e:
            print(f"Error setting gain: {e}")
            return False

    def disconnect_device(self):
        """디바이스 연결을 안전하게 해제합니다."""
        self._stream_active = False  # 스트림 상태 비활성화

        if self.device is not None:
            try:
                # 스트림이 실행 중이면 중지
                print(f"{TAB1}Stopping stream for device {self.SERIAL}")
                self.device.stop_stream()
            except Exception as e:
                print(f"{TAB1}Error stopping stream: {e}")

            try:
                # 트리거 스레드 중지
                if self.trigger_thread and self.trigger_thread.is_alive():
                    print(f"{TAB1}Stopping trigger thread for device {self.SERIAL}")
                    self.trigger_thread_stop.set()
                    self.trigger_thread.join(timeout=1.0)
            except Exception as e:
                print(f"{TAB1}Error stopping trigger thread: {e}")

            # 디바이스 참조 제거
            self.device = None
            print(f"{TAB1}Device {self.SERIAL} disconnected")

    def __del__(self):
        # 개별 디바이스 정리
        self.disconnect_device()

        # 전역 디바이스 매니저를 통한 전체 정리
        try:
            _device_manager.cleanup_devices()
        except Exception as e:
            print(f"{TAB1}Error in global device cleanup: {e}")

        print(f"{TAB1}LucidCamera {self.SERIAL} destroyed")


def get_global_device_manager():
    """전역 디바이스 매니저 인스턴스를 반환합니다."""
    return _device_manager


def list_all_lucid_devices():
    """연결된 모든 Lucid 디바이스를 출력합니다."""
    print("Scanning for Lucid devices...")
    device_info = _device_manager.get_device_info()

    if not device_info:
        print("No Lucid devices found.")
    else:
        print(f"Found {len(device_info)} Lucid device(s):")
        for info in device_info:
            print(f"  - {info['model']} (Serial: {info['serial']})")


def cleanup_all_lucid_devices():
    """모든 Lucid 디바이스를 강제로 정리합니다. 크래시나 예외 상황에서 사용하세요."""
    try:
        print("Emergency cleanup: Destroying all Lucid devices...")
        _device_manager.cleanup_devices()
        print("Emergency cleanup completed")
    except Exception as e:
        print(f"Error during emergency cleanup: {e}")


if __name__ == "__main__":
    lucid = LucidCamera(
        serial="253200234",
        ptpmode=True,
        type="TRI032S-C",
    )
    lucid.exposure_auto = True
    lucid.exposure_auto_target = 4
    lucid_right = LucidCamera(serial="224201585", ptpmode=True, master=False)
    lucid.connect_device()
    lucid_right.exposure_auto = True
    lucid_right.exposure_auto_target = 200
    lucid_right.connect_device()

    lucid_tof = LucidCamera(serial="252902574", type="HELIOS2")
    lucid_tof.connect_device()

    time.sleep(5)
    lucid.device_config_timestamp_base()
    lucid_right.timestamp_base = lucid.timestamp_base
    lucid_tof.timestamp_base = lucid.timestamp_base

    # Get PTP status safely
    ptp_status_left = lucid.get_node_value_safely("PtpStatus", "Unknown")
    ptp_status_right = lucid_right.get_node_value_safely("PtpStatus", "Unknown")
    print(f"Left camera PTP status: {ptp_status_left}")
    print(f"Right camera PTP status: {ptp_status_right}")
    lucid_right.open_stream()
    lucid.open_stream()
    lucid_tof.open_stream()

    for i in range(10):

        def collect_left():
            images = lucid.collect_images()
            print(f"Left : Collected {len(images)} images")
            if len(images) > 0:
                print("Left", round(images[0].timestamp_ns / 1e9, 4))

        def collect_right():
            images = lucid_right.collect_images()
            print(f"Right : Collected {len(images)} images")
            if len(images) > 0:
                print("Right", round(images[0].timestamp_ns / 1e9, 4))

        def collect_tof():
            images = lucid_tof.collect_images()
            print(f"TOF : Collected {len(images)} images")
            if len(images) > 0:
                print("TOF", round(images[0].timestamp_ns / 1e9, 4))
                print("TOF meta", images[0].metadata)

        thread_left = threading.Thread(target=collect_left, daemon=True)
        thread_right = threading.Thread(target=collect_right, daemon=True)
        thread_tof = threading.Thread(target=collect_tof, daemon=True)

        thread_left.start()
        thread_right.start()
        thread_tof.start()
        thread_left.join()
        thread_right.join()
        thread_tof.join()
        time.sleep(0.1)
    del lucid
