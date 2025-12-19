import threading
from typing import Union, Optional, Literal
from dataclasses import dataclass

import cv2
import numpy as np
from lucid_cam import LucidCamera, _device_manager, update_node_safely
import rclpy
from ..camera import Camera
import time


class CameraLucid24Single(Camera):

    @dataclass
    class State(Camera.State):
        exposure_auto: bool = False
        exposure_auto_target: int = 128

    def __init__(
        self,
        name: str,
        serial: str,
        ptpmode=False,
        ptpmaster=False,
        type: Literal["TRI054S-C", "TRI032S-C", "HELIOS2"] = "TRI054S-C",
        exposure_auto_target=128,
    ):
        # Initialize custom state with exposure auto settings
        state = self.State(
            exposure_auto=True, exposure_auto_target=exposure_auto_target
        )

        # TOF 카메라 여부를 먼저 확인
        self.is_tof = type == "HELIOS2"

        super().__init__(
            name,
            self.Const(
                srcs=1,
                raw_format="depth" if self.is_tof else "bayer",
                raw_bits=32 if self.is_tof else (24 if type == "TRI054S-C" else 12),
                width=640 if self.is_tof else (1440 if type == "TRI054S-C" else 1024),
                height=480 if self.is_tof else (928 if type == "TRI054S-C" else 768),
                preview_keys=["depth", "intensity"] if self.is_tof else ["image"],
            ),
        )

        # Replace the default state with our custom state
        self.state: "CameraLucid24Single.State" = state

        self.lucid_api = LucidCamera(
            serial, ptpmode=ptpmode, master=ptpmaster, type=type
        )

        self.wb = np.asarray(
            [
                2.0841475322429894,
                1.0,
                1.9215893014341496,
            ]
        )
        self.buffer_resolve_thread: Optional[threading.Thread] = None
        self.config = self.Config(self)
        self.state.preview_interval = 0.2
        self.lucid_api.exposure_auto = self.state.exposure_auto
        self.lucid_api.exposure_auto_target = self.state.exposure_auto_target

        # 비디오 스트림 방식을 위한 latest frame 저장
        self.frame_lock = threading.Lock()

    def start_stream(self):
        # if self.state.stream_on:
        #     return
        self.lucid_api.open_stream()
        self.state.stream_on = True
        if (
            self.buffer_resolve_thread is None
            or not self.buffer_resolve_thread.is_alive()
        ):
            self.buffer_resolve_loop_launch()

    def buffer_resolve_loop_launch(self):
        def buffer_resolve_loop():
            while rclpy.ok():
                try:
                    images = self.lucid_api.collect_images()
                except Exception as e:
                    print(f"Error collecting images from {self.name}: {e}")
                    time.sleep(1)
                    continue
                images.sort(key=lambda x: x.timestamp_ns)

                for image in images:
                    metadata= image.metadata if hasattr(image, "metadata") else {}
                    if "timestamp_ns" in metadata:
                        metadata["timestamp_ns_pc"] = metadata["timestamp_ns"] + self.lucid_api.timestamp_base
                    
                    if self.is_tof:

                        # TOF 데이터에서 depth와 intensity 분리
                        depth, intensity = self.extract_depth_from_tof(image.buffer_np)

                        frame = self.Frame(
                            image.timestamp_ns / 1e9,
                            {
                                "depth": depth,
                                "intensity": intensity,
                                "metadata": metadata,
                            },
                            {},
                            file_format={"depth": "npy", "intensity": "exr"},
                        )

                        # TOF 카메라는 Frame으로 콜백 호출
                        threading.Thread(
                            target=self.frame_callback,
                            args=(self, frame),
                        ).start()
                    else:
                        
                        frame = self.Frame(
                            image.timestamp_ns / 1e9,
                            {
                                "image": image.buffer_np,
                                "metadata": metadata,
                            },
                            {},
                            file_format={"image": "exr"},
                        )
                        threading.Thread(
                            target=self.frame_callback,
                            args=(self, frame),
                        ).start()
                    # RGB 카메라의 경우 LucidImage 객체는 images 리스트에 그대로 있음
                    # 원래 lucid_cam.py의 collect_images()가 반환하는 LucidImage 객체들을 그대로 사용

        self.buffer_resolve_thread = threading.Thread(
            target=buffer_resolve_loop, daemon=True
        )
        self.buffer_resolve_thread.start()

    def stop_stream(self):
        if not self.state.stream_on:
            return
        if (
            self.lucid_api.trigger_thread is not None
            and self.lucid_api.trigger_thread.is_alive()
        ):
            self.lucid_api.trigger_thread_stop.set()
            print("Waiting for stream thread to finish...")
            self.lucid_api.trigger_thread.join()
            self.state.stream_on = False

    def pose_process_thumbnail_12(self, frame):
        image = frame.reshape(-1, 1024, 2)
        image0 = cv2.cvtColor(image[..., 0], cv2.COLOR_BAYER_RG2RGB)
        image1 = cv2.cvtColor(image[..., 1], cv2.COLOR_BAYER_RG2RGB)
        image = image1.astype(np.float32) / 16 + image0.astype(np.float32) / 255 / 16
        image *= self.wb.reshape(1, 1, 3)
        image = image ** (1 / 2.2)
        return np.clip(image * 255, 0, 255).astype(np.uint8)

    def post_process_thumbnail(self, frame, frame_id: str):
        """썸네일 후처리 - RGB와 TOF를 구분하여 처리"""

        if self.is_tof:
            return self.post_process_tof_separated_thumbnail(frame, frame_id)
        if self.lucid_api.type == "TRI032S-C":
            return self.pose_process_thumbnail_12(frame)
        return self.post_process_rgb_thumbnail(frame)

    def post_process_tof_separated_thumbnail(self, frame_data, frame_id):
        """이미 분리된 depth와 intensity 데이터로 썸네일 생성"""
        if frame_id == "depth":
            # Depth를 JET 컬러맵으로 시각화 (멀리: 빨강, 가까이: 파랑)
            valid_mask = (frame_data > 0) & (
                frame_data < 40000
            )  # 0~40m 범위 내 유효한 값만
            depth_colorized = self.normalize_and_colorize(
                frame_data, cv2.COLORMAP_JET, valid_mask
            )
            return depth_colorized
        elif frame_id == "intensity":
            # Intensity를 grayscale로 시각화
            valid_mask = frame_data > 0
            intensity_colorized = self.create_intensity_image(frame_data, valid_mask)
            return intensity_colorized
        else:
            print(f"Unknown frame_id for TOF thumbnail: {frame_id}")
            return np.zeros((480, 640, 3), dtype=np.uint8)

    def post_process_rgb_thumbnail(self, frame):
        """RGB 카메라용 썸네일 후처리"""
        image = frame.reshape(-1, 1440, 3)
        image0 = cv2.cvtColor(image[..., 0], cv2.COLOR_BAYER_RG2RGB)
        image1 = cv2.cvtColor(image[..., 1], cv2.COLOR_BAYER_RG2RGB)
        image2 = cv2.cvtColor(image[..., 2], cv2.COLOR_BAYER_RG2RGB)
        image = (
            image2.astype(np.float32) / 255
            + image1.astype(np.float32) / 255 / 256
            + image0.astype(np.float32) / 255 / 256 / 256
        )
        image *= self.wb.reshape(1, 1, 3)
        image = (image * 10) ** (1 / 2.2)

        return np.clip(image * 255, 0, 255).astype(np.uint8)

    def set_exposure_time(self, exposure_us: float):
        """Set exposure time in microseconds for HDR burst imaging"""
        try:
            return self.lucid_api.set_exposure_time(int(exposure_us))
        except Exception as e:
            print(f"Failed to set exposure time for {self.name}: {e}")
            return False

    def set_gain(self, gain_value: float):
        """Set gain value for HDR high-gain burst imaging"""
        try:
            return self.lucid_api.set_gain(gain_value)
        except Exception as e:
            print(f"Failed to set gain for {self.name}: {e}")
            return False

    def get_latest_frame(self):
        """Get the most recent frame from the video stream buffer"""
        try:
            # 연결 상태 확인
            if self.state.device_status != "connected":
                return None

            # 스트림이 활성화되어 있는지 확인
            if not self.state.stream_on:
                print(f"Stream not active for {self.name}")
                return None

            # thread-safe하게 최신 프레임 반환
            with self.frame_lock:
                latest_frame = self.lucid_api.get_latest_frame()

            if latest_frame is not None:
                # TOF 카메라인지 확인 (인스턴스 변수 사용)
                metadata = getattr(latest_frame, "metadata", {})
                if self.is_tof:
                    # TOF 데이터에서 depth와 intensity 분리
                    depth, intensity = self.extract_depth_from_tof(
                        latest_frame.buffer_np
                    )

                    if depth is not None and intensity is not None:
                        # 분리된 데이터로 새로운 프레임 생성
                        return {
                            "timestamp_ns": latest_frame.timestamp_ns,
                            "depth": depth,
                            "intensity": intensity,
                            "metadata": metadata,
                        }
                    else:
                        # 분리 실패 시 원본 데이터 반환
                        return {
                            "timestamp_ns": latest_frame.timestamp_ns,
                            "tof": latest_frame.buffer_np,
                            "metadata": metadata,
                        }
                else:
                    # RGB 카메라의 경우 기존 방식
                    return latest_frame

            print(f"No recent frame available for {self.name}")
            return None

        except Exception as e:
            print(f"Failed to get latest frame from {self.name}: {e}")
            return None

    def trigger_device(self):
        """Trigger a single capture from the camera"""
        try:
            if not (
                self.lucid_api
                and hasattr(self.lucid_api, "device")
                and self.lucid_api.device
            ):
                print(f"Device not ready for {self.name}")
                return False

            # 연결 상태 확인
            if self.state.device_status != "connected":
                print(f"Device not connected for {self.name}")
                return False

            # 트리거 노드 확인 및 실행
            try:
                trigger_node = self.lucid_api.device.nodemap.get_node("TriggerSoftware")
                if trigger_node:
                    trigger_node.execute()
                    return True
                else:
                    print(f"TriggerSoftware node not found for {self.name}")
                    return False
            except Exception as trigger_e:
                print(f"Trigger execution failed for {self.name}: {trigger_e}")
                return False

        except Exception as e:
            print(f"Failed to trigger {self.name}: {e}")
            return False

    def refresh_config(self):
        if hasattr(self.config, "refresh_config"):
            self.config.refresh_config()

    def launch_device(self):
        if self.state.device_status not in ["disconnected", "error"]:
            return
        self.state.device_status = "connecting"
        try:
            result = self.lucid_api.connect_device()
            if result:
                self.state.device_status = "connected"

            else:
                self.state.device_status = "error"
        except Exception as e:
            self.state.device_status = "error"
            print(f"Error launching device {self.name}: {e}")

    def is_tof_camera(self, metadata=None):
        """카메라가 TOF인지 확인하는 메서드 - 생성자에서 설정된 값 반환"""
        return self.is_tof

    def extract_depth_from_tof(self, frame_data):
        """TOF 데이터에서 depth와 intensity 정보를 추출 (Coord3D_CY16 포맷)"""
        try:
            # Coord3D_CY16 포맷: 32 bits per pixel = C(depth, 16bits) + Y(intensity, 16bits)

            # frame_data shape 확인
            # print(frame_data.shape)
            if (
                len(frame_data.shape) == 3 and frame_data.shape[-1] == 4
            ):  # 32 bits = 4 bytes per pixel
                # 16비트씩 2채널로 분할 (C, Y)
                height, width = frame_data.shape[:2]
                frame_data = frame_data.astype(np.uint16)
                depth = frame_data[..., 0] + frame_data[..., 1] * 256  # C 채널 (depth)
                intensity = (
                    frame_data[..., 2] + frame_data[..., 3] * 256
                )  # Y 채널 (intensity)

                return depth, intensity
            else:
                print(f"Unexpected TOF data shape for CY16: {frame_data.shape}")
                return None, None

        except Exception as e:
            print(f"Error extracting depth from TOF CY16 data: {e}")
            return None, None

    def normalize_and_colorize(self, data, colormap=cv2.COLORMAP_JET, valid_mask=None):
        """데이터를 정규화하고 컬러맵을 적용"""
        try:
            if data is None:
                return None

            # 유효한 픽셀만 고려하여 정규화
            if valid_mask is not None:
                valid_data = data[valid_mask]
                if len(valid_data) > 0:
                    data_min, data_max = np.min(valid_data), np.max(valid_data)
                else:
                    data_min, data_max = 0, 1
            else:
                data_min, data_max = np.min(data), np.max(data)

            # 정규화
            if data_max > data_min:
                data_normalized = np.clip(
                    (data.astype(np.float32) - data_min) / (data_max - data_min), 0, 1
                )
            else:
                data_normalized = np.zeros_like(data, dtype=np.float32)

            # 8비트로 변환 후 컬러맵 적용
            data_8bit = (data_normalized * 255).astype(np.uint8)
            colorized = cv2.applyColorMap(data_8bit, colormap)

            # 무효한 픽셀을 검은색으로 처리
            if valid_mask is not None:
                colorized[~valid_mask] = [0, 0, 0]

            return colorized

        except Exception as e:
            print(f"Error normalizing and colorizing data: {e}")
            return None

    def create_intensity_image(self, intensity, valid_mask=None):
        """Intensity 데이터를 grayscale BGR 이미지로 변환"""
        try:
            if intensity is None:
                return None

            # 유효한 픽셀만 고려하여 정규화
            if valid_mask is not None:
                valid_data = intensity[valid_mask]
                if len(valid_data) > 0:
                    data_min, data_max = np.min(valid_data), np.max(valid_data)
                else:
                    data_min, data_max = 0, 1
            else:
                data_min, data_max = np.min(intensity), np.max(intensity)

            # 정규화
            if data_max > data_min:
                intensity_normalized = np.clip(
                    (intensity.astype(np.float32) - data_min) / (data_max - data_min),
                    0,
                    1,
                )
            else:
                intensity_normalized = np.zeros_like(intensity, dtype=np.float32)

            # 8비트 grayscale로 변환
            intensity_8bit = (intensity_normalized * 255).astype(np.uint8)

            # BGR로 변환 (grayscale을 3채널로)
            intensity_bgr = cv2.cvtColor(intensity_8bit, cv2.COLOR_GRAY2BGR)

            # 무효한 픽셀을 검은색으로 처리
            if valid_mask is not None:
                intensity_bgr[~valid_mask] = [0, 0, 0]

            return intensity_bgr

        except Exception as e:
            print(f"Error creating intensity image: {e}")
            return None

    def validate_device(self):
        if self.state.device_status == "connecting":
            return
        try:
            if (
                self.lucid_api.device is None
                or not self.lucid_api.device.is_connected()
            ):
                # 디바이스가 연결되지 않은 경우, 전역 매니저에서 재검색 시도
                # print(f"Device {self.name} disconnected, attempting reconnection...")
                # device = _device_manager.find_device_by_serial(self.lucid_api.SERIAL)
                # if device is not None:
                #     self.lucid_api.device = device
                #     self.state.device_status = "connected"
                #     print(f"Device {self.name} reconnected successfully")
                # else:
                self.state.device_status = "disconnected"
                self.state.stream_on = False
                print(f"Device {self.name} not found in available devices")
            else:
                self.state.device_status = "connected"
                if self.state.stream_on and (
                    self.buffer_resolve_thread is None
                    or not self.buffer_resolve_thread.is_alive()
                ):
                    self.state.stream_on = False
        except Exception as e:
            print(f"Error validating device {self.name}: {e}")
            self.state.device_status = "error"
            self.state.stream_on = False

    class Config(Camera.Config):
        def __init__(self, device: "CameraLucid24Single"):
            self.device = device

            # TOF 카메라인지 확인하여 다른 설정을 제공 (인스턴스 변수 사용)
            if device.is_tof:
                # TOF 카메라의 경우 exposure와 gain 설정 제외
                configs = {
                    "PixelFormat": self.Param(
                        name="PixelFormat",
                        value="Coord3D_CY16",
                        type="str",
                    ),
                }
            else:
                # RGB 카메라의 경우 기존 설정 유지
                configs = {
                    "ExposureTime": self.Param(
                        name="ExposureTime",
                        value=50000,
                        range=[0.1, 100000],
                        type="float",
                        unit="us",
                    ),
                    "Gain": self.Param(
                        name="Gain",
                        value=0.0,
                        range=[0.0, 20.0],
                        type="float",
                        unit="dB",
                    ),
                    "ExposureAuto": self.Param(
                        name="ExposureAuto",
                        value=device.state.exposure_auto,
                        type="bool",
                    ),
                    "GainAuto": self.Param(
                        name="GainAuto",
                        value=device.state.exposure_auto,
                        type="bool",
                    ),
                    "TargetBrightness": self.Param(
                        name="TargetBrightness",
                        value=device.state.exposure_auto_target,
                        range=[0, 255],
                        type="int",
                    ),
                }

            super().__init__(configs=configs)

        def update_config(self, name: str, value: Union[float, int, bool, str]):
            if name not in self.configs:
                print(f"Config parameter '{name}' not found for {self.device.name}")
                return

            param = self.configs[name]
            if param.type == "float":
                value = float(value)
            elif param.type == "int":
                value = int(value)
            elif param.type == "bool":
                value = bool(value)

            # Handle exposure auto parameters specially (RGB 카메라만)
            if not self.device.is_tof:
                if name == "ExposureAuto" or name == "GainAuto":
                    value = "Continuous" if value else "Off"

            # TOF 카메라는 exposure/gain 설정을 하지 않음
            if self.device.is_tof and name in [
                "ExposureTime",
                "Gain",
                "ExposureAuto",
                "GainAuto",
            ]:
                print(f"Skipping {name} setting for TOF camera")
                return

            update_node_safely(self.device.lucid_api.device, name, value)
            self.refresh_config(name)

        def refresh_config(self, name=None):
            if name is not None:
                if name not in self.configs:
                    return

                config = self.configs[name]

                # Handle regular lucid camera parameters
                if self.device.lucid_api.device is not None and hasattr(
                    self.device.lucid_api.device, "nodemap"
                ):
                    try:
                        node = self.device.lucid_api.device.nodemap.get_node(name)
                        if node is not None:
                            config.value = node.value
                            if (
                                config.name == "ExposureAuto"
                                or config.name == "GainAuto"
                            ):
                                config.value = config.value == "Continuous"
                    except Exception as e:
                        print(f"Error refreshing config {name}: {e}")
            else:
                for name, config in self.configs.items():
                    # Handle regular lucid camera parameters
                    if self.device.lucid_api.device is not None and hasattr(
                        self.device.lucid_api.device, "nodemap"
                    ):
                        try:
                            node = self.device.lucid_api.device.nodemap.get_node(name)
                            if node is not None:
                                config.value = node.value
                                if (
                                    config.name == "ExposureAuto"
                                    or config.name == "GainAuto"
                                ):
                                    config.value = config.value == "Continuous"
                                if (
                                    config.type == "float"
                                    and hasattr(node, "min")
                                    and hasattr(node, "max")
                                ):
                                    config.range = [node.min, node.max]
                        except Exception as e:
                            print(f"Error refreshing config {name}: {e}")
