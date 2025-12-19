import threading

import cv2
import numpy as np
from lucid_py_api import LucidPyAPI
import rclpy
from ..camera import Camera


class CameraLucid24(Camera):

    def __init__(self, name: str, lucid_api: LucidPyAPI):
        super().__init__(
            name,
            self.Const(
                srcs=2,
                raw_format="bayer",
                raw_bits=24,
                width=1440,
                height=928,
                preview_keys=["left", "right"],
            ),
        )
        self.lucid_api = lucid_api
        self.wb = np.asarray(
            [
                2.0841475322429894,
                1.0,
                1.9215893014341496,
            ]
        )

    def is_tof_camera(self, metadata=None):
        """카메라가 TOF인지 확인하는 메서드"""
        if metadata and "pixel_format" in metadata:
            return "Coord3D" in str(metadata["pixel_format"])
        return False

    def extract_depth_from_tof(self, frame_data):
        """TOF 데이터에서 depth와 intensity 정보를 추출 (Coord3D_CY16 포맷)"""
        try:
            # Coord3D_CY16 포맷: 32 bits per pixel = C(depth, 16bits) + Y(intensity, 16bits)

            # frame_data shape 확인
            if (
                len(frame_data.shape) == 3 and frame_data.shape[-1] == 4
            ):  # 32 bits = 4 bytes per pixel
                # 16비트씩 2채널로 분할 (C, Y)
                height, width = frame_data.shape[:2]
                frame_16bit = frame_data.view(np.uint16).reshape(height, width, 2)

                depth = frame_16bit[..., 0]  # C 채널 (depth)
                intensity = frame_16bit[..., 1]  # Y 채널 (intensity)

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

    def start_stream(self):
        self.lucid_api.open_stream()

        def buffer_resolve_loop():
            while rclpy.ok():
                buffers = self.lucid_api.collect_images()

                # TOF 카메라인지 확인 (단일 카메라로 처리)
                if len(buffers) == 1:  # TOF 카메라의 경우
                    buffers[0].sort(key=lambda x: x.timestamp_ns)
                    while len(buffers[0]) > 0:
                        buffer_tof = buffers[0].pop(0)

                        # TOF 데이터에서 depth와 intensity 분리
                        depth, intensity = self.extract_depth_from_tof(
                            buffer_tof.buffer_np
                        )

                        if depth is not None and intensity is not None:
                            threading.Thread(
                                target=self.frame_callback,
                                args=(
                                    self,
                                    self.Frame(
                                        buffer_tof.timestamp_ns / 1e9,
                                        {
                                            "depth": depth,
                                            "intensity": intensity,
                                        },
                                        getattr(
                                            buffer_tof, "metadata", {}
                                        ),  # TOF 메타데이터 포함
                                        file_format={
                                            "depth": "npy",
                                            "intensity": "npy",
                                        },
                                    ),
                                ),
                            ).start()
                        else:
                            # depth/intensity 분리 실패 시 원본 데이터 사용
                            threading.Thread(
                                target=self.frame_callback,
                                args=(
                                    self,
                                    self.Frame(
                                        buffer_tof.timestamp_ns / 1e9,
                                        {
                                            "tof": buffer_tof.buffer_np,
                                        },
                                        getattr(buffer_tof, "metadata", {}),
                                        file_format={"tof": "npy"},
                                    ),
                                ),
                            ).start()

                else:  # 기존 스테레오 카메라 처리
                    buffers[0].sort(key=lambda x: x.timestamp_ns)
                    buffers[1].sort(key=lambda x: x.timestamp_ns)
                    while len(buffers[0]) > 0 and len(buffers[1]) > 0:
                        if (
                            abs(buffers[0][0].timestamp_ns - buffers[1][0].timestamp_ns)
                            / 1e9
                        ) < 0.1:
                            buffer_left = buffers[0].pop(0)
                            buffer_right = buffers[1].pop(0)

                            threading.Thread(
                                target=self.frame_callback,
                                args=(
                                    self,
                                    self.Frame(
                                        buffer_left.timestamp_ns / 1e9,
                                        {
                                            "left": buffer_left.buffer_np,
                                            "right": buffer_right.buffer_np,
                                        },
                                        {},
                                        file_format={"left": "npy", "right": "npy"},
                                    ),
                                ),
                            ).start()
                        else:
                            if buffers[0][0].timestamp_ns < buffers[1][0].timestamp_ns:
                                buffers[0].pop(0)
                            else:
                                buffers[1].pop(0)

        threading.Thread(target=buffer_resolve_loop, daemon=True).start()

    def post_process_thumbnail(self, frame, frame_id: str):
        """썸네일 후처리 - RGB와 TOF를 구분하여 처리"""
        # frame에서 메타데이터 확인
        metadata = getattr(frame, "metadata", {}) if hasattr(frame, "metadata") else {}

        # TOF 카메라인지 확인 (depth/intensity 키가 있는지 또는 메타데이터로 확인)
        if hasattr(frame, "keys") and (
            "depth" in frame.keys() or "intensity" in frame.keys()
        ):
            print("Processing TOF camera thumbnail (depth/intensity separated)")
            return self.post_process_tof_separated_thumbnail(frame, frame_id, metadata)
        elif self.is_tof_camera(metadata):
            print("Processing TOF camera thumbnail (raw data)")
            return self.post_process_tof_thumbnail(frame, frame_id, metadata)
        else:
            # 기존 RGB 카메라 처리
            return self.post_process_rgb_thumbnail(frame, frame_id)

    def post_process_tof_separated_thumbnail(self, frame_data, frame_id: str, metadata):
        """이미 분리된 depth와 intensity 데이터로 썸네일 생성"""
        try:
            depth = frame_data.get("depth")
            intensity = frame_data.get("intensity")

            if depth is not None and intensity is not None:
                # 유효한 픽셀 마스크 생성 (intensity > 0인 픽셀들)
                valid_mask = intensity > 0

                # Depth를 JET 컬러맵으로 시각화 (멀리: 빨강, 가까이: 파랑)
                depth_colorized = self.normalize_and_colorize(
                    depth, cv2.COLORMAP_JET, valid_mask
                )

                # Intensity를 grayscale로 시각화
                intensity_colorized = self.create_intensity_image(intensity, valid_mask)

                if depth_colorized is not None and intensity_colorized is not None:
                    # 두 이미지를 좌우로 concat (depth | intensity)
                    combined = np.hstack([depth_colorized, intensity_colorized])
                    return combined
                elif depth_colorized is not None:
                    return depth_colorized
                elif intensity_colorized is not None:
                    return intensity_colorized
                else:
                    # 둘 다 실패한 경우
                    return np.zeros((480, 640, 3), dtype=np.uint8)
            else:
                # depth나 intensity가 없는 경우
                return np.zeros((480, 640, 3), dtype=np.uint8)

        except Exception as e:
            print(f"Error processing separated TOF thumbnail: {e}")
            return np.zeros((480, 640, 3), dtype=np.uint8)

    def post_process_tof_thumbnail(self, frame, frame_id: str, metadata):
        """TOF 카메라용 썸네일 후처리 - depth와 intensity를 좌우로 concat하여 표시"""
        try:
            # TOF 데이터에서 depth와 intensity 추출
            depth, intensity = self.extract_depth_from_tof(frame)

            if depth is not None and intensity is not None:
                # 유효한 픽셀 마스크 생성 (intensity > 0인 픽셀들)
                valid_mask = intensity > 0

                # Depth를 JET 컬러맵으로 시각화 (멀리: 빨강, 가까이: 파랑)
                depth_colorized = self.normalize_and_colorize(
                    depth, cv2.COLORMAP_JET, valid_mask
                )

                # Intensity를 grayscale로 시각화
                intensity_colorized = self.create_intensity_image(intensity, valid_mask)

                if depth_colorized is not None and intensity_colorized is not None:
                    # 두 이미지를 좌우로 concat (depth | intensity)
                    combined = np.hstack([depth_colorized, intensity_colorized])
                    return combined
                elif depth_colorized is not None:
                    # depth만 성공한 경우
                    return depth_colorized
                elif intensity_colorized is not None:
                    # intensity만 성공한 경우
                    return intensity_colorized
                else:
                    # 둘 다 실패한 경우 depth를 grayscale로 표시
                    depth_min, depth_max = np.min(depth), np.max(depth)
                    if depth_max > depth_min:
                        depth_normalized = (
                            (depth - depth_min) / (depth_max - depth_min) * 255
                        ).astype(np.uint8)
                    else:
                        depth_normalized = np.zeros_like(depth, dtype=np.uint8)
                    return cv2.cvtColor(depth_normalized, cv2.COLOR_GRAY2BGR)
            else:
                # depth/intensity 추출 실패 시 원본 데이터를 grayscale로 표시
                if len(frame.shape) == 3 and frame.shape[2] > 1:
                    # 첫 번째 채널만 사용
                    gray = frame[..., 0]
                else:
                    gray = frame.squeeze()

                gray_min, gray_max = np.min(gray), np.max(gray)
                if gray_max > gray_min:
                    gray_normalized = (
                        (gray - gray_min) / (gray_max - gray_min) * 255
                    ).astype(np.uint8)
                else:
                    gray_normalized = np.zeros_like(gray, dtype=np.uint8)
                return cv2.cvtColor(gray_normalized, cv2.COLOR_GRAY2BGR)

        except Exception as e:
            print(f"Error processing TOF thumbnail: {e}")
            # 오류 발생 시 기본 처리
            try:
                if len(frame.shape) == 3:
                    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                else:
                    gray = frame.astype(np.uint8)
                return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            except:
                # 최후의 수단: 빈 이미지 반환
                return np.zeros((480, 640, 3), dtype=np.uint8)

    def post_process_rgb_thumbnail(self, frame, frame_id: str):
        """기존 RGB 카메라용 썸네일 후처리"""
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
        image = image ** (1 / 2.2)

        return (image * 255).astype(np.uint8)
