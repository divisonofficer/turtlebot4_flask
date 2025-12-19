from curses import meta
import traceback
from typing import Dict, List, Literal, Optional
import cv2
import numpy as np
import os
import threading
import time
import json
import traceback
import gc

try:
    import psutil

    PSUTIL_AVAILABLE = True
except ImportError:
    print("Warning: psutil not available. Memory monitoring will be limited.")
    PSUTIL_AVAILABLE = False

try:
    import OpenEXR
    import Imath

    EXR_AVAILABLE = True
except ImportError:
    print("Warning: OpenEXR not available. Will use TIFF format instead.")
    EXR_AVAILABLE = False
EXR_AVAILABLE = False
from synchronized_queue import SQueue
from sensors.sensor import Sensor
from ouster_lidar.ouster_bridge import OusterLidarData
from lucid_py_api import LucidImage


class StereoCaptureItem:

    left: LucidImage
    right: LucidImage

    def __init__(
        self,
        left: LucidImage,
        right: LucidImage,
    ):
        self.left = left
        self.right = right

    def __del__(self):
        del self.left
        del self.right


class StereoMultiItem:
    rgb: StereoCaptureItem
    lidar: OusterLidarData
    timestamp: float
    id: str

    def __init__(
        self,
        id: str,
        timestamp: float,
        rgb: StereoCaptureItem,
        lidar: OusterLidarData,
    ):
        self.id = id
        self.rgb = rgb
        self.lidar = lidar
        self.timestamp = timestamp

    def __del__(self):
        del self.rgb
        del self.lidar
        del self.timestamp


import threading
import os
import time
from PIL import Image


def save_frame_metadata(frame_dir: str, frame_metadata: Dict[str, dict]):
    """
    프레임 전체의 메타데이터를 하나의 JSON 파일로 저장합니다.

    Args:
        frame_dir: 프레임 디렉토리 경로
        frame_metadata: 카메라별 메타데이터 딕셔너리 {'camera_name': {'exposure_time': ..., 'gain': ...}}
    """
    try:
        # 프레임 디렉토리에 metadata.json 파일 생성
        json_path = os.path.join(frame_dir, "metadata.json")

        # 타임스탬프 추가
        metadata_with_timestamp = {
            "frame_timestamp": time.time(),
            "frame_timestamp_ns": time.time_ns(),
            "cameras": frame_metadata,
        }

        # JSON 파일로 저장
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(metadata_with_timestamp, f, ensure_ascii=False, indent=2)

        print(f"Frame metadata saved: {json_path}")
        return json_path

    except Exception as e:
        print(f"Error saving frame metadata for {frame_dir}: {e}")
        return None


def create_hdr_from_exposures(
    images: List[np.ndarray],
    exposure_times: List[float],
    output_path: str,
    use_tone_mapping: bool = True,
) -> bool:
    """
    다중 노출 이미지들로부터 HDR 이미지를 생성

    Args:
        images: float32 형태의 이미지 리스트 (정규화된 0-1 범위)
        exposure_times: 각 이미지의 노출 시간 (마이크로초 단위)
        output_path: 출력 파일 경로
        use_tone_mapping: 톤 매핑 적용 여부

    Returns:
        성공 여부
    """
    try:
        if len(images) != len(exposure_times):
            print(
                f"Error: Image count ({len(images)}) doesn't match exposure times count ({len(exposure_times)})"
            )
            return False

        if len(images) < 2:
            print("Error: At least 2 images required for HDR")
            return False

        print(f"Creating HDR from {len(images)} exposures: {exposure_times}")

        # OpenCV용으로 이미지 변환 (0-255 범위의 uint8)
        cv_images = []
        for img in images:
            if img.dtype == np.float32:
                # float32 (0-1) -> uint8 (0-255)
                cv_img = np.clip(img * 255.0, 0, 255).astype(np.uint8)
            else:
                cv_img = img.astype(np.uint8)

            # 단일 채널이면 3채널로 변환
            if len(cv_img.shape) == 2:
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)
            elif cv_img.shape[2] == 1:
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)

            cv_images.append(cv_img)

        # 노출 시간을 초 단위로 변환 (마이크로초 -> 초)
        exposure_times_sec = np.array(exposure_times, dtype=np.float32) / 1_000_000.0

        print(f"Exposure times (seconds): {exposure_times_sec}")

        # HDR 합성
        merge_debevec = cv2.createMergeDebevec()
        hdr_image = merge_debevec.process(cv_images, exposure_times_sec)

        print(f"HDR image shape: {hdr_image.shape}, dtype: {hdr_image.dtype}")

        if use_tone_mapping:
            # 톤 매핑 적용 (Reinhard)
            tonemap = cv2.createTonemapReinhard(gamma=2.2)
            ldr_image = tonemap.process(hdr_image)

            # 0-255 범위로 변환
            ldr_image = np.clip(ldr_image * 255, 0, 255).astype(np.uint8)

            # LDR 이미지 저장 (JPG 또는 PNG)
            ldr_path = output_path.replace(".exr", "_tonemapped.jpg")
            cv2.imwrite(ldr_path, ldr_image)
            print(f"Tone-mapped HDR saved: {ldr_path}")

        # 원본 HDR 이미지 저장 (EXR 형식)
        if EXR_AVAILABLE:
            save_hdr_as_exr(hdr_image, output_path)
        else:
            # EXR을 사용할 수 없으면 32-bit TIFF로 저장
            tiff_path = output_path.replace(".exr", "_hdr.tiff")
            cv2.imwrite(tiff_path, hdr_image.astype(np.float32))
            print(f"HDR saved as TIFF: {tiff_path}")

        return True

    except Exception as e:
        print(f"Error creating HDR: {e}")
        return False


def save_hdr_as_exr(hdr_image: np.ndarray, file_path: str):
    """HDR 이미지를 EXR 형식으로 저장"""
    try:
        if not EXR_AVAILABLE:
            raise Exception("OpenEXR not available")

        height, width = hdr_image.shape[:2]

        # BGR에서 RGB로 변환
        if len(hdr_image.shape) == 3 and hdr_image.shape[2] == 3:
            rgb_image = cv2.cvtColor(hdr_image, cv2.COLOR_BGR2RGB)
        else:
            rgb_image = hdr_image

        # EXR 헤더 생성
        header = OpenEXR.Header(width, height)
        header["channels"] = {
            "R": Imath.Channel(Imath.PixelType(Imath.PixelType.FLOAT)),
            "G": Imath.Channel(Imath.PixelType(Imath.PixelType.FLOAT)),
            "B": Imath.Channel(Imath.PixelType(Imath.PixelType.FLOAT)),
        }

        # 디렉터리 생성
        os.makedirs(os.path.dirname(file_path), exist_ok=True)

        # EXR 파일 생성 및 저장
        exr = OpenEXR.OutputFile(file_path, header)

        if len(rgb_image.shape) == 3 and rgb_image.shape[2] >= 3:
            r_channel = rgb_image[:, :, 0].astype(np.float32).tobytes()
            g_channel = rgb_image[:, :, 1].astype(np.float32).tobytes()
            b_channel = rgb_image[:, :, 2].astype(np.float32).tobytes()
        else:
            # 단일 채널인 경우 RGB 모두 같은 값으로
            gray = rgb_image.astype(np.float32)
            r_channel = gray.tobytes()
            g_channel = gray.tobytes()
            b_channel = gray.tobytes()

        exr.writePixels({"R": r_channel, "G": g_channel, "B": b_channel})

        exr.close()
        print(f"HDR EXR saved: {file_path}")

    except Exception as e:
        print(f"Error saving HDR EXR: {e}")
        raise


class StereoStorage:
    FOLDER = "tmp/lucid"

    # 큐 및 메모리 관리 설정
    MAX_QUEUE_SIZE = 500  # 최대 큐 크기 (기본 1000장의 절반)
    CRITICAL_QUEUE_SIZE = 800  # 경고 임계값
    MAX_MEMORY_MB = 16192  # 최대 메모리 사용량 (8GB)
    CRITICAL_MEMORY_MB = 10144  # 메모리 경고 임계값 (6GB)

    def __init__(self):
        self.storage_queue: List = []  # 다양한 타입을 받을 수 있도록 수정
        self.storage_disabled = False  # 저장 비활성화 플래그
        self.last_memory_check = 0
        self.memory_check_interval = 1.0  # 1초마다 메모리 체크
        self.queue_full_warnings = 0
        # 실제로 디스크에 저장된 프레임(또는 HDR item) 카운터
        self.saved_count = 0

    def check_memory_usage(self):
        """메모리 사용량을 체크하고 임계치 초과 시 경고/조치"""
        if not PSUTIL_AVAILABLE:
            return False, 0

        try:
            process = psutil.Process()
            memory_mb = process.memory_info().rss / 1024 / 1024

            if memory_mb > self.CRITICAL_MEMORY_MB:
                print(
                    f"CRITICAL: Memory usage {memory_mb:.1f}MB exceeds critical threshold {self.CRITICAL_MEMORY_MB}MB"
                )
                gc.collect()  # 강제 가비지 컬렉션
                return True, memory_mb
            elif memory_mb > self.MAX_MEMORY_MB * 0.8:
                print(f"WARNING: High memory usage {memory_mb:.1f}MB")

            return False, memory_mb
        except Exception as e:
            print(f"Error checking memory: {e}")
            return False, 0

    def should_skip_storage(self):
        """저장을 건너뛸지 결정"""
        current_time = time.time()

        # 주기적으로 메모리 체크
        if current_time - self.last_memory_check > self.memory_check_interval:
            memory_critical, memory_mb = self.check_memory_usage()
            self.last_memory_check = current_time

            if memory_critical:
                self.storage_disabled = True
                print(
                    f"Storage DISABLED due to critical memory usage: {memory_mb:.1f}MB"
                )
                return True

        # 큐 크기 체크
        queue_size = len(self.storage_queue)

        if queue_size >= self.MAX_QUEUE_SIZE:
            if not self.storage_disabled:
                print(
                    f"Storage DISABLED: Queue size {queue_size} exceeds maximum {self.MAX_QUEUE_SIZE}"
                )
                self.storage_disabled = True
            return True
        elif queue_size >= self.CRITICAL_QUEUE_SIZE:
            self.queue_full_warnings += 1
            if self.queue_full_warnings % 10 == 1:  # 10번마다 한 번씩 경고
                print(
                    f"WARNING: Queue size {queue_size} approaching limit {self.MAX_QUEUE_SIZE}"
                )

        # 큐가 충분히 비었으면 저장 재활성화
        if self.storage_disabled and queue_size < self.MAX_QUEUE_SIZE * 0.5:
            print(f"Storage RE-ENABLED: Queue size reduced to {queue_size}")
            self.storage_disabled = False
            self.queue_full_warnings = 0

        return self.storage_disabled

    def enqueue(self, item: StereoMultiItem):
        if self.should_skip_storage():
            print(
                f"Skipping storage due to memory/queue limits. Queue size: {len(self.storage_queue)}"
            )
            # 메모리 정리를 위해 아이템 삭제
            del item
            return False

        self.storage_queue.append(item)
        return True

    def enqueue_hdr_data(self, storage_id: str, timestamp: str, hdr_data: dict):
        """HDR 데이터를 위한 별도의 enqueue 메서드"""
        if self.should_skip_storage():
            print(
                f"Skipping HDR storage due to memory/queue limits. Queue size: {len(self.storage_queue)}"
            )
            # 메모리 정리를 위해 HDR 데이터 삭제
            del hdr_data
            return False

        # HDR 데이터를 튜플 형태로 저장
        hdr_item = ("hdr", storage_id, timestamp, hdr_data)  # "hdr" 태그 추가
        self.storage_queue.append(hdr_item)
        print(
            f"HDR data enqueued: {storage_id}, {timestamp}, keys: {list(hdr_data.keys())}"
        )
        return True

    def queue_loop(self):
        """저장 큐를 처리하는 메인 루프 - 완전 비동기 처리 + 성능 최적화"""
        active_storage_threads = []
        max_concurrent_storage = 6  # 동시 저장 작업 제한 증가 (3 → 6)
        batch_size = 2  # 한 번에 처리할 항목 수

        print(
            f"Storage queue loop started. Max concurrent: {max_concurrent_storage}, Batch size: {batch_size}"
        )

        while True:
            # 완료된 저장 스레드들 정리
            active_storage_threads = [t for t in active_storage_threads if t.is_alive()]

            current_queue_size = len(self.storage_queue)
            available_slots = max_concurrent_storage - len(active_storage_threads)

            # 저장할 항목이 있고 사용 가능한 스레드 슬롯이 있는 경우
            if current_queue_size > 0 and available_slots > 0:
                # 배치 크기 결정 (사용 가능한 슬롯과 큐 크기 고려)
                items_to_process = min(batch_size, available_slots, current_queue_size)

                if current_queue_size > self.CRITICAL_QUEUE_SIZE:
                    # 큐가 임계 크기를 넘으면 더 적극적으로 처리
                    items_to_process = min(available_slots, current_queue_size)

                print(
                    f"Processing {items_to_process} items, queue length: {current_queue_size}, active threads: {len(active_storage_threads)}"
                )

                # 배치로 항목들 처리
                for _ in range(items_to_process):
                    if len(self.storage_queue) == 0:
                        break

                    item = self.storage_queue.pop(0)

                    # 저장 작업을 별도 스레드에서 비동기 실행
                    def async_storage_worker(storage_item):
                        try:
                            # HDR 데이터인지 확인
                            if (
                                isinstance(storage_item, tuple)
                                and len(storage_item) == 4
                                and storage_item[0] == "hdr"
                            ):
                                # HDR 데이터 저장
                                _, storage_id, timestamp, hdr_data = storage_item
                                self.store_hdr_item(storage_id, timestamp, hdr_data)
                            elif (
                                isinstance(storage_item, tuple)
                                and len(storage_item) == 3
                            ):
                                # 기존 방식 (일반 데이터)
                                self.store_queue_item(*storage_item)
                            else:
                                # StereoMultiItem 객체
                                if not isinstance(storage_item, tuple):
                                    self.store_item(storage_item.id, storage_item)
                        except Exception as e:
                            print(f"Storage error: {e}")
                            traceback.print_exc()
                        finally:
                            # 메모리 정리 강화
                            if hasattr(storage_item, "__del__"):
                                del storage_item
                            gc.collect()

                    # 비동기 저장 스레드 시작
                    storage_thread = threading.Thread(
                        target=async_storage_worker, args=(item,), daemon=True
                    )
                    storage_thread.start()
                    active_storage_threads.append(storage_thread)

            # 동적 슬립 시간 조정
            if current_queue_size > self.CRITICAL_QUEUE_SIZE:
                sleep_time = 0.005  # 큐가 가득 찬 경우 더 자주 체크
            elif current_queue_size > 100:
                sleep_time = 0.01  # 보통 부하일 때
            else:
                sleep_time = 0.02  # 낮은 부하일 때는 덜 자주 체크

            time.sleep(sleep_time)

    def convert_24bit_bayer_to_float32(self, image_data: np.ndarray) -> np.ndarray:
        """
        24비트 HDR Bayer 이미지 (H x W x 3 uint8)를 float32 1채널로 변환
        각 픽셀의 3바이트를 24비트 정수로 결합한 후 float32로 정규화
        """

        if image_data.dtype == np.float32:
            return image_data
        if len(image_data.shape) == 2:
            bitsize = image_data.dtype.itemsize * 8
            return image_data.astype(np.float32) / (2**bitsize - 1)

        if len(image_data.shape) != 3:
            raise ValueError(f"Expected H x W x 3 image, got shape {image_data.shape}")

        # shape[2]에 따라 비트 깊이 결정
        if image_data.shape[2] == 2:
            # 12비트 이미지: 2개의 uint8 채널을 12비트 정수로 결합
            combined = image_data[:, :, 0].astype(np.uint32) + (
                image_data[:, :, 1].astype(np.uint32) << 8
            )
            # 12비트 최대값 (2^12 - 1 = 4095)으로 정규화
            max_bits = 2**12 - 1
        elif image_data.shape[2] == 3:
            # 24비트 이미지: 3개의 uint8 채널을 24비트 정수로 결합
            # Little-endian 방식으로 결합: channel0 + (channel1 << 8) + (channel2 << 16)
            combined = (
                image_data[:, :, 0].astype(np.uint32)
                + (image_data[:, :, 1].astype(np.uint32) << 8)
                + (image_data[:, :, 2].astype(np.uint32) << 16)
            )
            # 24비트 최대값 (2^24 - 1 = 16777215)으로 정규화
            max_bits = 2**24 - 1
        else:
            raise ValueError(
                f"Unsupported channel count: {image_data.shape[2]}. Expected 2 (12-bit) or 3 (24-bit)"
            )

        # 정규화하여 float32 변환
        float_image = combined.astype(np.float32) / max_bits

        return float_image

    def save_float32_as_exr(self, filename: str, image_data: np.ndarray):
        """float32 이미지를 EXR 포맷으로 저장"""
        if EXR_AVAILABLE:
            try:
                # OpenEXR을 사용하여 저장
                height, width = image_data.shape
                header = OpenEXR.Header(width, height)
                header["channels"] = {
                    "Y": Imath.Channel(Imath.PixelType(Imath.PixelType.FLOAT))
                }

                # 이미지 데이터를 string으로 변환
                image_string = image_data.astype(np.float32).tobytes()

                exr_file = OpenEXR.OutputFile(filename, header)
                exr_file.writePixels({"Y": image_string})
                exr_file.close()

                print(f"Saved EXR: {filename}")
                return True
            except Exception as e:
                print(f"Failed to save EXR: {e}")
                return False
        else:
            # OpenEXR이 없으면 TIFF로 대체 저장
            try:
                cv2.imwrite(filename, image_data.astype(np.float32))
                print(f"Saved exr (fallback): {filename}")
                return True
            except Exception as e:
                tiff_filename = filename.replace(".exr", ".tiff")
                cv2.imwrite(tiff_filename, image_data)
                print(f"Saved TIFF (fallback): {tiff_filename}")
            return True

    def uint8buffer_to_uint32(self, buffer: np.ndarray) -> np.ndarray:
        result_buffer = np.zeros((buffer.shape[0], buffer.shape[1]), dtype=np.uint32)
        for idx in range(buffer.shape[-1]):
            result_buffer += buffer[:, :, idx].astype(np.uint32) << (8 * idx)
        return result_buffer

    def save_lidar(self, folder: str, item: OusterLidarData):
        lidar_reflectivity_uint8 = item.reflectivity.astype(np.uint8)
        cv2.imwrite(f"{folder}/lidar_reflectivity.png", lidar_reflectivity_uint8)
        lidar_range_uint8 = (item.ranges / 255.0).astype(np.uint8)
        cv2.imwrite(f"{folder}/lidar_range.png", lidar_range_uint8)
        Image.fromarray(item.reflectivity).save(f"{folder}/lidar_reflectivity.tiff")
        Image.fromarray(item.ranges).save(f"{folder}/lidar_range.tiff")

    def store_sensor_frame(self, folder, frame):
        # folder = f"{self.FOLDER}/{storage_id}/{time_stamp}/key"
        if isinstance(frame, dict):
            for key, data in frame.items():
                if key == "metadata":
                    continue
                np.save(f"{folder}/{key}.npy", data)
            return
            
        for src, data in frame.data.items():
            if src == "metadata":
                continue
            if frame.file_format[src] == "npy":
                np.save(f"{folder}/{src}.npy", data)
            if frame.file_format[src] == "exr":
                data = self.convert_24bit_bayer_to_float32(data)
                self.save_float32_as_exr(f"{folder}/{src}.exr", data)
            if frame.file_format[src] == "png":
                cv2.imwrite(f"{folder}/{src}.png", data)

    def store_queue_item(
        self, storage_id: str, timestamp: float, item: Dict[str, SQueue.Item]
    ):
        time_stamp = time.strftime("%H_%M_%S_", time.localtime(timestamp)) + str(
            int((timestamp % 1) * 1000)
        ).zfill(3)
        os.makedirs(f"{self.FOLDER}/{storage_id}/{time_stamp}", exist_ok=True)
        metadata = {}
        for key, data in item.items():
            frame: Sensor.Frame = data.data
            if "metadata" in frame.data:
                metadata[key] = frame.data["metadata"]
            os.makedirs(f"{self.FOLDER}/{storage_id}/{time_stamp}/{key}", exist_ok=True)
            self.store_sensor_frame(
                f"{self.FOLDER}/{storage_id}/{time_stamp}/{key}", frame
            )
            # for src, data in frame.data.items():
            #     if src == "metadata":
            #         metadata[key] = data  # 메타데이터 수집
            #         continue
            #     id = f"{key}_{src}"
            #     if frame.file_format[src] == "npy":
            #         np.save(f"{self.FOLDER}/{storage_id}/{time_stamp}/{id}.npy", data)
            #     if frame.file_format[src] == "exr":
            #         data = self.convert_24bit_bayer_to_float32(data)
            #         self.save_float32_as_exr(
            #             f"{self.FOLDER}/{storage_id}/{time_stamp}/{id}.exr", data
            #         )
            #     if frame.file_format[src] == "png":
            #         cv2.imwrite(
            #             f"{self.FOLDER}/{storage_id}/{time_stamp}/{id}.png", data
            #         )
        if metadata:
            save_frame_metadata(
                f"{self.FOLDER}/{storage_id}/{time_stamp}", metadata
            )  # 메타데이터 저장
        # 한 프레임(아이템) 저장이 완료되었음을 카운트
        try:
            self.saved_count += 1
        except Exception:
            pass

    def store_hdr_item(self, storage_id: str, timestamp: str, hdr_data: dict):
        """HDR 데이터를 저장하는 메서드"""
        try:
            # 타임스탬프에서 시간 정보 추출 (_hdr_burst 제거)
            clean_timestamp = timestamp.replace("_hdr_burst", "")

            # 디렉토리 생성
            root_dir = f"{self.FOLDER}/{storage_id}/{clean_timestamp}_hdr"
            os.makedirs(root_dir, exist_ok=True)

            print(f"Storing HDR data to: {root_dir}")

            # 프레임 전체의 메타데이터를 수집
            frame_metadata = {}

            # HDR 데이터 저장
            for sensor_name, sensor_data in hdr_data.items():
                sensor_dir = f"{root_dir}/{sensor_name}"
                os.makedirs(sensor_dir, exist_ok=True)

                # 센서별 메타데이터 수집
                sensor_metadata = {}
                print(f"Processing sensor: {sensor_name}, data type: {type(sensor_data)}")
                if isinstance(sensor_data, list):
                    # 카메라 데이터 (여러 노출시간의 프레임들)
                    for idx, frame in enumerate(sensor_data):
                        try:
                            if (
                                hasattr(frame, "buffer_np")
                                and frame.buffer_np is not None
                            ):
                                float_bayer = self.convert_24bit_bayer_to_float32(
                                    frame.buffer_np
                                )
                                # EXR 포맷으로 저장
                                filename = f"{sensor_dir}/exposure_{idx:02d}.exr"
                                success = self.save_float32_as_exr(
                                    filename, float_bayer
                                )
                                # LucidImage에 메타데이터가 있으면 수집
                                if hasattr(frame, "metadata") and frame.metadata:
                                    sensor_metadata[f"exposure_{idx:02d}"] = (
                                        frame.metadata
                                    )
                            elif hasattr(frame, "data") and isinstance(
                                frame.data, dict
                            ):
                                # Frame 객체 (다른 카메라 및 High-gain 모드)
                                if "metadata" in frame.data:
                                    metadata = frame.data["metadata"]
                                    sensor_metadata[f"exposure_{idx:02d}"] = metadata

                                if "image" in frame.data:
                                    image_data = frame.data["image"]
                                    filename = f"{sensor_dir}/exposure_{idx:02d}.png"
                                    cv2.imwrite(filename, image_data)
                                    print(f"Saved frame: {filename}")
                                else:
                                    print(
                                        f"No 'image' key in frame.data for {sensor_name}"
                                    )
                            else:
                                print(
                                    f"Unsupported frame type for {sensor_name}[{idx}]: {type(frame)}"
                                )

                        except Exception as frame_e:
                            print(
                                f"Error processing frame {idx} for {sensor_name}: {frame_e}"
                            )

                    # 센서별 메타데이터를 프레임 메타데이터에 추가
                    if sensor_metadata:
                        frame_metadata[sensor_name] = sensor_metadata

                else:
                    
                    self.store_sensor_frame(sensor_dir, sensor_data)

            # 프레임 전체의 메타데이터를 하나의 JSON 파일로 저장
            if frame_metadata:
                save_frame_metadata(root_dir, frame_metadata)
            print(f"HDR storage completed for {storage_id}")
            # HDR burst 저장이 완료된 것으로 간주하고 카운트
            try:
                self.saved_count += 1
            except Exception:
                pass

        except Exception as e:
            print(f"Error storing HDR data: {e}")

            traceback.print_exc()

    def store_item(self, id: str, item: StereoMultiItem):
        """개별 항목 저장 - 제한된 비동기 처리로 메모리 사용량 최적화"""
        time_stamp = time.strftime("%H_%M_%S_", time.localtime(item.timestamp)) + str(
            int((item.timestamp % 1) * 1000)
        ).zfill(3)
        dir_path = f"{self.FOLDER}/{id}/{time_stamp}"
        os.makedirs(dir_path, exist_ok=True)

        store_dict = {
            **item.lidar.__dict__(),
            "left": item.rgb.left.buffer_np,
            "right": item.rgb.right.buffer_np,
            "timestamp_ns": item.timestamp,
        }

        # 제한된 비동기 처리 - 큰 파일들은 비동기, 작은 파일들은 동기
        save_threads = []
        max_async_files = 4  # 비동기 파일 수 증가 (2 → 4)

        # 큰 파일들 (이미지 데이터)은 비동기로 처리
        async_items = []
        sync_items = []

        for k, v in store_dict.items():
            # 이미지 데이터와 라이다 데이터는 비동기로 처리
            if (
                k in ["left", "right"]
                or "range" in k
                or "intensity" in k
                or "reflectivity" in k
            ):
                async_items.append((k, v))
            else:  # 작은 데이터는 동기
                sync_items.append((k, v))

        # 동기 파일 저장 (작은 파일들)
        for k, v in sync_items:
            try:
                np.save(f"{dir_path}/{k}.npy", v)
            except Exception as e:
                print(f"Error saving sync file {k}: {e}")

        # 비동기 파일 저장 (큰 파일들만)
        def save_file_async(filename, data, key):
            try:
                np.save(filename, data)
                print(f"Async save completed: {key}")
            except Exception as e:
                print(f"Error saving async file {key}: {e}")
            finally:
                # 메모리 정리
                del data

        for k, v in async_items[:max_async_files]:  # 최대 개수 제한
            save_thread = threading.Thread(
                target=save_file_async,
                args=(f"{dir_path}/{k}.npy", v, k),
                daemon=True,
            )
            save_thread.start()
            save_threads.append(save_thread)

        # 비동기 파일들이 너무 많으면 나머지는 동기로 처리
        for k, v in async_items[max_async_files:]:
            try:
                np.save(f"{dir_path}/{k}.npy", v)
            except Exception as e:
                print(f"Error saving overflow file {k}: {e}")

        # 비동기 저장 스레드들이 완료되기를 기다림 (데이터 일관성 보장)
        for thread in save_threads:
            thread.join()  # timeout 제거 - 완전히 끝날 때까지 대기

        print(f"Frame storage completed: {id}/{time_stamp} ({len(store_dict)} files)")

        # 메모리 정리는 모든 저장 완료 후 수행
        del item
        del store_dict
        gc.collect()
        # 저장 완료 카운트
        try:
            self.saved_count += 1
        except Exception:
            pass

    def get_status(self):
        """현재 저장 시스템 상태 반환"""
        status = {
            "queue_size": len(self.storage_queue),
            "max_queue_size": self.MAX_QUEUE_SIZE,
            "critical_queue_size": self.CRITICAL_QUEUE_SIZE,
            "storage_disabled": self.storage_disabled,
            "queue_full_warnings": self.queue_full_warnings,
        }

        # 저장된 프레임(아이템) 카운트 추가
        try:
            status["saved_count"] = int(self.saved_count)
        except Exception:
            status["saved_count"] = 0

        if PSUTIL_AVAILABLE:
            try:
                process = psutil.Process()
                memory_mb = process.memory_info().rss / 1024 / 1024
                status.update(
                    {
                        "memory_usage_mb": round(memory_mb, 1),
                        "max_memory_mb": self.MAX_MEMORY_MB,
                        "critical_memory_mb": self.CRITICAL_MEMORY_MB,
                        "memory_percent": round(
                            (memory_mb / self.MAX_MEMORY_MB) * 100, 1
                        ),
                    }
                )
            except Exception as e:
                status["memory_error"] = str(e)
        else:
            status["memory_monitoring"] = "unavailable (psutil not installed)"

        return status

    def print_status(self):
        """상태 정보를 콘솔에 출력"""
        status = self.get_status()
        print(f"=== Storage System Status ===")
        print(
            f"Queue: {status['queue_size']}/{status['max_queue_size']} (Critical: {status['critical_queue_size']})"
        )
        print(f"Storage: {'DISABLED' if status['storage_disabled'] else 'ENABLED'}")

        if "memory_usage_mb" in status:
            print(
                f"Memory: {status['memory_usage_mb']}MB/{status['max_memory_mb']}MB ({status['memory_percent']}%)"
            )

        if status["queue_full_warnings"] > 0:
            print(f"Queue warnings: {status['queue_full_warnings']}")
        print("==============================")
