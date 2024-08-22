from typing import List, Literal
import cv2
from cv2.typing import MatLike
import numpy as np
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


class StereoStorage:
    FOLDER = "tmp/lucid"

    def __init__(self):
        self.storage_queue: List[StereoMultiItem] = []

    def enqueue(self, item: StereoMultiItem):
        self.storage_queue.append(item)

    def queue_loop(self):
        threads: List[threading.Thread] = []
        while True:
            if len(self.storage_queue) > 0:
                print("Storing item, queue length: ", len(self.storage_queue))
                item = self.storage_queue.pop(0)
                thread = threading.Thread(
                    target=self.store_item, args=(item.id, item), daemon=False
                )
                thread.start()
            time.sleep(0.1)

    def uint8buffer_to_uint32(self, buffer: np.ndarray) -> np.ndarray:
        result_buffer = np.zeros((buffer.shape[0], buffer.shape[1]), dtype=np.uint32)
        for idx in range(buffer.shape[-1]):
            result_buffer += buffer[:, :, idx].astype(np.uint32) << (8 * idx)
        return result_buffer

    def write_image(
        self, path: str, buffer: np.ndarray, format: Literal["png", "tiff"]
    ):
        if format == "png":
            cv2.imwrite(path, buffer)
        elif format == "tiff":
            Image.fromarray(self.uint8buffer_to_uint32(buffer)).save(path)

    def store_stereo_item(
        self, id: str, timestamp: str, channel: str, item: StereoCaptureItem
    ):
        root = f"{self.FOLDER}/{id}/{timestamp}"
        os.makedirs(root, exist_ok=True)
        root = f"{root}/{channel}"
        os.makedirs(root, exist_ok=True)

        threads = [
            threading.Thread(
                target=self.write_image,
                args=(
                    f"{root}/left.png",
                    item.left.buffer_np[:, :, 2],
                    "png",
                ),
            ),
            threading.Thread(
                target=self.write_image,
                args=(
                    f"{root}/right.png",
                    item.right.buffer_np[:, :, 2],
                    "png",
                ),
            ),
            threading.Thread(
                target=self.write_image,
                args=(
                    f"{root}/left.tiff",
                    item.left.buffer_np,
                    "tiff",
                ),
            ),
            threading.Thread(
                target=self.write_image,
                args=(
                    f"{root}/right.tiff",
                    item.right.buffer_np,
                    "tiff",
                ),
            ),
        ]

        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

    def save_lidar(self, folder: str, item: OusterLidarData):
        lidar_reflectivity_uint8 = item.reflectivity.astype(np.uint8)
        cv2.imwrite(f"{folder}/lidar_reflectivity.png", lidar_reflectivity_uint8)
        lidar_range_uint8 = (item.ranges / 255.0).astype(np.uint8)
        cv2.imwrite(f"{folder}/lidar_range.png", lidar_range_uint8)
        Image.fromarray(item.reflectivity).save(f"{folder}/lidar_reflectivity.tiff")
        Image.fromarray(item.ranges).save(f"{folder}/lidar_range.tiff")

    def store_item(self, id: str, item: StereoMultiItem):

        time_stamp = time.strftime("%H_%M_%S_", time.localtime(item.timestamp)) + str(
            int((item.timestamp % 1) * 1000)
        ).zfill(3)
        os.makedirs(f"{self.FOLDER}/{id}/{time_stamp}", exist_ok=True)

        np.savez(
            f"{self.FOLDER}/{id}/{time_stamp}/raw.npz",
            reflectivity=item.lidar.reflectivity,
            ranges=item.lidar.ranges,
            left=item.rgb.left.buffer_np,
            right=item.rgb.right.buffer_np,
            points=item.lidar.points,
        )

        del item
