from typing import Dict, List, Literal
import cv2

import numpy as np
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
                self.store_queue_item(*item)
                # self.store_item(item.id, item)
                # thread = threading.Thread(
                #     target=self.store_item, args=(item.id, item), daemon=False
                # )
                # thread.start()
                # thread.join()

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

    key_dict = {
        "lucid_stereo_left": "left",
        "lucid_stereo_right": "right",
        "ouster_points": "points",
    }

    def store_queue_item(
        self, storage_id: str, timestamp: float, item: Dict[str, SQueue.Item]
    ):
        time_stamp = time.strftime("%H_%M_%S_", time.localtime(timestamp)) + str(
            int((timestamp % 1) * 1000)
        ).zfill(3)
        os.makedirs(f"{self.FOLDER}/{storage_id}/{time_stamp}", exist_ok=True)
        for key, data in item.items():
            frame: Sensor.Frame = data.data
            for src, data in frame.data.items():
                id = f"{key}_{src}"
                if id in self.key_dict:
                    id = self.key_dict[id]
                if frame.file_format[src] == "npy":
                    np.save(f"{self.FOLDER}/{storage_id}/{time_stamp}/{id}.npy", data)
                if frame.file_format[src] == "png":
                    cv2.imwrite(
                        f"{self.FOLDER}/{storage_id}/{time_stamp}/{id}.png", data
                    )

    def store_item(self, id: str, item: StereoMultiItem):

        time_stamp = time.strftime("%H_%M_%S_", time.localtime(item.timestamp)) + str(
            int((item.timestamp % 1) * 1000)
        ).zfill(3)
        os.makedirs(f"{self.FOLDER}/{id}/{time_stamp}", exist_ok=True)
        store_dict = {
            **item.lidar.__dict__(),
            "left": item.rgb.left.buffer_np,
            "right": item.rgb.right.buffer_np,
            "timestamp_ns": item.timestamp,
        }
        threads = [
            threading.Thread(
                target=np.save,
                args=(f"{self.FOLDER}/{id}/{time_stamp}/{k}.npy", v),
            )
            for k, v in store_dict.items()
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        # np.savez(
        #     f"{self.FOLDER}/{id}/{time_stamp}/raw.npz",
        #     **store_dict,
        # )

        del item
