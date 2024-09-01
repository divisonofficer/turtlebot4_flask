from typing import Optional
import cv2
from cv2.typing import MatLike
import numpy as np

from ouster_lidar.ouster_bridge import OusterLidarData


class StereoCaptureItem:
    left: MatLike
    right: MatLike
    disparity: Optional[MatLike]
    disparity_color: Optional[MatLike]

    def __init__(
        self,
        left: MatLike,
        right: MatLike,
        disparity: Optional[MatLike] = None,
        disparity_color: Optional[MatLike] = None,
    ):
        self.left = left
        self.right = right
        self.disparity = disparity
        self.disparity_color = disparity_color

    def __del__(self):
        del self.left
        del self.right
        del self.disparity
        del self.disparity_color


class StereoMultiItem:
    rgb: StereoCaptureItem
    nir: StereoCaptureItem
    merged: MatLike
    timestamp: float

    def __init__(
        self,
        timestamp: float,
        rgb: StereoCaptureItem,
        nir: StereoCaptureItem,
        merged: MatLike,
    ):
        self.rgb = rgb
        self.nir = nir
        self.merged = merged
        self.timestamp = timestamp

    def __del__(self):
        del self.rgb
        del self.nir
        del self.merged
        del self.timestamp


import threading
import os
import time


class StereoStorage:
    FOLDER = "tmp/depth"

    def store_stereo_item(
        self, id: str, timestamp: str, channel: str, item: StereoCaptureItem
    ):
        root = f"{self.FOLDER}/{id}/{timestamp}"
        os.makedirs(root, exist_ok=True)
        root = f"{root}/{channel}"
        os.makedirs(root, exist_ok=True)
        cv2.imwrite(f"{root}/left.png", item.left)
        cv2.imwrite(f"{root}/right.png", item.right)
        if item.disparity is not None and item.disparity_color is not None:
            cv2.imwrite(f"{root}/disparity.png", item.disparity)
            cv2.imwrite(f"{root}/disparity_color.png", item.disparity_color)

    def store_item(
        self, id: str, item: StereoMultiItem, lidar: Optional[OusterLidarData] = None
    ):
        def store():
            time_stamp = time.strftime(
                "%H_%M_%S_", time.localtime(item.timestamp)
            ) + str(int((item.timestamp % 1) * 1000)).zfill(3)
            os.makedirs(self.FOLDER, exist_ok=True)
            os.makedirs(f"{self.FOLDER}/{id}", exist_ok=True)
            self.store_stereo_item(id, time_stamp, "rgb", item.rgb)
            self.store_stereo_item(id, time_stamp, "nir", item.nir)
            cv2.imwrite(f"{self.FOLDER}/{id}/{time_stamp}_merged.png", item.merged)
            if lidar is not None:
                np.savez(
                    f"{self.FOLDER}/{id}/{time_stamp}/lidar.npz", **lidar.__dict__()
                )

        thread = threading.Thread(target=store)
        thread.start()
        thread.join()

        del item
