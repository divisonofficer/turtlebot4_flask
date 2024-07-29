import cv2
from cv2.typing import MatLike


class StereoCaptureItem:
    left: MatLike
    right: MatLike
    disparity: MatLike
    disparity_color: MatLike

    def __init__(
        self,
        left: MatLike,
        right: MatLike,
        disparity: MatLike,
        disparity_color: MatLike,
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

    def __dell(self):
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
        cv2.imwrite(f"{root}/disparity.png", item.disparity)
        cv2.imwrite(f"{root}/disparity_color.png", item.disparity_color)

    def store_item(self, id: str, item: StereoMultiItem):
        def store():
            time_stamp = time.strftime("%H_%M_%S", time.localtime(item.timestamp))
            os.makedirs(self.FOLDER, exist_ok=True)
            os.makedirs(f"{self.FOLDER}/{id}", exist_ok=True)
            self.store_stereo_item(id, time_stamp, "rgb", item.rgb)
            self.store_stereo_item(id, time_stamp, "nir", item.nir)
            cv2.imwrite(f"{self.FOLDER}/{id}/{time_stamp}_merged.png", item.merged)

        thread = threading.Thread(target=store)
        thread.start()
        thread.join()

        del item
