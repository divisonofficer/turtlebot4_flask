import cv2
from cv2.typing import MatLike
from sensor_msgs.msg import PointCloud2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import pcl


class StereoCaptureItem:
    left: MatLike
    right: MatLike

    def __init__(
        self,
        left: MatLike,
        right: MatLike,
    ):
        self.left = left
        self.right = right

    def __del__(self):
        del self.left
        del self.right


class StereoMultiItem:
    rgb: StereoCaptureItem
    lidar: PointCloud2
    timestamp: float

    def __init__(
        self,
        timestamp: float,
        rgb: StereoCaptureItem,
        lidar: PointCloud2,
    ):
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
        cv2.imwrite(f"{root}/left.raw", item.left)
        cv2.imwrite(f"{root}/right.raw", item.right)

    def save_pointcloud(self, filename: str, item: PointCloud2):
        cloud = pcl.PointCloud()
        points_list = []

        for point in pc2.read_points(item, skip_nans=True):
            points_list.append([point[0], point[1], point[2]])

        cloud.from_list(points_list)

        # Save to PCD file
        pcl.save(cloud, filename)

    def store_item(self, id: str, item: StereoMultiItem):
        def store():
            time_stamp = time.strftime(
                "%H_%M_%S_", time.localtime(item.timestamp)
            ) + str(int((item.timestamp % 1) * 1000)).zfill(3)
            os.makedirs(self.FOLDER, exist_ok=True)
            os.makedirs(f"{self.FOLDER}/{id}", exist_ok=True)
            self.store_stereo_item(id, time_stamp, "rgb", item.rgb)
            self.save_pointcloud(
                f"{self.FOLDER}/{id}/{time_stamp}/lidar.pcd", item.lidar
            )

        thread = threading.Thread(target=store)
        thread.start()
        thread.join()

        del item
