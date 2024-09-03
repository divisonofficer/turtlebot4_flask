from typing import Literal, Optional
import cv2
from cv2.typing import MatLike
import numpy as np

from ouster_lidar.ouster_bridge import OusterLidarData

import threading
import os
import time
import open3d as o3d


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


class StereoStorageFrame:
    class FrameCapture:
        left_path: str
        right_path: str
        disparity_path: Optional[str] = None
        disparity_viz_path: Optional[str] = None

        def __dict__(self):
            return {
                "left_path": self.left_path,
                "right_path": self.right_path,
                "disparity_path": self.disparity_path,
                "disparity_viz_path": self.disparity_viz_path,
            }

    frame_id: str
    scene_id: str
    capture_rgb: FrameCapture
    capture_nir: FrameCapture
    lidar_path: Optional[str] = None
    lidar_ply_path: Optional[str] = None
    lidar_projected_path: Optional[str] = None

    def __dict__(self):
        return {
            "capture_rgb": self.capture_rgb.__dict__(),
            "capture_nir": self.capture_nir.__dict__(),
            "lidar_path": self.lidar_path,
            "lidar_ply_path": self.lidar_ply_path,
            "lidar_projected_path": self.lidar_projected_path,
            "frame_id": self.frame_id,
            "scene_id": self.scene_id,
        }


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

    def read_storage_list(self, root=FOLDER):
        scenes = [f for f in os.listdir(root) if os.path.isdir(os.path.join(root, f))]
        scenes.sort()
        return scenes

    def read_storage_scene(self, id: str, root=FOLDER):
        scene_folder = os.path.join(root, id)
        frame_list = [
            f
            for f in os.listdir(scene_folder)
            if os.path.isdir(os.path.join(scene_folder, f))
        ]
        frame_list.sort()
        return frame_list

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

    def read_frame_property(
        self,
        scene_id: str,
        frame_id: str,
        channel: Optional[Literal["rgb", "nir"]],
        property: str,
        root=FOLDER,
        file_type: Optional[str] = ".png",
    ):
        root = f"{root}/{scene_id}/{frame_id}"
        if channel is not None:
            root = f"{root}/{channel}"
        if file_type is not None:
            return f"{root}/{property}{file_type}"
        return f"{root}/{property}"

    def read_frame_info(self, scene_id: str, frame_id: str, root=FOLDER):
        root = f"{root}/{scene_id}/{frame_id}"
        frame = StereoStorageFrame()
        frame.frame_id = frame_id
        frame.scene_id = scene_id
        frame.capture_rgb = StereoStorageFrame.FrameCapture()
        frame.capture_nir = StereoStorageFrame.FrameCapture()
        for channel, capture in zip(
            ["rgb", "nir"], [frame.capture_rgb, frame.capture_nir]
        ):
            capture.left_path = f"{root}/{channel}/left.png"
            capture.right_path = f"{root}/{channel}/right.png"
            if os.path.exists(f"{root}/{channel}/disparity.npz"):
                capture.disparity_path = f"{root}/{channel}/disparity.npz"
                capture.disparity_viz_path = f"{root}/{channel}/disparity_color.png"

        if os.path.exists(f"{root}/lidar.npz"):
            frame.lidar_path = f"{root}/lidar.npz"
        if os.path.exists(f"{root}/lidar.ply"):
            frame.lidar_ply_path = f"{root}/lidar.ply"
        return frame

    def scene_update_calibration(self, root, scene_id, calibration_id):
        calibration = np.load(f"tmp/calibration/{calibration_id}/calibration.npz")
        calibration_formed = {
            "mtx_left": calibration["mtx_left"],
            "dist_left": calibration["dist_left"],
            "mtx_right": calibration["mtx_right"],
            "dist_right": calibration["dist_right"],
            "R": calibration["R"],
            "T": calibration["T"],
            "E": calibration["E"],
            "F": calibration["F"],
        }
        np.savez(f"{root}/{scene_id}/calibration.npz", **calibration_formed)

    def disparity_to_depth(self, fx, baseline, disparity):
        return fx * baseline / disparity

    def depth_to_pointcloud(self, depth, fx, fy, cx, cy):
        x = np.arange(depth.shape[1])
        y = np.arange(depth.shape[0])
        xx, yy = np.meshgrid(x, y)
        xx = (xx - cx) * depth / fx
        yy = (yy - cy) * depth / fy
        # Remove inf and nan values from depth
        points = np.stack([xx, yy, depth], axis=-1)
        points = points[np.isfinite(points).all(axis=2)]
        return points

    def post_process_pointcloud(self, root, scene_id, frame_id):
        points_path = f"{root}/{scene_id}/{frame_id}/lidar.npz"
        rgb_disparity_path = f"{root}/{scene_id}/{frame_id}/rgb/disparity.npz"

        calibration = np.load(f"{root}/{scene_id}/calibration.npz")
        fx = calibration["mtx_left"][0, 0]
        fy = calibration["mtx_left"][1, 1]
        bl = np.linalg.norm(calibration["T"])
        disparity = np.load(rgb_disparity_path)["arr_0"]
        depth = self.disparity_to_depth(fx, bl, disparity)
        cx = calibration["mtx_left"][0, 2]
        cy = calibration["mtx_left"][1, 2]
        camera_points = self.depth_to_pointcloud(depth, fx, fy, cx, cy)
        lidar_points = np.load(points_path)["points"].reshape(-1, 3) * 1000

        lidar_pcd = o3d.geometry.PointCloud()
        lidar_pcd.points = o3d.utility.Vector3dVector(lidar_points)

        camera_pcd = o3d.geometry.PointCloud()
        camera_pcd.points = o3d.utility.Vector3dVector(camera_points)

        lidar_pcd_path = f"{root}/{scene_id}/{frame_id}/lidar.ply"
        camera_pcd_path = f"{root}/{scene_id}/{frame_id}/camera_rgb.ply"

        o3d.io.write_point_cloud(lidar_pcd_path, lidar_pcd)
        o3d.io.write_point_cloud(camera_pcd_path, camera_pcd)
