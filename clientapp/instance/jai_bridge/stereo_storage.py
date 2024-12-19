from typing import Any, Callable, Dict, List, Literal, Optional, Tuple
from weakref import ref
import cv2

from deprecated import deprecated
import numpy as np
import tqdm

from ouster_lidar.ouster_bridge import OusterLidarData
from geometry_msgs.msg import Pose
import threading
import os
import time
import open3d as o3d
from PIL import Image
import h5py
from io import BytesIO


class StorageItem:
    id: str
    timestamp: float

    def __init__(self, timestamp: float):
        self.timestamp = timestamp


class HDRCaptureItem(StorageItem):
    """
    hdr:
        left:
            RGB: np.ndarray
            NIR: np.ndarray
        right:
            RGB: np.ndarray
            NIR: np.ndarray
    lidar: OusterLidarData
    odom: List[Pose]
    """

    hdr: Dict[str, Dict[str, np.ndarray]]

    lidar: OusterLidarData
    odom: List[Pose]
    timestamp: float

    def __init__(
        self,
        hdr: Dict[str, Dict[str, np.ndarray]],
        lidar: OusterLidarData,
        odom: List[Pose],
        timestamp: float,
    ):
        self.hdr = hdr
        self.lidar = lidar
        self.odom = odom
        self.timestamp = timestamp
        super().__init__(timestamp)

    def __del__(self):
        del self.hdr
        del self.lidar
        del self.odom

    def h5dict(self):
        return {
            "hdr": {
                "left": {
                    "rgb": self.hdr["left"]["rgb"],
                    "nir": self.hdr["left"]["nir"],
                },
                "right": {
                    "rgb": self.hdr["right"]["rgb"],
                    "nir": self.hdr["right"]["nir"],
                },
            },
            "lidar": {
                "attrs": self.lidar.meta_dict(),
                "imu": self.lidar.imu.dict(),
                "points": self.lidar.points,
            },
            "odom": {
                "position": np.array(
                    [
                        [pose.position.x, pose.position.y, pose.position.z]
                        for pose in self.odom
                    ]
                ),
                "orientation": np.array(
                    [
                        [
                            pose.orientation.x,
                            pose.orientation.y,
                            pose.orientation.z,
                            pose.orientation.w,
                        ]
                        for pose in self.odom
                    ]
                ),
            },
            "timestamp": self.timestamp,
        }


class StereoCaptureItem:
    left: np.ndarray
    right: np.ndarray

    disparity: Optional[np.ndarray]
    disparity_color: Optional[np.ndarray]

    exposure_left: Optional[float] = None
    exposure_right: Optional[float] = None

    def __init__(
        self,
        left: np.ndarray,
        right: np.ndarray,
        disparity: Optional[np.ndarray] = None,
        disparity_color: Optional[np.ndarray] = None,
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


class StereoMultiItem(StorageItem):
    rgb: StereoCaptureItem
    nir: StereoCaptureItem
    merged: Optional[np.ndarray]
    rgbd_rear: Optional[np.ndarray]
    rgbd_disp: Optional[np.ndarray]
    lidar: Optional[OusterLidarData]
    calibration: dict

    def __init__(
        self,
        timestamp: float,
        rgb: StereoCaptureItem,
        nir: StereoCaptureItem,
        merged: Optional[np.ndarray],
        calibration: dict,
        lidar: Optional[OusterLidarData] = None,
        rgbd_disp: Optional[np.ndarray] = None,
        rgbd_rear: Optional[np.ndarray] = None,
    ):
        self.rgb = rgb
        self.nir = nir
        self.merged = merged
        self.timestamp = timestamp
        self.lidar = lidar
        self.calibration = calibration
        self.rgbd_disp = rgbd_disp
        self.rgbd_rear = rgbd_rear
        super().__init__(timestamp)

    def __del__(self):
        del self.rgb
        del self.nir
        del self.merged
        del self.timestamp
        del self.lidar
        del self.calibration
        del self.rgbd_rear
        del self.rgbd_disp


class StereoStorageFrame:
    class FrameCapture:
        left_path: str
        right_path: str
        disparity_path: Optional[str] = None
        disparity_viz_path: Optional[str] = None

        def to_dict(self):
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

    def to_dict(self) -> Dict[str, Any]:
        return {
            "capture_rgb": self.capture_rgb.to_dict(),
            "capture_nir": self.capture_nir.to_dict(),
            "lidar_path": self.lidar_path,
            "lidar_ply_path": self.lidar_ply_path,
            "lidar_projected_path": self.lidar_projected_path,
            "frame_id": self.frame_id,
            "scene_id": self.scene_id,
        }


class StereoStorage:
    FOLDER = "tmp/depth"

    def __init__(self):
        self.storage_queue: List[StorageItem] = []
        self.item_store_time = 0.0

    def enqueue(self, item: StorageItem):
        self.storage_queue.append(item)

    def queue_loop(self):
        threads: List[threading.Thread] = []
        while True:
            if len(self.storage_queue) > 0:
                print("Storing item, queue length: ", len(self.storage_queue))
                item = self.storage_queue.pop(0)
                if isinstance(item, StereoMultiItem):
                    self.store_item(item.id, item)
                if isinstance(item, HDRCaptureItem):
                    self.store_hdr_item(item.id, item)
                # thread = threading.Thread(
                #     target=self.store_item, args=(item.id, item), daemon=False
                # )
                # thread.start()
            # time.sleep(0.01)

    def store_stereo_item(
        self, id: str, timestamp: str, channel: str, item: StereoCaptureItem
    ):
        root = f"{self.FOLDER}/{id}/{timestamp}"
        root = f"{root}/{channel}"
        os.makedirs(root, exist_ok=True)
        cv2.imwrite(f"{root}/left_distorted.png", item.left)
        cv2.imwrite(f"{root}/right_distorted.png", item.right)
        if item.disparity is not None and item.disparity_color is not None:
            cv2.imwrite(f"{root}/disparity.png", item.disparity)
            cv2.imwrite(f"{root}/disparity_color.png", item.disparity_color)

    def read_storage_list(self, root=FOLDER):
        scenes = [f for f in os.listdir(root) if os.path.isdir(os.path.join(root, f))]
        scenes.sort()
        return scenes

    def read_storage_scene(self, id: str, root=FOLDER):
        scene_folder = os.path.join(root, id)
        frame_list: List[str] = []
        if os.path.exists(os.path.join(scene_folder, "0.hdf5")):
            hdf5_files = [f for f in os.listdir(scene_folder) if f.endswith(".hdf5")]
            for hdf5_file in hdf5_files:
                frame_list += self.read_scene_h5_frame_list(
                    os.path.join(scene_folder, hdf5_file)
                )
        else:
            frame_list = [
                f for f in os.listdir(scene_folder) if f.split("_")[-1].isdigit()
            ]
        frame_list.sort()
        return frame_list

    def read_scene_h5_frame_list(self, h5_file: str):
        with h5py.File(h5_file, "r") as f:
            return list(f["frame"].keys())

    def get_scene_h5_file_for_store(self, scene_id: str, root=FOLDER):
        os.makedirs(os.path.join(root, scene_id), exist_ok=True)
        hdf5_files = [
            f for f in os.listdir(os.path.join(root, scene_id)) if f.endswith(".hdf5")
        ]
        if len(hdf5_files) == 0:
            return h5py.File(os.path.join(root, scene_id, "0.hdf5"), "w")
        hdf5_files.sort(key=lambda x: int(x.split("/")[-1].split(".")[0]))
        with h5py.File(os.path.join(root, scene_id, hdf5_files[-1]), "a") as f:
            if len(f["frame"].keys()) < 100:
                f.close()
                return h5py.File(os.path.join(root, scene_id, hdf5_files[-1]), "a")
            f.close()
        return h5py.File(os.path.join(root, scene_id, f"{len(hdf5_files)}.hdf5"), "w")

    def write_dict_to_h5(self, h5group: h5py.Group, data_dict):
        """
        재귀적으로 딕셔너리를 탐색하여 HDF5 그룹과 데이터셋을 생성합니다.

        Args:
            h5group: 현재 HDF5 그룹.
            data_dict: 저장할 데이터가 포함된 딕셔너리.
        """
        for key, value in data_dict.items():
            if value is None:
                continue
            if key == "attrs" and isinstance(value, dict):
                for attr_key, attr_value in value.items():
                    h5group.attrs[attr_key] = attr_value
            elif isinstance(value, dict):
                subgroup = h5group.create_group(key)
                self.write_dict_to_h5(subgroup, value)
            else:
                # Numpy 배열이나 다른 데이터 타입을 데이터셋으로 저장
                h5group.create_dataset(key, data=value)

    @deprecated(version="0.1.0", reason="Use write_dict_to_h5 instead")
    def store_item_to_h5(
        self,
        id: str,
        timestamp: float,
        calibration: dict,
        lidar_meta: dict,
        lidar_points: np.ndarray,
        lidar_reflectivity: Optional[np.ndarray] = None,
        rgbd_disp: Optional[np.ndarray] = None,
        disparity: Optional[Tuple[np.ndarray, np.ndarray]] = None,
        exposure_times: Optional[Tuple[float, float, float, float]] = None,
        lidar_imu_data: Optional[dict] = None,
        root=FOLDER,
    ):
        time_stamp = time.strftime("%H_%M_%S_", time.localtime(timestamp)) + str(
            int((timestamp % 1) * 1000)
        ).zfill(3)
        with self.get_scene_h5_file_for_store(id, root) as f:
            if not "calibration" in f:
                f.create_group("calibration")
                for k, v in calibration.items():
                    f["calibration"].attrs[k] = v
            try:
                frame = f.create_group(f"frame/{time_stamp}")
            except Exception as e:
                print(e)
                f.close()
                return
            frame.create_group("image")
            frame["image"].attrs["timestamp"] = timestamp
            frame["image"].attrs["rgb_left_path"] = os.path.join(
                time_stamp, "rgb", "left.png"
            )
            frame["image"].attrs["rgb_right_path"] = os.path.join(
                time_stamp, "rgb", "right.png"
            )
            frame["image"].attrs["nir_left_path"] = os.path.join(
                time_stamp, "nir", "left.png"
            )
            frame["image"].attrs["nir_right_path"] = os.path.join(
                time_stamp, "nir", "right.png"
            )
            if exposure_times is not None:
                frame["image"].attrs["rgb_exposure_left"] = exposure_times[0]
                frame["image"].attrs["rgb_exposure_right"] = exposure_times[1]
                frame["image"].attrs["nir_exposure_left"] = exposure_times[2]
                frame["image"].attrs["nir_exposure_right"] = exposure_times[3]

            if disparity is not None:
                frame["image"].attrs["rgb_disparity_path"] = os.path.join(
                    time_stamp, "rgb/disparity.png"
                )
                frame["image"].attrs["nir_disparity_path"] = os.path.join(
                    time_stamp, "nir/disparity.png"
                )
                disparity_group = frame.create_group("disparity")
                disparity_group.create_dataset("rgb", data=disparity[0])
                disparity_group.create_dataset("nir", data=disparity[1])

            frame.create_group("lidar")
            for k, v in lidar_meta.items():
                frame["lidar"].attrs[k] = v
            if lidar_imu_data is not None:
                imu = frame["lidar"].create_group("imu")
                for k, v in lidar_imu_data.items():
                    imu.attrs[k] = v
            if lidar_reflectivity is not None:
                frame.create_dataset("lidar/reflectivity", data=lidar_reflectivity)
            if rgbd_disp is not None:
                frame.create_dataset("rgbd_disp", data=rgbd_disp)
            frame.create_dataset("lidar/points", data=lidar_points)

            f.close()

    def store_hdr_item(self, id: str, item: HDRCaptureItem):
        time_stamp = time.strftime("%H_%M_%S_", time.localtime(item.timestamp)) + str(
            int((item.timestamp % 1) * 1000)
        ).zfill(3)
        with self.get_scene_h5_file_for_store(id, self.FOLDER) as f:
            frame = f.create_group(f"frame/{time_stamp}")
            self.write_dict_to_h5(frame, item.h5dict())
        print(item.hdr["left"]["rgb"].shape, item.hdr["left"]["rgb"].dtype)
        threads = [
            threading.Thread(
                target=cv2.imwrite,
                args=(
                    f"{self.FOLDER}/{id}/{time_stamp}/{side}/{src}_fusion.png",
                    item.hdr[side][src],
                ),
            )
            for side in item.hdr
            for src in item.hdr[side]
        ]

        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()

    def store_item(self, id: str, item: StereoMultiItem):
        time_begin = time.time()
        time_stamp = time.strftime("%H_%M_%S_", time.localtime(item.timestamp)) + str(
            int((item.timestamp % 1) * 1000)
        ).zfill(3)

        os.makedirs(f"{self.FOLDER}/{id}", exist_ok=True)

        threads = [
            threading.Thread(
                target=self.store_stereo_item,
                args=(id, time_stamp, "rgb", item.rgb),
            ),
            threading.Thread(
                target=self.store_stereo_item,
                args=(id, time_stamp, "nir", item.nir),
            ),
        ]
        if item.rgbd_rear is not None:
            t = threading.Thread(
                target=cv2.imwrite,
                args=(f"{self.FOLDER}/{id}/{time_stamp}/rgbd_rear.png", item.rgbd_rear),
            )
            threads.append(t)

        for thread in threads:
            thread.start()
        if item.lidar is not None:
            with self.get_scene_h5_file_for_store(id, self.FOLDER) as f:
                frame = f.create_group(f"frame/{time_stamp}")
                self.write_dict_to_h5(
                    frame,
                    {
                        "image": {
                            "attrs": {
                                "timestamp": item.timestamp,
                                "rgb_left_path": f"{time_stamp}/rgb/left.png",
                                "rgb_right_path": f"{time_stamp}/rgb/right.png",
                                "nir_left_path": f"{time_stamp}/nir/left.png",
                                "nir_right_path": f"{time_stamp}/nir/right.png",
                                "rgb_exposure_left": item.rgb.exposure_left,
                                "rgb_exposure_right": item.rgb.exposure_right,
                                "nir_exposure_left": item.nir.exposure_left,
                                "nir_exposure_right": item.nir.exposure_right,
                            },
                            "rgbd_disp": item.rgbd_disp,
                        },
                        "lidar": {
                            "attrs": item.lidar.meta_dict(),
                            "imu": {
                                "attrs": item.lidar.imu.dict(),
                            },
                            "points": item.lidar.points,
                            "reflectivity": item.lidar.reflectivity,
                        },
                    },
                )

            # self.store_item_to_h5(
            #     id,
            #     item.timestamp,
            #     item.calibration,
            #     item.lidar.meta_dict(),
            #     item.lidar.points,
            #     None,
            #     rgbd_disp=item.rgbd_disp,
            #     exposure_times=(
            #         item.rgb.exposure_left,
            #         item.rgb.exposure_right,
            #         item.nir.exposure_left,
            #         item.nir.exposure_right,
            #     ),  # type: ignore
            #     lidar_imu_data=item.lidar.imu.dict(),
            # )
        for thread in threads:
            thread.join()

        del item
        self.item_store_time = time.time() - time_begin

    def npz_to_h5(
        self,
        scene_id: str,
        root=FOLDER,
        progress_callback: Optional[Callable[[float], None]] = None,
    ):
        scene_root_path = os.path.join(root, scene_id)
        frames = [
            f
            for f in os.listdir(scene_root_path)
            if os.path.isdir(os.path.join(scene_root_path, f))
        ]
        frames.sort()

        for idx, frame in tqdm.tqdm(enumerate(frames)):
            if progress_callback is not None:
                progress_callback(idx / len(frames))
            frame_root = os.path.join(scene_root_path, frame)
            if not os.path.exists(os.path.join(frame_root, "post.npz")):
                continue

            with np.load(os.path.join(frame_root, "post.npz")) as post:
                calibration = {}
                for k in [
                    "mtx_left",
                    "dist_left",
                    "mtx_right",
                    "dist_right",
                    "R",
                    "T",
                    "E",
                    "F",
                ]:
                    calibration[k] = post[k]
                timestamp = int("".join(frame.split("_"))) / 1000.0
                lidar_meta = {
                    "beam_altitude_angles": post["beam_altitude_angles"],
                    "beam_azimuth_angles": post["beam_azimuth_angles"],
                    "imu_to_sensor_transform": post["imu_to_sensor_transform"],
                    "lidar_to_sensor_transform": post["lidar_to_sensor_transform"],
                    "lidar_origin_to_beam_origin_mm": post[
                        "lidar_origin_to_beam_origin_mm"
                    ],
                    "beam_to_lidar_transform": post["beam_to_lidar_transform"],
                }
                lidar_points = post["points"]
                lidar_reflectivity = post["reflectivity"]
                self.store_item_to_h5(
                    scene_id,
                    timestamp,
                    calibration,
                    lidar_meta,
                    lidar_points,
                    lidar_reflectivity,
                    disparity=(
                        post["disparity_rgb"],
                        post["disparity_nir"],
                    ),
                    root=root,
                )

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
        if file_type == ".png":
            image = Image.open(f"{root}/{property}.png")
            image.thumbnail((128, 128))
            img_bytes = BytesIO()
            image.save(img_bytes, format="PNG")
            return img_bytes.getvalue()
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
            if os.path.exists(f"{root}/{channel}/disparity.png"):
                capture.disparity_path = f"{root}/post.npz"
                capture.disparity_viz_path = f"{root}/{channel}/disparity.png"

        if os.path.exists(f"{root}/lidar.npz"):
            frame.lidar_path = f"{root}/lidar.npz"
        elif os.path.exists(os.path.join(root, scene_id, "0.hdf5")):
            frame.lidar_path = f"{root}/post.npz"
        if os.path.exists(f"{root}/lidar.ply"):
            frame.lidar_ply_path = f"{root}/lidar.ply"
        return frame

    # deprecated
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

    # deprecated
    def post_process_pointcloud(self, root, scene_id, frame_id):
        rgb_disparity_path = f"{root}/{scene_id}/{frame_id}/rgb/disparity.npz"
        if os.path.exists(f"{root}/{scene_id}/{frame_id}/post.npz"):
            post_dict = np.load(f"{root}/{scene_id}/{frame_id}/post.npz")
            lidar_points = post_dict["points"].reshape(-1, 3) * 1000
            calibration = post_dict
        else:
            points_path = f"{root}/{scene_id}/{frame_id}/lidar.npz"
            lidar_points = np.load(points_path)["points"].reshape(-1, 3) * 1000
            calibration = np.load(f"{root}/{scene_id}/calibration.npz")
        fx = calibration["mtx_left"][0, 0]
        fy = calibration["mtx_left"][1, 1]
        cx = calibration["mtx_left"][0, 2]
        cy = calibration["mtx_left"][1, 2]
        bl = np.linalg.norm(calibration["T"])
        disparity = np.load(rgb_disparity_path)["arr_0"]
        depth = self.disparity_to_depth(fx, bl, disparity)

        camera_points = self.depth_to_pointcloud(depth, fx, fy, cx, cy)

        lidar_pcd = o3d.geometry.PointCloud()
        lidar_pcd.points = o3d.utility.Vector3dVector(lidar_points)

        camera_pcd = o3d.geometry.PointCloud()
        camera_pcd.points = o3d.utility.Vector3dVector(camera_points)

        lidar_pcd_path = f"{root}/{scene_id}/{frame_id}/lidar.ply"
        camera_pcd_path = f"{root}/{scene_id}/{frame_id}/camera_rgb.ply"

        o3d.io.write_point_cloud(lidar_pcd_path, lidar_pcd)
        o3d.io.write_point_cloud(camera_pcd_path, camera_pcd)
