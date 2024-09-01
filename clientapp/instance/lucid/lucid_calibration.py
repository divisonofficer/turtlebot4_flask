import os
from typing import List, Literal
import matplotlib.pyplot as plt

import numpy as np
import cv2
import torch
import tqdm
from lucid_postprocess import LucidPostProcess

import sys

sys.path.append("../jai_bridge/modules/RAFT_Stereo")
from core.raft_stereo import RAFTStereo
import open3d as o3d


class LucidCalibration:

    def __init__(self, args):
        self.args = args
        self.sift = cv2.SIFT.create()
        self.bf = cv2.BFMatcher()
        self.postprocess = LucidPostProcess()
        self.image_shape = (0, 0)

    def init_calibration_input(self, image_shape):
        cx = image_shape[0] / 2
        cy = image_shape[1] / 2
        fx = 500
        fy = 500

        self.k_left = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        self.k_right = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
        self.d_left = np.zeros(5)
        self.d_right = np.zeros(5)

    def extract_feature_pair(self, left: np.ndarray, right: np.ndarray):
        key_l, des_l = self.sift.detectAndCompute(left, None)
        key_r, des_r = self.sift.detectAndCompute(right, None)
        matches = self.bf.match(des_l, des_r)
        pst_l = np.array([key_l[m.queryIdx].pt for m in matches]).astype(np.float32)
        pst_r = np.array([key_r[m.trainIdx].pt for m in matches]).astype(np.float32)
        return pst_l, pst_r

    def stereo_calibration(
        self,
        image_shape,
        objp: List[np.ndarray],
        pst_l: List[np.ndarray],
        pst_r: List[np.ndarray],
    ):
        if not hasattr(self, "k_left"):
            self.init_calibration_input(image_shape)
        k_left = self.k_left
        d_left = self.d_left
        k_right = self.k_right
        d_right = self.d_right

        retval, k_left, d_left, k_right, d_right, R, T, E, F = cv2.stereoCalibrate(
            objp,
            pst_l,
            pst_r,
            k_left,
            d_left,
            k_right,
            d_right,
            image_shape,
            flags=cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_ASPECT_RATIO,
        )
        return k_left, d_left, k_right, d_right, R, T

    def read_image_pair(self, folder: str):
        raw = np.load(folder + "/raw.npz")
        left = raw["left"]
        right = raw["right"]
        toned = self.postprocess.rawUint8ToTonemappedBgr(
            np.concatenate([left, right], axis=1)
        )
        left = toned[:, : left.shape[1], :]
        right = toned[:, left.shape[1] :, :]
        if self.image_shape == (0, 0):
            self.image_shape = (left.shape[1], left.shape[0])
        return left, right

    def calibrate(self, folder_list: List[str]):
        pst_l_list = []
        pst_r_list = []
        objp_list = []
        for folder in tqdm.tqdm(folder_list):
            try:
                left, right = self.read_image_pair(folder)
                pst_l, pst_r = self.extract_feature_pair(left, right)
                if pst_l.shape[0] < 10:
                    continue
                pst_l_list.append(pst_l)
                pst_r_list.append(pst_r)
                objp = np.zeros((pst_l.shape[0], 3), np.float32)
                objp[:, :2] = pst_l
                objp_list.append(objp)
            except Exception as e:
                print(e)
                continue

        k_left, d_left, k_right, d_right, R, T = self.stereo_calibration(
            self.image_shape, objp_list, pst_l_list, pst_r_list
        )
        self.k_left = k_left
        self.d_left = d_left
        self.k_right = k_right
        self.d_right = d_right

        return k_left, d_left, k_right, d_right, R, T

    def extract_folders(self, folder: str, depth=0):
        folders = os.listdir(folder)
        folders.sort()
        output: List[str] = []

        for f in folders:
            if os.path.isdir(os.path.join(folder, f)):

                if os.path.exists(os.path.join(folder, f, "raw.npz")):
                    output.append(os.path.join(folder, f))
                elif depth < 3:
                    output += self.extract_folders(os.path.join(folder, f), depth + 1)

        return output

    def calibrate_from_folder(self, folder: str):
        folders = self.extract_folders(folder)

        for i, index in enumerate(range(0, len(folders), self.args.max_frames)):

            if index + self.args.max_frames > len(folders):
                folder_batch = folders[index:]
            else:
                folder_batch = folders[index : index + self.args.max_frames]
            k_left, d_left, k_right, d_right, R, T = self.calibrate(folder_batch)
            print(f"Batch {i+1}/{len(folders)//self.args.max_frames}")
            print(f"K_left: {k_left}")
            print(f"D_left: {d_left}")
            print(f"K_right: {k_right}")
            print(f"D_right: {d_right}")
            if self.args.save_on_iteration:
                np.savez(
                    f"calibration_{i}.npz",
                    k_left=k_left,
                    d_left=d_left,
                    k_right=k_right,
                    d_right=d_right,
                    R=R,
                    T=T,
                )

        return self.k_left, self.d_left, self.k_right, self.d_right, R, T


class StereoDepth:
    def __init__(self):
        class Args:
            def __init__(self):
                self.model = (
                    "../jai_bridge/modules/RAFT_Stereo/models/raftstereo-realtime.pth"
                )
                self.n_downsample = 3
                self.n_gru_layers = 2
                self.slow_fast_gru = False
                self.valid_iters = 7
                self.mixed_precision = True
                self.hidden_dims = [128] * 3
                self.context_norm = "batch"
                self.corr_levels = 4
                self.corr_radius = 4
                self.shared_backbone = True
                self.corr_implementation = "reg_cuda"

        args = Args()
        model = torch.nn.DataParallel(RAFTStereo(args), device_ids=[0]).cuda()
        model.load_state_dict(torch.load(args.model))
        model.eval()
        self.raft_stereo = model.module

    def disparity_matching(self, left: np.ndarray, right: np.ndarray):
        left_tensor = (
            torch.from_numpy(left).permute(2, 0, 1).unsqueeze(0).float().cuda()
        )
        right_tensor = (
            torch.from_numpy(right).permute(2, 0, 1).unsqueeze(0).float().cuda()
        )
        with torch.no_grad():
            _, flow = self.raft_stereo(
                left_tensor, right_tensor, iters=20, test_mode=True
            )
        return -flow.cpu().numpy()[0, 0]


class LidarCalibration:
    def __init__(self, args):
        self.args = args
        self.stereo_depth = StereoDepth()
        calibration_path = args.calibration
        self.calibration = np.load(calibration_path)
        self.k_left = self.calibration["mtx_left"]
        self.dist_left = self.calibration["dist_left"]
        self.baseline = np.linalg.norm(self.calibration["T"][0])
        self.postprocess = LucidPostProcess()

        self.trans_init = np.array(
            [
                [1, 0, 0, -60],
                [0, 1, 0, -240],
                [0, 0, 1, 80],
                [0, 0, 0, 1],
            ]
        )

    def calibrate_frame_chessboard(self, folder: str):
        raw_np = np.load(folder + "/raw.npz")
        left = self.image_get_tonemapped(folder, "left")
        right = self.image_get_tonemapped(folder, "right")
        left_gray = cv2.cvtColor(left, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(left_gray, (3, 3), None)
        if not ret:
            return None
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_refined = cv2.cornerSubPix(
            left_gray, corners, (3, 3), (-1, -1), criteria
        )
        lidar_points = raw_np["points"].reshape(-1, 3)
        lidar_pcd = o3d.geometry.PointCloud()
        lidar_pcd.points = o3d.utility.Vector3dVector(lidar_points)
        plane_model, inliner = lidar_pcd.segment_plane(
            distance_threshold=0.1, ransac_n=3, num_iterations=1000
        )
        plane_points = np.array(lidar_points)[inliner]

        fig = plt.figure(figsize=(12, 6))
        ax1 = fig.add_subplot(121, projection="3d")
        ax1.scatter(
            plane_points[:, 0],
            plane_points[:, 1],
            plane_points[:, 2],
            c="b",
            marker="o",
        )
        ax1.set_title("Plane Points")
        ax1.set_xlabel("X")
        ax1.set_ylabel("Y")
        ax1.set_zlabel("Z")
        fig.savefig(folder + "/plane_points.png")

    def image_get_tonemapped(self, folder: str, side: Literal["left", "right"]):
        if not self.args.overlap and os.path.exists(folder + f"/{side}_tonemapped.png"):
            return cv2.imread(folder + f"/{side}_tonemapped.png")
        raw_np = np.load(folder + "/raw.npz")
        raw = np.concatenate((raw_np["left"], raw_np["right"]), axis=1)
        tonemapped = self.postprocess.rawUint8ToTonemappedBgr(raw)
        tonemapped = (
            tonemapped[:, : raw_np["left"].shape[1], :]
            if side == "left"
            else tonemapped[:, raw_np["left"].shape[1] :, :]
        )
        cv2.imwrite(folder + f"/{side}_tonemapped.png", tonemapped)
        return tonemapped

    def calibrate_frame(self, folder: str):
        print(folder)
        raw_np = np.load(folder + "/raw.npz")
        left = self.image_get_tonemapped(folder, "left")
        right = self.image_get_tonemapped(folder, "right")
        ranges = raw_np["ranges"]

        if not hasattr(self, "Q"):
            imageSize = (1440, 928)
            # imageSize = (left.shape[1], left.shape[0])
            R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
                self.k_left,
                self.dist_left,
                self.calibration["mtx_right"],
                self.calibration["dist_right"],
                imageSize,
                self.calibration["R"],
                self.calibration["T"],
                alpha=0,
            )
            self.Q = Q
            self.map1x, self.map1y = cv2.initUndistortRectifyMap(
                self.k_left, self.dist_left, R1, P1, imageSize, cv2.CV_32FC1
            )
            self.map2x, self.map2y = cv2.initUndistortRectifyMap(
                self.calibration["mtx_right"],
                self.calibration["dist_right"],
                R2,
                P2,
                imageSize,
                cv2.CV_32FC1,
            )
            self.remap_mask_left = cv2.remap(
                np.ones(left.shape[:2]), self.map1x, self.map1y, cv2.INTER_LINEAR
            )

        image_height, image_width = left.shape[:2]

        left = cv2.remap(left, self.map1x, self.map1y, cv2.INTER_LINEAR)
        right = cv2.remap(
            right,
            self.map2x,
            self.map2y,
            cv2.INTER_LINEAR,
        )
        left = left[:image_height, :image_width]
        right = right[:image_height, :image_width]
        disparity = self.stereo_depth.disparity_matching(left, right)
        depth = self.disparity_to_depth(disparity)
        remap_mask_left = self.remap_mask_left[:image_height, :image_width]
        disparity[remap_mask_left == 0] = 0
        depth[remap_mask_left == 0] = 0

        cv2.imwrite(folder + "/left_rectified.png", left)
        cv2.imwrite(folder + "/right_rectified.png", right)

        if self.args.save_depth:
            MAXDIS = 64.0
            MAXDEPTH = 24000
            disparity_color = np.clip(disparity, 0, MAXDIS)
            disparity_color = cv2.applyColorMap(
                (disparity_color / MAXDIS * 255).astype(np.uint8), cv2.COLORMAP_MAGMA
            )
            cv2.imwrite(folder + "/disparity.png", disparity_color)
            depth_color = np.clip(depth, 0, MAXDEPTH)
            depth_color = cv2.applyColorMap(
                (depth_color / MAXDEPTH * 255).astype(np.uint8), cv2.COLORMAP_MAGMA
            )
            cv2.imwrite(folder + "/depth.png", depth_color)

            updated_data = {
                "depth": depth,
                "disparity": disparity,
                "k_left": self.k_left,
                "d_left": self.dist_left,
                "k_right": self.calibration["mtx_right"],
                "d_right": self.calibration["dist_right"],
                "R": self.calibration["R"],
                "T": self.calibration["T"],
            }
            np.savez(folder + "/post.npz", **updated_data)
            return None

        # points_camera = cv2.reprojectImageTo3D(disparity, self.Q).reshape(-1, 3)
        points_camera = self.depth_to_points(depth)
        points_lidar = self.lidar_depth_project(ranges)

        points_lidar = points_lidar.reshape(-1, 3)
        if self.args.save_depth:
            self.plot(points_camera, points_lidar, folder + "/points_plot.png")

        transform = self.compute_matches(points_camera, points_lidar)
        return transform

    def disparity_to_depth(self, disparity: np.ndarray):
        depth = self.baseline * self.k_left[0, 0] / disparity
        depth[np.isnan(depth)] = 0
        depth[np.isinf(depth)] = 0
        return depth

    def depth_to_points(self, depth: np.ndarray):

        h, w = depth.shape
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        u = u.flatten()
        v = v.flatten()
        z = depth.flatten()
        x = (u - self.k_left[0, 2]) * z / self.k_left[0, 0]
        y = (v - self.k_left[1, 2]) * z / self.k_left[1, 1]
        points = np.stack([x, y, z], axis=-1)
        points = points[
            np.logical_and.reduce(np.abs(points) <= 50000, axis=1)
            & np.logical_not(np.isnan(points).any(axis=1))
            & np.logical_not(np.isinf(points).any(axis=1))
            & (points[:, 2] > 0)
        ]
        return points

    def calibrate(self, folder_list: List[str]):
        rvec_list = []
        tvec_list = []
        for folder in tqdm.tqdm(folder_list):
            # self.calibrate_frame_chessboard(folder)
            self.calibrate_frame(folder)
            # rvec = self.calibrate_frame(folder)
            # rvec_list.append(rvec)

        return rvec_list

    def compute_matches(self, points_camera, points_lidar):

        pcd_camera = o3d.geometry.PointCloud()
        pcd_camera.points = o3d.utility.Vector3dVector(points_camera)
        pcd_lidar = o3d.geometry.PointCloud()
        pcd_lidar.points = o3d.utility.Vector3dVector(points_lidar)

        threshold = 2000

        reg_p2p = o3d.pipelines.registration.registration_icp(
            pcd_lidar,
            pcd_camera,
            threshold,
            self.trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        )
        self.trans_init = reg_p2p.transformation

        return reg_p2p.transformation

    def plot(self, points_1, points_2, save_path="points_plot.png"):

        # 3D 플롯을 위한 설정
        fig = plt.figure(figsize=(12, 6))

        # 첫 번째 point cloud 시각화
        ax1 = fig.add_subplot(121, projection="3d")
        ax1.scatter(points_1[:, 0], points_1[:, 2], points_1[:, 1], c="b", marker="o")
        ax1.set_title("Point Cloud 1")
        ax1.set_xlabel("X")
        ax1.set_ylabel("Z")
        ax1.set_zlabel("Y")

        # 두 번째 point cloud 시각화
        ax2 = fig.add_subplot(122, projection="3d")
        ax2.scatter(points_2[:, 0], points_2[:, 1], points_2[:, 2], c="r", marker="o")
        ax2.set_title("Point Cloud 2")
        ax2.set_xlabel("X")
        ax2.set_ylabel("Y")
        ax2.set_zlabel("Z")

        plt.tight_layout()
        plt.savefig(save_path)

    def compute_stereo_depth(self, left, right):
        disparity = self.stereo_depth.disparity_matching(left, right)
        disparity[disparity < 0] = 0
        fx = self.k_left[0, 0]
        baseline = self.baseline
        depth = fx * baseline / disparity
        depth[depth < 0] = 0
        depth[np.isnan(depth)] = 0
        depth[np.isinf(depth)] = 0
        return depth

    def lidar_depth_project(self, ranges: np.ndarray):
        WIDTH = 1024
        HEIGHT = 128
        V_FOV = np.pi / 8
        V_FOV_MIN = -V_FOV / 2
        V_FOV_MAX = V_FOV / 2

        phi = np.linspace(V_FOV_MIN, V_FOV_MAX, HEIGHT)
        theta = np.linspace(0, 2 * np.pi, WIDTH, endpoint=False)

        x = ranges * np.cos(phi[:, np.newaxis]) * np.cos(theta)
        y = ranges * np.cos(phi[:, np.newaxis]) * np.sin(theta)
        z = ranges * np.sin(phi[:, np.newaxis])

        return np.stack([x, y, z], axis=-1)


if __name__ == "__main__":

    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=str, required=True)
    parser.add_argument("--stereo_calibration", action="store_true")
    parser.add_argument("--lidar_calibration", action="store_true")
    parser.add_argument("--calibration", type=str, default="calibration.npz")
    parser.add_argument("--max_frames", type=int, default=50)
    parser.add_argument("--output", type=str, default="calibration.npz")
    parser.add_argument("--save_on_iteration", action="store_true")
    parser.add_argument("--save_tonemap", action="store_true")
    parser.add_argument("--save_depth", action="store_true")
    parser.add_argument("--overlap", action="store_true")
    args = parser.parse_args()
    calibration = LucidCalibration(args)

    if args.stereo_calibration:
        k_left, d_left, k_right, d_right, R, T = calibration.calibrate_from_folder(
            args.input
        )
        print(f"K_left: {k_left}")
        print(f"D_left: {d_left}")
        print(f"K_right: {k_right}")
        print(f"D_right: {d_right}")
        print(f"R: {R}")
        print(f"T: {T}")

        np.savez(
            "calibration.npz",
            k_left=k_left,
            d_left=d_left,
            k_right=k_right,
            d_right=d_right,
            R=R,
            T=T,
        )
    if args.lidar_calibration:
        lidar_calibration = LidarCalibration(args)
        folders = calibration.extract_folders(args.input)
        R, T = lidar_calibration.calibrate(folders)
        print(f"R: {R}")
        print(f"T: {T}")
        np.savez("lidar_calibration.npz", R=R, T=T)
