import time
from typing import List, Tuple

import cv2
import numpy as np

import os
import json
from calibration_type import CalibrationOutput


class CalibrationStorage:
    def __init__(self):
        self.ROOT_FOLDER = "tmp/calibration/"

    def get_colmap_parameter(self, mtx, dist):
        return [
            mtx[0, 0],
            mtx[1, 1],
            mtx[0, 2],
            mtx[1, 2],
            dist[0],
            dist[1],
            dist[2],
            dist[3],
        ]

    def save_calibration(
        self,
        id: int,
        output: CalibrationOutput,
        image_points_left,
        image_points_right,
        image_pairs,
        image_pairs_origin,
        chessboard_shape,
    ):
        folder = f"{self.ROOT_FOLDER}{id}/"
        os.makedirs(folder, exist_ok=True)
        meta_dict = {
            "id": id,
            "timestamp": time.time(),
            "month": time.strftime("%m"),
            "day": time.strftime("%d"),
            "hour": time.strftime("%H"),
            "image_count": len(image_points_left),
            "colmap_left": self.get_colmap_parameter(
                output.mtx_left, output.dist_left[0]
            ),
            "colmap_right": self.get_colmap_parameter(
                output.mtx_right, output.dist_right[0]
            ),
            "chessboard_shape": chessboard_shape,
        }
        np.savez(
            f"{folder}calibration.npz",
            ret=output.ret,
            mtx_left=output.mtx_left,
            dist_left=output.dist_left,
            mtx_right=output.mtx_right,
            dist_right=output.dist_right,
            R=output.R,
            T=output.T,
            E=output.E,
            F=output.F,
            left_rvecs=output.rvecs_left,
            left_tvecs=output.tvecs_left,
            right_rvecs=output.rvecs_right,
            right_tvecs=output.tvecs_right,
            image_points_left=image_points_left,
            image_points_right=image_points_right,
            shape=chessboard_shape,
        )
        for idx, (img_left, img_right) in enumerate(image_pairs):
            print(f"Saving {folder}{idx}.png")
            cv2.imwrite(f"{folder}left_{idx}.png", img_left)
            cv2.imwrite(f"{folder}right_{idx}.png", img_right)

        for idx, (img_left, img_right) in enumerate(image_pairs_origin):
            print(f"Saving {folder}{idx}.png")
            cv2.imwrite(f"{folder}left_origin_{idx}.png", img_left)
            cv2.imwrite(f"{folder}right_origin_{idx}.png", img_right)
        json.dump(meta_dict, open(f"{folder}meta.json", "w"))

    def list_calibrations(self):
        calibrations = []
        for folder in os.listdir(self.ROOT_FOLDER):
            if os.path.isdir(f"{self.ROOT_FOLDER}{folder}"):
                meta = json.load(open(f"{self.ROOT_FOLDER}{folder}/meta.json"))
                calibrations.append(meta)
        return calibrations

    def load_calibration(self, id: int):
        folder = f"{self.ROOT_FOLDER}{id}/"
        if not os.path.exists(folder):
            return None
        data = np.load(f"{folder}calibration.npz")
        output = CalibrationOutput(
            ret=data["ret"],
            mtx_left=data["mtx_left"],
            dist_left=data["dist_left"],
            mtx_right=data["mtx_right"],
            dist_right=data["dist_right"],
            R=data["R"],
            T=data["T"],
            E=data["E"],
            F=data["F"],
            rvecs_left=data["left_rvecs"],
            tvecs_left=data["left_tvecs"],
            rvect_right=data["right_rvecs"],
            tvect_right=data["right_tvecs"],
        )
        shape = data["shape"] if "shape" in data else (12, 6, 21.9)

        image_points_left = [x for x in data["image_points_left"]]
        image_points_right = [x for x in data["image_points_right"]]
        image_pairs: List[Tuple[np.ndarray, np.ndarray]] = []
        image_pairs_origin: List[Tuple[np.ndarray, np.ndarray]] = []
        for i in range(len(image_points_left)):
            img_left = cv2.imread(f"{folder}left_{i}.png")
            img_right = cv2.imread(f"{folder}right_{i}.png")
            img_pair = (img_left, img_right)
            image_pairs.append(img_pair)
            img_left = cv2.imread(f"{folder}left_origin_{i}.png")
            img_right = cv2.imread(f"{folder}right_origin_{i}.png")
            img_pair = (img_left, img_right)
            image_pairs_origin.append(img_pair)
        return (
            output,
            image_points_left,
            image_points_right,
            image_pairs,
            image_pairs_origin,
            shape,
        )

    def get_new_id(self):
        id = 0
        while os.path.exists(f"{self.ROOT_FOLDER}{id}/"):
            id += 1
        return id
