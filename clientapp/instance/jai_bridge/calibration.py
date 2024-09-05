from typing import List, Optional, Tuple, Union

import cv2
import numpy as np
from calibration_type import CalibrationOutput
from calibration_storage import CalibrationStorage


class Calibration:
    def __init__(self):
        self.init_calibration_points()

        self.output = None
        self.reprojection_errors_mean = 0.0
        self.reprojection_errors: list[tuple[float, float]] = []
        self.storage = CalibrationStorage()

        self.calibration_id = self.storage.get_new_id()

    def init_calibration_points(self):
        self.CHESS_CELL_WIDTH = 21.9
        self.CHESS_SHAPE: Tuple[int, int] = (12, 6)
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.objpoints: list[np.ndarray] = []
        self.imgpoints_left: list[np.ndarray] = []
        self.imgpoints_right: list[np.ndarray] = []
        self.chessboard_images: list[tuple[np.ndarray, np.ndarray]] = []
        self.chessboard_image_origins: list[tuple[np.ndarray, np.ndarray]] = []
        self.update_objp()

    def update_objp(self):
        self.objp = np.zeros((self.CHESS_SHAPE[0] * self.CHESS_SHAPE[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[
            0 : self.CHESS_SHAPE[0], 0 : self.CHESS_SHAPE[1]
        ].T.reshape(-1, 2)
        self.objp *= self.CHESS_CELL_WIDTH

    def append_calibration_points(
        self,
        im_left: np.ndarray,
        im_left_corner: np.ndarray,
        corners_left: np.ndarray,
        im_right: np.ndarray,
        im_right_corner: np.ndarray,
        corners_right: np.ndarray,
    ):
        self.objpoints = [self.objp for _ in range(len(self.objpoints) + 1)]
        self.imgpoints_left.append(corners_left)
        self.imgpoints_right.append(corners_right)
        self.chessboard_images.append((im_left_corner, im_right_corner))
        self.chessboard_image_origins.append((im_left, im_right))
        if len(self.objpoints) > 50:
            max_error, max_error_idx = 0.0, -1
            for i in range(len(self.reprojection_errors)):
                error_left, error_right = self.reprojection_errors[i]
                if error_left > max_error:
                    max_error = error_left
                    max_error_idx = i
                if error_right > max_error:
                    max_error = error_right
                    max_error_idx = i
            self.delete_chessboard_array(max_error_idx)

    def compute_reprojection_error(self, rvecs, tvecs, mtx, dist, imgpoints):
        imgpoints2, _ = cv2.projectPoints(self.objp, rvecs, tvecs, mtx, dist)
        error = cv2.norm(imgpoints, imgpoints2, cv2.NORM_L2)
        mse = (error**2) / len(imgpoints)
        rsme = np.sqrt(mse)
        return rsme

    def compute_reprojection_errors(self):
        mean_error = 0.0
        error_list: list[tuple[float, float]] = []

        if not self.output:
            return []

        for i in range(len(self.imgpoints_left)):
            error_left = self.compute_reprojection_error(
                self.output.rvecs_left[i],
                self.output.tvecs_left[i],
                self.output.mtx_left,
                self.output.dist_left,
                self.imgpoints_left[i],
            )
            error_right = self.compute_reprojection_error(
                self.output.rvecs_right[i],
                self.output.tvecs_right[i],
                self.output.mtx_right,
                self.output.dist_right,
                self.imgpoints_right[i],
            )
            error_list.append((error_left, error_right))

        return mean_error / len(self.imgpoints_left), error_list

    def delete_chessboard_array(self, idx: int):
        if len(self.chessboard_images) > idx:
            self.chessboard_images.pop(idx)
            self.objpoints.pop(idx)
            self.imgpoints_left.pop(idx)
            self.imgpoints_right.pop(idx)
            self.chessboard_image_origins.pop(idx)

    def delete_chessboard_image(self, idx: int):
        self.delete_chessboard_array(idx)
        if len(self.chessboard_images) > 0:
            return self.calibrate_camera(None, None)
        return None

    def extract_chessboard_corners(self, im_left: np.ndarray, im_right: np.ndarray):
        gray_left = cv2.cvtColor(im_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(im_right, cv2.COLOR_BGR2GRAY)
        rect_left, corners_left = cv2.findChessboardCorners(gray_left, self.CHESS_SHAPE)
        rect_right, corners_right = cv2.findChessboardCorners(
            gray_right, self.CHESS_SHAPE
        )

        if rect_left and rect_right:
            corners_left = cv2.cornerSubPix(
                gray_left, corners_left, self.CHESS_SHAPE, (-1, -1), self.criteria
            )
            corners_right = cv2.cornerSubPix(
                gray_right, corners_right, self.CHESS_SHAPE, (-1, -1), self.criteria
            )
            im_left_corner = im_left.copy()
            im_right_corner = im_right.copy()
            im_left_corner = cv2.drawChessboardCorners(
                im_left_corner, self.CHESS_SHAPE, corners_left, rect_left
            )
            im_right_corner = cv2.drawChessboardCorners(
                im_right_corner, self.CHESS_SHAPE, corners_right, rect_right
            )
            self.append_calibration_points(
                im_left,
                im_left_corner,
                corners_left,
                im_right,
                im_right_corner,
                corners_right,
            )
            return True
        return False

    def calibrate_camera(
        self,
        im_left: Optional[Union[np.ndarray, List[np.ndarray]]],
        im_right: Optional[Union[np.ndarray, List[np.ndarray]]],
    ):

        if im_left is not None and im_right is not None:
            if not isinstance(im_left, list):
                im_left = [im_left]
            if not isinstance(im_right, list):
                im_right = [im_right]
            for left, right in zip(im_left, im_right):
                self.extract_chessboard_corners(left, right)
        if len(self.chessboard_images) < 1:
            return None, None, None
        image_shape = self.chessboard_images[0][0].shape[:2]
        image_shape = (image_shape[1], image_shape[0])
        ret_left, mtx_left, dist_left, rvecs_left, tvecs_left = cv2.calibrateCamera(
            self.objpoints, self.imgpoints_left, image_shape, None, None  # type: ignore
        )  # type: ignore
        (
            ret_right,
            mtx_right,
            dist_right,
            rvecs_right,
            tvecs_right,
        ) = cv2.calibrateCamera(
            self.objpoints, self.imgpoints_right, image_shape, None, None  # type: ignore
        )
        (
            ret,
            mtx_left,
            dist_left,
            mtx_right,
            dist_right,
            R,
            T,
            E,
            F,
        ) = cv2.stereoCalibrate(
            self.objpoints,
            self.imgpoints_left,
            self.imgpoints_right,
            mtx_left,
            dist_left,
            mtx_right,
            dist_right,
            image_shape,
        )
        lidar_RT = self.output.Lidar_RT if self.output else np.eye(4)
        self.output = CalibrationOutput(
            ret,
            mtx_left,
            dist_left,
            rvecs_left,
            tvecs_left,
            mtx_right,
            dist_right,
            rvecs_right,
            tvecs_right,
            R,
            T,
            E,
            F,
            lidar_RT,
            image_shape,
        )
        self.reprojection_errors_mean, self.reprojection_errors = (
            self.compute_reprojection_errors()
        )
        # self.reprojection_errors = [
        #     (ret_left, ret_right) for x in range(len(self.objpoints))
        # ]

        return self.output, self.chessboard_images[-1][0], self.chessboard_images[-1][1]

    def depth(self, image_left, image_right):
        if not self.output:
            return None
        gray_left = cv2.cvtColor(image_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(image_right, cv2.COLOR_BGR2GRAY)
        rectify_left, rectify_right, proj_left, proj_right, Q, roi_left, roi_right = (
            cv2.stereoRectify(
                self.output.mtx_left,
                self.output.dist_left,
                self.output.mtx_right,
                self.output.dist_right,
                gray_left.shape[::-1],
                self.output.R,
                self.output.T,
            )
        )
        map_left_x, map_left_y = cv2.initUndistortRectifyMap(
            self.output.mtx_left,
            self.output.dist_left,
            rectify_left,
            proj_left,
            gray_left.shape[::-1],
            cv2.CV_32FC1,
        )
        map_right_x, map_right_y = cv2.initUndistortRectifyMap(
            self.output.mtx_right,
            self.output.dist_right,
            rectify_right,
            proj_right,
            gray_right.shape[::-1],
            cv2.CV_32FC1,
        )
        left_rectified = cv2.remap(gray_left, map_left_x, map_left_y, cv2.INTER_LINEAR)
        right_rectified = cv2.remap(
            gray_right, map_right_x, map_right_y, cv2.INTER_LINEAR
        )
        stereo = cv2.StereoBM.create(numDisparities=16, blockSize=15)
        disparity = stereo.compute(left_rectified, right_rectified)
        focal_length = proj_left[0, 0]
        baseline = np.linalg.norm(self.output.T)
        disparity[disparity <= 0] = 1e-6
        depth = focal_length * baseline / disparity
        return depth

    def save_calibration(self):
        if self.output is None:
            return -1
        self.storage.save_calibration(
            self.calibration_id,
            self.output,
            self.imgpoints_left,
            self.imgpoints_right,
            self.chessboard_images,
            self.chessboard_image_origins,
            (self.CHESS_SHAPE[0], self.CHESS_SHAPE[1], self.CHESS_CELL_WIDTH),
        )
        return self.calibration_id

    def load_calibration(self, id: int):
        output = self.storage.load_calibration(id)
        if output is None:
            return False
        (self.output, _, _, chessboard_images, chessboard_image_origins, shape) = output
        self.chessboard_images = []
        self.objpoints = []
        self.imgpoints_left = []
        self.imgpoints_right = []
        self.calibration_id = id
        self.CHESS_CELL_WIDTH = shape[2]
        self.CHESS_SHAPE = (int(shape[0]), int(shape[1]))
        self.chessboard_image_origins = []
        self.update_objp()
        im_lefts = [x[0] for x in chessboard_image_origins]
        im_rights = [x[1] for x in chessboard_image_origins]
        self.calibrate_camera(im_lefts, im_rights)

        self.reprojection_errors_mean, self.reprojection_errors = (
            self.compute_reprojection_errors()
        )

        return True
