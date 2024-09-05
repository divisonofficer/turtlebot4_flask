import numpy as np


class CalibrationOutput:
    mtx_left_stereo: np.ndarray
    dist_left_stereo: np.ndarray
    mtx_right_stereo: np.ndarray
    dist_right_stereo: np.ndarray

    def __init__(
        self,
        ret: float,
        mtx_left: np.ndarray,
        dist_left: np.ndarray,
        rvecs_left: np.ndarray,
        tvecs_left: np.ndarray,
        mtx_right: np.ndarray,
        dist_right: np.ndarray,
        rvect_right: np.ndarray,
        tvect_right: np.ndarray,
        R: np.ndarray,
        T: np.ndarray,
        E: np.ndarray,
        F: np.ndarray,
        Lidar_RT: np.ndarray,
        image_size: tuple[int, int],
    ):
        self.ret = ret
        self.mtx_left = mtx_left
        self.dist_left = dist_left
        self.mtx_right = mtx_right
        self.dist_right = dist_right
        self.R = R
        self.T = T
        self.E = E
        self.F = F
        self.rvecs_left = rvecs_left
        self.tvecs_left = tvecs_left
        self.rvecs_right = rvect_right
        self.tvecs_right = tvect_right
        self.Lidar_RT = Lidar_RT
        self.image_size = image_size
