from typing import Callable, List

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points
import numpy as np
import cv2
import cv2


class Point2Depth:
    def __init__(self):

        pass

    image_callback: Callable[[np.ndarray], None]

    def set_intrinsics(self, K: np.ndarray):
        self.K = K

    def set_transform(self, T_lc: np.ndarray):
        self.T_lc = T_lc

    def set_image_resolution(self, image_width: int, image_height: int):
        self.image_width = image_width
        self.image_height = image_height

    def transform_to_camera_frame(self, points):
        points_homogeneous = np.hstack((points, np.ones((points.shape[0], 1))))
        points_camera_frame = self.T_lc @ points_homogeneous.T
        return points_camera_frame[:3, :].T

    def project_to_image_plane(self, points_camera):
        # print(points_camera)
        point_camera_homogeneous = points_camera.T

        point_camera_homogeneous = (
            point_camera_homogeneous / point_camera_homogeneous[2]
        )

        x = point_camera_homogeneous[0] * self.K[0, 0] + self.K[0, 2]
        y = point_camera_homogeneous[1] * self.K[1, 1] + self.K[1, 2]

        points_2d = np.vstack((x, y)).T

        depth_values = points_camera[:, 2]  # z 좌표 (깊이 값)

        return points_2d, depth_values

    def create_depth_image(self, points_2d, depth_values):
        depth_image = np.full((self.image_height // 8, self.image_width // 8), np.inf)

        u = np.round(points_2d[:, 0]).astype(int)
        v = np.round(points_2d[:, 1]).astype(int)

        valid_indices = (
            (u >= 0) & (u < self.image_width) & (v >= 0) & (v < self.image_height)
        )

        v = v // 8
        u = u // 8

        for i in np.where(valid_indices)[0]:
            if depth_image[v[i], u[i]] > depth_values[i]:
                depth_image[v[i], u[i]] = depth_values[i]

        depth_mask = depth_image == np.inf
        depth_image[depth_mask] = 0
        return depth_image, depth_mask

    def pointcloud2depthImage(self, msg: PointCloud2):
        # PointCloud2 메시지로부터 포인트 클라우드를 추출
        points_2d, depth_values = self.pointcloud2depth(msg)

        # 카메라 평면의 뎁스 이미지 생성
        depth_image, depth_mask = self.create_depth_image(points_2d, depth_values)

        self.image_callback(self.depth_image_visualization(depth_image, depth_mask))

    def pointcloud2depth(self, msg: PointCloud2):
        points: List[List[float]] = []
        for point in read_points(msg, field_names=["x", "y", "z"], skip_nans=True):
            x, y, z = point

            points.append([x, -z, y])

        points = np.array(points) * 1000.0  # type: ignore

        # 필터링된 포인트 클라우드 추출
        filtered_points = points[~np.isnan(points).any(axis=1)]
        # 포인트 클라우드를 카메라 좌표계로 변환
        points_camera_frame = self.transform_to_camera_frame(filtered_points)

        # 카메라 이미지 평면으로 투영하고 뎁스 값 가져오기
        points_2d, depth_values = self.project_to_image_plane(points_camera_frame)
        return points_2d, depth_values

    def depth_image_visualization(self, depth_image, depth_mask):

        depth_image = depth_image / 25.0
        depth_image[depth_image < 0] = 0

        depth_image = depth_image.astype(np.uint8)
        # depth_image = cv2.inpaint(
        #     depth_image,
        #     depth_mask.astype(np.uint8),
        #     7,
        #     cv2.INPAINT_TELEA,
        #     dst=depth_image,
        # )

        # print(depth_image.min(), depth_image.max(), depth_image.mean())
        # print(depth_image)

        depth_image = cv2.applyColorMap(depth_image, cv2.COLORMAP_MAGMA)

        return depth_image
