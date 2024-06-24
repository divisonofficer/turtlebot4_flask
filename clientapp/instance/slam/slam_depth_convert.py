import random
import numpy as np
from sensor_msgs.msg import LaserScan


def depth_to_laser_scan(camera_info, depth_image):
    height = camera_info.height
    width = camera_info.width
    K = np.array(camera_info.k).reshape(3, 3)
    fov = 2 * np.arctan(width / (2 * K[0, 0]))
    angle_increment = fov / 300
    angle_min = -fov / 2
    angle_max = fov / 2
    num_angles = int(fov / angle_increment + 1)
    angles = np.linspace(angle_min, angle_max, num_angles)
    center_x = K[0, 2]
    center_y = K[1, 2]
    f_x = K[0, 0]
    f_y = K[1, 1]
    P = np.array(camera_info.p).reshape(3, 4)
    R = P[:, :3]
    T = P[:, 3]
    P_inv = np.linalg.pinv(P)
    ranges = [float("inf")] * num_angles
    # 이미지의 각도 범위 내 각도에 해당하는 거리를 계산
    points: list[np.ndarray] = []
    for x in range(width):
        for y in range(100, height - 100, 10):
            dx = x - center_x
            dy = y - center_y
            distance = depth_image[y, x] / 1000.0

            if distance <= 0.0:
                continue

            Z = depth_image[y, x]
            X = (x - center_x) * Z / f_x
            Y = (y - center_y) * Z / f_y
            point_3d_homogenous = np.array([X, Y, Z])
            point_3d = R @ point_3d_homogenous
            # point_3d += T
            point_3d /= 1000000.0
            point_3d[2] *= 1000

            if point_3d[2] <= 0.5:
                continue
            if point_3d[2] > 2.0:
                continue

            if random.random() < 0.1:
                points.append(point_3d)

            angle = -np.arctan2(dx, f_x)
            index = int((angle - angle_min) / angle_increment)
            if 0 <= index < num_angles:
                if distance < ranges[index]:
                    ranges[index] = distance
    print(len(points))
    points_np = np.array(points)
    scan = LaserScan()
    scan.header.stamp = camera_info.header.stamp
    scan.header.frame_id = "base_link"
    scan.angle_min = angle_min
    scan.angle_max = angle_max
    scan.angle_increment = angle_increment
    scan.range_min = 0.0
    scan.range_max = float("inf")
    scan.ranges = ranges
    return scan, points_np
