from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import cv2

from videostream import VideoStream


stream = VideoStream()


def slam_map_opencv(
    map: OccupancyGrid,
    position: PoseWithCovarianceStamped,
    markers: list = None,
    lidar_position: list = None,
):

    # 임의의 데이터를 생성합니다. 실제 사용 시에는 ROS 메시지의 data를 사용해야 합니다.
    # 여기서 -1은 알 수 없는 공간, 0은 빈 공간, 1은 점유된 공간을 의미합니다.
    data = map.data

    # 알 수 없는 공간은 (127, 127, 127)로, 빈 공간은 (255, 255, 255)로, 점유된 공간은 (0, 0, 0)으로 변환합니다.
    data = [
        (127, 127, 127) if x == -1 else (255, 255, 255) if x == 0 else (0, 0, 0)
        for x in data
    ]

    # numpy 배열로 변환하고 이미지의 형태로 재조정합니다.
    image_data = np.array(data, dtype=np.uint8).reshape(
        (map.info.height, map.info.width, 3)
    )

    # OpenCV에서 사용할 수 있도록 데이터 타입을 변경합니다.
    image_data = image_data.astype(np.uint8)

    origin = map.info.origin.position
    if position is not None:
        robot_pose = position.pose.pose.position

        image_data = cv2.flip(image_data, 1)

        robot_x = int(map.info.width - (robot_pose.x - origin.x) / map.info.resolution)
        robot_y = int((robot_pose.y - origin.y) / map.info.resolution)

        cv2.circle(image_data, (robot_x, robot_y), 2, (128, 0, 0), -1)
        radius = int(map.info.width / 300)
        if lidar_position:
            for position in lidar_position:
                lidar_x = robot_x - int(position[0] / map.info.resolution)
                lidar_y = robot_y + int(position[1] / map.info.resolution)
                if radius < 1:
                    image_data[lidar_y, lidar_x] = (0, 0, 255)
                else:
                    cv2.circle(image_data, (lidar_x, lidar_y), radius, (0, 0, 255), -1)

    for marker in markers or []:
        pos = marker["pose"]
        marker_x = int(map.info.width - (pos["x"] - origin.x) / map.info.resolution)
        marker_y = int((pos["y"] - origin.y) / map.info.resolution)

        cv2.circle(image_data, (marker_x, marker_y), 2, (0, 128, 0), -1)

    stream.cv_ndarray_callback(image_data)
