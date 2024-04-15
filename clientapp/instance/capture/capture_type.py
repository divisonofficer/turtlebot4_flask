from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from dataclasses import dataclass

from slam_types import Pose3D


class ImageBytes:
    width: int
    height: int
    data: bytes

    def __init__(self, image: Image, topic: str = "/oakd/rgb/preview/image_raw"):
        self.width = image.width
        self.height = image.height
        self.data = image.data.tolist()
        self.image = CvBridge().imgmsg_to_cv2(image, desired_encoding="passthrough")
        if topic == "/stereo/depth":
            self.image = cv2.normalize(
                self.image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1
            )
            self.image = cv2.convertScaleAbs(self.image)

    def to_dict(self):
        return {
            "width": self.width,
            "height": self.height,
        }


class CaptureLiDAR:
    angle_min: float
    angle_max: float
    angle_increment: float
    ranges: list[float]

    def __init__(self, angle_min, angle_max, angle_increment, ranges):
        self.angle_min = angle_min
        self.angle_max = angle_max
        self.angle_increment = angle_increment
        self.ranges = ranges

    def to_dict(self):
        return {
            "angle_min": self.angle_min,
            "angle_max": self.angle_max,
            "angle_increment": self.angle_increment,
            "ranges": self.ranges,
        }


class CaptureSingleScene:
    capture_id: int
    scene_id: int
    timestamp: int
    robot_pose: Pose3D
    lidar_position: CaptureLiDAR
    picture_oakd_mono: ImageBytes

    def __init__(
        self,
        capture_id,
        scene_id,
        timestamp,
        robot_pose,
        lidar_position,
        picture_oakd_mono,
    ):
        self.capture_id = capture_id
        self.scene_id = scene_id
        self.timestamp = timestamp
        self.robot_pose = robot_pose
        self.lidar_position = lidar_position
        self.picture_oakd_mono = picture_oakd_mono

    def to_dict(self):
        return {
            "capture_id": self.capture_id,
            "scene_id": self.scene_id,
            "timestamp": self.timestamp,
            "robot_pose": (
                self.robot_pose.to_dict()
                if type(self.robot_pose) == Pose3D
                else self.robot_pose
            ),
            "lidar_position": (
                self.lidar_position.to_dict()
                if type(self.lidar_position) == CaptureLiDAR
                else self.lidar_position
            ),
            "picture_oakd_mono": (
                self.picture_oakd_mono.to_dict()
                if type(self.picture_oakd_mono) == ImageBytes
                else self.picture_oakd_mono
            ),
        }
