from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image, LaserScan, CompressedImage
from slam_pb2 import Pose3D
from typing import List, Optional, Union
from capture_pb2 import *
from google.protobuf import json_format
import numpy as np


class ImageBytes:
    width: int
    height: int
    # data: List[int]
    image: cv2.typing.MatLike
    topic: str

    def __init__(
        self,
        image: Union[Image, CompressedImage],
        topic: str = "/oakd/rgb/preview/image_raw",
        bayerInterpolation: bool = False,
    ):
        self.topic = topic
        if type(image) == CompressedImage:
            # print(f"converting {topic} to cv2")
            np_arr = np.frombuffer(image.data, np.uint8)

            if bayerInterpolation:
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BayerBG2BGR)
            else:
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.image = cv_image

        elif type(image) == Image:
            self.width = image.width
            self.height = image.height
            # self.data = image.data.tolist()
            self.image = CvBridge().imgmsg_to_cv2(image, desired_encoding="passthrough")

        # if topic == "/stereo/depth":
        #     self.image = cv2.normalize(
        #         self.image, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8UC1
        #     )
        #     self.image = cv2.convertScaleAbs(self.image)

    def to_dict(self):
        return {
            "width": self.width,
            "height": self.height,
            "topic": self.topic,
        }


class CaptureLiDAR:
    angle_min: float
    angle_max: float
    angle_increment: float
    ranges: list[float]
    range_max: float

    def __init__(self, lidar_msg: LaserScan):
        self.angle_min = lidar_msg.angle_min
        self.angle_max = lidar_msg.angle_max
        self.angle_increment = lidar_msg.angle_increment
        self.ranges = lidar_msg.ranges.tolist()
        self.range_max = max([x for x in self.ranges if x != float("inf")])
        self.ranges = [self.range_max if x == float("inf") else x for x in self.ranges]

    def to_dict(self):
        return {
            "angle_min": self.angle_min,
            "angle_max": self.angle_max,
            "angle_increment": self.angle_increment,
            "ranges": self.ranges,
            "range_max": self.range_max,
        }

    def toProto(self):
        dict = self.to_dict()
        proto = LidarPosition()
        json_format.ParseDict(dict, proto)
        return proto


class CaptureSingleScene:
    capture_id: int
    scene_id: int
    timestamp: int
    robot_pose: Optional[Pose3D]
    lidar_position: Optional[CaptureLiDAR]
    picture_list: List[ImageBytes]

    def __init__(
        self,
        capture_id: int,
        scene_id: int,
        timestamp: int,
        robot_pose: Optional[Pose3D] = None,
        lidar_position: Optional[CaptureLiDAR] = None,
        picture_list: List[ImageBytes] = [],
    ):
        self.capture_id = capture_id
        self.scene_id = scene_id
        self.timestamp = timestamp
        self.robot_pose = robot_pose
        self.lidar_position = lidar_position
        self.picture_list = picture_list

    def to_dict_light(self):
        images = [
            x.topic.replace("/", "_")
            for x in self.picture_list
            if type(x) == ImageBytes
        ]
        return CaptureAppScene(
            capture_id=self.capture_id,
            scene_id=self.scene_id,
            timestamp=self.timestamp,
            robot_pose=self.robot_pose,
            images=images,
            lidar_position=(
                self.lidar_position.toProto() if self.lidar_position else None
            ),
        )
