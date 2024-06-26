from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image, LaserScan, CompressedImage
from slam_pb2 import Pose3D
from typing import List, Optional, Union
from capture_pb2 import *
from google.protobuf import json_format
import numpy as np
from cv2.typing import MatLike
from videostream import decode_jai_compressedImage


class ImageBytes:
    width: int
    height: int
    # data: List[int]
    image: MatLike
    topic: str

    def __init__(
        self,
        image: Union[Image, CompressedImage, MatLike, List[CompressedImage]],
        topic: str = "/oakd/rgb/preview/image_raw",
        bayerInterpolation: bool = False,
    ):
        self.topic = topic

        if type(image) == np.ndarray:
            self.width = image.shape[1]
            self.height = image.shape[0]
            self.image = image

        elif type(image) == CompressedImage:
            # print(f"converting {topic} to cv2")
            np_arr = np.frombuffer(image.data, np.uint8)

            if "bit" in image.header.frame_id:
                cv_image = decode_jai_compressedImage(image)
            else:
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

        elif type(image) == list:
            images = [decode_jai_compressedImage(x).astype(np.uint32) for x in image]

            # self.image = np.sum(images, axis=0) / len(images)
            self.image = np.median(images, axis=0)
            self.image = self.image.astype(np.uint16)
            self.width = self.image.shape[1]
            self.height = self.image.shape[0]

    def to_dict(self):
        return {
            "width": self.width,
            "height": self.height,
            "topic": self.topic,
        }

    def __del__(self):
        del self.image


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

    def __del__(self):
        for img in self.picture_list:
            del img
