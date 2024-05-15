from time import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from typing import List, Optional, Any
from capture_msg_def import CaptureMessageDefinition
from capture_pb2 import CaptureMessageDefGroup


class CaptureMessage:
    lidar_msg: Optional[LaserScan]
    pose_msg: Optional[PoseWithCovarianceStamped]
    messages_received: bool
    messages_received_second: bool
    timestamp: float
    timestamp_second: float
    image_topics: List[str]
    use_second_timestamp: dict[str, float] = {}

    msg_dict: dict[str, Any] = {}

    def __init__(self, definition: CaptureMessageDefinition):
        self.msg_dict = {}
        self.lidar_msg = None
        self.pose_msg = None
        self.messages_received = False
        self.messages_received_second = False
        self.timestamp = time()
        self.timestamp_second = time()
        self.use_second_timestamp = {}
        self.definition = definition

    def get_message(self, group: CaptureMessageDefGroup):
        return [
            self.msg_dict[x.topic] if x.topic in self.msg_dict else None
            for x in group.messages
        ]

    def check_messages_received(self):

        keys = [
            [
                msg.topic
                for group in self.definition.entries()
                if group.enabled
                for msg in group.messages
                if msg.topic not in self.use_second_timestamp.keys()
            ],
            self.use_second_timestamp.keys(),
        ]
        msg_flag = [True, len(keys[1]) > 0]
        for i in range(2):
            for k in keys[i]:
                if k in self.msg_dict and self.msg_dict[k]:
                    continue
                msg_flag[i] = False
                break
        self.messages_received, self.messages_received_second = msg_flag
        # if self.messages_received:
        #     print(keys[0], self.msg_dict.keys())
        # if self.messages_received_second:
        #     print(keys[1], self.msg_dict.keys())

    #     if self.lidar_msg is not None and self.pose_msg is not None:
    #         if len(self.image_msg_dict.values()) == len(self.image_topics):
    #             self.messages_received = True
    #             for msg in [self.lidar_msg, self.pose_msg] + list(
    #                 self.image_msg_dict.values()
    #             ):
    #                 if (
    #                     msg.header.stamp.sec + msg.header.stamp.nanosec / 1000000000
    #                     < self.timestamp
    #                 ):
    #                     self.messages_received = False
    #                     break

    def update_msg(self, topic: str, msg):

        # if "oakd" in topic and time():
        #     print(topic, " behind :", time() - msg.header.stamp.sec)
        #     # ,
        #     #     msg.header.stamp,
        #     #     "entire_begin:",
        #     #     self.timestamp,
        #     #     "sub_begin: ",
        #     #     self.timestamp_second,
        #     #     "now: ",
        #     #     time(),
        #     # )
        if topic in self.msg_dict:
            return

        if msg.header.stamp.sec + msg.header.stamp.nanosec / 1000000000 >= (
            self.timestamp_second
            if topic in self.use_second_timestamp
            else self.timestamp
        ):
            self.msg_dict[topic] = msg
            self.check_messages_received()

    # def get_msg_received(self):
    #     if self.pose_msg is None:
    #         raise NoPoseSignal("No pose signal received")

    #     pose = self.pose_msg.pose.pose
    #     pose = RosProtoConverter().rosPoseToProtoPose3D(pose)
    #     # get lidar information

    #     if self.lidar_msg is None:
    #         raise NoLidarSignal("No lidar signal received")

    #     lidar = CaptureLiDAR(self.lidar_msg)

    #     # get image msgs

    #     if len(self.image_msg_dict.values()) < len(self.image_topics):
    #         raise NoImageSignal(
    #             f"{len(self.image_msg_dict.values())} images received out of {len(self.image_topics)}"
    #         )
    #     image_list = [
    #         ImageBytes(msg, topic, bayerInterpolation="channel_0" in topic)
    #         for topic, msg in self.image_msg_dict.items()
    #     ]
    #     return pose, lidar, image_list

    def vacate_second_msg_dict(self, group: CaptureMessageDefGroup):
        self.messages_received_second = False
        self.use_second_timestamp = {}
        for msg in group.messages:
            if msg.topic in self.msg_dict:
                del self.msg_dict[msg.topic]
            self.use_second_timestamp[msg.topic] = True
        self.timestamp_second = time()
