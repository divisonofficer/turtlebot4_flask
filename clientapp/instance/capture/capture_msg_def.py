from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from typing import Any, TypeVar
from capture_pb2 import CaptureMessageDef, CaptureMessageDefGroup
from google.protobuf import json_format

MsgType = TypeVar("MsgType")


class CaptureMessageDefinition:

    hyperSpectral1 = CaptureMessageDefGroup(
        name="hyperSpectral1",
        messages=[
            CaptureMessageDef(
                topic="/jai_1600/channel_0",
                format="bayer_rg8",
                ros_msg_type=CaptureMessageDef.RosMsgType.Image,
            ),
            CaptureMessageDef(
                topic="/jai_1600/channel_1",
                format="mono8",
                ros_msg_type=CaptureMessageDef.RosMsgType.Image,
            ),
        ],
        enabled=False,
    )
    oakd = CaptureMessageDefGroup(
        name="oakd",
        messages=[
            CaptureMessageDef(
                topic="/oakd/rgb/preview/image_raw",
                format="rgb",
                ros_msg_type=CaptureMessageDef.RosMsgType.Image,
            ),
        ],
        enabled=False,
    )

    slam = CaptureMessageDefGroup(
        name="slam",
        messages=[
            CaptureMessageDef(
                topic="/scan",
                format="lidar",
                ros_msg_type=CaptureMessageDef.RosMsgType.Image,
            ),
            CaptureMessageDef(
                topic="/pose",
                format="Pose3D",
                ros_msg_type=CaptureMessageDef.RosMsgType.PoseWithCovarianceStamped,
            ),
        ],
        enabled=False,
    )

    def __init__(self):
        pass

    def entries(self):
        return [self.oakd, self.hyperSpectral1, self.slam]

    def to_dict(self):
        def_dict = {}
        for group in self.entries():
            def_dict[group.name] = json_format.MessageToDict(group)
        return def_dict

    def resolve_ros_type(self, type: CaptureMessageDef.RosMsgType.ValueType):
        if type == CaptureMessageDef.RosMsgType.Image:
            return Image
        if type == CaptureMessageDef.RosMsgType.LaserScan:
            return LaserScan
        if type == CaptureMessageDef.RosMsgType.PoseWithCovarianceStamped:
            return PoseWithCovarianceStamped

    def update_enable(self, group, status):
        self.__getattribute__(group).enabled = status
