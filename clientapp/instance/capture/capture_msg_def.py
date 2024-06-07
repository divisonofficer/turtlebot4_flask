from sensor_msgs.msg import Image, LaserScan, CompressedImage
from geometry_msgs.msg import PoseWithCovarianceStamped
from typing import Any, TypeVar
from capture_pb2 import (
    CaptureMessageDef,
    CaptureMessageDefGroup,
    CaptureScenarioHyperparameter,
)
from google.protobuf import json_format

MsgType = TypeVar("MsgType")


class CaptureMessageDefinition:
    ANGLES = [30, 60, 270, 315]
    MultiChannel_Left = CaptureMessageDefGroup(
        name="MultiChannel_Left",
        messages=[
            CaptureMessageDef(
                topic="/jai_1600_left/channel_0",
                format="bayer_rg8",
                ros_msg_type=CaptureMessageDef.RosMsgType.CompressedImage,
                interpolation=5,
            ),
            CaptureMessageDef(
                topic="/jai_1600_left/channel_1",
                format="mono8",
                ros_msg_type=CaptureMessageDef.RosMsgType.CompressedImage,
                interpolation=5,
            ),
        ],
        enabled=False,
    )
    MultiChannel_Right = CaptureMessageDefGroup(
        name="MultiChannel_Right",
        messages=[
            CaptureMessageDef(
                topic="/jai_1600_right/channel_0",
                format="bayer_rg8",
                ros_msg_type=CaptureMessageDef.RosMsgType.CompressedImage,
                interpolation=5,
            ),
            CaptureMessageDef(
                topic="/jai_1600_right/channel_1",
                format="mono8",
                ros_msg_type=CaptureMessageDef.RosMsgType.CompressedImage,
                interpolation=5,
            ),
        ],
        enabled=False,
    )
    oakd = CaptureMessageDefGroup(
        name="oakd",
        messages=[
            # CaptureMessageDef(
            #     topic="/oakd/rgb/image_raw",
            #     format="rgb",
            #     ros_msg_type=CaptureMessageDef.RosMsgType.Image,
            # ),
            CaptureMessageDef(
                topic="/oakd/rgb/image_raw/compressed",
                format="rgb",
                ros_msg_type=CaptureMessageDef.RosMsgType.CompressedImage,
                delay=2.0,
            ),
            # CaptureMessageDef(
            #     topic="/oakd/stereo/image_raw/compressedDepth",
            #     format="depth",
            #     ros_msg_type=CaptureMessageDef.RosMsgType.CompressedImage,
            # ),
            CaptureMessageDef(
                topic="/oakd/left/image_raw/compressed",
                format="rgb",
                ros_msg_type=CaptureMessageDef.RosMsgType.CompressedImage,
                delay=2.0,
            ),
            CaptureMessageDef(
                topic="/oakd/right/image_raw/compressed",
                format="rgb",
                ros_msg_type=CaptureMessageDef.RosMsgType.CompressedImage,
                delay=2.0,
            ),
            # CaptureMessageDef(
            #     topic="/oakd/rgb/image_raw/compressedDepth",
            #     format="depth",
            #     ros_msg_type=CaptureMessageDef.RosMsgType.CompressedImage,
            # ),
        ],
        enabled=False,
    )

    slam = CaptureMessageDefGroup(
        name="slam",
        messages=[
            CaptureMessageDef(
                topic="/scan",
                format="lidar",
                ros_msg_type=CaptureMessageDef.RosMsgType.LaserScan,
            ),
            CaptureMessageDef(
                topic="/pose",
                format="Pose3D",
                ros_msg_type=CaptureMessageDef.RosMsgType.PoseWithCovarianceStamped,
            ),
        ],
        enabled=False,
    )

    topic_map: dict[str, CaptureMessageDef]

    def __init__(self):
        self.topic_map = {}
        for group in self.entries():
            for msg in group.messages:
                self.topic_map[msg.topic] = msg

    def entries(self):
        return [self.oakd, self.MultiChannel_Left, self.MultiChannel_Right, self.slam]

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
        if type == CaptureMessageDef.RosMsgType.CompressedImage:
            return CompressedImage

    def update_enable(self, group, status):
        self.__getattribute__(group).enabled = status

    def resolve_topic(self, topic: str):
        if topic in self.topic_map:
            return self.topic_map[topic]
        return None


class ScenarioHyperParameter:
    RotationQueueCount = CaptureScenarioHyperparameter.HyperParameter(
        name="RotationQueueCount",
        value=30,
        gap=1,
        range=[1, 50],
    )
    RotationSpeed = CaptureScenarioHyperparameter.HyperParameter(
        name="RotationSpeed",
        value=0.25,
        gap=0.025,
        range=[0.1, 0.5],
    )
    RotationInterval = CaptureScenarioHyperparameter.HyperParameter(
        name="RotationInterval",
        value=0.6,
        gap=0.05,
        range=[0.1, 1.0],
    )

    JaiInterpolationNumber = CaptureScenarioHyperparameter.HyperParameter(
        name="JaiInterpolationNumber",
        value=3,
        gap=1,
        range=[1, 10],
    )

    def __init__(self):
        for entry in self.entries():
            setattr(self, "_" + entry.name, entry.value)

    def entries(self):
        return [
            self.RotationQueueCount,
            self.RotationSpeed,
            self.RotationInterval,
            self.JaiInterpolationNumber,
        ]

    def update(self, name, value):
        entry: CaptureScenarioHyperparameter.HyperParameter = self.__getattribute__(
            name
        )
        if entry.range[0] <= value <= entry.range[1]:
            entry.value = value

    def to_msg(self):
        msg = CaptureScenarioHyperparameter()
        for entry in self.entries():
            msg.hyperparameters.append(entry)
        return msg
