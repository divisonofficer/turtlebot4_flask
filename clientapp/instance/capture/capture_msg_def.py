from sensor_msgs.msg import Image, LaserScan, CompressedImage
from geometry_msgs.msg import PoseWithCovarianceStamped
from typing import Any, Optional, TypeVar
from capture_pb2 import (
    CaptureMessageDef,
    CaptureMessageDefGroup,
    CaptureScenarioHyperparameter,
)
from google.protobuf import json_format

MsgType = TypeVar("MsgType")


class CaptureMessageDefinition:
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

    Ellipsis = CaptureMessageDefGroup(
        name="Ellipsis",
        enabled=False,
    )

    HDR = CaptureMessageDefGroup(
        name="HDR",
        enabled=False,
    )

    topic_map: dict[str, CaptureMessageDef]

    def __init__(self):
        self.topic_map = {}
        for group in self.entries():
            for msg in group.messages:
                self.topic_map[msg.topic] = msg

    def entries(self):
        return [
            self.oakd,
            self.MultiChannel_Left,
            self.MultiChannel_Right,
            self.slam,
            self.Ellipsis,
            self.HDR,
        ]

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


class CaptureModeEnum:
    Single = "Single"
    RotatingCapture = "Rotating Capture"
    DrivingCapture = "Driving Capture"
    DrivingSideScanning = "Driving Side Scanning"
    MemoryStressTest = "Memory Stress Test"

    def as_list(self):
        return [
            self.Single,
            self.RotatingCapture,
            self.DrivingCapture,
            self.DrivingSideScanning,
            self.MemoryStressTest,
        ]

    def __getitem__(self, item):
        return self.as_list()[item]


CaptureMode = CaptureModeEnum()


class ScenarioHyperParameter:
    RotationQueueCount = CaptureScenarioHyperparameter.HyperParameter(
        name="RotationQueueCount",
        value=30,
        gap=1,
        range=[1, 50],
        type=CaptureScenarioHyperparameter.ParameterType.DOUBLE,
    )
    RotationAngleMargin = CaptureScenarioHyperparameter.HyperParameter(
        name="RotationAngleMargin",
        value=5,
        gap=0.1,
        range=[0, 90],
        type=CaptureScenarioHyperparameter.ParameterType.DOUBLE,
    )

    DriveDistance = CaptureScenarioHyperparameter.HyperParameter(
        name="DriveDistance",
        value=1,
        gap=0.1,
        range=[0.1, 2],
        type=CaptureScenarioHyperparameter.ParameterType.DOUBLE,
    )

    DriveMargin = CaptureScenarioHyperparameter.HyperParameter(
        name="DriveMargin",
        value=0.1,
        gap=0.01,
        range=[0.01, 1],
        type=CaptureScenarioHyperparameter.ParameterType.DOUBLE,
    )

    SideAngleCount = CaptureScenarioHyperparameter.HyperParameter(
        name="SideAngleCount",
        value=5,
        gap=1,
        range=[1, 10],
        type=CaptureScenarioHyperparameter.ParameterType.DOUBLE,
    )

    SideAngleRange = CaptureScenarioHyperparameter.HyperParameter(
        name="SideAngleRange",
        value=45,
        gap=1,
        range=[5, 90],
        type=CaptureScenarioHyperparameter.ParameterType.DOUBLE,
    )

    JaiInterpolationNumber = CaptureScenarioHyperparameter.HyperParameter(
        name="JaiInterpolationNumber",
        value=3,
        gap=1,
        range=[1, 10],
        type=CaptureScenarioHyperparameter.ParameterType.DOUBLE,
    )

    JaiAutoExpose = CaptureScenarioHyperparameter.HyperParameter(
        name="JaiAutoExpose",
        value=1,
        gap=1,
        range=[0, 1],
        type=CaptureScenarioHyperparameter.ParameterType.BOOLEAN,
    )

    CaptureQueueMode = CaptureScenarioHyperparameter.HyperParameter(
        name="CaptureQueueMode",
        value=0,
        enum_values=CaptureMode.as_list(),
        type=CaptureScenarioHyperparameter.ParameterType.ENUM,
    )
    EllRotationDegree = CaptureScenarioHyperparameter.HyperParameter(
        name="EllRotationDegree",
        value_array=[30, 60, 270, 315],
        type=CaptureScenarioHyperparameter.ParameterType.DOUBLE_ARRAY,
    )

    HdrExposureTime = CaptureScenarioHyperparameter.HyperParameter(
        name="HdrExposureTime",
        value_array=[1000, 4000, 16000, 64000],
        type=CaptureScenarioHyperparameter.ParameterType.DOUBLE_ARRAY,
    )
    HdrExposureTimeNir = CaptureScenarioHyperparameter.HyperParameter(
        name="HdrExposureTimeNIR",
        value_array=[1000, 4000, 16000, 64000],
        type=CaptureScenarioHyperparameter.ParameterType.DOUBLE_ARRAY,
    )

    def __init__(self):
        for entry in self.entries():
            setattr(self, "_" + entry.name, entry.value)

    def entries(self):
        return [
            self.RotationQueueCount,
            self.RotationAngleMargin,
            self.JaiInterpolationNumber,
            self.JaiAutoExpose,
            self.CaptureQueueMode,
            self.EllRotationDegree,
            self.HdrExposureTime,
            self.HdrExposureTimeNir,
            self.DriveDistance,
            self.DriveMargin,
            self.SideAngleCount,
            self.SideAngleRange,
        ]

    def update(self, name, value=None, value_array: Optional[list[float]] = None):
        entry: CaptureScenarioHyperparameter.HyperParameter = self.__getattribute__(
            name
        )
        if value is not None:
            if entry.type == CaptureScenarioHyperparameter.ParameterType.DOUBLE:
                if entry.range[0] <= value <= entry.range[1]:
                    entry.value = value
            else:
                entry.value = value
        if value_array is not None:
            entry.value_array[:] = value_array

    def to_msg(self):
        msg = CaptureScenarioHyperparameter()
        for entry in self.entries():
            msg.hyperparameters.append(entry)
        return msg
