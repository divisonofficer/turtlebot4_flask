from dataclasses import dataclass
from typing import Dict, Union, Any
import math
from geometry_msgs.msg import Point, Pose, Quaternion


@dataclass
class EulierAngle:
    def __init__(self, roll: float, pitch: float, yaw: float):
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def __str__(self) -> str:
        return f"roll: {self.roll}, pitch: {self.pitch}, yaw: {self.yaw}"

    def to_dict(self) -> Dict[str, float]:
        return {"roll": self.roll, "pitch": self.pitch, "yaw": self.yaw}

    def __dict__(self):
        return self.to_dict()


@dataclass
class QuaternionAngle:
    def __init__(self, w: float, x: float, y: float, z: float):
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    @classmethod
    def from_dict(cls, dict: Dict[str, float]) -> "QuaternionAngle":
        return cls(dict["w"], dict["x"], dict["y"], dict["z"])

    @classmethod
    def from_msg(cls, msg: Quaternion) -> "QuaternionAngle":
        return cls(msg.w, msg.x, msg.y, msg.z)

    def __str__(self) -> str:
        return f"w: {self.w}, x: {self.x}, y: {self.y}, z: {self.z}"

    def to_dict(self) -> Dict[str, float]:
        return {"w": self.w, "x": self.x, "y": self.y, "z": self.z}

    def to_euler(self) -> EulierAngle:
        q = self

        w, x, y, z = (
            q.w,
            q.x,
            q.y,
            q.z,
        )  # Fix: Assign values from q to variables w, x, y, z
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return EulierAngle(roll_x, pitch_y, yaw_z)

    def __dict__(self):
        return self.to_dict()


@dataclass
class Point3D:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self) -> str:
        return f"x: {self.x}, y: {self.y}, z: {self.z}"

    @classmethod
    def from_dict(cls, dict: Dict[str, float]) -> "Point3D":
        return cls(dict["x"], dict["y"], dict["z"])

    @classmethod
    def from_msg(cls, msg: Point) -> "Point3D":
        return cls(msg.x, msg.y, msg.z)

    def to_dict(self) -> Dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z}

    def __dict__(self):
        return self.to_dict()


@dataclass
class Pose3D:
    def __init__(
        self, position: Point3D, orientation: Union[EulierAngle, QuaternionAngle]
    ):
        self.position = position
        if isinstance(orientation, EulierAngle):
            self.orientation = orientation
            self.orientation_quaternion = None
        else:
            self.orientation_quaternion = orientation
            self.orientation = orientation.to_euler()

    @classmethod
    def from_msg(cls, msg: Pose) -> "Pose3D":
        return cls(
            Point3D.from_msg(msg.position), QuaternionAngle.from_msg(msg.orientation)
        )

    def to_dict(self) -> Dict[str, Union[Dict[str, float], Dict[str, float]]]:
        return {
            "position": self.position.to_dict(),
            "orientation": self.orientation.to_dict(),
        }

    def to_dict_point(
        self,
    ) -> Dict[
        str, Union[Dict[str, float], Dict[str, float], Dict[str, float], float, None]
    ]:
        return {
            "x": self.position.x,
            "y": self.position.y,
            "z": self.position.z,
            "orientation_quaternion": (
                self.orientation_quaternion.to_dict()
                if self.orientation_quaternion
                else None
            ),
            "orientation": self.orientation.to_dict(),
        }

    def __dict__(self):
        return self.to_dict()


@dataclass
class MapMarker:
    def __init__(
        self,
        id: int,
        position: Point3D,
        orientation: Union[EulierAngle, QuaternionAngle],
    ):
        self.id = id
        self.position = position
        if isinstance(orientation, EulierAngle):
            self.orientation = orientation
        else:
            self.orientation = orientation.to_euler()

    def to_dict(self) -> Dict[str, Union[int, Dict[str, float]]]:
        return {
            "id": self.id,
            "pose": self.position.to_dict(),
            "orientation": self.orientation.to_dict(),
        }

    def __dict__(self):
        return self.to_dict()


@dataclass
class MapInfo:
    """
    width, height, resolution
    """

    def __init__(self, width: int, height: int, resolution: float):
        self.width = width
        self.height = height
        self.resolution = resolution

    def to_dict(self) -> Dict[str, Union[int, float]]:
        return {
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
        }

    def __dict__(self):
        return self.to_dict()


def dictValueConversion(data: Dict[str, Any]) -> Dict[str, Any]:
    for key, value in data.items():
        if isinstance(value, dict):
            data[key] = dictValueConversion(value)
        elif isinstance(value, list):
            for i, v in enumerate(value):
                value[i] = dictValueConversion(v)
        elif hasattr(value, "__dict__"):
            data[key] = value.__dict__()
    return data
