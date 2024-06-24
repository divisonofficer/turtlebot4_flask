import math
from slam_types import EulerAngle, QuaternionAngle
from geometry_msgs.msg import Quaternion
from typing import Union


def quaternion_to_euler(q: Union[QuaternionAngle, Quaternion]) -> EulerAngle:
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """

    if isinstance(q, Quaternion):
        q = QuaternionAngle.from_msg(q)

    w, x, y, z = q.w, 0, q.y, 0  # Fix: Assign values from q to variables w, x, y, z
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

    return EulerAngle(roll_x, pitch_y, yaw_z)
