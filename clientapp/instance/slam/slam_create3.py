import json
import math
import subprocess
from typing import Any, Optional, Union
from flask_socketio import SocketIO
from spinner import Spinner
from slam_pb2 import Point3D, Pose3D, Pose3DArray, RobotQuaternionOrientation
from tf2_msgs.msg import TFMessage
from irobot_create_msgs.action import NavigateToPosition, DriveDistance, RotateAngle
from irobot_create_msgs.srv import EStop
from geometry_msgs.msg import TransformStamped, Point, Quaternion
from google.protobuf import json_format
from slam_node import SlamApp
from rclpy.action import ActionClient
from rclpy.node import Node
from slam_types import RosProtoConverter


class SlamCreate3(Node):
    tf_poses: dict[str, Pose3D] = {}

    pose_stamps: Pose3DArray = Pose3DArray()

    def __init__(self, socket: SocketIO, spinner: Spinner):
        super().__init__("slam_create3")  # type: ignore
        self.socket = socket
        spinner.add_node(self)
        self.spinner = spinner

        self.subscribe_tf()

        self.create_timer(1, self.emit_poses)

    def subscribe_tf(self):
        self.subscription_tf = self.create_subscription(
            TFMessage, "/tf", self.tf_callback, 10
        )
        self.action_nav = ActionClient(self, NavigateToPosition, "navigate_to_position")
        self.action_estop = self.create_client(EStop, "/e_stop")
        self.action_drive = ActionClient(self, DriveDistance, "drive_distance")
        self.action_rotate = ActionClient(self, RotateAngle, "rotate_angle")

    def tf_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            if isinstance(transform, TransformStamped):
                quaternion = RobotQuaternionOrientation(
                    x=transform.transform.rotation.x,
                    y=transform.transform.rotation.y,
                    z=transform.transform.rotation.z,
                    w=transform.transform.rotation.w,
                )
                self.tf_poses[transform.child_frame_id] = Pose3D(
                    position=Point3D(
                        x=transform.transform.translation.x,
                        y=transform.transform.translation.y,
                        z=transform.transform.translation.z,
                    ),
                    orientation=quaternion,
                    orientation_euler=RosProtoConverter().quaternionToEuler(quaternion),
                )

    def emit_poses(self):
        self.socket.emit(
            "create3_poses",
            self.get_current_pose_dict(),
            namespace="/slam",
        )

    def get_current_pose(self):
        if not "base_link" in self.tf_poses:
            return None
        return self.tf_poses["base_link"]

    def get_current_pose_dict(self):
        pose_dict = self.get_current_pose()
        if not pose_dict:
            return None
        pose_dict = json_format.MessageToDict(pose_dict)
        for key, value in pose_dict.items():
            for key_, value_ in value.items():
                if value_ == None:
                    value[key_] = 0
        return pose_dict

    def navigate_robot(self, goal: Union[Pose3D, Point3D]):
        if isinstance(goal, Point3D):
            current_orientation = self.tf_poses["base_link"].orientation
            goal = Pose3D(
                position=goal,
                orientation=current_orientation,
            )
        print("Sending Navigation Goal")
        self.extend_pose_stamps()
        MoveGoal(self).move(goal)

    def rotate_robot(self, angle: float):
        self.extend_pose_stamps()
        MoveGoal(self).rotate(angle)

    def drive_robot(self, distance: float):
        self.extend_pose_stamps()
        MoveGoal(self).drive(distance)

    def estop(self):
        subprocess.run(
            [
                "ros2",
                "service",
                "call",
                "/e_stop",
                "irobot_create_msgs/srv/EStop",
                "{e_stop_on: true}",
            ]
        )
        # print("E-Stop")
        # request = EStop.Request()
        # request.e_stop_on = True
        # self.action_estop.call(EStop.Request())

    def estop_release(self):
        subprocess.run(
            [
                "ros2",
                "service",
                "call",
                "/e_stop",
                "irobot_create_msgs/srv/EStop",
                "{e_stop_on: false}",
            ]
        )

    def extend_pose_stamps(self):
        current_pose = self.get_current_pose()
        if not current_pose:
            return
        self.pose_stamps.poses.append(current_pose)
        self.socket.emit(
            "create3_pose_stamps",
            self.pose_stamps.SerializeToString(),
            namespace="/slam",
        )


class MoveGoal:
    def __init__(self, node: SlamCreate3):
        self.node = node
        self.pose = node.tf_poses["base_link"]

    def move(self, goal: Pose3D):
        orientation = self.pose.orientation_euler.yaw
        origin = self.pose.position
        target_angle = math.atan2(
            goal.position.y - origin.y, goal.position.x - origin.x
        )
        target_angle = math.pi - target_angle
        self.rotate(target_angle - orientation)

        target_distance = math.sqrt(
            (goal.position.x - origin.x) ** 2 + (goal.position.y - origin.y) ** 2
        )
        self.drive(target_distance)

    def rotate(self, angle: float):
        goal_angle = RotateAngle.Goal()
        goal_angle.angle = angle
        if goal_angle.angle > math.pi:
            goal_angle.angle -= 2 * math.pi
        if goal_angle.angle < -math.pi:
            goal_angle.angle += 2 * math.pi
        goal_angle.max_rotation_speed = 0.3
        self.node.action_rotate.send_goal(goal_angle)

    def drive(self, distance: float):
        goal = DriveDistance.Goal()
        goal.distance = distance
        goal.max_translation_speed = 0.13
        self.node.action_drive.send_goal(goal)
