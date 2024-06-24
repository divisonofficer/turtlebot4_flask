import math
import subprocess
from typing import Optional, Union
from flask_socketio import SocketIO
import numpy as np
from videostream import VideoStream
from spinner import Spinner
from slam_pb2 import (
    Point3D,
    Point3DArray,
    Pose3D,
    Pose3DArray,
    RobotQuaternionOrientation,
)
from tf2_msgs.msg import TFMessage
from irobot_create_msgs.action import NavigateToPosition, DriveDistance, RotateAngle
from irobot_create_msgs.srv import EStop
from geometry_msgs.msg import (
    TransformStamped,
    PoseStamped,
    Transform,
)
from google.protobuf import json_format
from rclpy.action import ActionClient
from rclpy.node import Node
from slam_types import RosProtoConverter
from sensor_msgs.msg import PointCloud2, Image, LaserScan, CameraInfo
from cv_bridge import CvBridge
from irobot_create_msgs.srv import ResetPose
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
)
from slam_depth_convert import depth_to_laser_scan


class StereoDepth:

    frame_left: Optional[np.ndarray] = None
    frame_right: Optional[np.ndarray] = None
    orientation_yaw: float = 0
    point_vector: np.ndarray = np.array([])
    pose_odometry: Optional[PoseStamped] = None
    pose_robot: Optional[Odometry] = None
    pose_tf: Optional[Transform] = None
    pointcloud: Optional[PointCloud2] = None
    tf_robot: Optional[Transform] = None

    oak_depth: Optional[Image] = None
    oak_depth_camera_info: Optional[CameraInfo] = None

    def __init__(self, socketIO: SocketIO, node: Node):
        self.videostream = VideoStream()
        self.depth_stream = VideoStream()
        self.socketIO = socketIO
        self.obstacle_map: dict[tuple[int, int], int] = {}
        self.root_node = node

        self.scan_publisher = node.create_publisher(LaserScan, "/scan", 10)

    def msg_tf_callback(self, msg: TFMessage):
        for transform in msg.transforms:
            if transform.header.frame_id == "world":
                self.pose_tf = transform.transform

    def pointcloud_to_laserscan(self, pointcloud):
        point_ranges = []
        for point in pointcloud:
            dir = math.atan2(point[1], point[0])
            distance = math.sqrt(point[0] ** 2 + point[1] ** 2)
            point_ranges.append((dir, distance))
        point_ranges.sort(key=lambda x: x[0])
        dir_min = -math.pi * 3 / 8 / 2
        dir_max = math.pi * 3 / 8 / 2
        range_max = float("inf")
        dir_increment = math.pi / 100
        ranges = [range_max] * int((dir_max - dir_min) / dir_increment + 1)
        for i, point in enumerate(point_ranges):
            i = int((point[0] - dir_min) / dir_increment)
            if i < 0 or i >= len(ranges):
                continue
            ranges[i] = min(ranges[i], point[1])
        laserScan = LaserScan()
        laserScan.header.stamp = self.root_node.get_clock().now().to_msg()
        laserScan.angle_increment = dir_increment
        laserScan.angle_min = dir_min
        laserScan.angle_max = dir_max
        laserScan.range_max = range_max
        laserScan.ranges = ranges
        laserScan.header.frame_id = "base_link"
        return laserScan

    def update_robot_pose(self, pose: Odometry):
        self.pose_robot = pose

    def msg_callback_pointcloud(self, msg: PointCloud2):
        self.pointcloud = msg
        self.process_pointcloud(msg)

    def msg_callback_odometry(self, msg: PoseStamped):
        self.pose_odometry = msg

    def process_pointcloud(self, msg: PointCloud2):
        # Convert PointCloud2 message to numpy array
        pointcloud = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 3)

        # Extract x, y, z coordinates from pointcloud
        x = pointcloud[:, 0]
        y = pointcloud[:, 1]
        z = pointcloud[:, 2]

        # Create 3D vector from x, y, z coordinates
        vector_3d = np.column_stack((x, y, z))
        vector_3d = vector_3d[vector_3d[:, 2] > 0.3]
        # vector_3d = vector_3d[vector_3d[:, 2] < 4]

        vector_3d *= 20
        vector_3d = vector_3d.astype(int)
        vector_3d = np.unique(vector_3d, axis=0)
        vector_3d = vector_3d.astype(float) / 20.0

        vector_3d = vector_3d[vector_3d[:, 0] != vector_3d[:, 1]]
        vector_3d = vector_3d[vector_3d[:, 0] != vector_3d[:, 2]]
        vector_3d = vector_3d[vector_3d[:, 1] != vector_3d[:, 2]]

        pose_robot = self.tf_robot
        pose_camera = self.pose_odometry.pose if self.pose_odometry else None
        # Transform vector_3d coordinates from camera to robot
        if pose_camera and pose_robot:
            # Convert pose_camera to a transformation matrix

            def pose_to_homogeneous_matrix(position=None, quaternion=None):
                """
                위치와 쿼터니언을 받아 4x4 호모지니어스 변환 행렬을 반환합니다.
                """
                # 변환 행렬의 회전 부분
                if position is None:
                    position = [0, 0, 0]
                else:
                    position = [position.x, position.y, position.z]
                if quaternion is None:
                    quaternion = [0, 0, 0, 1]
                else:
                    quaternion = [
                        0,
                        0,
                        quaternion.z,
                        quaternion.w,
                    ]

                rotation_matrix = R.from_quat(quaternion).as_matrix()

                # 변환 행렬
                transformation_matrix = np.eye(4)
                transformation_matrix[:3, :3] = rotation_matrix
                transformation_matrix[:3, 3] = position

                return transformation_matrix

            def transform_point_cloud(point_cloud, transformation_matrix):
                """
                주어진 변환 행렬을 사용하여 포인트 클라우드를 변환합니다.
                """
                # 포인트 클라우드를 호모지니어스 좌표로 확장합니다.
                num_points = point_cloud.shape[0]
                homogeneous_points = np.hstack((point_cloud, np.ones((num_points, 1))))

                # 변환 행렬을 적용합니다.
                transformed_points = (transformation_matrix @ homogeneous_points.T).T

                # 호모지니어스 좌표를 다시 3D 좌표로 변환합니다.
                transformed_points = transformed_points[:, :3]

                return transformed_points

            robot_matrix = pose_to_homogeneous_matrix(
                quaternion=pose_robot.rotation,
            )
            robot_posiiton_matrix = pose_to_homogeneous_matrix(
                position=pose_robot.translation,
            )

            camera_matrix = pose_to_homogeneous_matrix(
                position=pose_camera.position,
                quaternion=pose_camera.orientation,
            )
            camera_position_matrix = pose_to_homogeneous_matrix(
                position=pose_camera.position,
            )
            vector_3d = transform_point_cloud(vector_3d, np.linalg.inv(camera_matrix))
            vector_3d = transform_point_cloud(
                vector_3d,
                np.array(
                    [
                        [0, 1, 0, 0],
                        [-1, 0, 0, 0],
                        [0, 0, 1, 0],
                        [
                            0,
                            0,
                            0,
                            1,
                        ],
                    ]
                ),
            )

            vector_3d = vector_3d[np.abs(vector_3d[:, 0]) < 8]
            vector_3d = vector_3d[np.abs(vector_3d[:, 1]) < 8]

            # laserScan = self.pointcloud_to_laserscan(vector_3d)
            # laserScan.header.stamp = msg.header.stamp
            # self.scan_publisher.publish(laserScan)

            vector_3d = transform_point_cloud(vector_3d, robot_matrix)

            vector_3d = transform_point_cloud(vector_3d, robot_posiiton_matrix)

        else:
            print("Missing pose_camera or pose_robot")

        self.point_vector = vector_3d
        point_msg = Point3DArray()
        for point in vector_3d:
            proto_point = Point3D(
                x=point[0],
                y=point[1],
                z=point[2],
            )
            self.obstacle_map_put(point)
            point_msg.points.append(proto_point)

        self.socketIO.emit(
            "create3_pointcloud", point_msg.SerializeToString(), namespace="/slam"
        )

        obstacle_map_msg = Point3DArray()

        for key in self.obstacle_map.keys():
            x, y = key  # type: ignore
            proto_point = Point3D(
                x=float(x) / 10, y=float(y) / 10, z=self.obstacle_map[key]
            )
            obstacle_map_msg.points.append(proto_point)
        self.socketIO.emit(
            "create3_obstacle_map",
            obstacle_map_msg.SerializeToString(),
            namespace="/slam",
        )

    def scan_callback(self, msg: LaserScan):
        angle_yaw = self.orientation_yaw
        points_proto = Point3DArray()
        for i, range in enumerate(msg.ranges):
            angle = angle_yaw + msg.angle_min + i * msg.angle_increment
            x = range * math.cos(angle)
            y = range * math.sin(angle)
            if self.tf_robot:
                x = x + self.tf_robot.translation.x
                y = y + self.tf_robot.translation.y
            points_proto.points.append(Point3D(x=x, y=y, z=0))
        self.socketIO.emit(
            "create3_scan", points_proto.SerializeToString(), namespace="/slam"
        )

    def obstacle_map_put(self, point):

        x = point[0]
        y = point[1]
        z = point[2]
        if self.pose_odometry is None or self.pose_robot is None:
            return
        odo = self.pose_robot.pose.pose
        x_odo = odo.position.x
        y_odo = odo.position.y
        distance = math.sqrt((x - x_odo) ** 2 + (y - y_odo) ** 2)
        if distance < 1.5 and z > 0:
            x = int(x * 10)
            y = int(y * 10)
            if (x, y) in self.obstacle_map:
                self.obstacle_map[(x, y)] += 1
            else:
                self.obstacle_map[(x, y)] = 1

    def msg_callback_depth(self, msg: Image):
        if self.oak_depth_camera_info is None:
            return
        frame = CvBridge().imgmsg_to_cv2(msg)
        scan, points = depth_to_laser_scan(self.oak_depth_camera_info, frame)
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = "oak_rgb_camera_optical_frame"
        self.scan_publisher.publish(scan)
        if self.pose_robot is None:
            print("No robot pose")
            return
        print(
            points.shape,
            points[:, 0].min(),
            points[:, 0].max(),
            points[:, 1].min(),
            points[:, 1].max(),
        )
        robot_position = self.pose_robot.pose.pose.position
        robot_quaternion = self.pose_robot.pose.pose.orientation
        robot_rotation_matrix = R.from_quat(
            [
                robot_quaternion.x,
                robot_quaternion.y,
                robot_quaternion.z,
                robot_quaternion.w,
            ]
        ).as_matrix()

        rotate_90_clockwise_4x4 = np.array(
            [
                [0, 1, 0, 0],
                [1, 0, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ]
        )

        robot_rotation_matrix = np.vstack([robot_rotation_matrix, np.array([0, 0, 0])])
        robot_matrix = np.hstack(
            [
                robot_rotation_matrix,
                np.array([[robot_position.x], [robot_position.y], [0], [1]]),
            ]
        )
        if points.shape[1] == 3:
            points = np.hstack([points, np.ones((points.shape[0], 1))])
        else:
            points[:, 3] = 1
        points[:, 0] -= (points[:, 0].min() + points[:, 0].max()) / 2
        print(points.shape, robot_matrix.shape)
        points = rotate_90_clockwise_4x4 @ points.T
        points = robot_matrix @ points

        point_proto = Point3DArray()
        for point in points.T:
            point_proto.points.append(Point3D(x=point[0], y=point[1], z=point[2]))

        self.socketIO.emit(
            "create3_pointcloud",
            point_proto.SerializeToString(),
            namespace="/slam",
        )

    def msg_callback_depth_camera_info(self, msg: CameraInfo):
        self.oak_depth_camera_info = msg


class SlamCreate3(Node):
    tf_poses: dict[str, Pose3D] = {}
    odom_poses: dict[str, Odometry] = {}

    pose_stamps: Pose3DArray = Pose3DArray()

    def __init__(self, socket: SocketIO, spinner: Spinner):
        super().__init__("slam_create3")  # type: ignore
        self.socket = socket
        spinner.add_node(self)
        self.spinner = spinner
        self.depthStream = StereoDepth(socket, self)
        self.subscribe_tf()
        self.client_reset_pose = self.create_client(ResetPose, "/reset_pose")
        self.create_timer(1, self.emit_poses)

    def reset_pose(self):
        if self.depthStream.pose_odometry:
            pose_service = ResetPose.Request()
            pose_service.pose = self.depthStream.pose_odometry
            print("Reset Pose", pose_service.pose)
            self.client_reset_pose.call(pose_service)

    def subscribe_tf(self):
        self.subscription_tf = self.create_subscription(
            TFMessage, "/tf", self.tf_callback, 10
        )
        self.subscription_odom = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            ),
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
                        x=-transform.transform.translation.x,
                        y=transform.transform.translation.y,
                        z=transform.transform.translation.z,
                    ),
                    orientation=quaternion,
                    orientation_euler=RosProtoConverter().quaternionToEuler(quaternion),
                )
                if transform.child_frame_id == "base_link":
                    self.depthStream.orientation_yaw = self.tf_poses[
                        "base_link"
                    ].orientation_euler.yaw
                    self.depthStream.tf_robot = transform.transform

    def odom_callback(self, msg: Odometry):
        self.odom_poses[msg.child_frame_id] = msg
        self.depthStream.update_robot_pose(msg)

    def emit_poses(self):
        self.socket.emit(
            "create3_poses",
            self.get_current_pose_dict(),
            namespace="/slam",
        )

    def get_current_pose(self):
        odom = self.odom_poses.get("base_link")

        if not odom:
            return None
        pose = odom.pose.pose
        quaternion = RobotQuaternionOrientation(
            x=pose.orientation.x,
            y=pose.orientation.y,
            z=pose.orientation.z,
            w=pose.orientation.w,
        )
        euler = RosProtoConverter().quaternionToEuler(quaternion)

        return Pose3D(
            position=Point3D(
                x=pose.position.x,
                y=pose.position.y,
                z=pose.position.z,
            ),
            orientation=quaternion,
            orientation_euler=euler,
        )

    def pose_to_pose3d_dict(self, position, orientation):
        position = {
            "x": position.x,
            "y": position.y,
            "z": position.z,
        }
        quaternion = RobotQuaternionOrientation(
            x=orientation.x,
            y=orientation.y,
            z=orientation.z,
            w=orientation.w,
        )
        euler = RosProtoConverter().quaternionToEuler(quaternion)
        pose = Pose3D(
            position=Point3D(
                x=position["x"],
                y=position["y"],
                z=position["z"],
            ),
            orientation=quaternion,
            orientation_euler=euler,
        )
        return pose

    def get_current_pose_dict(self):
        pose_dict = self.get_current_pose()
        if not pose_dict:
            return None
        pose_dict = json_format.MessageToDict(pose_dict)
        for key, value in pose_dict.items():
            for key_, value_ in value.items():
                if value_ == None:
                    value[key_] = 0
        self.emit_pose_stamps()
        poses = [pose_dict]
        if self.depthStream.pose_tf:
            pose_tf = self.depthStream.pose_tf
            pose_dict_tf = self.pose_to_pose3d_dict(
                pose_tf.translation, pose_tf.rotation
            )
            poses.append(json_format.MessageToDict(pose_dict_tf))
        return poses

    def navigate_robot(self, goal: Union[Pose3D, Point3D]):
        if isinstance(goal, Point3D):
            current_orientation = self.tf_poses["base_link"].orientation
            goal = Pose3D(
                position=goal,
                orientation=current_orientation,
            )
        print("Sending Navigation Goal")
        self.extend_pose_stamps()
        MoveGoal(self).move(goal, self.check_drive_safety)

    def check_drive_safety(self, destination: Point3D):
        safety, points = self.compute_drive_safety(destination)
        if safety:
            print("YES I CAN MOVE")
            return True
        if not points is None:

            self.socket.emit(
                "create3_obstacles",
                Point3DArray(
                    points=[
                        Point3D(x=point[0], y=point[1], z=point[2]) for point in points
                    ]
                ).SerializeToString(),
                namespace="/slam",
            )
        print("NO I CAN'T MOVE")
        return False

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
        self.emit_pose_stamps()

    def emit_pose_stamps(self):
        self.socket.emit(
            "create3_pose_stamps",
            self.pose_stamps.SerializeToString(),
            namespace="/slam",
        )

    def compute_drive_safety(self, destination: Point3D):
        ROBOT_RADIUS = 0.5
        DETECT_ANGLE = math.pi / 8
        current_pose = self.get_current_pose()
        if not current_pose:
            return False, None
        distance = math.sqrt(
            (destination.x - current_pose.position.x) ** 2
            + (destination.y - current_pose.position.y) ** 2
        )
        point_in_distance = 0
        point_detected = []
        for point in self.depthStream.point_vector:
            x = point[0]
            y = point[1]
            if (
                math.sqrt(
                    (x - current_pose.position.x) ** 2
                    + (y - current_pose.position.y) ** 2
                )
                < distance + ROBOT_RADIUS
            ):
                point_in_distance += 1

                point_direction = math.atan2(
                    y - current_pose.position.y,
                    x - current_pose.position.x,
                )
                angle_gap = abs(
                    point_direction - current_pose.orientation_euler.yaw + math.pi
                )
                if angle_gap > math.pi:
                    angle_gap = 2 * math.pi - angle_gap
                if angle_gap < DETECT_ANGLE:
                    point_detected.append(point)

        if len(point_detected) > 0:
            return False, point_detected
        return True, None


class MoveGoal:
    def __init__(self, node: SlamCreate3):
        self.node = node
        self.pose = node.tf_poses["base_link"]

    def move(self, goal: Pose3D, check_drive_safety):
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
        if check_drive_safety(goal.position):
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
