import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose2D
from slam_opencv import slam_map_opencv

from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SaveMap, DeserializePoseGraph, SerializePoseGraph

from slam_types import RosProtoConverter

from slam_pb2 import *

import math

import time
from typing import Union, List, Optional, Tuple

from spinner import Spinner
from std_msgs.msg import String


from google.protobuf import json_format


class SlamNodeMeta:

    map_origin: Optional[Point3D] = None
    markers: list[MapMarker] = []
    position: Optional[Pose3D] = None

    map_msg: Optional[OccupancyGrid] = None
    map_size: Optional[MapInfo] = None
    lidar_position: Optional[list[tuple[float, float]]] = None
    slam_metadata = {
        "pose": {
            "interval": 9999,
            "timestamp": time.time(),
            "app_time": time.time(),
        },
        "map": {
            "interval": 9999,
            "timestamp": time.time(),
            "app_time": time.time(),
        },
        "lidar": {
            "interval": 9999,
            "timestamp": time.time(),
            "app_time": time.time(),
        },
    }

    def __init__(self):
        pass

    def updateTimestamp(self, topic: str, timestamp: float):
        self.slam_metadata[topic]["interval"] = (
            timestamp - self.slam_metadata[topic]["timestamp"]
        )
        self.slam_metadata[topic]["timestamp"] = timestamp
        self.slam_metadata[topic]["appTime"] = time.time()

    def toProto(self):
        meta = SlamMetaData()
        json_format.ParseDict(self.slam_metadata, meta)
        return SlamState(
            robot_pose=(
                RobotPose(
                    position=self.position.position,
                    orientation=self.position.orientation_euler,
                )
                if self.position
                else None
            ),
            map_origin=self.map_origin,
            map_size=self.map_size,
            markers=self.markers,
            slam_metadata=meta,
        )


class SlamApp(Node):
    """
    This class represents the SlamApp node.

    Args:
        sockets: The sockets used for communication.

    Attributes:
        sockets: The sockets used for communication.
        timer_: The timer for the timer callback.
        map_subscription: The subscription for the map topic.
        pos_subscription: The subscription for the position topic.
        __position: The current position.
        __map_origin: The origin of the map.
        __map_size: The size of the map.
        __markers: The list of markers.
        topic_timestamps: The timestamps of the topics.
    """

    state = SlamNodeMeta()
    rosProto = RosProtoConverter()

    def __init__(self, sockets, launch, spinner: Spinner):
        """
        Initializes the SlamApp node.

        Args:
            sockets: The sockets used for communication.
        """
        super().__init__("client_slam_node")

        spinner.add_node(self)
        self.spinner = spinner

        self.sockets = sockets
        self.launch = launch
        self.timer_ = self.create_timer(30, self.timer_callback)
        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )
        self.pos_subscription = self.create_subscription(
            PoseWithCovarianceStamped, "/pose", self.position_callback, 10
        )
        self.lidar_subscription = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, 10
        )

    ############################################################################################################
    # Topic Callbacks
    ############################################################################################################

    def map_callback(self, msg: OccupancyGrid):
        """
        Callback function for the map topic.

        Args:
            msg: The map message.
        """
        self.state.map_size = MapInfo(
            width=msg.info.width, height=msg.info.height, resolution=msg.info.resolution
        )
        slam_map_opencv(
            msg, self.state.map_msg, self.state.markers, self.state.lidar_position
        )

        self.state.updateTimestamp("map", msg.header.stamp.nanosec / 1000000)

    def position_callback(self, msg: PoseWithCovarianceStamped):
        """
        Callback function for the position topic.

        Args:
            msg: The position message.
        """
        self.state.updateTimestamp("pose", msg.header.stamp.nanosec / 1000000)

        self.state.position = self.rosProto.rosPoseToProtoPose3D(msg.pose.pose)
        if self.state.position:
            self.sockets.emit(
                "robot_pose",
                json_format.MessageToDict(self.state.position),
                namespace="/slam",
            )

        self.emit_slam_status()

    def lidar_callback(self, msg: LaserScan):
        self.state.updateTimestamp("lidar", msg.header.stamp.nanosec / 1000000)

        self.state.lidar_position = self.laser_to_position_array(msg)

    ############################################################################################################
    # Service Call
    ############################################################################################################

    def service_call(self, service_type, service_name, request):
        client = self.create_client(service_type, service_name)
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")
        self.get_logger().info(f"Service available, calling service... {service_name}")
        future = client.call_async(request)
        result = self.spinner.call_future_sync(service_name, future)

        self.get_logger().info(f"Service call result: {result}")

        if hasattr(result, "result") and result.result == 255:
            return None
        return result

    def service_call_save_map_png(self, filename: str):
        request = SaveMap.Request()
        request_name = String()
        request_name.data = filename
        request.name = request_name

        return self.service_call(SaveMap, "/slam_toolbox/save_map", request)

    def service_call_save_map(self, filename: str):
        request = SerializePoseGraph.Request()
        request.filename = filename
        return self.service_call(
            SerializePoseGraph, "/slam_toolbox/serialize_map", request
        )

    def service_call_load_map(self, filename: str):
        if not self.state.position:
            return "Error"
        request = DeserializePoseGraph.Request()

        request.filename = filename
        request.match_type = DeserializePoseGraph.Request.START_AT_GIVEN_POSE
        request.initial_pose = Pose2D()
        request.initial_pose.x, request.initial_pose.y = (
            self.state.position.position.x,
            self.state.position.position.y,
        )

        request.initial_pose.theta = self.state.position.orientation_euler.yaw
        return self.service_call(
            DeserializePoseGraph, "/slam_toolbox/deserialize_map", request
        )

    ############################################################################################################
    # Marker Functions
    ############################################################################################################

    def add_marker(self, msg: PoseWithCovarianceStamped):
        """
        Adds a marker to the list of markers.

        Args:
            msg: The marker message.
        """
        pose = self.rosProto.rosPoseToProtoPose3D(msg.pose)
        self.state.markers.append(
            MapMarker(
                id=int(time.time().real),
                position=pose.position,
                orientation=pose.orientation_euler,
            )
        )

    def request_add_marker(self):
        """
        Requests to add a marker.
        """
        self.get_logger().info("Request to add marker")
        self.add_marker(self.state.position)
        self.sockets.emit(
            "markers",
            [json_format.MessageToDict(x) for x in self.state.markers],
            namespace="/slam",
        )

    def add_marker_by_position(self, x, y):
        """
        Adds a marker to the list of markers by position.

        Args:
            x: The x-coordinate of the marker.
            y: The y-coordinate of the marker.
        """

        self.state.markers.append(
            MapMarker(
                id=int(time.time().real),
                position=Point3D(x=x, y=y, z=0),
                orientation=RobotEuilerOrientation(
                    roll=0,
                    pitch=0,
                    yaw=0,
                ),
            )
        )
        self.sockets.emit(
            "markers",
            [json_format.MessageToDict(x) for x in self.state.markers],
            namespace="/slam",
        )

    def delete_marker(self, id):
        """
        Deletes a marker from the list of markers.

        Args:
            id: The id of the marker.
        """
        self.state.markers = [
            marker for marker in self.state.markers if marker.id != id
        ]
        self.sockets.emit(
            "markers",
            [json_format.MessageToDict(x) for x in self.state.markers],
            namespace="/slam",
        )

    ############################################################################################################
    # Position Functions
    ############################################################################################################

    def laser_to_position_array(self, msg: LaserScan) -> List[Tuple[float, float]]:
        if self.state.position is None:
            return []

        angleStart = (
            msg.angle_min + self.state.position.orientation_euler.yaw + math.pi / 2
        )
        angleIncrement = msg.angle_increment
        ranges = msg.ranges
        positionArray: List[Tuple[float, float]] = []
        for i, distance in enumerate(ranges):
            if math.isinf(distance):
                continue
            angle = angleStart + i * angleIncrement
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            positionArray.append((x, y))
        return positionArray

    ############################################################################################################
    # Timer Functions
    ############################################################################################################

    def timer_callback(self):
        """
        Timer callback function.
        """
        if self.launch.process:
            self.get_logger().info("Slam App is Running")
        else:
            self.get_logger().info("Slam App is not Running")
        self.emit_slam_status()

    #############################################################################################################
    # Node Dignostics
    ############################################################################################################

    def emit_slam_status(self):
        """
        Emits the slam status.
        """
        message = self.get_slam_status()
        if self.launch.process:
            if self.state.position:
                # self.get_logger().info(f"Robot Pose: {self.state.position}")
                pass
            if self.state.map_origin:
                # self.get_logger().info(f"Map Origin: {self.state.map_origin}")
                pass
            message.status = SlamState.SUCCESS
            message.message = "Slam is Running"
        else:
            message.status = SlamState.OFFLINE
            message.message = "Slam app is not running"
        self.sockets.emit(
            "slam_status",
            json_format.MessageToDict(message),
            namespace="/slam",
        )

    def get_slam_status(self):

        if (
            time.time() - self.state.slam_metadata["pose"]["timestamp"]
            > self.state.slam_metadata["pose"]["interval"] * 2
        ):
            self.state.slam_metadata["pose"]["interval"] = (
                time.time() - self.state.slam_metadata["pose"]["timestamp"]
            )
        return self.state.toProto()

    def get_map_json(self):
        if self.state.map_msg is None:
            return {
                "status": "error",
                "message": "Map not available",
            }
        data = list(self.state.map_msg.data)
        return {
            "map_origin": self.point_to_dict(self.state.map_origin),
            "map_size": self.state.map_size,
            "markers": self.state.markers,
            "map_metadata": [
                {
                    "resolution": x.resolution,
                    "width": x.width,
                    "height": x.height,
                    "origin": self.point_to_dict(x.origin.position),
                }
                for x in [self.state.map_msg.info]
            ][0],
            "map_data": data,
        }

    def point_to_dict(self, p):
        return {"x": p.x, "y": p.y}
