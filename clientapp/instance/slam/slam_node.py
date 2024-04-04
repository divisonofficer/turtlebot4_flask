import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from slam_opencv import slam_map_opencv

from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SaveMap

from slam_types import (
    QuaternionAngle,
    MapMarker,
    EulierAngle,
    Point3D,
    MapInfo,
    Pose3D,
    dictValueConversion,
)

import math

import time
from typing import Union, List, Optional, Tuple


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

    __map_origin: Optional[Point3D]
    __euler_orientation: Optional[EulierAngle]
    __markers: List[MapMarker]
    __map_msg: Optional[OccupancyGrid]
    __map_size: Optional[MapInfo]
    __lidar_position: List[Tuple[float, float]]

    def __init__(self, sockets, launch):
        """
        Initializes the SlamApp node.

        Args:
            sockets: The sockets used for communication.
        """
        rclpy.init()
        super().__init__("client_slam_node")

        self.sockets = sockets
        self.launch = launch
        self.timer_ = self.create_timer(10, self.timer_callback)
        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )
        self.pos_subscription = self.create_subscription(
            PoseWithCovarianceStamped, "/pose", self.position_callback, 10
        )
        self.lidar_subscription = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, 10
        )
        self.init_members()

    def init_members(self):
        self.__map_origin = None
        self.__euler_orientation = None
        self.__markers = []

        self.__map_msg = None
        self.__map_size = None
        self.__position = None

        self.topic_timestamps = {}

    ############################################################################################################
    # Topic Callbacks
    ############################################################################################################

    def map_callback(self, msg: OccupancyGrid):
        """
        Callback function for the map topic.

        Args:
            msg: The map message.
        """
        slam_map_opencv(msg, self.__position, self.__markers, self.__lidar_position)
        self.__map_origin = Point3D.from_msg(msg.info.origin.position)
        self.__map_size = MapInfo(msg.info.width, msg.info.height, msg.info.resolution)
        self.__map_msg = msg
        self.topic_timestamps["map"] = msg.header.stamp

    def position_callback(self, msg: PoseWithCovarianceStamped):
        """
        Callback function for the position topic.

        Args:
            msg: The position message.
        """
        self.__position = Pose3D.from_msg(msg.pose.pose)
        self.__euler_orientation = QuaternionAngle.from_msg(
            msg.pose.pose.orientation
        ).to_euler()
        self.sockets.emit(
            "robot_pose",
            self.__position.to_dict_point(),
            namespace="/slam",
        )
        self.topic_timestamps["pose"] = msg.header.stamp

    def lidar_callback(self, msg: LaserScan):
        self.__lidar_position = self.laser_to_position_array(msg)

    ############################################################################################################
    # Service Call
    ############################################################################################################

    def service_call_save_map(self, filename: str):
        request = SaveMap.Request()
        request.name = filename
        client = self.create_client(SaveMap, "/slam_toolbox/save_map")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().success
        else:
            return False

    ############################################################################################################
    # Marker Functions
    ############################################################################################################

    def add_marker(self, msg: PoseWithCovarianceStamped):
        """
        Adds a marker to the list of markers.

        Args:
            msg: The marker message.
        """
        self.__markers.append({"id": time.time().real, "pose": self.pose_to_dict(msg)})

    def request_add_marker(self):
        """
        Requests to add a marker.
        """
        self.get_logger().info("Request to add marker")
        self.add_marker(self.__position)
        self.sockets.emit(
            "markers",
            dictValueConversion(self.__markers),
            namespace="/slam",
        )

    def add_marker_by_position(self, x, y):
        """
        Adds a marker to the list of markers by position.

        Args:
            x: The x-coordinate of the marker.
            y: The y-coordinate of the marker.
        """

        self.__markers.append(
            MapMarker(
                id=time.time().real,
                position=Point3D(x, y, 0),
                orientation=QuaternionAngle(0, 0, 0, 1),
            )
        )
        self.sockets.emit(
            "markers",
            dictValueConversion(self.__markers),
            namespace="/slam",
        )

    def delete_marker(self, id):
        """
        Deletes a marker from the list of markers.

        Args:
            id: The id of the marker.
        """
        self.__markers = [marker for marker in self.__markers if marker.id != id]
        self.sockets.emit(
            "markers",
            dictValueConversion(self.__markers),
            namespace="/slam",
        )

    ############################################################################################################
    # Position Functions
    ############################################################################################################

    def laser_to_position_array(self, msg: LaserScan) -> List[Tuple[float, float]]:
        if self.__euler_orientation is None:
            return []

        angleStart = msg.angle_min + self.__euler_orientation.roll + math.pi / 2
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
        self.emit_slam_status()

    #############################################################################################################
    # Node Dignostics
    ############################################################################################################

    def emit_slam_status(self):
        """
        Emits the slam status.
        """
        if self.launch.process:
            self.get_logger().info("Slam App is Running")
            if self.__position:
                self.get_logger().info(f"Robot Pose: {self.__position}")
            if self.__map_origin:
                self.get_logger().info(f"Map Origin: {self.__map_origin}")
            self.sockets.emit(
                "slam_status",
                dictValueConversion(self.get_slam_status()),
                namespace="/slam",
            )
        else:
            self.get_logger().info("Slam App is not Running")
            self.sockets.emit(
                "slam_status",
                {"status": "error", "message": "Slam not running"},
                namespace="/slam",
            )

    def get_slam_status(self):
        return {
            "status": "success",
            "message": "Slam running",
            "robot_pose": (
                self.__position.to_dict_point() if self.__position else None
            ),
            "map_origin": self.__map_origin,
            "map_size": self.__map_size,
            "markers": [marker for marker in self.__markers],
        }

    def get_map_json(self):
        if self.__map_msg is None:
            return {
                "status": "error",
                "message": "Map not available",
            }
        data = list(self.__map_msg.data)
        return {
            "map_origin": self.point_to_dict(self.__map_origin),
            "map_size": self.__map_size,
            "markers": self.__markers,
            "map_metadata": [
                {
                    "resolution": x.resolution,
                    "width": x.width,
                    "height": x.height,
                    "origin": self.point_to_dict(x.origin.position),
                }
                for x in [self.__map_msg.info]
            ][0],
            "map_data": data,
        }

    def point_to_dict(self, p):
        return {"x": p.x, "y": p.y}
