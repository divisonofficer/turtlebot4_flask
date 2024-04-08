import rclpy
import math
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
from videostream import VideoStream
from rclpy.node import Node
from flask_socketio import SocketIO

RESOLUTION = 512
DOSPLAY_RESOLUTION = 512


class LidarNode(Node):
    __counter = 0
    EMIT_INTERVAL = 10
    is_running = False

    def __init__(self, socketio: SocketIO):
        super().__init__("client_lidar_node")
        self.create_timer(20, self.timer_callback)
        self.socketio = socketio
        self.subscription = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.__bridgeCallback = None
        self.stream = VideoStream()
        self.__bridgeCallback = self.stream.cv_ndarray_callback
        self.is_running = False  # Add an on-off switch variable

    def timer_callback(self):
        if self.is_running:
            print("Lidar Node is Running")

    def scan_callback(self, msg: LaserScan):
        if not self.is_running:
            return

        self.__counter += 1
        if self.__counter > self.EMIT_INTERVAL:
            self.__counter = 0
            self.socketio.emit(
                "/lidar/scan",
                {
                    "range_max": msg.range_max,
                    "angle_increment": msg.angle_increment,
                    "angle_min": msg.angle_min,
                    "angle_max": msg.angle_max,
                },
                namespace="/socket/ros",
            )
        max_range = msg.range_max * 0.5
        ranges = msg.ranges
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min
        dots = []
        for i, distance in enumerate(ranges):
            # if distance is inf, continue
            if math.isinf(distance):
                continue

            if distance > max_range:
                continue

            angle = angle_min + i * angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)

            # Normalize x and y values to range from -1 to 1
            x_normalized = (x) / max_range
            y_normalized = (y) / max_range

            # Use the normalized x and y values for further processing
            dots.append((x_normalized, y_normalized))

        # Create a 2d image array. Each element in the array represents a pixel in the image. Image resolution is RESOLUTION x RESOLUTION

        image = np.full((RESOLUTION, RESOLUTION, 3), (255, 255, 255), dtype=np.uint8)

        # Iterate over the image array and set the corresponding pixels in the image

        for dot in dots:
            x = int((dot[0] + 1) * RESOLUTION / 2)
            y = int((dot[1] + 1) * RESOLUTION / 2)
            cv2.circle(image, (x, y), 2, (0, 0, 255), -1)

        if self.__bridgeCallback:
            self.__bridgeCallback(image)

    def start(self):
        self.is_running = True

    def stop(self):
        self.is_running = False
