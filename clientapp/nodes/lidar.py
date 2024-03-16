import rclpy
import math
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
from videostream import VideoStream
from rclpy.node import Node
from flask_socketio import SocketIO


RESOLUTION = 256
DOSPLAY_RESOLUTION = 512


class LidarNode(Node):

    def __init__(self, socketio: SocketIO):
        super().__init__("client_lidar_node")
        self.socketio = socketio
        self.subscription = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.__bridgeCallback = None
        self.stream = VideoStream()
        self.__bridgeCallback = self.stream.cv_ndarray_callback

    def scan_callback(self, msg: LaserScan):

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
        self.image_array = [[0] * RESOLUTION for _ in range(RESOLUTION)]
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

        for dot in dots:
            x_normalized, y_normalized = dot
            # Convert normalized x and y values to pixel coordinates
            x_pixel = int((x_normalized + 1) * RESOLUTION / 2)
            y_pixel = int((y_normalized + 1) * RESOLUTION / 2)
            # Set the corresponding pixel in the image array to 1
            self.image_array[y_pixel][x_pixel] = 1

        # Create a blank image with size RESOLUTION x RESOLUTION and 3 channels (RGB)
        image = np.zeros((RESOLUTION * 2, RESOLUTION * 2, 3), dtype=np.uint8)

        # Iterate over the image array and set the corresponding pixels in the image
        for y in range(RESOLUTION):
            for x in range(RESOLUTION):
                if self.image_array[y][x] == 1:
                    # Draw a circle with radius 3 pixels
                    cv2.circle(image, (x * 2, y * 2), 3, (255, 255, 255), -1)

        if self.__bridgeCallback:
            self.__bridgeCallback(image)
