import rclpy
import math
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
from .videostream import VideoStream
from rclpy.node import Node


class LidarNode(Node):
    def __init__(self):
        super().__init__("client_lidar_node")
        self.subscription = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.stream = VideoStream(
            lambda x: setattr(self, "__bridgeCallback", x) or self.subscription
        )

    def scan_callback(self, msg: LaserScan):
        max_range = msg.range_max
        ranges = msg.ranges
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

        dots = []

        print("Lidar data received")

        for i, distance in enumerate(ranges):
            angle = angle_min + i * angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)

            # Normalize x and y values to range from -1 to 1
            x_normalized = (x - max_range) / max_range
            y_normalized = (y - max_range) / max_range

            # Use the normalized x and y values for further processing
            dots.append((x_normalized, y_normalized))

        # Create a 2d image array. Each element in the array represents a pixel in the image. Image resolution is 256 x 256
        image_array = [[0] * 256 for _ in range(256)]
        for dot in dots:
            x_normalized, y_normalized = dot
            # Convert normalized x and y values to pixel coordinates
            x_pixel = int((x_normalized + 1) * 127.5)
            y_pixel = int((y_normalized + 1) * 127.5)
            # Set the corresponding pixel in the image array to 1
            image_array[y_pixel][x_pixel] = 1

        # Create a blank image with size 256 x 256 and 3 channels (RGB)
        image = np.zeros((256, 256, 3), dtype=np.uint8)

        # Iterate over the image array and set the corresponding pixels in the image
        for y in range(256):
            for x in range(256):
                if image_array[y][x] == 1:
                    # Set the pixel to white (255, 255, 255)
                    image[y, x] = (255, 255, 255)

        # Convert the image to JPEG format
        _, jpeg_image = cv2.imencode(".jpg", image)

        if self.__bridgeCallback:
            self.__bridgeCallback(jpeg_image)
