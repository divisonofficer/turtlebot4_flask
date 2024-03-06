import socketio

import threading
from requests import post
from flask_socketio import SocketIO


# (logger=True, engineio_logger=True)
ROS_SERVER = "http://192.168.185.2:5000"

import json

import logging

sio = socketio.Client()


class SocketIoClientManager:
    def __init__(self):
        self.event_dict = {}

    def open_topic_publish(self, topic):
        response = post(f"{ROS_SERVER}/manual/topic", json={"topic_name": topic})

    def ros_socket_open(self):
        try:
            # Replace 'http://localhost:5000' with your Socket.IO server URL
            sio.connect(ROS_SERVER, namespaces=["/manual"])
            # You might need to adjust the namespace based on your server setup
        except Exception as e:
            print(f"Connection error: {e}")

    def ros_socket_thread(self, topics: dict):
        self.ros_socket_open()

        for topic, callback in topics.items():
            self.open_topic_publish(topic)

            sio.on(topic, callback, namespace="/manual")
            print(f"Subscribed to {topic}")

    def socket_event_thread(self, events: dict):
        for event, callback in events.items():
            print(event, callback)
            sio.on(event, handler=callback, namespace="/manual")

    def ros_socket_add_event(self, fsio: SocketIO, event: str):
        print(f"Adding event {event}")
        if event in self.event_dict:
            return
        self.event_dict[event] = True

        sio.on(
            event,
            handler=lambda x: fsio.emit(event, x, namespace="/socket/ros"),
            namespace="/manual",
        )

    def ros_socket_launch_thread(self, fsio: SocketIO):

        self.ros_socket_thread(
            {
                # "/oakd/rgb/preview/camera_info": lambda x: fsio.emit(
                #     "camera_info", x, namespace="/socket/ros"
                # ),
                # "/ip": lambda x: print(x) or fsio.emit("/ip", x, namespace="/socket/ros"),
                # # "/color/image": lambda x: request_object_detection_payloader(x, fsio),
            }
        )
        self.socket_event_thread(
            {
                "status_monitoring": lambda x: fsio.emit(
                    "/status_monitoring", json.dumps(x), namespace="/socket/ros"
                ),
                "pkg_monitoring": lambda x: fsio.emit(
                    "/pkg_monitoring", json.dumps(x), namespace="/socket/ros"
                ),
            }
        )

    def ros_socket_update_events(self, events: str, fsio: SocketIO):
        # decode json to list of dict
        for event in events:
            if event["running"]:
                self.ros_socket_add_event(fsio, event["topic"])


socketIoClientManager = SocketIoClientManager()

import numpy as np
import cv2
from time import time


prev_detection_request = 0


def request_object_detection_payloader(response, fsio):
    global prev_detection_request
    if time() - prev_detection_request < 3:
        return

    fsio.emit(
        "camera_detection",
        request_object_detection(response_to_image_file(response)),
        namespace="/socket/ros",
    )
    prev_detection_request = time()


def request_object_detection(image_bytes):
    """
    the route "detect' need
    # Get the image
        image_file = request.files["image"]
    """
    response = post(f"http://localhost:5300/detect", files={"image": image_bytes})
    begin = time()
    print(f"Time: {time() - begin}")
    return response.json()


def response_to_image_file(data):

    # Extract image information
    height = data["height"]
    width = data["width"]
    image_data = data["data"]
    # Convert the flat list to a 3D numpy array (height, width, channels)
    # The data is in BGR format, and each color channel is 8 bits, so we use uint8
    image_array = np.array(image_data, dtype=np.uint8).reshape((height, width, 3))

    # Create an image using OpenCV
    cv2.imwrite("output_image.jpg", image_array)

    # If you need to send this image as part of a POST request, you can read it back as bytes
    with open("output_image.jpg", "rb") as image_file:
        image_bytes = image_file.read()

    # Now, `image_bytes` can be used as the body of your POST request
    return image_bytes
