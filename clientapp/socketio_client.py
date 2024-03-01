import socketio

import threading
from requests import post
from flask_socketio import SocketIO


sio = socketio.Client()  # (logger=True, engineio_logger=True)
ROS_SERVER = "http://192.168.185.2:5000"


@sio.event
def connect():
    print("### open ###")
    sio.emit("my message", {"data": "I'm connected!"})


@sio.event
def connect_error(data):
    print("Connection failed")


@sio.event
def disconnect():
    print("### closed ###")


CALLBACK_CAMERA_INFO = None


def open_topic_publish(topic):
    response = post(f"{ROS_SERVER}/manual/topic", json={"topic_name": topic})


def ros_socket_open():
    try:
        # Replace 'http://localhost:5000' with your Socket.IO server URL
        sio.connect(ROS_SERVER, namespaces=["/manual"])
        # You might need to adjust the namespace based on your server setup
    except Exception as e:
        print(f"Connection error: {e}")


def ros_socket_thread(topics: dict):
    ros_socket_open()

    for topic, callback in topics.items():
        open_topic_publish(topic)

        sio.on(topic, callback, namespace="/manual")
        print(f"Subscribed to {topic}")


def ros_socket_launch(fsio: SocketIO):
    threading.Thread(target=ros_socket_launch_thread, args=(fsio,)).start()


def ros_socket_launch_thread(fsio: SocketIO):

    ros_socket_thread(
        {
            "/oakd/rgb/preview/camera_info": lambda x: fsio.emit(
                "camera_info", x, namespace="/socket/ros"
            ),
            "/ip": lambda x: print(x) or fsio.emit("/ip", x, namespace="/socket/ros"),
            "/color/image": lambda x: fsio.emit(
                "camera_detection",
                request_object_detection(response_to_image_file(x)),
                namespace="/socket/ros",
            ),
        }
    )


import numpy as np
import cv2
from time import time


def request_object_detection(image_bytes):
    """
    the route "detect' need
    # Get the image
        image_file = request.files["image"]
    """
    response = post(f"http://localhost:5300/detect", files={"image": image_bytes})
    begin = time()
    print(response.json())
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
