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
    ros_socket_thread(
        {
            "/oakd/rgb/preview/camera_info": lambda x: fsio.emit(
                "camera_info", x, namespace="/socket/ros"
            ),
            "/ip": lambda x: print(x) or fsio.emit("/ip", x, namespace="/socket/ros"),
            "/color/image": lambda x: 0,
        }
    )
