import socketio

from requests import post
from flask_socketio import SocketIO
from public.publicresolver import getNetInfo

# (logger=True, engineio_logger=True)
ROS_SERVER = "http://" + getNetInfo()["ROBOT_FLASK_SERVER"]

sio = socketio.Client()


class SocketIoClientManager:
    def __init__(self):
        self.event_dict = {}

    def open_topic_publish(self, topic):
        response = post(f"{ROS_SERVER}/manual/topic", json={"topic_name": topic})

    def ros_socket_open(self):
        try:
            # Replace 'http://localhost:5000' with your Socket.IO server URL
            sio.connect(ROS_SERVER, namespaces=["/manual", "/ros"])
            print("Connected to ROS server")
            # You might need to adjust the namespace based on your server setup
        except Exception as e:
            print(f"Connection error: {e}")

    def socket_event_thread(self, events: dict):
        for event, callback in events.items():
            print(event, callback)
            sio.on(event, handler=callback, namespace="/manual")

    def ros_socket_add_event(self, fsio: SocketIO, event: str, event_type: str):
        print(f"Adding event {event} {event_type}")
        if event in self.event_dict:
            return
        self.event_dict[event] = True

        callback = lambda x: fsio.emit(event, x, namespace="/socket/ros")

        if "/CompressedImage" in event_type or "/msg/Image" in event_type:
            callback = lambda x: on_preview(x) or fsio.emit(
                event,
                {
                    "width": x["width"],
                    "height": x["height"],
                    "frame_id": x["header"]["frame_id"],
                },
                namespace="/socket/ros",
            )

        sio.on(
            event,
            handler=callback,
            namespace="/manual",
        )

    def ros_socket_launch_thread(self, fsio: SocketIO):
        self.ros_socket_open()
        self.socket_event_thread(
            {
                # "status_monitoring": lambda x: fsio.emit(
                #     "/status_monitoring", json.dumps(x), namespace="/socket/ros"
                # ),
                "pkg_monitoring": lambda x: fsio.emit(
                    "/pkg_monitoring", x, namespace="/socket/ros"
                ),
            }
        )

    def ros_socket_update_events(self, events: dict, fsio: SocketIO):
        # decode json to list of dict
        for event in events:
            if event["running"]:
                self.ros_socket_add_event(fsio, event["topic"], event["type"][0])

    def ros_socket_emit(self, event, data, namespace="/manual"):
        sio.emit(event, data, namespace=namespace)


socketIoClientManager = SocketIoClientManager()

import numpy as np
import cv2
from time import time
import time


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


current_frame = {}


def on_preview(data):
    global current_frame
    # Decode the frame

    height, width = data["height"], data["width"]
    frame_name = data["header"]["frame_id"]
    frame_data = np.array(data["data"], dtype=np.uint8).reshape(
        (height, width, 3)
    )  # Assuming 'bgr8' encoding
    current_frame[frame_name] = frame_data


def gen_frames(frame_name):
    global current_frame
    while True:
        if frame_name in current_frame and current_frame[frame_name] is not None:
            ret, buffer = cv2.imencode(".jpg", current_frame[frame_name])
            frame = buffer.tobytes()
            yield (b"--frame\r\n" b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        else:
            # You may want to add a small sleep here to avoid busy waiting
            time.sleep(5)
            continue
