import socketio
import threading

# Create a Socket.IO client

sio = socketio.AsyncClient(logger=True, engineio_logger=True)
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


@sio.on("camera_info")
def on_event_camera_info(data):
    if CALLBACK_CAMERA_INFO is not None:
        CALLBACK_CAMERA_INFO(data)


CALLBACK_CAMERA_INFO = None


def ros_socket_open():
    try:
        # Replace 'http://localhost:5000' with your Socket.IO server URL
        sio.connect(ROS_SERVER, namespaces=["/socket/camera/info"])
        # You might need to adjust the namespace based on your server setup
    except Exception as e:
        print(f"Connection error: {e}")


def ros_socket_thread(
    callback_camera_info=None,
):
    global CALLBACK_CAMERA_INFO
    CALLBACK_CAMERA_INFO = callback_camera_info
    ros_socket_open()
