from flask import Flask, jsonify, Response

from flask import Flask
from flask_socketio import SocketIO
import requests
from socketio_client import ros_socket_launch


ROS_SERVER = "http://192.168.185.2:5000"


app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")


@app.route("/ros/stop_motor", methods=["POST"])
def post_stop_motor():
    """
    request http call of /stop_motor on ROS_SERVER
    """
    response = requests.post(f"{ROS_SERVER}/stop_motor")
    return jsonify(response.json())


@app.route("/ros/camera/preview")
def camera_color_preview_proxy():
    video_stream_url = (
        f"{ROS_SERVER}/camera/preview"  # URL of the video stream on server B
    )
    req = requests.get(video_stream_url, stream=True)

    return Response(
        req.iter_content(chunk_size=1024), content_type=req.headers["Content-Type"]
    )


@app.route("/ros/camera/color")
def camera_color_color_proxy():
    video_stream_url = (
        f"{ROS_SERVER}/camera/color"  # URL of the video stream on server B
    )
    req = requests.get(video_stream_url, stream=True)

    return Response(
        req.iter_content(chunk_size=1024), content_type=req.headers["Content-Type"]
    )


@socketio.on("connect", namespace="/socket/ros")
def handle_connect_camera_info():
    print("ReactApp connected")


if __name__ == "__main__":
    ros_socket_launch(socketio)
    socketio.run(app, debug=True, port=5001)
