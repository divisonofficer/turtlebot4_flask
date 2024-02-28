from flask import Flask, request, jsonify, render_template, Response
from flask_socketio import SocketIO
from controller import Controller
import time
from ros_call import executor_thread
from camera_function.preview_stream import (
    generate_preview,
    create_ros_preview_subscriber,
)

app = Flask(__name__)
controller = Controller()
socketio = SocketIO(app)


@app.route("/stop_motor", methods=["POST"])
def post_stop_motor():
    response = controller.run_command("stop_motor")
    return jsonify(response)


@socketio.on("connect", namespace="/camera/info")
def handle_connect_camera_info():
    print("Client connected")


@app.route("/resource/<resource>", methods=["GET"])
def get_resource(resource):
    """
    return files in resource/ folder
    """
    return app.send_static_file("resource/" + resource)


@app.route("/", methods=["GET"])
def index():
    return "Hello world"


def onCallbackCameraInfo(msg):
    socketio.emit("camera_info", msg, namespace="/camera/info")


@app.route("/preview_video_feed")
def preview_video_feed():
    return Response(
        generate_preview(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":

    executor_thread(
        [
            create_ros_preview_subscriber(controller),
            controller.con_subscribe_camera_info(None, onCallbackCameraInfo),
        ]
    )
    socketio.run(app, host="0.0.0.0", port=5000, debug=True)
