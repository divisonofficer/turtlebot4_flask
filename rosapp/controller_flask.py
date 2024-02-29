from flask import Flask, request, jsonify, render_template, Response
from flask_socketio import SocketIO
from controller import Controller
import time
from ros_call import executor_thread
from camera_function.preview_stream import VideoStream
from socket_emit import SocketEmit

app = Flask(__name__)
app.config["SECRET_KEY"] = "secret!"
controller = Controller()
socketio = SocketIO(app, cors_allowed_origins="*", logger=True, engineio_logger=True)


@app.route("/stop_motor", methods=["POST"])
def post_stop_motor():
    response = controller.run_command("stop_motor")
    return jsonify(response)


@socketio.on(message="connect", namespace="/socket/camera/info")
def handle_connect_camera_info():
    print("ClientApp connected")


@app.route("/resource/<resource>", methods=["GET"])
def get_resource(resource):
    """
    return files in resource/ folder
    """
    return app.send_static_file("resource/" + resource)


@app.route("/", methods=["GET"])
def index():
    return "Hello world"


@app.route("/service/default/<service_name>", methods=["POST"])
def post_open_service(service_name, request_data=None):
    """
    HTTP POST request to call a ROS2 service
    ex) curl -X POST http://localhost:5000/service/default/stop_motor
    {
        "rosType": "std_srvs/srv/Empty",
        "body": {}
    }
    """
    rosType = request_data.get("rosType")
    requestBody = request_data.get("body")
    if requestBody is None:
        requestBody = {}
    if rosType is None:
        return "rosType is required", 400
    response = controller.manual_service_call(service_name, rosType, requestBody)
    return jsonify(response)


def onCallbackCameraInfo(msg):
    socketio.emit("camera_info", msg, namespace="/socket/camera/info")


colorStream = VideoStream(controller.con_subscribe_camera_color)
previewStream = VideoStream(controller.con_subscribe_camera_preview)


@app.route("/camera/color")
def color_video_feed():
    return Response(
        colorStream.generate_preview(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/camera/preview")
def preview_video_feed():
    return Response(
        previewStream.generate_preview(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


if __name__ == "__main__":

    executor_thread(
        [
            colorStream.subscriber,
            previewStream.subscriber,
            controller.con_subscribe_camera_info(
                None,
                SocketEmit(socketio, "/socket/camera/info", 1).getCallback(
                    "camera_info"
                ),
            ),
        ]
    )
    socketio.run(app, host="0.0.0.0", port=5000, debug=True)
