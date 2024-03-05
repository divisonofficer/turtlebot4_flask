from flask import Flask, request, jsonify, render_template, Response
from flask_socketio import SocketIO
from controller import Controller
from camera_function.preview_stream import VideoStream
from socket_emit import SocketEmit

from ros_executor import RosExecutor
from manual_topic import ManualTopicManager

from config_load import configManager

from ros_topic_diagnostic import RosTopicDiagnostic

app = Flask(__name__)
app.config["SECRET_KEY"] = "secret!"
controller = Controller()
socketio = SocketIO(app, cors_allowed_origins="*")


@app.route("/stop_motor", methods=["POST"])
def post_stop_motor():
    response = controller.run_command("stop_motor")
    return jsonify(response)


@socketio.on(message="connect", namespace="/socket/camera/info")
def handle_connect_camera_info():
    print("ClientApp connected")


@socketio.on(message="connect", namespace="/manual")
def handle_connect_manual():
    print("Manual Topic Receiver connected")


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


executor = RosExecutor()
manualTopicManager = ManualTopicManager(socketio, controller, executor)


@app.route("/manual/topic/", methods=["POST"])
def get_manual_topic():
    """
    get manual topic
    """
    topic_name = request.json.get("topic_name")
    if topic_name[0] != "/":
        topic_name = "/" + topic_name
    topic_type = configManager.ros2_topics[topic_name]

    node = manualTopicManager.register_topic(topic_name, topic_type)
    if node is None:
        return "Topic already exists", 400
    return Response(status=200)


@app.route("/manual/topic/<topic_name>", methods=["DELETE"])
def delete_manual_topic(topic_name):
    """
    delete manual topic
    """
    topic_name = "/" + topic_name
    if topic_name not in manualTopicManager.topics:
        return "Topic does not exist", 400
    manualTopicManager.delete_topic(topic_name)
    return Response(status=200)


rosDiagnostic = RosTopicDiagnostic(socketio)


@app.route("/pkg/node/list", methods=["GET"])
def get_node_list():
    """
    get node list
    """
    return jsonify(rosDiagnostic.NODE_CHECK_DICT)


@app.route("/pkg/node/<pkg_name>/<node_name>", methods=["POST"])
def post_node_launch(pkg_name, node_name):
    """
    launch node
    """
    status, response = rosDiagnostic.launch_node(pkg_name, node_name)
    if status == 500:
        return Response(response, status=500)
    return Response(status=200)


@app.route("/pkg/node/<node_name>", methods=["DELETE"])
def delete_node_kill(node_name):
    """
    delete node
    """
    rosDiagnostic.kill_node(node_name)
    return Response(status=200)


@app.route("/pkg/node/<node_name>", methods=["GET"])
def get_node_status(node_name):
    """
    get node status
    """
    return jsonify({"logs": rosDiagnostic.get_log(node_name)})


if __name__ == "__main__":

    executor.executor_thread(
        [
            # colorStream.subscriber,
            # previewStream.subscriber,
            controller.con_subscribe_camera_info(
                None,
                SocketEmit(socketio, "/socket/camera/info", 1).getCallback(
                    "camera_info"
                ),
            ),
        ]
    )
    rosDiagnostic.spin()
    socketio.run(app, host="0.0.0.0", port=5000, debug=True)
