from flask import Flask, request, jsonify, render_template, Response
from flask_socketio import SocketIO
from share.controller import Controller
from camera_function.preview_stream import VideoStream

from config_load import configManager

from ros_topic_diagnostic import RosTopicDiagnostic


from geometry_msgs.msg import Twist


app = Flask(__name__)
app.config["SECRET_KEY"] = "secret!"
controller = Controller()
socketio = SocketIO(app, cors_allowed_origins="*")


@socketio.on(message="connect", namespace="/manual")
def handle_connect_manual():
    print("Manual Topic Receiver connected")


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


# ROS2 Topic Management


rosDiagnostic = RosTopicDiagnostic(socketio)


### ROS Node Managments


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
    options = request.json.get("options")
    status, response = rosDiagnostic.launch_node(pkg_name, node_name, options)
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


with app.app_context():
    controller.init(socketio, "/manual")
    rosDiagnostic.spin()


##### ROS SOCKET


@socketio.on("connect", namespace="/ros")
def handle_ros_connect():
    print("ROS Socket Connected")


@socketio.on("drive", namespace="/ros")
def handle_drive(data):
    message = Twist()
    message.linear.x = float(data["linear"]["x"])
    message.linear.y = float(data["linear"]["y"])
    message.linear.z = float(data["linear"]["z"])
    message.angular.x = float(data["angular"]["x"])
    message.angular.y = float(data["angular"]["y"])
    message.angular.z = float(data["angular"]["z"])
    controller.manual_topic_emit("/cmd_vel", "geometry_msgs/msg/Twist", message)


if __name__ == "__main__":
    socketio.run(app, host="0.0.0.0", port=5000)
