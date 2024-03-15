from flask import Flask, jsonify, Response, request, send_from_directory

from flask import Flask
from flask_cors import CORS
from flask_socketio import SocketIO
import requests
from socketio_client import socketIoClientManager, gen_frames

from public.publicresolver import getNetInfo


ROS_SERVER = "http://" + getNetInfo()["ROBOT_FLASK_SERVER"]


app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

from share.controller import Controller

controller = Controller()


@app.route("/", methods=["GET"])
def get_template():
    return send_from_directory("build", "index.html")


@app.route("/static/css/<path:filename>", methods=["GET"])
def get_template_static_css(filename):
    return send_from_directory("build/static/css", filename)


@app.route("/static/js/<path:filename>", methods=["GET"])
def get_template_static_cs(filename):
    return send_from_directory("build/static/js", filename)


@app.route("/ros/stop_motor", methods=["POST"])
def post_stop_motor():
    """
    request http call of /stop_motor on ROS_SERVER
    """
    response = requests.post(f"{ROS_SERVER}/stop_motor")
    if response.status_code == 200:
        return Response(status=200)


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


@app.route("/ros/topic/list/", methods=["GET"])
def get_topic_list():
    return jsonify(controller.get_topic_list())


@app.route("/ros/topic/type/format", methods=["POST"])
def get_topic_type_format():
    topic_type = request.json.get("topic_type")
    return jsonify(controller.rospy.get_type_json_format(topic_type))


@app.route("/ros/topic/publish", methods=["POST"])
def publish_topic_once():
    topic_name = request.json.get("topic_name")
    topic_type = request.json.get("topic_type")
    data = request.json.get("message")
    controller.manual_topic_emit(topic_name, topic_type, data)
    return Response(status=200)


@app.route("/ros/topic", methods=["POST"])
def post_topic_subscribe():
    topic_name = request.json.get("topic_name")
    topic_type = request.json.get("topic_type")

    res = controller.manualTopicManager.register_topic(topic_name, topic_type)
    if res == True:
        return Response(status=200)
    return Response(status=400, response=res)


@app.route("/ros/topic/logs", methods=["POST"])
def post_topic_get_logs():
    topic_name = request.json.get("topic_name")
    return jsonify(controller.get_topic_logs(topic_name))


@app.route("/ros/topic/nodes", methods=["POST"])
def post_topic_nodes_get():
    topic_name = request.json.get("topic_name")
    return jsonify(controller.rospy.get_topic_nodes_list(topic_name))


@app.route("/ros/topic/preview/<frame_id>")
def ros_preview_feed(frame_id):
    return Response(
        gen_frames(frame_id),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/ros/topic/delete", methods=["POST"])
def delete_topic_unsubscribe():
    topic_name = request.json.get("topic_name")

    ret = controller.manualTopicManager.delete_topic(topic_name)
    if ret == 200:
        return Response(status=200)
    return Response(status=400, response=ret)


@app.route("/ros/node/list/", methods=["GET"])
def get_node_pkg_list():
    response = requests.get(f"{ROS_SERVER}/pkg/node/list")
    if response.json():
        return jsonify(response.json())
    return jsonify({"status": "success"})


@app.route("/ros/node/<pkg_name>/<node_name>", methods=["POST"])
def post_node_start(pkg_name, node_name):
    response = requests.post(
        f"{ROS_SERVER}/pkg/node/{pkg_name}/{node_name}", json=request.json
    )
    return Response(
        response.content,
        status=response.status_code,
    )


@app.route("/ros/node/<pkg_name>/<node_name>", methods=["DELETE"])
def delete_node_stop(pkg_name, node_name):
    response = requests.delete(f"{ROS_SERVER}/pkg/node/{node_name}")
    return Response(
        response.content,
        status=response.status_code,
    )


@app.route("/ros/node/<pkg_name>/<node_name>", methods=["GET"])
def get_node_status(pkg_name, node_name):
    # Send request to the ROS server
    response = requests.get(f"{ROS_SERVER}/pkg/node/{node_name}")

    # Check if the request was successful
    if response.status_code == 200:
        # Return the content of the response, status code, and headers
        return Response(
            response.content,
            status=response.status_code,
        )
    else:
        # Handle errors or unexpected response codes here, for example:
        return {"error": "Node not found or error in server"}, 404


@app.route("/ros/service/list/", methods=["GET"])
def get_ros_service_list():
    return jsonify(controller.rospy.get_services_list())


@app.route("/ros/service/call", methods=["POST"])
def get_ros_service_call():
    service_name = request.json.get("service_name")
    service_type = request.json.get("service_type")
    request_data = request.json.get("request_data")

    result = controller.manual_service_call(service_name, service_type, request_data)
    return jsonify(result)


@app.route("/ros/video/lidar")
def get_ros_video_lidar_stream():
    return Response(
        controller.lidar.stream.generate_preview(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@socketio.on("connect", namespace="/socket/ros")
def handle_connect_camera_info():
    print("ReactApp connected")


@socketio.on("/drive", namespace="/socket/ros")
def handle_drive(data):
    socketIoClientManager.ros_socket_emit("drive", data, namespace="/ros")


with app.app_context():
    controller.init(socketio, "/socket/ros")

if __name__ == "__main__":
    socketIoClientManager.ros_socket_launch_thread(socketio)
    socketio.run(app, port=5001, host="0.0.0.0")
