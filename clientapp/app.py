from flask import Flask, jsonify, Response, request

from flask import Flask
from flask_cors import CORS
from flask_socketio import SocketIO
import requests
from socketio_client import socketIoClientManager, gen_frames


ROS_SERVER = "http://192.168.185.2:5000"


app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")


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
    response = requests.get(f"{ROS_SERVER}/manual/topic/list")
    # Check if the request was successful
    if response.status_code == 200:
        # Return the content of the response, status code, and headers
        socketIoClientManager.ros_socket_update_events(response.json(), socketio)
        return Response(
            response.content,
            status=response.status_code,
        )
    else:
        # Handle errors or unexpected response codes here, for example:
        return {"error": response.content}, response.status_code


@app.route("/ros/topic", methods=["POST"])
def post_topic_subscribe():
    topic_name = request.json.get("topic_name")
    topic_type = request.json.get("topic_type")
    response = requests.post(
        f"{ROS_SERVER}/manual/topic",
        json={"topic_name": topic_name, "topic_type": topic_type},
    )
    socketIoClientManager.ros_socket_add_event(socketio, topic_name, topic_type)
    return Response(
        response.content,
        status=response.status_code,
    )


@app.route("/ros/topic/nodes", methods=["POST"])
def post_topic_nodes_get():
    topic_name = request.json.get("topic_name")
    response = requests.post(
        f"{ROS_SERVER}/manual/topic/nodes", json={"topic_name": topic_name}
    )
    return Response(
        response.content,
        status=response.status_code,
    )


@app.route("/ros/topic/preview/<frame_id>")
def ros_preview_feed(frame_id):
    return Response(
        gen_frames(frame_id),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/ros/topic/delete", methods=["POST"])
def delete_topic_unsubscribe():
    topic_name = request.json.get("topic_name")
    response = requests.post(
        f"{ROS_SERVER}/manual/topic/delete", json={"topic_name": topic_name}
    )
    return Response(
        response.content,
        status=response.status_code,
    )


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


@socketio.on("connect", namespace="/socket/ros")
def handle_connect_camera_info():
    print("ReactApp connected")


@socketio.on("/drive", namespace="/socket/ros")
def handle_drive(data):
    socketIoClientManager.ros_socket_emit("drive", data, namespace="/ros")


if __name__ == "__main__":
    socketIoClientManager.ros_socket_launch_thread(socketio)
    socketio.run(app, port=5001)
