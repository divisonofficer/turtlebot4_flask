from crypt import methods
from flask import Flask, Response, request, send_file
from flask_socketio import SocketIO

import sys

sys.path.append("../../../public/proto/python")
sys.path.append("../public/proto/python")
from slam_create3 import SlamCreate3
from slam_pb2 import Point3D


import os
import subprocess
import threading
import rclpy

from slam_opencv import stream
from spinner import Spinner
from slam_node import SlamApp
from slam_repo import SlamRepo
from typing import Optional
from google.protobuf import json_format
import logging

logging.basicConfig(
    format="SlamApp %(message)s",
)


class SlamLaunch:
    def __init__(self, sockets):
        self.command = "ros2 launch turtlebot4_navigation slam.launch.py"
        self.process = None
        self.thread = None
        self.sockets = sockets

        self.isCreate3UsingMode = False
        self.isSlamUsingMode = False

    def std_callback(self, msg):
        print(msg)

    def launch(self, map_name: Optional[str] = None):
        """
        run subprocess asynchronously using Threading
        and lively get stdout, stderr.
        then pass them to  std_callback
        """

        def run_subprocess_async(command):
            self.process = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True
            )
            while True:
                stdout = self.process.stdout
                if not stdout:
                    break
                output = stdout.readline().decode().strip()
                if output == "" and self.process.poll() is not None:
                    break
                if output:
                    self.std_callback(output)
            rc = self.process.poll()
            return rc

        self.thread = threading.Thread(
            target=run_subprocess_async, args=(self.command,)
        )
        self.isSlamUsingMode = True
        self.thread.start()

    def cancel_subprocess(self):
        os.system(
            "ps aux | grep -i 'ros2 launch turtlebot4_navigation slam.launch.py' | awk '{print $2}' | xargs kill -9"
        )

        if self.thread:
            self.thread.join()
            self.thread = None
            if self.process:
                self.process.terminate()
                self.process = None
        if not self.process:
            self.isSlamUsingMode = False

    def __del__(self):
        self.cancel_subprocess()


app = Flask(__name__)
rclpy.init()
sockets = SocketIO(
    app,
    cors_allowed_origins="*",
    async_mode="threading",
    # logger=True,
    # engineio_logger=True,
)
spinner = Spinner()
spinner_create3 = Spinner()
launch = SlamLaunch(sockets)
node = SlamApp(sockets, launch, spinner)
node_create3 = SlamCreate3(sockets, spinner_create3)
repo = SlamRepo()


@app.route("/launch", methods=["GET"])
def launch_slam():
    if not launch.isSlamUsingMode and not launch.isCreate3UsingMode:
        launch.launch()
        return {
            "status": "success",
            "message": "Slam launched successfully",
        }
    return {
        "status": "error",
        "message": "Slam already running",
    }


@app.route("/launch/<map_name>", methods=["GET"])
def launch_slam_open_map(map_name):
    if not launch.process and not launch.isCreate3UsingMode:
        launch.launch(map_name)
        return {
            "status": "success",
            "message": "Slam launched successfully",
        }
    return {
        "status": "error",
        "message": "Slam already running",
    }


@app.route("/cancel", methods=["GET"])
def cancel_slam():

    if launch.process:
        print("Trying to cancel slam Service")
        launch.cancel_subprocess()
        return {
            "status": "success",
            "message": "Slam cancelled successfully",
        }
    return {
        "status": "error",
        "message": "Slam not running",
    }


@sockets.on("connect", namespace="/slam")
def handle_connect_camera_info():
    print("ReactApp connected")
    node.emit_slam_status()


@app.route("/map/add_marker_self", methods=["POST"])
def add_marker_self():
    node.request_add_marker()
    return {"status": "success", "message": "Marker added"}


@app.route("/map/add_marker", methods=["POST"])
def add_marker():
    x = request.form.get("x", "0")
    y = request.form.get("y", "0")
    if not x or not y:
        return {"status": "error", "message": "Invalid x or y"}
    node.add_marker_by_position(float(x), float(y))
    return {"status": "success", "message": "Marker added"}


@app.route("/map/stream/<timestamp>")
def map_preview_stream(timestamp):
    return Response(
        stream.generate_preview(timestamp),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/map/stream/<timestamp>", methods=["DELETE"])
def cancel_map_preview(timestamp):
    stream.stop(timestamp)
    return {"status": "success", "message": "Map preview stopped"}


@app.route("/map/marker/<id>", methods=["DELETE"])
def delete_marker(id):
    node.delete_marker(id)
    return {"status": "success", "message": "Marker deleted"}


@app.route("/map/data", methods=["GET"])
def get_map_data():
    if not launch.process:
        return {
            "status": "error",
            "message": "Slam not running",
        }
    data = node.get_map_json()
    print(data)
    return data


@app.route("/map/save", methods=["POST"])
def save_map():
    file = request.json.get("filename", "map") if request.json else None
    overlap_possible = request.json.get("overwrite", False) if request.json else None

    filename = repo.save_map_available(str(file))

    if not filename or not overlap_possible:
        return Response(
            status=400,
            response="Filename unavailable",
        )

    if not node.service_call_save_map(filename):
        return Response(
            status=500,
            response="Failed to save map",
        )
    if not node.service_call_save_map_png(filename):
        return Response(
            status=500,
            response="Failed to save map image",
        )
    return {
        "status": "success",
        "message": "Map saved",
    }


@app.route("/map/load", methods=["POST"])
def load_serialized_map():
    file = request.json.get("filename", "map") if request.json else None
    filename = repo.load_map_available(str(file))
    if not filename:
        return {
            "status": "error",
            "message": "Filename unavailable",
        }
    if not node.service_call_load_map(filename):
        return {
            "status": "error",
            "message": "Failed to load map",
        }
    return {
        "status": "success",
        "message": "Map loaded",
    }


@app.route("/map/list", methods=["GET"])
def get_saved_map_list():
    return [json_format.MessageToDict(x) for x in repo.get_map_list()]


@app.route("/map/<map_name>", methods=["GET"])
def get_map_metadata(map_name):
    metadata = repo.get_map_metadata(map_name)
    if metadata:
        return json_format.MessageToDict(metadata)
    return Response(
        status=404,
        response="Map not found",
    )


@app.route("/map/<map_name>/image", methods=["GET"])
def get_map_image(map_name):
    path = repo.get_map_image(map_name)
    if path:
        return send_file(path, mimetype="image/jpeg")
    return Response(
        status=404,
        response="Map image not found",
    )


@app.route("/create3", methods=["GET"])
def get_create3_data():
    status = node_create3.get_current_pose_dict()
    if not status:
        return {
            "status": "error",
            "message": "Create3 not running",
        }
    return status


@app.route("/create3/launch", methods=["POST"])
def launch_create3():
    if not launch.process and not launch.isSlamUsingMode:
        launch.isCreate3UsingMode = True
        # todo : prepare create3 launch command
        return {
            "status": "success",
            "message": "Create3 launched successfully",
        }
    return {
        "status": "error",
        "message": "Create3 already running",
    }


@app.route("/create3/action/navigate_to_position", methods=["POST"])
def create3_action_navigate_to_position():
    x = float(request.json["x"]) if request.json else None
    y = float(request.json["y"]) if request.json else None

    if x is None or y is None:
        return {"status": "error", "message": "Invalid x or y"}

    node_create3.navigate_robot(Point3D(x=x, y=y, z=0))

    return {"status": "success", "message": "Robot navigated to position"}


@app.route("/create3/action/rotate", methods=["POST"])
def create3_action_rotate():
    angle = float(request.json["angle"]) if request.json else None
    if angle is None:
        return {"status": "error", "message": "Invalid angle"}

    node_create3.rotate_robot(angle)
    return {"status": "success", "message": "Robot rotated"}


@app.route("/create3/action/drive", methods=["POST"])
def create3_action_drive():
    distance = float(request.json["distance"]) if request.json else None
    if distance is None:
        return {"status": "error", "message": "Invalid distance"}

    node_create3.drive_robot(distance)
    return {"status": "success", "message": "Robot drove"}


@app.route("/create3/action/estop", methods=["POST"])
def create3_action_estop():
    node_create3.estop()
    return {"status": "success", "message": "Robot estopped"}


@app.route("/create3/action/estop/release", methods=["POST"])
def create3_action_estop_release():
    node_create3.estop_release()
    return {"status": "success", "message": "Robot estop released"}


if __name__ == "__main__":

    spinner.spin_async()
    spinner_create3.spin_async()
    sockets.run(app, port=5010, host="0.0.0.0", allow_unsafe_werkzeug=True)  # type: ignore
