from flask import Flask, Response, request
from flask_socketio import SocketIO


import os
import subprocess
import threading

from slam_opencv import stream
import rclpy
from slam_node import SlamApp


class SlamLaunch:
    def __init__(self, sockets):
        self.command = "ros2 launch turtlebot4_navigation slam.launch.py"
        self.process = None
        self.thread = None
        self.sockets = sockets

    def std_callback(self, msg):
        print(msg)

    def launch(self):
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

    def __del__(self):
        self.cancel_subprocess()


app = Flask(__name__)
sockets = SocketIO(app, cors_allowed_origins="*", async_mode="threading")


launch = SlamLaunch(sockets)
node = SlamApp(sockets, launch)


def spin_ros2_node():

    def spin():
        rclpy.spin(node)

    threading.Thread(target=spin).start()


@app.route("/launch", methods=["GET"])
def launch_slam():
    if not launch.process:
        launch.launch()
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


if __name__ == "__main__":
    spin_ros2_node()
    sockets.run(app, port=5010, host="0.0.0.0", allow_unsafe_werkzeug=True)
