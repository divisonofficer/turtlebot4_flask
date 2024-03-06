import threading
import time
from rclpy.node import Node
from flask_socketio import SocketIO


TOPIC_CHECK_LIST = {
    "oakd": "/oakd/rgb/preview/image_raw",
    "color": "color/preview/image",
    "lidar": "/stop_motor",
    "motion": "/cmd_vel",
    "battery": "/battery_state",
    "stereo": "/stereo/depth",
    "hmi": "/hmi/led",
}


import subprocess
import json


class RosTopicDiagnostic:
    def __init__(self, socketio: SocketIO):
        self.socketio = socketio

    def get_topics_dict_list(self):
        return [{"topic": topic, "type": type} for topic, type in self.TOPIC_NAME_TYPE]

    def monitor_ros2_services(self):
        ros2_node = Node("ros2_monitoring_node")

        # from ros2_node.json
        with open("ros2_node.json", "r") as f:
            self.NODE_CHECK_DICT = json.load(f)

        while True:
            # Example: Check if specific topics are available
            self.TOPIC_NAME_TYPE = ros2_node.get_topic_names_and_types()

            status = {topic_name: False for topic_name in TOPIC_CHECK_LIST.keys()}

            for topic_name, _ in self.TOPIC_NAME_TYPE:
                for key in TOPIC_CHECK_LIST.keys():
                    if TOPIC_CHECK_LIST[key] in topic_name:
                        status[key] = True

            # Emit status over SocketIO
            self.socketio.emit("status_monitoring", status, namespace="/manual")

            # Sleep for a bit before checking again
            time.sleep(10)

            pkg_status = {
                node: False for node in [x["name"] for x in self.NODE_CHECK_DICT]
            }

            # ps aux | grep ros2 subprocess
            # Execute the ps command and grep for ros2
            process = subprocess.Popen(["ps", "aux"], stdout=subprocess.PIPE)
            output, error = subprocess.Popen(
                ["grep", "ros2"], stdin=process.stdout, stdout=subprocess.PIPE
            ).communicate()

            # Decode output for Python 3
            output = output.decode("utf-8")
            for node in self.NODE_CHECK_DICT:

                if node["node"] + ".launch.py" in output:
                    pkg_status[node["name"]] = True

            self.socketio.emit("pkg_monitoring", pkg_status, namespace="/manual")

    # Run the monitoring function in a daemon thread

    def spin(self):
        if hasattr(self, "daemon_thread"):
            if self.daemon_thread.is_alive():
                return
        self.daemon_thread = threading.Thread(
            target=self.monitor_ros2_services, daemon=True
        )
        self.daemon_thread.start()

    def launch_node(self, pkg_name, node_name):
        # <project_directory>/bash/launch.sh
        path = "bash/launch.sh"
        output, error = subprocess.Popen(
            [path, pkg_name, node_name], stdout=subprocess.PIPE
        ).communicate()
        output = output.decode("utf-8")
        if error:
            return 500, error
        if "Error" in output:
            return 500, output
        return 200, None

    def kill_node(self, node_name):
        # bash/knode.sh <node_name>
        subprocess.Popen(["bash/knode.sh", node_name])
        return True

    def get_log(self, node_name):
        with open(f"/tmp/ros2_launch/log/{node_name}.log", "r") as f:
            return f.read()
