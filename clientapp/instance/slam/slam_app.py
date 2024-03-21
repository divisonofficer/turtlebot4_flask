from flask import Flask, Response
import os
import subprocess
import threading

from slam_opencv import slam_map_opencv, stream
from rclpy.node import Node
import rclpy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped


class SlamApp(Node):
    def __init__(self):
        super().__init__("client_slam_node")
        self.publisher_ = self.create_publisher(String, "/client/slam", 10)
        self.timer_ = self.create_timer(20, self.timer_callback)
        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )
        self.pos_subscription = self.create_subscription(
            PoseWithCovarianceStamped, "/pose", self.position_callback, 10
        )
        self.__position = None
        self.__map_origin = None

    def map_callback(self, msg: OccupancyGrid):
        slam_map_opencv(msg, self.__position)
        self.__map_origin = msg.info.origin.position

    def position_callback(self, msg: PoseWithCovarianceStamped):
        self.__position = msg

    def timer_callback(self):
        msg = String()
        msg.data = "Running"
        self.publisher_.publish(msg)
        self.get_logger().info("Published 'Running' on /client/slam")
        if self.__position:
            self.get_logger().info(f"Robot Pose: {self.__position.pose.pose}")
        if self.__map_origin:
            self.get_logger().info(f"Map Origin: {self.__map_origin}")


def spin_ros2_node():
    rclpy.init()
    node = SlamApp()

    def spin():
        rclpy.spin(node)

    threading.Thread(target=spin).start()


class SlamLaunch:
    def __init__(self):
        self.command = "ros2 launch turtlebot4_navigation slam.launch.py"
        self.process = None
        self.thread = None

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
                output = self.process.stdout.readline().decode().strip()
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
        # ps aux | grep -i "ros2 launch turtlebot4_navigation slam.launch.py" | awk '{print $2}' | xargs kill -9
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


launch = SlamLaunch()


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


@app.route("/map")
def map_preview_stream():
    return Response(
        stream.generate_preview(), mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    spin_ros2_node()
    app.run(port=5010)
