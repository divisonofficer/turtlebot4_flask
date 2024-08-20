from typing import List

from sensor_msgs.msg import PointCloud2
import numpy as np
import rclpy
from rclpy.node import Node
from videostream import VideoStream
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from rclpy.executors import SingleThreadedExecutor

from points2depth import Point2Depth


class Point2DepthNode(Node):

    def timer_callback(self):
        # print(self.get_topic_names_and_types())
        pass

    def __init__(self):
        super().__init__("lidar_to_depth_image")
        self.timer = self.create_timer(1, self.timer_callback)
        self.p2depth = Point2Depth()

        self.depth_viz_stream = VideoStream()

        self.p2depth.image_callback = self.depth_viz_stream.cv_ndarray_callback

        self.p2depth.set_intrinsics(
            np.array(
                [
                    [1764.0609770407675, 0, 712.9065552882782],
                    [0, 1764.8799163219228, 562.1627377324658],
                    [0, 0, 1],
                ]
            )
        )
        self.p2depth.set_transform(
            np.array(
                [
                    [1, 0, 0, -180],
                    [0, 1, 0, -180],
                    [0, 0, 1, -20],
                ]
            )
        )

        self.p2depth.set_image_resolution(1440, 1080)

        self.subscription = self.create_subscription(
            PointCloud2,
            "/ouster/points",  # 실제 토픽 이름으로 변경
            self.p2depth.pointcloud2depthImage,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            ),
        )
        self.subscription  # prevent unused variable warning


from flask import Flask, Response
import threading

app = Flask(__name__)
depthNode: Point2DepthNode


def spin():
    rclpy.init()
    global depthNode
    depthNode = Point2DepthNode()

    def spin_node(nodes: List[Node]):
        executor = SingleThreadedExecutor()
        for node in nodes:
            executor.add_node(node)
        executor.spin()

    threading.Thread(target=spin_node, args=([[depthNode]])).start()


@app.route("/depth_image")
def depth_image():
    return Response(
        depthNode.depth_viz_stream.generate_preview(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


if __name__ == "__main__":
    spin()
    app.run(port=5020)
