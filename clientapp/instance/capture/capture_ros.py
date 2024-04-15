from time import time, sleep
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from typing import Union
import os
import threading
import json
import cv2
import rclpy

CAPTURE_TEMP = "/tmp/oakd_capture"
if not os.path.exists(CAPTURE_TEMP):
    os.mkdir(CAPTURE_TEMP)


class CaptureNode(Node):
    space_id: int
    capture_id: int

    image_topic: str = "/oakd/rgb/preview/image_raw"

    def __init__(self):
        super().__init__("client_capture_node")

        self.subscription_pose = self.create_subscription(
            PoseWithCovarianceStamped, "/pose", self.pose_callback, 10
        )
        self.subscription_lidar = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, 3
        )
        self.subscription_mono_image = None

        self.publisher_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

        self.space_id = 0
        self.flag_capture_running = False
        if not os.path.exists(f"{CAPTURE_TEMP}/{self.space_id}"):
            os.mkdir(f"{CAPTURE_TEMP}/{self.space_id}")

    def run_capture_queue_thread(self):
        if self.subscription_mono_image:
            self.destroy_subscription(self.subscription_mono_image)
            self.subscription_mono_image = None

        self.subscription_mono_image = self.create_subscription(
            Image, self.image_topic, self.mono_image_callback, 3
        )

        self._logger.info(f"Capture started on topic {self.image_topic}")

        thread = threading.Thread(target=self.run_capture_queue)
        thread.start()
        return {
            "status": "success",
            "space_id": self.space_id,
            "capture_id": self.capture_id,
        }

    def run_capture_queue(self):
        self.flag_capture_running = True
        self.flag_abort = False
        self.capture_id = int(time())

        capture_dir = f"{CAPTURE_TEMP}/{self.space_id}/{self.capture_id}"
        if not os.path.exists(capture_dir):
            os.mkdir(capture_dir)
        scenes = []
        for scene_id in range(10):
            if self.flag_abort:
                self.get_logger().info("Capture aborted")
                break
            try:
                scene_dir = f"{capture_dir}/{scene_id}"
                os.mkdir(scene_dir)
                scene = self.run_single_capture()
                cv2.imwrite(f"{scene_dir}/oakd_mono.jpg", scene.picture_oakd_mono.image)
                with open(f"{scene_dir}/oakd_mono_data.json", "w") as f:
                    json.dump(scene.picture_oakd_mono.data, f)
                with open(f"{scene_dir}/meta.json", "w") as f:
                    json.dump(scene.to_dict(), f)
                scenes.append(scene)
                self.turn_right()
            except NoImageSignal:
                self.get_logger().info("No image signal received")

            except NoPoseSignal:
                self.get_logger().info("No pose signal received")

            except NoLidarSignal:
                self.get_logger().info("No lidar signal received")

        self.flag_capture_running = False

    def pose_callback(self, msg):
        self.pose_msg = msg
        # self.get_logger().info("Pose signal received")
        self.check_messages_received()

    def lidar_callback(self, msg):
        self.lidar_msg = msg
        # self.get_logger().info("Lidar signal received")
        self.check_messages_received()

    def mono_image_callback(self, msg):
        self.mono_image_msg = msg
        # self.get_logger().info("Mono image signal received")
        self.check_messages_received()

    def check_messages_received(self):
        if self.pose_msg and self.lidar_msg and self.mono_image_msg:
            self.messages_received = True

    def run_single_capture(self):

        # get map pose information
        self.messages_received = False
        self.pose_msg = None
        self.lidar_msg = None
        self.mono_image_msg = None
        self.timestamp = time()
        # 필요한 모든 메시지가 도착할 때까지 기다림
        while not self.messages_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time() - self.timestamp > 10:
                break
            sleep(1)

        if self.pose_msg is None:
            raise NoPoseSignal("No pose signal received")

        pose = Pose3D.from_msg(self.pose_msg.pose.pose)

        # get lidar information

        if self.lidar_msg is None:
            raise NoLidarSignal("No lidar signal received")

        lidar = CaptureLiDAR(
            self.lidar_msg.angle_min,
            self.lidar_msg.angle_max,
            self.lidar_msg.angle_increment,
            self.lidar_msg.ranges.tolist(),
        )

        # get oakd mono image

        if self.mono_image_msg is None:
            raise NoImageSignal("No image signal received")

        mono_image = ImageBytes(self.mono_image_msg, topic=self.image_topic)

        return CaptureSingleScene(
            capture_id=0,
            scene_id=0,
            timestamp=int(time()),
            robot_pose=pose,
            lidar_position=lidar,
            picture_oakd_mono=mono_image,
        )

    def turn_right(self):
        twist = Twist()
        twist.angular.z = 0.5
        self.publisher_cmd_vel.publish(twist)
        rclpy.spin_once(self, timeout_sec=5)
