from time import time, sleep
from rclpy.node import Node, Subscription
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from typing import List, Optional, Any
import os
import threading
import rclpy
from capture_type import ImageBytes, CaptureLiDAR, CaptureSingleScene
from slam_types import Pose3D
from capture_storage import CaptureStorage, CAPTURE_TEMP


class NoLidarSignal(Exception):
    pass


class NoImageSignal(Exception):
    pass


class NoPoseSignal(Exception):
    pass


class CaptureMessage:
    image_msg_dict: dict[str, Any]
    lidar_msg: Optional[LaserScan]
    pose_msg: Optional[PoseWithCovarianceStamped]
    messages_received: bool
    timestamp: float
    image_topics: List[str]

    def __init__(self, image_topics: List[str] = []):
        self.image_msg_dict = {}
        self.lidar_msg = None
        self.pose_msg = None
        self.messages_received = False
        self.timestamp = time()
        self.image_topics = image_topics

    def check_messages_received(self):
        if self.lidar_msg is not None and self.pose_msg is not None:
            if len(self.image_msg_dict.values()) == len(self.image_topics):
                self.messages_received = True


class CaptureNode(Node):
    space_id: int
    capture_id: int
    capture_msg: Optional[CaptureMessage]
    image_topics: List[str] = ["/oakd/rgb/preview/image_raw"]
    subscriptions_image: List[Subscription]

    def __init__(self, storage: CaptureStorage):
        super().__init__("client_capture_node")

        self.subscription_pose = self.create_subscription(
            PoseWithCovarianceStamped, "/pose", self.pose_callback, 10
        )
        self.subscription_lidar = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, 3
        )
        self.subscriptions_image = []

        self.publisher_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

        self.space_id = 0
        self.flag_capture_running = False

        self.storage = storage

        if not os.path.exists(f"{CAPTURE_TEMP}/{self.space_id}"):
            os.mkdir(f"{CAPTURE_TEMP}/{self.space_id}")

    def init_space(self, space_id: Optional[int] = None):
        if not space_id:
            space_id = int(time())
        self.space_id = space_id

    def init_subscriptions(self):
        """
        Initialize image subscriptions
        If there are existing subscriptions, destroy them
        for each image topic, create a new subscription
        """
        if len(self.subscriptions_image) > 0:
            for sub in self.subscriptions_image:
                sub.destroy()
            self.subscriptions_image = []

        self.subscriptions_image = []
        for topic in self.image_topics:
            self.subscriptions_image.append(
                self.create_subscription(
                    Image, topic, self.get_image_callback(topic), 3
                )
            )

    def run_capture_queue_thread(self):
        self.init_subscriptions()

        self._logger.info(f"Capture started on topic {self.image_topic}")

        thread = threading.Thread(target=self.run_capture_queue)
        thread.start()
        return {
            "status": "success",
            "space_id": self.space_id,
            "capture_id": self.capture_id,
        }

    def run_capture_queue_single(self):
        """
        Request Single Scene Capture
        return : bool
        """
        self.init_subscriptions()
        self.capture_id = int(time())
        scene = self.run_single_capture()
        if not scene:
            return None
        self.storage.store_captured_scene(self.space_id, self.capture_id, 0, scene)
        return {
            "status": "success",
            "space_id": self.space_id,
            "capture_id": self.capture_id,
        }

    def run_capture_queue(self):
        """
        Request Multiple Scene Capture on single position
        This function triggers capture thread
        """
        self.flag_capture_running = True
        self.flag_abort = False
        self.capture_id = int(time())

        scenes = []
        for scene_id in range(10):
            if self.flag_abort:
                self.get_logger().info("Capture aborted")
                break

            scene = self.run_single_capture()
            if not scene:
                continue
            self.storage.store_captured_scene(
                self.space_id, self.capture_id, scene_id, scene
            )
            scenes.append(scene)
            self.turn_right()

        self.flag_capture_running = False

    def pose_callback(self, msg):
        if not self.capture_msg:
            return
        self.capture_msg.pose_msg = msg
        self.capture_msg.check_messages_received()

    def lidar_callback(self, msg):
        if not self.capture_msg:
            return
        self.capture_msg.lidar_msg = msg
        self.capture_msg.check_messages_received()

    def get_image_callback(self, topic):
        def callback(msg):
            if not self.capture_msg:
                return
            self.capture_msg.image_msg_dict[topic] = msg
            self.capture_msg.check_messages_received()

        return callback

    def run_single_capture(self):
        try:
            # get map pose information
            self.capture_msg = CaptureMessage(self.image_topics)
            # 필요한 모든 메시지가 도착할 때까지 기다림
            while not self.capture_msg.messages_received:
                rclpy.spin_once(self, timeout_sec=0.1)
                if time() - self.capture_msg.timestamp > 10:
                    break
                sleep(1)
            capture_msg = self.capture_msg
            if capture_msg.pose_msg is None:
                raise NoPoseSignal("No pose signal received")

            pose = Pose3D.from_msg(capture_msg.pose_msg.pose.pose)

            # get lidar information

            if capture_msg.lidar_msg is None:
                raise NoLidarSignal("No lidar signal received")

            lidar = CaptureLiDAR(capture_msg.lidar_msg)

            # get image msgs

            if len(capture_msg.image_msg_dict.values()) < len(self.image_topics):
                raise NoImageSignal(
                    f"{len(capture_msg.image_msg_dict.values())} images received out of {len(capture_msg.image_topics)}"
                )

            return CaptureSingleScene(
                capture_id=0,
                scene_id=0,
                timestamp=int(time()),
                robot_pose=pose,
                lidar_position=lidar,
                picture_list=[
                    ImageBytes(msg, topic)
                    for topic, msg in capture_msg.image_msg_dict.items()
                ],
            )
        except NoImageSignal:
            self.get_logger().info("No image signal received")

        except NoPoseSignal:
            self.get_logger().info("No pose signal received")

        except NoLidarSignal:
            self.get_logger().info("No lidar signal received")
        return None

    def turn_right(self):
        twist = Twist()
        twist.angular.z = 0.5
        time_begin = time()
        while rclpy.ok():

            # Turn right the robot
            # self.publisher_cmd_vel.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
            sleep(0.1)
            if time() - time_begin > 5:
                break
