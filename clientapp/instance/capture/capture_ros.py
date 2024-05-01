from time import time, sleep
from rclpy.node import Node, Subscription
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from typing import List, Optional, Any, Dict
import os
import threading
import rclpy
from capture_type import ImageBytes, CaptureLiDAR, CaptureSingleScene
from slam_types import Pose3D
from capture_storage import CaptureStorage, CAPTURE_TEMP
from flask_socketio import SocketIO


from slam_source import SlamSource


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
    space_id: Optional[int]
    capture_id: int
    capture_msg: Optional[CaptureMessage]
    image_topics: List[str] = ["/oakd/rgb/preview/image_raw"]
    subscriptions_image: Dict[str, Subscription]
    socketIO: SocketIO

    def __init__(self, storage: CaptureStorage, socketIO: SocketIO = None):
        super().__init__("client_capture_node")

        # Subscribe to Pose and LiDAR topics
        self.subscription_pose = self.create_subscription(
            PoseWithCovarianceStamped, "/pose", self.pose_callback, 10
        )
        self.subscription_lidar = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, 3
        )
        self.subscriptions_image = {}

        self.publisher_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

        self.space_id = None
        self.flag_capture_running = False

        self.storage = storage
        self.socketIO = socketIO

        self.slam_source = SlamSource()

    def init_space(
        self, space_id: Optional[int] = None, space_name: Optional[str] = None
    ):
        if self.space_id:
            return {"status": "error", "message": "Space is already initialized"}

        if not space_id:
            space_id = int(time())
            if not space_name:
                space_name = f"Space {space_id}"
            self.storage.store_space_metadata(space_id, space_name)
            if self.slam_source.request_map_save(space_id):
                return {"status": "error", "message": "Failed to init slam"}
        else:
            meta = self.storage.get_space_metadata(space_id)
            if not meta:
                return {"status": "error", "message": "Space not found"}
            space_name = meta["space_name"]
            self.slam_source.request_map_load(space_id)

        self.space_id = space_id
        self.space_name = space_name
        return {
            "status": "success",
            "space_id": self.space_id,
        }

    def empty_space(self):
        if self.space_id:
            self.storage.store_space_metadata(self.space_id)
            self.slam_source.request_map_save_and_clean(self.space_id)
        else:
            return None
        self.space_id = None
        return True

    def init_subscriptions(self):
        """
        Initialize image subscriptions
        If there are existing subscriptions, destroy them
        for each image topic, create a new subscription
        """
        for topic in self.image_topics:
            if not topic in self.subscriptions_image:
                self.subscriptions_image[topic] = self.create_subscription(
                    Image, topic, self.get_image_callback(topic), 3
                )

    def run_capture_queue_thread(self):
        """
        Request Capture Rotation Queue
        Robot begins to rotate and capture images, asynchrnously
        """
        with self.check_capture_call_available() as error:
            if error:
                return error
        self.init_subscriptions()

        self._logger.info(f"Capture started on topic {self.image_topic}")

        thread = threading.Thread(target=self.run_capture_queue)
        thread.start()
        return {
            "status": "success",
            "space_id": self.space_id,
            "capture_id": self.capture_id,
        }

    def capture_single_job(self):
        self.init_subscriptions()
        self.capture_id = int(time())
        scene = self.run_single_capture()
        if not scene:
            return None
        self.storage.store_captured_scene(
            space_id=self.space_id, capture_id=self.capture_id, scene_id=0, scene=scene
        )

    def run_capture_queue_single(self):
        """
        Request Single Scene Capture

        """
        error = self.check_capture_call_available()
        if error:
            return error

        threading.Thread(target=self.capture_single_job).start()

        return {
            "status": "success",
            "space_id": self.space_id,
            "capture_id": self.capture_id,
        }

    def check_capture_call_available(self):
        if self.space_id is None:
            return {
                "status": "error",
                "message": "Space ID is not initialized",
            }
        if self.flag_capture_running:
            return {
                "status": "error",
                "message": "Capture is already running",
            }
        return None

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

            scene = self.run_single_capture(scene_id=scene_id)
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

    def run_single_capture(self, scene_id=0):
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
            scene = CaptureSingleScene(
                capture_id=self.capture_id,
                scene_id=scene_id,
                timestamp=int(time()),
                robot_pose=pose,
                lidar_position=lidar,
                picture_list=[
                    ImageBytes(msg, topic)
                    for topic, msg in capture_msg.image_msg_dict.items()
                ],
            )
            if self.socketIO:
                serialized_scene = scene.to_dict_light()
                serialized_scene["space_id"] = self.space_id
                self.socketIO.emit(
                    "/recent_scene", serialized_scene, namespace="/socket"
                )

            return scene
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
