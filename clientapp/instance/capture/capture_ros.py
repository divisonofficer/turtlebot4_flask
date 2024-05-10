from time import time, sleep
from rclpy.node import Node, Subscription
from sensor_msgs.msg import Image, LaserScan, Joy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from typing import List, Optional, Any, Dict
import os
import threading
import rclpy
from capture_type import ImageBytes, CaptureLiDAR, CaptureSingleScene
from capture_pb2 import CaptureTaskProgress
from slam_types import Pose3D
from capture_storage import CaptureStorage, CAPTURE_TEMP
from flask_socketio import SocketIO
from google.protobuf import json_format
import requests


from slam_source import SlamSource


class NoLidarSignal(Exception):
    pass


class NoImageSignal(Exception):
    pass


class NoPoseSignal(Exception):
    pass


class PolarizerError(Exception):
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
                for msg in [self.lidar_msg, self.pose_msg] + list(
                    self.image_msg_dict.values()
                ):
                    if msg.header.stamp.sec < self.timestamp:
                        self.messages_received = False
                        break

    def get_msg_received(self):
        if self.pose_msg is None:
            raise NoPoseSignal("No pose signal received")

        pose = Pose3D.from_msg(self.pose_msg.pose.pose)

        # get lidar information

        if self.lidar_msg is None:
            raise NoLidarSignal("No lidar signal received")

        lidar = CaptureLiDAR(self.lidar_msg)

        # get image msgs

        if len(self.image_msg_dict.values()) < len(self.image_topics):
            raise NoImageSignal(
                f"{len(self.image_msg_dict.values())} images received out of {len(self.image_topics)}"
            )
        image_list = [
            ImageBytes(msg, topic) for topic, msg in self.image_msg_dict.items()
        ]
        return pose, lidar, image_list

    def clear_image_dict(self):
        self.image_msg_dict = {}
        self.messages_received = False
        self.timestamp = time()


class Ell14:
    def __init__(self):
        pass

    def polarizer_turn(self, home=None):
        """
        Request ell14 server to turn the rotation stage of polarizer
        If home is True, then turn to home position
        """
        if home:
            requests.post("http://localhost/ell/angle/home")
            return
        if (
            requests.post(
                "http://localhost/ell/angle",
                json={"angle": 45},
                headers={"Content-Type": "application/json"},
            ).status_code
            != 200
        ):
            raise PolarizerError("Failed to turn polarizer")
        sleep(1)


class CaptureNode(Node):
    space_id: Optional[int]
    capture_id: int
    capture_msg: Optional[CaptureMessage] = None
    image_topics: List[str] = ["/oakd/rgb/preview/image_raw"]
    image_topics_polarized: List[str] = [
        "/jai_1600/channel_0",
        "/jai_1600/channel_1",
    ]
    subscriptions_image: Dict[str, Subscription]
    socketIO: SocketIO

    def __init__(self, storage: CaptureStorage, socketIO: SocketIO = None):
        super().__init__("client_capture_node")
        self.storage = storage
        self.socketIO = socketIO
        self.slam_source = SlamSource()
        self.ell = Ell14()
        self.lock = threading.Lock()

        self.init_sensor_subscriptions()

        self.space_id = None
        self.flag_capture_running = False

    ########################################################
    ### Public Methods
    #######################################################

    def set_capture_flag(self, flag: bool):
        with self.lock:
            self.flag_capture_running = flag

    def init_space(
        self, space_id: Optional[int] = None, space_name: Optional[str] = None
    ):
        """
        Prepare space information for capture
        If space_id is not provided, create a new space_id
        If space_id is provided, load the existing space information including map information
        """
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
            space_name = meta.space_name
            self.slam_source.request_map_load(space_id)

        self.space_id = space_id
        self.space_name = space_name
        return {
            "status": "success",
            "space_id": self.space_id,
        }

    def empty_space(self):
        """
        vacate existing space information and save the map
        """
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
        This function will trigger sub thread that runs capture queue for 20 times
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

    def run_capture_queue_single(self):
        """
        Request Single Scene Capture
        Robot captures a single scene and stores it in the storage
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
        """
        Check if the capture call is available
        To run the capture, the space_id should be initialized and the other capture should not be running
        return error message if the capture call is not available
        return None if the capture call is available
        """
        with self.lock:
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

    ########################################################
    ### Private Initialization
    #######################################################
    def init_sensor_subscriptions(self):
        # Subscribe to Pose and LiDAR topics
        self.subscription_pose = self.create_subscription(
            PoseWithCovarianceStamped, "/pose", self.pose_callback, 10
        )
        self.subscription_lidar = self.create_subscription(
            LaserScan, "/scan", self.lidar_callback, 3
        )

        self.subscription_joy = self.create_subscription(
            Joy, "/joy", self.joy_callback, qos_profile=10
        )

        self.subscriptions_image = {}

        self.publisher_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)

        self.image_topics = self.image_topics_polarized
        self.init_subscriptions()

    ########################################################
    ### Topic Callbacks
    #######################################################

    def pose_callback(self, msg):
        with self.lock:
            if not self.capture_msg:
                return

            self.capture_msg.pose_msg = msg
            self.capture_msg.check_messages_received()

    def lidar_callback(self, msg):
        with self.lock:
            if not self.capture_msg:
                return

            self.capture_msg.lidar_msg = msg
            self.capture_msg.check_messages_received()

    def joy_callback(self, msg: Joy):
        """
        Detect Joystick's active button and trigger the capture actions
        """
        if self.check_capture_call_available():
            return
        if msg.buttons[3] == 1:  # X button
            if msg.buttons[4] == 1:  # L1 Button
                self.run_capture_queue()
            else:
                self.run_capture_queue_single()

    def get_image_callback(self, topic):
        def callback(msg):
            if not self.capture_msg:
                return
            with self.lock:
                self.capture_msg.image_msg_dict[topic] = msg
                self.capture_msg.check_messages_received()

        return callback

    ########################################################
    ### Image Capture Tasks
    #######################################################

    def run_capture_queue(self):
        """
        Request Multiple Scene Capture on single position
        This function triggers capture thread
        For each iteration, the camera captures scene and robot turns right for about 10 degrees
        """
        self.set_capture_flag(True)
        self.flag_abort = False
        self.capture_id = int(time())

        scenes = []
        for scene_id in range(20):
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
        self.set_capture_flag(False)

    def capture_single_job(self):
        self.init_subscriptions()
        self.set_capture_flag(True)
        self.capture_id = int(time())
        scene = self.run_single_capture()
        if not scene:
            return None
        self.storage.store_captured_scene(
            space_id=self.space_id, capture_id=self.capture_id, scene_id=0, scene=scene
        )
        self.set_capture_flag(False)

    def run_polarized_capture(self):
        """
        Capture the multispectral camera four times with a 45-degree rotation of the polarizer.
        Repeat the process of rotating by 45 degrees and capturing the image.
        """
        image_list = []
        self.ell.polarizer_turn(home=True)
        for deg in [0, 45, 90, 135]:
            self.socket_progress(
                deg * 15 // 45 + 30, None, None, f"Capturing {deg} degrees"
            )
            self.get_logger().info(f"Capturing {deg} degrees")
            if self.capture_msg:
                self.capture_msg.clear_image_dict()
                while True:

                    if time() - self.capture_msg.timestamp > 10:
                        break
                    with self.lock:
                        if self.capture_msg.messages_received:
                            break
                    sleep(1)
                capture_msg = self.capture_msg
                for topic in self.image_topics_polarized:
                    if capture_msg.image_msg_dict[topic] is None:
                        continue
                    image_list.append(
                        ImageBytes(
                            capture_msg.image_msg_dict[topic],
                            topic + "/" + str(deg),
                        )
                    )

            self.ell.polarizer_turn()

        self.ell.polarizer_turn(home=True)
        return image_list

    def run_single_capture(self, scene_id=0):
        """
        vacate capture_msg and wait for all messages to arrive
        create CaptureSingleScene object with lidar, pose, and images
        get polarized images if available
        emit the scene to the socket
        """
        try:
            self.socket_progress(0, scene_id, None, f"Scene {scene_id} Capture started")
            # get map pose information
            self.capture_msg = CaptureMessage(self.image_topics)
            # 필요한 모든 메시지가 도착할 때까지 기다림

            while True:
                with self.lock:
                    if self.capture_msg.messages_received:
                        break
                if time() - self.capture_msg.timestamp > 10:
                    break
                sleep(1)
            pose, lidar, image_list = self.capture_msg.get_msg_received()
            self.socket_progress(
                30, scene_id, None, f"Scene {scene_id} Basic Capture completed"
            )
            # Capture polarized images
            if self.image_topics_polarized[0] in self.capture_msg.image_topics:
                image_list += self.run_polarized_capture()

            scene = CaptureSingleScene(
                capture_id=self.capture_id,
                scene_id=scene_id,
                timestamp=int(time()),
                robot_pose=pose,
                lidar_position=lidar,
                picture_list=image_list,
            )
            self.socket_progress(
                100, scene_id, None, f"Scene {scene_id}  Capture completed"
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
            self.socket_progress(100, scene_id, None, "No image signal received")
        except NoPoseSignal:
            self.get_logger().info("No pose signal received")
            self.socket_progress(100, scene_id, None, "No pose signal received")

        except NoLidarSignal:
            self.get_logger().info("No lidar signal received")
            self.socket_progress(100, scene_id, None, "No lidar signal received")
        except PolarizerError:
            self.get_logger().info("Polarizer Error")
            self.socket_progress(100, scene_id, None, "Polarizer Error")
            self.ell.polarizer_turn(home=True)
        return None

    def socket_progress(
        self,
        progress: int,
        scene_id: Optional[int],
        images: Optional[List[ImageBytes]],
        msg: Optional[str],
    ):
        if self.socketIO:
            self.socketIO.emit(
                "/progress",
                json_format.MessageToDict(
                    CaptureTaskProgress(
                        space_id=self.space_id if self.space_id else 0,
                        capture_id=self.capture_id,
                        progress=progress,
                        scene_id=scene_id if scene_id else 0,
                        message=msg if msg else "",
                        images=images,
                    )
                ),
                namespace="/socket",
            )

    def turn_right(self):
        twist = Twist()
        twist.angular.z = 0.5
        time_begin = time()
        while rclpy.ok():

            # Turn right the robot
            # self.publisher_cmd_vel.publish(twist)
            sleep(0.5)
            if time() - time_begin > 5:
                break
