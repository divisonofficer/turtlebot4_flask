from time import time, sleep
from rclpy.node import Node, Subscription
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from typing import List, Optional, Dict, TypeVar
import threading
import rclpy
from capture_type import ImageBytes, CaptureSingleScene
from capture_pb2 import CaptureTaskProgress, CaptureMessageDef
from capture_storage import CaptureStorage
from flask_socketio import SocketIO
from capture_msg_def import CaptureMessageDefinition

from capture_state import CaptureMessage
import requests


from slam_source import SlamSource
from capture_scenario import PolarizerError, CaptureSingleScenario
from socket_progress import SocketProgress


MsgType = TypeVar("MsgType")


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
    use_slam: bool = False

    messageDef = CaptureMessageDefinition()

    subscriptions_image: Dict[str, Subscription]
    socketIO: SocketIO

    def __init__(self, storage: CaptureStorage, socketIO: SocketIO):
        super().__init__(node_name="client_capture_node")  # type: ignore
        self.storage = storage
        self.socketIO = socketIO
        self.socket_progress = SocketProgress(socketIO, self.get_logger())
        self.slam_source = SlamSource()
        self.ell = Ell14()
        self.lock = threading.Lock()

        self.init_sensor_subscriptions()

        self.space_id = None
        self.flag_capture_running = False

    ########################################################
    ### Public Methods
    #######################################################

    def update_capture_topic(self, topic_group_name: str, status: bool):
        self.messageDef.update_enable(topic_group_name, status)

    def get_msg_group_info(self):
        return self.messageDef.to_dict()

    def get_status(self):
        return {
            "space_ready": self.space_id is not None,
            "space_id": self.space_id,
            "space_name": self.space_name if self.space_id else None,
            "message_def": self.get_msg_group_info(),
            "use_slam": self.use_slam,
        }

    def set_capture_flag(self, flag: bool):
        with self.lock:
            self.flag_capture_running = flag

    def init_space(
        self,
        space_id: Optional[int] = None,
        space_name: Optional[str] = None,
        use_slam: Optional[bool] = None,
    ):
        """
        Prepare space information for capture
        If space_id is not provided, create a new space_id
        If space_id is provided, load the existing space information including map information
        """
        self.use_slam = use_slam if use_slam else False
        if self.space_id:
            return {"status": "error", "message": "Space is already initialized"}

        if not space_id:
            space_id = int(time())
            if not space_name:
                space_name = f"Space {space_id}"
            if use_slam:
                self.storage.store_space_metadata(space_id, space_name)
                if self.slam_source.request_map_save(str(space_id)):
                    return {"status": "error", "message": "Failed to init slam"}
        else:
            meta = self.storage.get_space_metadata(space_id)
            if not meta:
                return {"status": "error", "message": "Space not found"}
            space_name = meta.space_name
            self.use_slam = meta.use_slam
            if self.use_slam:
                self.slam_source.request_map_load(str(space_id))

        self.space_id = space_id
        self.space_name = space_name
        self.messageDef.slam.enabled = self.use_slam
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
            self.slam_source.request_map_save_and_clean(str(self.space_id))
        else:
            return None
        self.space_id = None
        return True

    def run_capture_queue_thread(self):
        """
        Request Capture Rotation Queue
        Robot begins to rotate and capture images, asynchronously
        This function will trigger sub thread that runs capture queue for 20 times
        """
        error = self.check_capture_call_available()
        if error:
            return error

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
    def get_msg_callback(self, msgDef: CaptureMessageDef):
        def callback(msg):
            self.capture_msg.update_msg(msgDef.topic, msg) if self.capture_msg else None

        return callback

    def init_sensor_subscriptions(self):
        self.subscription_joy = self.create_subscription(
            Joy, "/joy", self.joy_callback, qos_profile=10
        )
        for group in self.messageDef.entries():
            for msg_def in group.messages:
                setattr(
                    self,
                    "subscription_" + msg_def.topic.replace("/", "_"),
                    self.create_subscription(
                        self.messageDef.resolve_ros_type(msg_def.ros_msg_type),
                        msg_def.topic,
                        self.get_msg_callback(msg_def),
                        10,
                    ),
                )

        self.publisher_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.publisher_jai_trigger = self.create_publisher(
            Bool, "/jai_1600/stream_trigger", 1
        )

    ########################################################
    ### Topic Callbacks
    #######################################################

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

    ########################################################
    ### Image Capture Tasks
    #######################################################

    def store_scene_thread(self, scene: CaptureSingleScene):
        threading.Thread(
            target=self.storage.store_captured_scene,
            args=(self.space_id, self.capture_id, scene.scene_id, scene),
        ).start()

    def run_capture_queue(self):
        if not self.space_id:
            return
        """
        Request Multiple Scene Capture on single position
        This function triggers capture thread
        For each iteration, the camera captures scene and robot turns right for about 10 degrees
        """
        self.set_capture_flag(True)
        self.flag_abort = False
        self.capture_id = int(time())

        self.socket_progress(
            0,
            uuid=f"{self.capture_id}/root",
            action=CaptureTaskProgress.INIT,
            msg="Begin Rotating Capture",
            capture_id=self.capture_id,
        )
        scenes = []
        for scene_id in range(20):
            if self.flag_abort:
                self.get_logger().info("Capture aborted")
                self.socket_progress(
                    100,
                    uuid=f"{self.capture_id}/root",
                    action=CaptureTaskProgress.ERROR,
                    msg="Capture Aborted",
                    capture_id=self.capture_id,
                )
                break
            self.socket_progress(
                scene_id * 5,
                uuid=f"{self.capture_id}/root",
                action=CaptureTaskProgress.DONE,
                msg=f"{scene_id}th take",
                capture_id=self.capture_id,
            )

            self.capture_msg = CaptureMessage(self.messageDef)
            scene = CaptureSingleScenario(
                self.open_jai_stream,
                self.socket_progress,
                [self.space_id, self.capture_id, scene_id],
                self.capture_msg,
                self.lock,
                self.socketIO,
                self.ell,
            ).run_single_capture()
            if not scene:
                continue
            scenes.append(scene)
            self.store_scene_thread(scene)

            self.socket_progress(
                scene_id * 5 + 3,
                uuid=f"{self.capture_id}/root",
                action=CaptureTaskProgress.DONE,
                msg=f"{scene_id}th take done. Turn right",
                capture_id=self.capture_id,
            )
            self.turn_right()
        self.socket_progress(
            100,
            uuid=f"{self.capture_id}/root",
            action=CaptureTaskProgress.DONE,
            msg="Rotating Capture Done",
            capture_id=self.capture_id,
        )
        self.set_capture_flag(False)

    def capture_single_job(self):
        if not self.space_id:
            return
        self.set_capture_flag(True)
        self.capture_id = int(time())
        self.capture_msg = CaptureMessage(self.messageDef)
        scene = CaptureSingleScenario(
            self.open_jai_stream,
            self.socket_progress,
            [self.space_id, self.capture_id, 0],
            self.capture_msg,
            self.lock,
            self.socketIO,
            self.ell,
        ).run_single_capture()
        if not scene:
            return None
        self.storage.store_captured_scene(
            space_id=self.space_id, capture_id=self.capture_id, scene_id=0, scene=scene
        )
        self.set_capture_flag(False)

    def open_jai_stream(self, open: bool):
        self.publisher_jai_trigger.publish(Bool(data=open))

    def turn_right(self):
        twist = Twist()
        twist.angular.z = 0.25
        time_begin = time()
        while rclpy.ok():

            # Turn right the robot
            self.publisher_cmd_vel.publish(twist)
            sleep(0.2)
            if time() - time_begin > 0.5:
                break
