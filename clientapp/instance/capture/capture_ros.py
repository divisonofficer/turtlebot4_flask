import gc
import math
from time import time, sleep

import rclpy.action
from rclpy.node import Node, Subscription
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from typing import Optional, Dict
import threading
import rclpy
from capture_jai_controller import CaptureJaiController
from capture_type import CaptureSingleScene
from capture_pb2 import CaptureTaskProgress, CaptureMessageDef, CaptureTopicTimestampLog
from capture_storage import CaptureStorage
from flask_socketio import SocketIO
from capture_msg_def import (
    CaptureMessageDefinition,
    CaptureMode,
    ScenarioHyperParameter,
)

from capture_state import CaptureMessage
import requests


from slam_source import SlamSource
from capture_scenario import PolarizerError, CaptureSingleScenario
from socket_progress import SocketProgress
from irobot_create_msgs.action import RotateAngle


class Ell14:
    def __init__(self):
        pass

    def polarizer_turn(self, home=None, angle=None):
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
                json={"angle": angle if angle else 45},
                headers={"Content-Type": "application/json"},
            ).status_code
            != 200
        ):
            raise PolarizerError("Failed to turn polarizer")
        sleep(1)


class CaptureNode(Node):
    space_id: Optional[int]
    capture_id: Optional[int] = None
    capture_msg: Optional[CaptureMessage] = None
    use_slam: bool = False

    messageDef = CaptureMessageDefinition()
    scenario_hyper = ScenarioHyperParameter()

    subscriptions_image: Dict[str, Subscription]
    socketIO: Optional[SocketIO]
    jai_controller = CaptureJaiController()

    def __init__(
        self, storage: CaptureStorage, socketIO: Optional[SocketIO], isSubNode=False
    ):
        if isSubNode:
            node_name = "client_capture_node" + str(time())
        else:
            node_name = "client_capture_node"
        super().__init__(node_name=node_name)  # type: ignore
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
            self.capture_id = None
            # if self.messageDef.oakd.enabled:
            #     self.oakd_camera_command(start=flag)

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
            self.storage.store_space_metadata(space_id, space_name)
            if use_slam:

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

        # requests.post("http://localhost/jai/device/init/all")

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
            if self.use_slam:
                self.slam_source.request_map_save_and_clean(str(self.space_id))
        else:
            return None
        self.space_id = None
        return True

    def init_capture_thread(self, capture_mode: Optional[str] = None):
        """
        Request Capture Rotation Queue
        Robot begins to rotate and capture images, asynchronously
        This function will trigger sub thread that runs capture queue for 20 times
        """

        if capture_mode is None:
            capture_mode_idx = int(self.scenario_hyper.CaptureQueueMode.value)
            capture_mode = CaptureMode[:][capture_mode_idx]

        error = self.check_capture_call_available()
        if error:
            return error
        self.capture_id = int(time())

        thread = threading.Thread(
            target=self.run_capture_thread, args=[capture_mode, self.capture_id]
        )
        thread.start()

        return {
            "status": "success",
            "space_id": self.space_id,
            "capture_id": self.capture_id,
        }

    def run_capture_thread(
        self, capture_mod: str = CaptureMode.Single, capture_id: int = -1
    ):
        try:
            #### Pre Capture Task
            if self.messageDef.Ellipsis.enabled:
                self.ell.polarizer_turn(home=True)
            self.set_capture_flag(True)

            self.run_capture_queue(capture_id, capture_mod)

            ### Post Capture Task
            if self.messageDef.Ellipsis.enabled:
                self.ell.polarizer_turn(home=True)
            self.set_capture_flag(False)
            if self.scenario_hyper.JaiAutoExpose.value == 1:
                self.jai_controller.unfreeze_auto_exposure()
        except Exception as e:
            if self.messageDef.Ellipsis.enabled:
                self.ell.polarizer_turn(home=True)
            self.set_capture_flag(False)

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

        self.client_create3_rotate = rclpy.action.client.ActionClient(
            self, RotateAngle, "/rotate_angle"
        )
        self.jai_controller.publisher_jai_trigger = self.create_publisher(
            Bool, "/jai_1600/stream_trigger", 1
        )
        self.client_oakd_start = self.create_client(Trigger, "/oakd/start_camera")
        self.client_oakd_stop = self.create_client(Trigger, "/oakd/stop_camera")

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
                self.init_capture_thread("queue")
            else:
                self.init_capture_thread("single")

    ########################################################
    ### Image Capture Tasks
    #######################################################

    def run_capture_queue(self, capture_id: int, capture_mod: str):
        if not self.space_id:
            return
        """
        Request Multiple Scene Capture on single position
        This function triggers capture thread
        For each iteration, the camera captures scene and robot turns right for about 10 degrees
        """
        self.flag_abort = False
        self.capture_id = int(time())

        space_id = self.space_id

        self.socket_progress(
            0,
            uuid=f"{capture_id}/root",
            action=CaptureTaskProgress.Action.INIT,
            msg="Begin Rotating Capture",
            capture_id=capture_id,
        )
        capture_count = round(self.scenario_hyper.RotationQueueCount.value)
        if capture_mod == CaptureMode.Single:
            capture_count = 1

        if capture_mod == CaptureMode.DrivingCapture:
            capture_count = round(
                self.scenario_hyper.DriveDistance.value
                / self.scenario_hyper.DriveMargin.value
            )

        if capture_mod == CaptureMode.DrivingSideScanning:
            capture_count *= int(self.scenario_hyper.SideAngleCount.value)
            self.turn_right(-self.scenario_hyper.SideAngleRange.value / 2 + 90)

        for scene_id in range(capture_count):
            if self.flag_abort:
                self.get_logger().info("Capture aborted")
                self.socket_progress(
                    100,
                    uuid=f"{capture_id}/root",
                    action=CaptureTaskProgress.Action.ERROR,
                    msg="Capture Aborted",
                    capture_id=capture_id,
                )
                break
            self.socket_progress(
                round(scene_id * (100 / self.scenario_hyper.RotationQueueCount.value)),
                uuid=f"{capture_id}/root",
                action=CaptureTaskProgress.Action.ACTIVE,
                msg=f"{scene_id}th take",
                capture_id=capture_id,
            )

            if self.scenario_hyper.JaiAutoExpose.value == 1:
                params = self.jai_controller.freeze_auto_exposure()
                if isinstance(params, dict):
                    if "jai_1600_left" in params:
                        print(
                            "Jai 1600 Left Auto Exposure: ",
                            params["jai_1600_left"]["ExposureTime"],
                        )
                    if "jai_1600_right" in params:
                        print(
                            "Jai 1600 Right Auto Exposure: ",
                            params["jai_1600_right"]["ExposureTime"],
                        )

            self.capture_msg = CaptureMessage(self.messageDef, self.scenario_hyper)
            scene = self.run_single_capture_scene(capture_id, scene_id)
            timestamp_log = self.capture_msg.timestamp_log
            del self.capture_msg
            if not scene:
                continue

            self.socket_progress(
                int(scene_id * 20 / self.scenario_hyper.RotationQueueCount.value) + 3,
                uuid=f"{capture_id}/root",
                action=CaptureTaskProgress.Action.ACTIVE,
                msg=f"{scene_id}th take done. Turn right",
                capture_id=capture_id,
            )
            time_begin_turn = time()
            if capture_mod == CaptureMode.RotatingCapture:
                if scene_id != self.scenario_hyper.RotationQueueCount.value - 1:
                    self.turn_right()
            elif capture_mod == CaptureMode.DrivingCapture:
                if (
                    scene_id
                    < self.scenario_hyper.DriveDistance.value
                    / self.scenario_hyper.DriveMargin.value
                    - 1
                ):
                    self.drive_forward()
            elif capture_mod == CaptureMode.DrivingSideScanning:
                if (scene_id + 1) % int(self.scenario_hyper.SideAngleCount.value) == 0:
                    self.turn_right(-self.scenario_hyper.SideAngleRange.value / 2 - 90)
                    self.drive_forward()
                    self.turn_right(-self.scenario_hyper.SideAngleRange.value / 2 + 90)
                else:
                    self.turn_right(
                        self.scenario_hyper.SideAngleRange.value
                        / (self.scenario_hyper.SideAngleCount.value - 1)
                    )

            if self.scenario_hyper.JaiAutoExpose.value == 1:
                self.jai_controller.unfreeze_auto_exposure()
            if self.messageDef.Ellipsis.enabled:
                self.ell.polarizer_turn(home=True)
            timestamp_log.logs.append(
                CaptureTopicTimestampLog.TimestampLog(
                    topic="Moving",
                    timestamp=time_begin_turn,
                    delay_to_system=time() - time_begin_turn,
                )
            )

            time_begin_store = time()
            self.storage.store_captured_scene(
                space_id=space_id, capture_id=capture_id, scene_id=scene_id, scene=scene
            )
            timestamp_log.logs.append(
                CaptureTopicTimestampLog.TimestampLog(
                    topic="Storage",
                    timestamp=time_begin_store,
                    delay_to_system=time() - time_begin_store,
                )
            )
            if self.socketIO:
                self.socketIO.emit(
                    "/timestamp_logs",
                    timestamp_log.SerializeToString(),
                    namespace="/socket",
                )
            del timestamp_log

        self.socket_progress(
            100,
            uuid=f"{capture_id}/root",
            action=CaptureTaskProgress.Action.DONE,
            msg="Rotating Capture Done",
            capture_id=capture_id,
        )

    def run_single_capture_scene(self, capture_id: int, scene_id):
        if not self.space_id:
            raise Exception("Space ID is not initialized")
        if not self.capture_msg:
            raise Exception("Capture is not running")
        scene = CaptureSingleScenario(
            self.jai_controller,
            self.socket_progress,
            [self.space_id, capture_id, scene_id],
            self.capture_msg,
            self.lock,
            self.socketIO,
            self.ell,
            self.storage,
        ).run_single_capture()
        gc.collect()
        return scene

    def turn_right(self, angle=None):
        action = RotateAngle.Goal()
        if angle is not None:
            action.angle = math.radians(angle)
        else:
            action.angle = 2 * math.pi / self.scenario_hyper.RotationQueueCount.value
        requests.post(
            "http://localhost/slam/create3/action/rotate",
            json={
                "angle": action.angle,
            },
        )

    def drive_forward(self):
        requests.post(
            "http://localhost/slam/create3/action/drive",
            json={"distance": self.scenario_hyper.DriveMargin.value},
        )

    def oakd_camera_command(self, start: Optional[bool] = None):
        if start == True:
            self.client_oakd_start.call(Trigger.Request())
        if start == False:
            self.client_oakd_stop.call(Trigger.Request())


import argparse
import json

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--space_id", type=int, required=True)
    parser.add_argument("--capture_id", type=int, required=True)
    parser.add_argument("--topic_option_dict", type=str, required=True)

    args = parser.parse_args()

    rclpy.init()
    node = CaptureNode(CaptureStorage(), None, isSubNode=True)
    node.space_id = args.space_id
    node.capture_id = args.capture_id
    topic_option_dict: dict[str, bool] = json.loads(args.topic_option_dict)
    for topic, status in topic_option_dict.items():
        node.update_capture_topic(topic, status)

    def spin():
        rclpy.spin(node)

    thread = threading.Thread(target=spin)
    thread.start()

    thread.join()
