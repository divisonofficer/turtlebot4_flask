from asyncio import Future
import json
from typing import Callable, List, Literal, Optional
from dataclasses import dataclass, asdict, field
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
import threading
from stereo_storage import (
    HDRCaptureItem,
)
import time
from stereo_hdr_queue import StereoHDRQueue
from geometry_msgs.msg import Pose
from rclpy.action import ActionClient
from irobot_create_msgs.action import RotateAngle, DriveDistance
import rclpy
from rclpy.client import Client
from jai_rosbridge.action import HDRTrigger
from std_srvs.srv import Trigger
from rclpy.action.client import ClientGoalHandle


class JaiTimeoutError(Exception):
    pass


class RangerTimeoutError(Exception):
    pass


class LiDARTimeoutError(Exception):
    pass


class GoalRejectedError(Exception):
    pass


class TapoTimeoutError(Exception):
    pass


class PoseNoDataError(Exception):
    pass


class JaiHDRCaptureAgent:
    @dataclass
    class Config:
        rotate_angle: int = 90
        capture_cnt: int = 6
        side_move_cnt: int = 1
        side_move_distance: float = 0.1
        timeout: int = 10
        timeout_hdr: int = 10
        ROOT: str = "/home/cglab/project/turtlebot4_flask/clientapp/tmp/stereo/hdr/"
        lidar: bool = True

    @dataclass
    class Log:
        @dataclass
        class ProgressRoot:
            idx: int
            status: Literal["ready", "running", "done", "error"]
            task: Literal["rotate", "hdr", "ambient"] = "rotate"

        progress_root: ProgressRoot = field(
            default_factory=lambda: JaiHDRCaptureAgent.Log.ProgressRoot(0, "ready")
        )
        progress_sub: dict = field(default_factory=lambda: {"type": "hdr", "idx": 0})
        hdr_error_msgs: List[dict] = field(default_factory=list)

    def update_config(self, config: dict):
        if self.hdr_thread is not None and self.hdr_thread.is_alive():
            raise Exception("HDR capture is running")
        for key, value in config.items():
            setattr(self.config, key, value)
        return asdict(self.config)

    def __init__(
        self,
        stereo_hdr_queue: StereoHDRQueue,
        action_client_rotate: ActionClient,
        action_client_hdr_trigger: ActionClient,
        action_client_drive: ActionClient,
        service_client_tapo_trigger: tuple[Client, Client],
        config: Config,
        topic_callback: Callable,
        log_callback: Callable,
    ):
        self.stereo_hdr_queue = stereo_hdr_queue
        self.config = config
        self.hdr_thread: Optional[threading.Thread] = None
        self.topic_callback = topic_callback
        self.log_callback = log_callback
        self.pose_list: List[Pose] = []
        self.action_client = action_client_rotate
        self.action_client_hdr_trigger = action_client_hdr_trigger
        self.service_client_tapo_on, self.service_client_tapo_off = (
            service_client_tapo_trigger
        )
        self.action_client_drive = action_client_drive
        self.log = self.Log()

    def capture_thread(self, space_id: str):
        if self.hdr_thread is not None and self.hdr_thread.is_alive():
            raise Exception("HDR capture is already running")
        self.hdr_thread = threading.Thread(
            target=self.capture_hdr, args=(space_id,), daemon=True
        )
        self.hdr_thread.start()

    def capture_hdr(self, space_id: str):
        """
        Clear HDR Queue
        """
        self.trigger_tapo(True)
        self.stereo_hdr_queue.clear()
        self.log.progress_root.status = "running"
        angle_current = 0

        for side_idx in range(self.config.side_move_cnt):
            if self.config.capture_cnt > 1:
                try:
                    self.turn_right(-self.config.rotate_angle / 2)
                except PoseNoDataError as e:
                    self.log.hdr_error_msgs.append(
                        {
                            "type": "error_no_pose",
                            "data": {"cause": type(e).__name__, "msg": str(e)},
                        }
                    )
                    self.publish_log()

            try:

                for i in range(self.config.capture_cnt):
                    frame_id = time.strftime("%H_%M_%S_", time.localtime()) + str(
                        int((time.time() % 1) * 1000)
                    ).zfill(3)
                    # Logger : Turn Right
                    self.log.progress_root.idx = i + side_idx * self.config.capture_cnt
                    self.log.progress_root.task = "rotate"
                    self.publish_log()
                    try:
                        if i != 0:
                            angle_current += 1
                            self.turn_right(
                                self.config.rotate_angle / (self.config.capture_cnt - 1)
                            )
                    except PoseNoDataError as e:
                        self.log.hdr_error_msgs.append(
                            {
                                "type": "error_no_pose",
                                "data": {"cause": type(e).__name__, "msg": str(e)},
                            }
                        )
                        self.publish_log()
                    # get topics from HDR Queue

                    self.get_hdr_frame(f"{space_id}/{frame_id}", i)

            except (
                JaiTimeoutError,
                RangerTimeoutError,
                GoalRejectedError,
                TapoTimeoutError,
                LiDARTimeoutError,
            ) as e:
                self.log.hdr_error_msgs.append(
                    {
                        "type": "error_crash",
                        "data": {"cause": type(e).__name__, "msg": str(e)},
                    }
                )
                self.publish_log()

            except Exception as e:
                self.log.hdr_error_msgs.append({"msg": str(e)})
                self.publish_log()

            if self.config.capture_cnt > 1:
                try:
                    self.turn_right(
                        -(
                            angle_current
                            * self.config.rotate_angle
                            / (self.config.capture_cnt - 1)
                            - self.config.rotate_angle / 2
                        )
                    )
                except PoseNoDataError as e:
                    self.log.hdr_error_msgs.append(
                        {
                            "type": "error_no_pose",
                            "data": {"cause": type(e).__name__, "msg": str(e)},
                        }
                    )
                    self.publish_log()

            angle_current = (
                0  # Reset angle to 0 after returning to the original position
            )
            if side_idx != self.config.side_move_cnt - 1:
                self.drive_side(self.config.side_move_distance)

        self.log.progress_root.status = "done"
        self.publish_log()

    def get_hdr_frame(self, frame_id, idx: int):
        time_begin = time.time()
        hdr_frame = None
        """
        Get HDR Frame
        """
        self.log.progress_root.task = "hdr"

        self.publish_log()
        self.trigger_hdr_action(frame_id + "/hdr")
        """
        Get Ambient Frame
        """
        self.trigger_tapo(False)
        time.sleep(1)
        self.log.progress_root.task = "ambient"
        self.publish_log()
        self.trigger_hdr_action(frame_id + "/ambient")
        self.trigger_tapo(True)
        """
        Get LiDAR Frame
        """
        if self.config.lidar:
            while time.time() - time_begin < self.config.timeout:
                hdr_frame = self.stereo_hdr_queue.get_item(time_begin)
                if hdr_frame is not None:
                    break
                time.sleep(1)
            if hdr_frame is None:
                raise LiDARTimeoutError("LiDAR Timeout")
            timestamp = (
                hdr_frame["lidar"].header.stamp.sec
                + hdr_frame["lidar"].header.stamp.nanosec * 1e-9
            )

            hdr_item = HDRCaptureItem(
                {}, hdr_frame["lidar"], self.pose_list, timestamp, frame_id=frame_id
            )
            self.topic_callback(hdr_item)

    def publish_log(self):
        if len(self.log.hdr_error_msgs) > 10:
            self.log.hdr_error_msgs = self.log.hdr_error_msgs[-10:]
        self.log_callback(asdict(self.log))

    def trigger_tapo(self, on=True):
        client = self.service_client_tapo_on if on else self.service_client_tapo_off
        request = Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self.action_client._node, future, timeout_sec=1
        )
        response: Trigger.Response = future.result()
        if response is None or not response.success:
            raise TapoTimeoutError(response.message if response is not None else "None")

    def trigger_hdr_action(
        self, space_id: str, mode: Literal["hdr", "ambient"] = "hdr"
    ):
        goal = HDRTrigger.Goal()
        goal.space_id = f"{self.config.ROOT}{space_id}"

        def hdr_feedback_callback(call_feedback):
            try:
                log_json = json.loads(call_feedback.feedback.feedback_message)
            except json.JSONDecodeError:
                log_json = {
                    "msg": call_feedback.feedback.feedback_message,
                    "type": "msg",
                }
            print(call_feedback.feedback.feedback_message)
            if "error_" in log_json["type"]:
                self.log.hdr_error_msgs.append(log_json)
            if "info_" in log_json["type"]:
                if log_json["type"] == "info_collect_hdr_images":
                    self.log.progress_sub["idx"] = log_json["data"]["exp_idx"]
                    self.log.progress_sub["exposure"] = log_json["data"]["exposure"]
            self.publish_log()

        future: Future[ClientGoalHandle] = (
            self.action_client_hdr_trigger.send_goal_async(
                goal, feedback_callback=hdr_feedback_callback
            )
        )
        begin_time = time.time()
        while not future.done():
            rclpy.spin_once(self.action_client_hdr_trigger._node, timeout_sec=0.1)
        goal_handle = future.result()  # goal이 accepted 되었는지 확인
        if not goal_handle.accepted:
            raise GoalRejectedError("Goal Rejected")

        # result를 비동기적으로 받기 위해 get_result_async 호출
        result_future = goal_handle.get_result_async()

        # result를 받을 때까지 루프
        while not result_future.done():
            rclpy.spin_once(self.action_client_hdr_trigger._node, timeout_sec=0.1)
            if time.time() - begin_time > self.config.timeout:
                raise JaiTimeoutError("Action Timeout")

    def turn_right(self, angle: float):
        self.pose_list.clear()
        # Action Call : Turn Right
        goal = RotateAngle.Goal()
        goal.angle = angle

        def feedback_callback(feedback: RotateAngle.Feedback):
            pass
            # self.pose_list.append(feedback)

        future: Future[ClientGoalHandle] = self.action_client.send_goal_async(
            goal, feedback_callback=feedback_callback
        )
        begin_time = time.time()
        while not future.done():
            rclpy.spin_once(self.action_client._node, timeout_sec=0.1)
            if time.time() - begin_time > self.config.timeout:
                raise RangerTimeoutError("Action Timeout")
        goal_handle = future.result()  # goal이 accepted 되었는지 확인
        if not goal_handle.accepted:
            raise GoalRejectedError("Goal Rejected")

        # result를 비동기적으로 받기 위해 get_result_async 호출
        result_future: Future[RotateAngle.Result] = goal_handle.get_result_async()

        # result를 받을 때까지 루프
        while not result_future.done():
            rclpy.spin_once(self.action_client_hdr_trigger._node, timeout_sec=0.1)
            if time.time() - begin_time > self.config.timeout:
                raise RangerTimeoutError("Action Timeout")
        try:
            pose = result_future.result().pose.pose
            if pose is not None:
                self.pose_list.append(pose)
            else:
                raise PoseNoDataError("Ranger: No Pose Data")
        except Exception as e:
            raise PoseNoDataError("Ranger: No Pose Data")

    def drive_side(self, distance: float):
        goal = DriveDistance.Goal()
        goal.distance = distance

        def feedback_callback(feedback: DriveDistance.Feedback):
            pass
            # self.pose_list.append(feedback)

        future: Future[ClientGoalHandle] = self.action_client_drive.send_goal_async(
            goal, feedback_callback=feedback_callback
        )
        begin_time = time.time()
        while not future.done():
            rclpy.spin_once(self.action_client._node, timeout_sec=0.1)
            if time.time() - begin_time > self.config.timeout:
                raise RangerTimeoutError("Action Timeout")
        goal_handle = future.result()
        if not goal_handle.accepted:
            raise GoalRejectedError("Goal Rejected")

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            rclpy.spin_once(self.action_client._node, timeout_sec=0.1)
            if time.time() - begin_time > self.config.timeout:
                raise RangerTimeoutError("Action Timeout")
