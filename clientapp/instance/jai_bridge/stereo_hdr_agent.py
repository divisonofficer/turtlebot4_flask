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
from irobot_create_msgs.action import RotateAngle
import rclpy
from jai_rosbridge.action import HDRTrigger


class JaiHDRCaptureAgent:
    @dataclass
    class Config:
        rotate_angle = 90
        capture_cnt = 6
        timeout = 20
        ROOT = "/home/cglab/project/turtlebot4_flask/clientapp/tmp/stereo/hdr/"

    @dataclass
    class Log:
        @dataclass
        class ProgressRoot:
            idx: int
            status: Literal["ready", "running", "done", "error"]
            task: Literal["rotate", "hdr"] = "rotate"

        progress_root: ProgressRoot = field(
            default_factory=lambda: JaiHDRCaptureAgent.Log.ProgressRoot(0, "ready")
        )
        progress_sub: dict = field(default_factory=lambda: {"type": "hdr", "idx": 0})
        hdr_error_msgs: List[dict] = field(default_factory=list)

    def __init__(
        self,
        stereo_hdr_queue: StereoHDRQueue,
        action_client_rotate: ActionClient,
        action_client_hdr_trigger: ActionClient,
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
        self.stereo_hdr_queue.clear()
        self.log.progress_root.status = "running"

        try:
            for i in range(self.config.capture_cnt):
                # Logger : Turn Right
                self.log.progress_root.idx = i
                self.log.progress_root.task = "rotate"
                self.publish_log()
                if i != 0:

                    self.turn_right(self.config.rotate_angle / self.config.capture_cnt)

                # get topics from HDR Queue
                self.log.progress_root.task = "hdr"
                self.publish_log()
                self.get_hdr_frame(space_id, i)
        except Exception as e:
            self.log.hdr_error_msgs.append({"msg": str(e)})
            self.log.progress_root.status = "error"
            self.publish_log()
            return
        self.log.progress_root.status = "done"
        self.publish_log()

    def hdr_reshape_concated_image(self, hdr_frame: CompressedImage, is_color: bool):
        buffer_np = np.frombuffer(hdr_frame.data, np.uint8)
        height, width = 1080, 1440
        image = cv2.imdecode(buffer_np, cv2.IMREAD_GRAYSCALE)
        image_split = np.split(image, image.shape[0] // height, axis=0)
        if is_color:
            image_split = [
                cv2.cvtColor(img, cv2.COLOR_BayerRG2RGB) for img in image_split
            ]
        frame_id = hdr_frame.header.frame_id
        exposure_times = [float(x) for x in frame_id.split("hdr_")[-1].split("_")]
        return [(img, exposure_times[i]) for i, img in enumerate(image_split)] + [
            (image, "concat")
        ]

    def get_hdr_frame(self, space_id, idx: int):
        time_begin = time.time()
        hdr_frame = None
        self.trigger_hdr_action(space_id)
        while time.time() - time_begin < self.config.timeout:
            hdr_frame = self.stereo_hdr_queue.get_item(time_begin)
            if hdr_frame is not None:
                break
            time.sleep(1)
        if hdr_frame is None:
            raise Exception("HDR Frame is not received")
        timestamp = (
            hdr_frame["lidar"].header.stamp.sec
            + hdr_frame["lidar"].header.stamp.nanosec * 1e-9
        )

        hdr_item = HDRCaptureItem(
            {},
            hdr_frame["lidar"],
            self.pose_list,
            timestamp,
        )
        self.topic_callback(hdr_item)

    def publish_log(self):
        if len(self.log.hdr_error_msgs) > 10:
            self.log.hdr_error_msgs = self.log.hdr_error_msgs[-10:]
        self.log_callback(asdict(self.log))

    def trigger_hdr_action(self, space_id: str):
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

        future = self.action_client_hdr_trigger.send_goal_async(
            goal, feedback_callback=hdr_feedback_callback
        )
        begin_time = time.time()
        while not future.done():
            rclpy.spin_once(self.action_client_hdr_trigger._node, timeout_sec=0.1)
        goal_handle = future.result()  # goal이 accepted 되었는지 확인
        if not goal_handle.accepted:
            raise Exception("Goal Rejected")

        # result를 비동기적으로 받기 위해 get_result_async 호출
        result_future = goal_handle.get_result_async()

        # result를 받을 때까지 루프
        while not result_future.done():
            rclpy.spin_once(self.action_client_hdr_trigger._node, timeout_sec=0.1)
            if time.time() - begin_time > self.config.timeout:
                raise Exception("Action Timeout")

    def turn_right(self, angle: float):
        self.pose_list.clear()
        # Action Call : Turn Right
        goal = RotateAngle.Goal()
        goal.angle = angle

        def feedback_callback(feedback):
            self.pose_list.append(feedback.current_pose)

        future = self.action_client.send_goal_async(
            goal, feedback_callback=feedback_callback
        )
        begin_time = time.time()
        while not future.done():
            rclpy.spin_once(self.action_client._node, timeout_sec=0.1)
            if time.time() - begin_time > self.config.timeout:
                raise Exception("Action Timeout")
        goal_handle = future.result()  # goal이 accepted 되었는지 확인
        if not goal_handle.accepted:
            raise Exception("Goal Rejected")

        # result를 비동기적으로 받기 위해 get_result_async 호출
        result_future = goal_handle.get_result_async()

        # result를 받을 때까지 루프
        while not result_future.done():
            rclpy.spin_once(self.action_client_hdr_trigger._node, timeout_sec=0.1)
            if time.time() - begin_time > self.config.timeout:
                raise Exception("Action Timeout")
