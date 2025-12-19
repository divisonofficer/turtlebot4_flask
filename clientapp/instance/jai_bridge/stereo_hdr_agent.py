from asyncio import Future
import json
from typing import Callable, List, Literal, Optional
from dataclasses import dataclass, asdict, field
import numpy as np
import cv2
from piper_client import PiperClient
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
from irobot_create_msgs.action._rotate_angle import RotateAngle_GetResult_Response
from irobot_create_msgs.action._drive_distance import DriveDistance_GetResult_Response
import rclpy
from rclpy.client import Client
from jai_rosbridge.action import HDRTrigger
from std_srvs.srv import Trigger
from rclpy.action.client import ClientGoalHandle
import math


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
        drive_mode: Literal["side", "forward", "arc"] = "side"
        arc_radius: float = 1
        arc_angle: float = 10
        arc_forward : bool = False
        drive_forward: bool = False

        timeout: int = 20
        timeout_hdr: int = 20
        ROOT: str = "/home/cglab/project/turtlebot4_flask/clientapp/tmp/stereo/hdr/"
        lidar: bool = False
        
        use_piper: bool = False
        
        skip_jai : bool = False
        
        
        

    @dataclass
    class GraphItem:
        pass

    @dataclass
    class CaptureGraphItem(GraphItem):
        space_id: str
        frame_idx: int

    @dataclass
    class RotateGraphItem(GraphItem):
        rotate_angle: float

    @dataclass
    class RotateBackGraphItem(GraphItem):
        pass

    @dataclass
    class DriveGraphItem(GraphItem):
        distance: float
        direction: Literal["forward", "left"]

    @dataclass
    class DriveArcGraphItem(GraphItem):
        radius: float
        angle: float
        
    @dataclass
    class PiperGraphItem(GraphItem):
        move_idx: int

    @dataclass
    class Log:
        @dataclass
        class ProgressRoot:
            idx: int
            status: Literal["ready", "running", "done", "error", "pause", "abort"]
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
        action_clients: List[ActionClient],
        service_client_tapo_trigger: tuple[Client, Client],
        config: Config,
        topic_callback: Callable,
        log_callback: Callable,
        piper_client: Optional[PiperClient] = None,
    ):
        self.stereo_hdr_queue = stereo_hdr_queue
        self.config = config
        self.hdr_thread: Optional[threading.Thread] = None
        self.topic_callback = topic_callback
        self.log_callback = log_callback
        self.pose_list: List[Pose] = []
        (
            self.action_client_rotate,
            self.action_client_hdr_trigger,
            self.action_client_drive_side,
            self.action_client_drive_forward,
        ) = action_clients
        self.service_client_tapo_on, self.service_client_tapo_off = (
            service_client_tapo_trigger
        )
        self.piper_client = piper_client

        self.log = self.Log()
        self.sig_stop = threading.Event()
        self.sig_pause = threading.Event()
        self.sig_stop.clear()
        
        # 로그 전송용 스레드 풀 (비동기 전송)
        from concurrent.futures import ThreadPoolExecutor
        self._log_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="log_publisher")

    def capture_thread(self, space_id: str, callback: Callable):
        if self.hdr_thread is not None and self.hdr_thread.is_alive():
            self.log.progress_root.status = "running"
            raise Exception("HDR capture is already running")
        self.hdr_thread = threading.Thread(
            target=self.capture_hdr,
            args=(
                space_id,
                callback,
            ),
        )
        self.hdr_thread.start()

    def run_graph_item(self, item: GraphItem):
        if isinstance(item, self.CaptureGraphItem) and not self.config.skip_jai:
            self.log.progress_root.idx = item.frame_idx
            frame_id = time.strftime("%H_%M_%S_", time.localtime()) + str(
                int((time.time() % 1) * 1000)
            ).zfill(3)
            self.get_hdr_frame(f"{item.space_id}/{frame_id}", item.frame_idx)
        elif isinstance(item, self.RotateGraphItem):
            try:
                self.turn_right(item.rotate_angle)
            except PoseNoDataError as e:
                self.log.hdr_error_msgs.append(
                    {
                        "type": "error_no_pose",
                        "data": {"cause": type(e).__name__, "msg": str(e)},
                    }
                )
                self.publish_log()
            finally:
                self.angle_current += item.rotate_angle
        elif isinstance(item, self.DriveGraphItem):
            if item.direction == "forward":
                self.drive_side(
                    item.distance,
                    self.action_client_drive_forward,
                )
            else:
                self.drive_side(item.distance, self.action_client_drive_side)
        elif isinstance(item, self.RotateBackGraphItem):
            self.turn_right(-self.angle_current)
            self.angle_current = 0
        elif isinstance(item, self.DriveArcGraphItem):
            self.drive_arc(
                item.radius,
                item.angle,
                self.action_client_drive_side,
                self.action_client_rotate,
                forward = self.config.arc_forward
            )
        elif isinstance(item, self.PiperGraphItem):
            self.move_piper(item.move_idx)

    def move_piper(self, move_idx: int):
        """
        Piper 로봇 암을 지정된 위치로 이동하고 안정화를 위해 대기.
        
        Args:
            move_idx: 프리셋 포즈 인덱스 (0 = home, 1~4 = 촬영 위치)
        """
        if self.piper_client is None:
            raise GoalRejectedError("PiperClient가 초기화되지 않았습니다.")
        
        # 이미 해당 위치에 있으면 스킵
        if self.piper_client.current_pose_idx == move_idx:
            self.log.progress_sub = {"type": "piper", "idx": move_idx, "status": "skipped"}
            self.publish_log()
            return
        
        # Progress 업데이트: piper 이동 시작
        self.log.progress_sub = {"type": "piper", "idx": move_idx, "status": "moving"}
        self.log.progress_root.task = "rotate"  # 기존 task 타입 활용
        self.publish_log()
        
        # 로봇 암 이동 명령
        success = self.piper_client.piper_move_arm(move_idx)
        if not success:
            # 로봇이 움직이는 중이라 거부된 경우, 잠시 대기 후 재시도
            self.get_logger().info(f"Piper 이동 실패 (idx={move_idx}), 2초 후 재시도...")
            self.log.progress_sub = {"type": "piper", "idx": move_idx, "status": "retry"}
            self.publish_log()
            time.sleep(2)
            success = self.piper_client.piper_move_arm_force(move_idx)
            if not success:
                self.log.progress_sub = {"type": "piper", "idx": move_idx, "status": "failed"}
                self.publish_log()
                raise GoalRejectedError(f"Piper 이동 실패: move_idx={move_idx}")
        
        # 4번에서 0번으로 이동하는 경우는 이후 ranger 이동이 있으므로 대기 생략
        if move_idx == 0 and self.piper_client.current_pose_idx == 4:
            self.log.progress_sub = {"type": "piper", "idx": move_idx, "status": "done"}
            self.publish_log()
            self.get_logger().info(f"Piper 이동 완료 (대기 생략): idx={move_idx}")
            return
        
        # 추가 안정화 대기 시간 (5초) - 진행 상황을 1초마다 업데이트
        self.get_logger().info("Waiting for piper moving")
        stabilize_time = 3
        for wait_sec in range(stabilize_time):
            self.log.progress_sub = {
                "type": "piper", 
                "idx": move_idx, 
                "status": "stabilizing",
                "wait": wait_sec + 1,
                "total_wait": stabilize_time
            }
            self.publish_log()
            time.sleep(1)
        
        self.log.progress_sub = {"type": "piper", "idx": move_idx, "status": "done"}
        self.publish_log()
        self.get_logger().info(f"Piper 이동 완료: idx={move_idx}")

    def get_logger(self):
        """로거 반환 (piper_client 또는 action_client에서)"""
        if self.piper_client is not None:
            return self.piper_client.get_logger()
        return self.action_client_rotate._node.get_logger()

    def pause(self):
        self.sig_pause.set()
        self.log.progress_root.status = "pause"

    def resume(self):
        self.sig_pause.clear()
        self.log.progress_root.status = "running"

    def abort(self):
        self.sig_stop.set()
        self.log.progress_root.status = "abort"

    def capture_hdr(self, space_id: str, callback: Optional[Callable] = None):
        """
        Clear HDR Queue
        """
        # self.trigger_tapo(True)
        self.stereo_hdr_queue.clear()
        self.log.progress_root.status = "running"
        self.publish_log()
        self.angle_current = 0.0

        graph_items: List[JaiHDRCaptureAgent.GraphItem] = []
        
        # 전체 캡처 인덱스 (piper와 별개로 관리)
        capture_idx = 0
        
        if self.config.drive_mode == "arc":
            for arc_idx in range(self.config.capture_cnt):
                if self.config.use_piper:
                    # Piper 모드: 한 arc 위치에서 5번의 piper 움직임과 캡처
                    for piper_idx in range(5):
                        graph_items.append(self.PiperGraphItem(piper_idx))
                        graph_items.append(self.CaptureGraphItem(f"{space_id}", capture_idx))
                        capture_idx += 1
                    # piper를 홈(0)으로 복귀 후 다음 arc 이동
                    graph_items.append(self.PiperGraphItem(0))
                else:
                    graph_items.append(self.CaptureGraphItem(f"{space_id}", capture_idx))
                    capture_idx += 1
                graph_items.append(
                    self.DriveArcGraphItem(
                        self.config.arc_radius, self.config.arc_angle / self.config.capture_cnt
                    )
                )
        else:
            for side_idx in range(self.config.side_move_cnt):
                if self.config.capture_cnt > 1 and self.config.side_move_cnt > 1:
                    graph_items.append(
                        self.RotateGraphItem(-self.config.rotate_angle / 2)
                    )
                for i in range(self.config.capture_cnt):

                    if i != 0:
                        graph_items.append(
                            self.RotateGraphItem(
                                self.config.rotate_angle / (self.config.capture_cnt - 1)
                            )
                        )
                    if self.config.use_piper:
                        # Piper 모드: 한 위치에서 5번의 piper 움직임과 캡처
                        for piper_idx in range(5):
                            graph_items.append(self.PiperGraphItem(piper_idx))
                            graph_items.append(self.CaptureGraphItem(f"{space_id}", capture_idx))
                            capture_idx += 1
                        # piper를 홈(0)으로 복귀 후 다음 이동
                        graph_items.append(self.PiperGraphItem(0))
                    else:
                        graph_items.append(
                            self.CaptureGraphItem(
                                f"{space_id}", capture_idx
                            )
                        )
                        capture_idx += 1
                    if (
                        self.config.capture_cnt - 1 == i
                        and self.config.side_move_cnt > 1
                        and i > 0
                    ):
                        graph_items.append(self.RotateBackGraphItem())
                if side_idx != self.config.side_move_cnt - 1:
                    if self.config.drive_forward:
                        graph_items.append(
                            self.DriveGraphItem(
                                self.config.side_move_distance, "forward"
                            )
                        )
                    else:
                        graph_items.append(
                            self.DriveGraphItem(self.config.side_move_distance, "left")
                        )

        for item in graph_items:
            try:
                while self.sig_pause.is_set():
                    time.sleep(1)

                if self.sig_stop.is_set():
                    self.sig_stop.clear()
                    raise GoalRejectedError("Manually Stopped")

                self.run_graph_item(item)

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
                self.log.progress_root.status = "error"
                self.publish_log()
                
                # Piper 모드: 에러 발생 시 홈 위치(0)로 복귀
                if self.config.use_piper:
                    try:
                        self.move_piper(0)
                    except Exception as piper_e:
                        self.get_logger().error(f"Piper 홈 복귀 실패: {piper_e}")
                
                if (
                    self.config.side_move_cnt > 1
                    and self.config.capture_cnt > 1
                    and self.angle_current != 0
                ):
                    self.run_graph_item(self.RotateBackGraphItem())

                break

        # Piper 모드: 완료 후 홈 위치(0)로 복귀
        if self.config.use_piper:
            try:
                self.move_piper(0)
            except Exception as piper_e:
                self.get_logger().error(f"Piper 홈 복귀 실패: {piper_e}")
        
        self.log.progress_root.status = "done"
        self.publish_log()
        if callback is not None:
            callback()

    def get_hdr_frame(self, frame_id, idx: int):
        time_begin = time.time()
        hdr_frame = None
        """
        Get HDR Frame
        """
        self.log.progress_root.task = "hdr"

        self.publish_log()
        self.trigger_hdr_action(frame_id)
        # """
        # Get Ambient Frame
        # """
        # self.trigger_tapo(False)
        # time.sleep(1)
        # self.log.progress_root.task = "ambient"
        # self.publish_log()
        # self.trigger_hdr_action(frame_id + "/ambient")
        # self.trigger_tapo(True)
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
            lidar = hdr_frame["lidar"]
        else:
            lidar = None
            timestamp = time.time()
        hdr_item = HDRCaptureItem(
            {}, lidar, self.pose_list, timestamp, frame_id=frame_id
        )
        self.topic_callback(hdr_item)

    def publish_log(self):
        """로그를 비동기로 전송 (블로킹 방지)"""
        if len(self.log.hdr_error_msgs) > 10:
            self.log.hdr_error_msgs = self.log.hdr_error_msgs[-10:]
        # 로그 데이터를 복사하여 비동기 전송
        log_data = asdict(self.log)
        self._log_executor.submit(self.log_callback, log_data)

    def trigger_tapo(self, on=True):
        client = self.service_client_tapo_on if on else self.service_client_tapo_off
        request = Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(
            self.action_client_rotate._node, future, timeout_sec=1
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
        # self.pose_list.clear()
        # Action Call : Turn Right
        goal = RotateAngle.Goal()
        goal.angle = angle

        def feedback_callback(feedback: RotateAngle.Feedback):
            pass
            # self.pose_list.append(feedback)

        future: Future[ClientGoalHandle] = self.action_client_rotate.send_goal_async(
            goal, feedback_callback=feedback_callback
        )
        begin_time = time.time()
        while not future.done():
            rclpy.spin_once(self.action_client_rotate._node, timeout_sec=0.1)
            if time.time() - begin_time > self.config.timeout:
                raise RangerTimeoutError("Action Timeout")
        goal_handle = future.result()  # goal이 accepted 되었는지 확인
        if not goal_handle.accepted:
            raise GoalRejectedError("Goal Rejected")

        # result를 비동기적으로 받기 위해 get_result_async 호출
        result_future: Future[RotateAngle_GetResult_Response] = (
            goal_handle.get_result_async()
        )

        # result를 받을 때까지 루프
        while not result_future.done():
            rclpy.spin_once(self.action_client_hdr_trigger._node, timeout_sec=0.1)
            if time.time() - begin_time > self.config.timeout:
                raise RangerTimeoutError("Action Timeout")

    def drive_side(self, distance: float, client: ActionClient):
        goal = DriveDistance.Goal()
        goal.distance = distance
        # self.pose_list.clear()

        def feedback_callback(feedback: DriveDistance.Feedback):
            pass
            # self.pose_list.append(feedback)

        future: Future[ClientGoalHandle] = client.send_goal_async(
            goal, feedback_callback=feedback_callback
        )
        begin_time = time.time()
        while not future.done():
            rclpy.spin_once(self.action_client_rotate._node, timeout_sec=0.1)
            if time.time() - begin_time > self.config.timeout:
                raise RangerTimeoutError("Action Timeout")
        goal_handle = future.result()
        if not goal_handle.accepted:
            raise GoalRejectedError("Goal Rejected")

        result_future: Future[DriveDistance_GetResult_Response] = (
            goal_handle.get_result_async()
        )
        while not result_future.done():
            rclpy.spin_once(self.action_client_rotate._node, timeout_sec=0.1)
            if time.time() - begin_time > self.config.timeout:
                raise RangerTimeoutError("Action Timeout")
        time.sleep(0.3)
        result = result_future.result().result

    def drive_arc(
        self,
        radius: float,
        angle: float,
        drive_client: ActionClient,
        rotate_client: ActionClient,
        forward: bool = True,
    ):
        print(radius, angle, forward)

        angle_radian = math.radians(angle)
        chord = 2.0 * radius * np.sin(angle_radian / 2.0)

        if forward:
            # Forward-facing arc
            self.turn_right(float(angle) / 2)
            self.drive_side(chord, drive_client)
            self.turn_right(float(angle) / 2)
        else:
            # Backward-facing arc (reverse orientation)
            # 뒤를 보고 있다면 arc의 방향을 반대로 적용
            self.turn_right(float(angle) / 2)
            self.drive_side(-chord, drive_client)  # 반대 방향으로 이동
            self.turn_right(float(angle) / 2)
