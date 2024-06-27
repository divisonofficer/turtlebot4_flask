from typing import Callable, Optional
from capture_pb2 import *
from capture_state import CaptureMessage
from capture_type import (
    CaptureLiDAR,
    ImageBytes,
    CaptureSingleScene,
    Image,
    CompressedImage,
)
import threading
from capture_storage import CaptureStorage
from slam_types import RosProtoConverter
from time import time, sleep
from flask_socketio import SocketIO
from socket_progress import SocketProgress
from capture_jai_controller import CaptureJaiController


class NoLidarSignal(Exception):
    pass


class NoImageSignal(Exception):
    pass


class NoPoseSignal(Exception):
    pass


class PolarizerError(Exception):
    pass


DEBUG_LAST_CAPTURE_TIME = 0.0


class CaptureSingleScenario:

    def __init__(
        self,
        jai_controller: CaptureJaiController,
        socket_progress: SocketProgress,
        id: list[int],
        capture_msg: CaptureMessage,
        lock: threading.Lock,
        socketIO: Optional[SocketIO],
        ell,
        storage: CaptureStorage,
    ):
        self.jai_controller = jai_controller
        self.socket_progress = socket_progress
        self.space_id, self.capture_id, self.scene_id = id
        self.capture_msg = capture_msg
        self.messageDef = capture_msg.definition
        self.lock = lock
        self.socketIO = socketIO
        self.ell = ell
        self.storage = storage
        self.ANGLES = capture_msg.hyperparamter.EllRotationDegree.value_array[:]

    def get_basic_msgs(self, scene: CaptureSingleScene):
        while True:
            with self.lock:
                if self.capture_msg.messages_received:
                    break
            if time() - self.capture_msg.timestamp > 10:
                break
            sleep(0.25)

        if self.messageDef.slam.enabled:
            pose, lidar = self.capture_msg.get_message(self.messageDef.slam)
            if not pose or not lidar:
                raise NoPoseSignal()
            scene.robot_pose = RosProtoConverter().rosPoseToProtoPose3D(pose.pose.pose)
            scene.lidar_position = CaptureLiDAR(lidar)

        if self.messageDef.oakd.enabled:
            scene.picture_list += [
                ImageBytes(image=x, topic=self.messageDef.oakd.messages[idx].topic)
                for idx, x in enumerate(
                    self.capture_msg.get_message(self.messageDef.oakd)
                )
                if isinstance(x, Image) or isinstance(x, CompressedImage)
            ]
        if (
            self.messageDef.MultiChannel_Left.enabled
            and not self.messageDef.Ellipsis.enabled
        ):
            scene.picture_list += [
                ImageBytes(
                    image=x,
                    topic=self.messageDef.MultiChannel_Left.messages[idx].topic,
                )
                for idx, x in enumerate(
                    self.capture_msg.get_message(self.messageDef.MultiChannel_Left)
                )
                if isinstance(x, Image)
                or isinstance(x, CompressedImage)
                or (isinstance(x, list) and isinstance(x[0], CompressedImage))
            ]
        if self.messageDef.MultiChannel_Right.enabled:
            scene.picture_list += [
                ImageBytes(
                    image=x,
                    topic=self.messageDef.MultiChannel_Right.messages[idx].topic,
                )
                for idx, x in enumerate(
                    self.capture_msg.get_message(self.messageDef.MultiChannel_Right)
                )
                if isinstance(x, Image)
                or isinstance(x, CompressedImage)
                or (isinstance(x, list) and isinstance(x[0], CompressedImage))
            ]

    def run_single_capture(self):
        global DEBUG_LAST_CAPTURE_TIME
        if DEBUG_LAST_CAPTURE_TIME > 0:
            self.capture_msg.timestamp_log.logs.append(
                CaptureTopicTimestampLog.TimestampLog(
                    topic="Next Capture Preparation",
                    timestamp=DEBUG_LAST_CAPTURE_TIME,
                    delay_to_system=time() - DEBUG_LAST_CAPTURE_TIME,
                )
            )
        DEBUG_LAST_CAPTURE_TIME = time()
        """
        vacate capture_msg and wait for all messages to arrive
        create CaptureSingleScene object with lidar, pose, and images
        get polarized images if available
        emit the scene to the socket
        """
        self.jai_controller.open_stream()
        try:
            capture_begin_time = time()
            self.socket_progress(
                0,
                scene_id=self.scene_id,
                space_id=None,
                msg=f"Scene {self.scene_id} Capture started",
                action=CaptureTaskProgress.Action.INIT,
                capture_id=self.capture_id,
            )
            scene = CaptureSingleScene(
                capture_id=self.capture_id,
                scene_id=self.scene_id,
                timestamp=int(time()),
            )

            # 필요한 모든 메시지가 도착할 때까지 기다림

            # Capture polarized images
            if self.messageDef.Ellipsis.enabled:
                scene.picture_list += self.run_polarized_capture()

            if self.messageDef.HDR.enabled:
                scene.picture_list += self.run_hdr_capture()

            self.get_basic_msgs(scene)
            self.socket_progress(
                100,
                scene_id=self.scene_id,
                space_id=None,
                msg=f"Scene {self.scene_id}  Capture completed",
                action=CaptureTaskProgress.Action.DONE,
                capture_id=self.capture_id,
            )
            if self.socketIO:
                serialized_scene = scene.to_dict_light()
                serialized_scene.space_id = self.space_id
                self.socketIO.emit(
                    "/recent_scene",
                    serialized_scene.SerializeToString(),
                    namespace="/socket",
                )
                self.capture_msg.timestamp_log.logs.append(
                    CaptureTopicTimestampLog.TimestampLog(
                        topic="Full Capture Process",
                        timestamp=capture_begin_time,
                        delay_to_system=time() - capture_begin_time,
                    )
                )

            self.jai_controller.close_stream()
            return scene
        except NoImageSignal:
            self.socket_progress(
                100,
                scene_id=self.scene_id,
                space_id=None,
                msg="No image signal received",
                action=CaptureTaskProgress.Action.ERROR,
                capture_id=self.capture_id,
            )
        except NoPoseSignal:
            self.socket_progress(
                100,
                scene_id=self.scene_id,
                space_id=None,
                msg="No pose signal received",
                action=CaptureTaskProgress.Action.ERROR,
                capture_id=self.capture_id,
            )
        except PolarizerError:
            self.socket_progress(
                100,
                scene_id=self.scene_id,
                space_id=None,
                msg="Polarizer Error",
                capture_id=self.capture_id,
                action=CaptureTaskProgress.Action.ERROR,
            )
        if self.messageDef.Ellipsis.enabled:
            self.ell.polarizer_turn(home=True)
        self.jai_controller.close_stream()
        return None

    def run_hdr_capture(self):
        exposure_viz = self.capture_msg.hyperparamter.HdrExposureTime.value_array
        exposure_nir = self.capture_msg.hyperparamter.HdrExposureTimeNir.value_array

        if len(exposure_viz) != len(exposure_nir):
            raise ValueError("Exposure time array length mismatch")
        self.socket_progress(
            10,
            scene_id=self.scene_id,
            msg="HDR Capture started",
            action=CaptureTaskProgress.Action.ACTIVE,
        )
        self.jai_controller.freeze_auto_exposure()
        image_list: list[ImageBytes] = []
        for idx, (viz, nir) in enumerate(zip(exposure_viz, exposure_nir)):

            self.jai_controller.close_stream()
            self.jai_controller.set_exposure(viz, nir)
            self.socket_progress(
                int(10 + 80 * idx / len(exposure_viz)),
                scene_id=self.scene_id,
                msg=f"Capturing {idx} of {len(exposure_viz)}",
                action=CaptureTaskProgress.Action.ACTIVE,
            )
            sleep(0.5)
            self.capture_msg.vacate_second_msg_dict(self.messageDef.MultiChannel_Left)
            self.capture_msg.vacate_second_msg_dict(self.messageDef.MultiChannel_Right)
            while True:
                if time() - self.capture_msg.timestamp_second > 10:
                    break
                with self.lock:
                    if self.capture_msg.messages_received_second:
                        break
                sleep(0.1)
            if not self.capture_msg.messages_received_second:
                raise NoImageSignal()
            for topic in (
                self.messageDef.MultiChannel_Left.messages[:]
                + self.messageDef.MultiChannel_Right.messages[:]
            ):
                image_list.append(
                    ImageBytes(
                        self.capture_msg.msg_dict[topic.topic],
                        topic.topic
                        + "/"
                        + str(int(viz if "bayer" in topic.format else nir)),
                        bayerInterpolation="bayer" in topic.format,
                    )
                )
        self.socket_progress(
            100,
            scene_id=self.scene_id,
            msg=f"HDR Capturing Queue Done",
            action=CaptureTaskProgress.Action.DONE,
        )
        return image_list

    def run_polarized_capture(self):
        if self.ANGLES[0] > 0:
            self.ell.polarizer_turn(angle=self.ANGLES[0])
            sleep(1)
        """
        Capture the multispectral camera four times with a 45-degree rotation of the polarizer.
        Repeat the process of rotating by 45 degrees and capturing the image.
        """
        if not self.messageDef.MultiChannel_Left.enabled:
            return []
        image_list: list[ImageBytes] = []
        for idx, deg in enumerate(self.ANGLES):
            self.socket_progress(
                int(deg) * 15 // 45 + 30,
                scene_id=self.scene_id,
                space_id=self.space_id,
                msg=f"Capturing {deg} degrees",
                action=CaptureTaskProgress.Action.ACTIVE,
                capture_id=self.capture_id,
            )
            if not self.capture_msg:
                raise NoImageSignal()

            self.capture_msg.vacate_second_msg_dict(self.messageDef.MultiChannel_Left)
            while True:
                if time() - self.capture_msg.timestamp_second > 10:
                    break
                with self.lock:
                    if self.capture_msg.messages_received_second:
                        break
                sleep(0.1)

            capture_msg = self.capture_msg
            if not capture_msg.messages_received_second:
                raise NoImageSignal()
            self.capture_msg.timestamp_log.logs.append(
                CaptureTopicTimestampLog.TimestampLog(
                    topic=f"Polarized Capture {deg} degrees",
                    timestamp=capture_msg.timestamp_second,
                    delay_to_system=time() - capture_msg.timestamp_second,
                )
            )

            for topic in self.messageDef.MultiChannel_Left.messages:
                image_list.append(
                    ImageBytes(
                        capture_msg.msg_dict[topic.topic],
                        topic.topic + "/" + str(int(deg)),
                        bayerInterpolation="bayer" in topic.format,
                    )
                )

            if idx < len(self.ANGLES) - 1:
                self.ell.polarizer_turn(angle=self.ANGLES[idx + 1] - self.ANGLES[idx])

            self.storage.store_captured_scene_image(
                image_list,
                self.space_id,
                self.capture_id,
                self.scene_id,
            )
            image_list = []
        return image_list
