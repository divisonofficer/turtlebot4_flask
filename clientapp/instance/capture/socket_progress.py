from typing import List, Optional
from capture_pb2 import CaptureTaskProgress
from sensor_msgs.msg import Image
from flask_socketio import SocketIO
from capture_type import ImageBytes


class SocketProgress:

    def __init__(self, socketIO: SocketIO, logger):
        self.socketIO = socketIO
        self.logger = logger

    def __call__(
        self,
        progress: int,
        scene_id: Optional[int] = None,
        images: Optional[List[ImageBytes]] = None,
        msg: Optional[str] = None,
        action: Optional[CaptureTaskProgress.Action.ValueType] = None,
        uuid: Optional[str] = None,
        space_id: Optional[int] = None,
        capture_id: Optional[int] = None,
    ):
        self.logger.info(msg)
        if not action:
            action = (
                CaptureTaskProgress.Action.ACTIVE
                if uuid
                else CaptureTaskProgress.Action.DEBUG
            )

        if not uuid:
            uuid = f"{capture_id}/{scene_id if scene_id else 'root'}"
        if self.socketIO:
            self.socketIO.emit(
                "/progress",
                CaptureTaskProgress(
                    space_id=space_id if space_id else 0,
                    capture_id=capture_id if capture_id else 0,
                    progress=progress,
                    scene_id=scene_id if scene_id else 0,
                    message=msg if msg else "",
                    images=[x.topic for x in images] if images else [],
                    action=action,
                    uid=uuid,
                ).SerializeToString(),
                namespace="/socket",
            )
