from sensors.sensor import Sensor
from sensors.camera import Camera
from flask_socketio import SocketIO

import cv2
import base64


class ThumbStream:
    def __init__(
        self,
        socketio: SocketIO,
    ):
        self.socketio = socketio

    def yield_thumbnails(self, data, sensor: Sensor, frame_id: str):
        thumb = sensor.post_process_thumbnail(data, frame_id)
        cv2.imwrite(f"thumb_{sensor.name}_{frame_id}.jpg", thumb)

        topic_id = f"/thumb/{sensor.name}/{frame_id}"
        _, buffer = cv2.imencode(".jpg", thumb)
        encoded_thumb = base64.b64encode(buffer).decode("utf-8")
        self.socketio.emit(topic_id, encoded_thumb)
