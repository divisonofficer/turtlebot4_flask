import threading

from typing import Union

import cv2
import numpy as np
from ouster_lidar.ouster_bridge import OusterBridge, OusterLidarData
from ..sensor import Sensor


class SensorOuster(Sensor):

    def __init__(self, name, ouster_bridge: OusterBridge):
        self.ouster_bridge = ouster_bridge
        const = Sensor.Const(preview_enabled=True, preview_keys=["points"])

        super().__init__(name, const, self.State())

        self.viz_max_depth = 20000
        self.cam2lidar = np.array(
            [
                [
                    0.8643344180119186,
                    0.5027809366545345,
                    -0.011719367593503036,
                    -158.7326471248788,
                ],
                [
                    -0.012185371637328744,
                    -0.0023593352234670107,
                    -0.9999229721610432,
                    -7.9801565543126065,
                ],
                [
                    -0.5027698584422295,
                    0.864410645049017,
                    0.004087317928107839,
                    -777.6752812183358,
                ],
                [0, 0, 0, 1],
            ]
        )

    def start_stream(
        self,
    ):
        if hasattr(self, "thread") and self.thread.is_alive():
            return
        self.thread = threading.Thread(
            target=self.ouster_bridge.collect_data, args=(self.__on_lidar_frame,)
        )
        self.thread.start()

    def __on_lidar_frame(self, msg: Union[OusterLidarData, Exception]):
        if isinstance(msg, OusterLidarData):
            frame = self.Frame(
                msg.timestamp_ns / 1e9,
                {"points": msg.points, "pose": msg.pose},
                {},
                {
                    "points": "npy",
                    "pose": "npy",
                },
            )
            self.frame_callback(self, frame)
        pass

    def register_callback(self, callback):
        self.frame_callback = callback

    def post_process_thumbnail(self, frame, frame_id: str):

        if frame_id == "points":
            points = frame
            points = points.reshape(-1, 3) * 1000
            lidar2cam = self.cam2lidar

            points = (
                lidar2cam
                @ np.concatenate([points, np.ones((points.shape[0], 1))], axis=1).T
            ).T

            points = points[:, :3]

            fx = 1346
            cx = 720
            cy = 464

            points[..., 0] = points[..., 0] * fx / points[..., 2] + cx
            points[..., 1] = points[..., 1] * fx / points[..., 2] + cy
            depth_map = np.zeros((928, 1440, 3), dtype=np.uint8)

            points = points[
                (points[..., 0] > 0)
                & (points[..., 0] < 1440 - 3)
                & (points[..., 1] > 0)
                & (points[..., 1] < 928 - 3)
                & (points[..., 2] > 0)
            ]
            points[..., 2] = points[..., 2] * 255 / self.viz_max_depth

            colors = cv2.applyColorMap(
                (points[..., 2]).astype(np.uint8), cv2.COLORMAP_JET
            )
            for i in range(9):
                depth_map[
                    points[..., 1].astype(int) + i % 3,
                    points[..., 0].astype(int) + i // 3,
                ] = colors.reshape(-1, 3)
            frame = depth_map
        return frame
