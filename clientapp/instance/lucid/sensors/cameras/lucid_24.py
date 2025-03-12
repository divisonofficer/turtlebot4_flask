import threading

import cv2
import numpy as np
from lucid_py_api import LucidPyAPI
import rclpy
from ..camera import Camera


class CameraLucid24(Camera):

    def __init__(self, name: str, lucid_api: LucidPyAPI):
        super().__init__(
            name,
            self.Const(
                srcs=2,
                raw_format="bayer",
                raw_bits=24,
                width=1440,
                height=928,
                preview_keys=["left", "right"],
            ),
        )
        self.lucid_api = lucid_api
        self.wb = np.asarray(
            [
                2.0841475322429894,
                1.0,
                1.9215893014341496,
            ]
        )

    def start_stream(self):
        self.lucid_api.open_stream()

        def buffer_resolve_loop():
            while rclpy.ok():
                buffers = self.lucid_api.collect_images()
                buffers[0].sort(key=lambda x: x.timestamp_ns)
                buffers[1].sort(key=lambda x: x.timestamp_ns)
                while len(buffers[0]) > 0 and len(buffers[1]) > 0:
                    if (
                        abs(buffers[0][0].timestamp_ns - buffers[1][0].timestamp_ns)
                        / 1e9
                    ) < 0.1:
                        buffer_left = buffers[0].pop(0)
                        buffer_right = buffers[1].pop(0)

                        threading.Thread(
                            target=self.frame_callback,
                            args=(
                                self,
                                self.Frame(
                                    buffer_left.timestamp_ns / 1e9,
                                    {
                                        "left": buffer_left.buffer_np,
                                        "right": buffer_right.buffer_np,
                                    },
                                    {},
                                    file_format={"left": "npy", "right": "npy"},
                                ),
                            ),
                        ).start()
                    else:
                        if buffers[0][0].timestamp_ns < buffers[1][0].timestamp_ns:
                            buffers[0].pop(0)
                        else:
                            buffers[1].pop(0)

        threading.Thread(target=buffer_resolve_loop, daemon=True).start()

    def post_process_thumbnail(self, frame, frame_id: str):
        image = frame.reshape(-1, 1440, 3)
        image0 = cv2.cvtColor(image[..., 0], cv2.COLOR_BAYER_RG2RGB)
        image1 = cv2.cvtColor(image[..., 1], cv2.COLOR_BAYER_RG2RGB)
        image2 = cv2.cvtColor(image[..., 2], cv2.COLOR_BAYER_RG2RGB)
        image = (
            image2.astype(np.float32) / 255
            + image1.astype(np.float32) / 255 / 256
            + image0.astype(np.float32) / 255 / 256 / 256
        )
        image *= self.wb.reshape(1, 1, 3)
        image = image ** (1 / 2.2)

        return (image * 255).astype(np.uint8)
