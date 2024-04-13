import cv2
import threading
from cv_bridge import CvBridge
import time
from typing import Union

from sensor_msgs.msg import Image, CompressedImage
from typing import Union
import numpy as np


class VideoStream:
    def __init__(self, create_subscriber=None):
        self.running = {}
        self.bridge = CvBridge()
        self.topic_time = time.time()
        self.yield_time = time.time()
        if create_subscriber is None:
            return
        self.subscriber = create_subscriber(self.cv_raw_callback)

    output_frame = None
    interval_callback = None
    lock = threading.Lock()

    def cv_raw_callback(self, msg: Union[Image, CompressedImage]):

        if isinstance(msg, CompressedImage):
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        else:

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.output_frame = cv_image

    def cv_ndarray_callback(self, image):
        self.output_frame = image

    def stop(self, timeStamp: str):
        self.running[timeStamp] = False

    def generate_preview(self, timeStamp: Union[str, None] = None, isGrayScale=False):
        # if timeStamp:
        #     self.running[timeStamp] = True
        while True:
            # if not self.running[timeStamp]:
            #     break
            with self.lock:
                if self.output_frame is None:
                    continue
                self.topic_time_interval = time.time() - self.topic_time
                self.topic_time = time.time()
                if isGrayScale:
                    self.output_frame = cv2.cvtColor(
                        self.output_frame, cv2.COLOR_BGR2GRAY
                    )

                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 50]
                (flag, encodedImage) = cv2.imencode(
                    ".jpg", self.output_frame, encode_param
                )

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + bytearray(encodedImage) + b"\r\n"
            )

            self.yield_time_interval = time.time() - self.yield_time
            self.yield_time = time.time()
            self.output_frame = None

            if self.interval_callback:
                self.interval_callback(
                    self.topic_time_interval, self.yield_time_interval
                )
