import cv2
import threading
from cv_bridge import CvBridge
import time
from typing import Union

from sensor_msgs.msg import Image, CompressedImage
from typing import Union
import numpy as np


def decode_jai_compressedImage(msg: CompressedImage):
    np_arr = np.frombuffer(msg.data, np.uint8)
    frame_format = ""
    frame_bits = 8

    frame_format = msg.header.frame_id.split("_")[0]
    frame_bits = int(msg.header.frame_id.split("_")[1].split("bit")[0])

    if frame_bits != 8:
        cv_image = np.reshape(np_arr, (2160, 1440, 2)).astype(np.uint16)
        cv_image = cv_image[:1080, :, 0] + (cv_image[:1080, :, 1] << 8)

    else:
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)

    if frame_format == "bayer":
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BayerBG2BGR)

    if frame_bits != 8:
        cv_image = cv_image << (16 - frame_bits)

    return cv_image


class VideoStream:
    def __init__(
        self,
        create_subscriber=None,
        preview_compress: bool = False,
        timestampWatermark: bool = False,
    ):
        self.bridge = CvBridge()
        self.topic_time = time.time()
        self.yield_time = time.time()
        self.timestampWatermark = timestampWatermark
        self.preview_compress = preview_compress
        if create_subscriber is None:
            return
        self.subscriber = create_subscriber(self.cv_raw_callback)

    output_frame = None
    cv_image = None
    interval_callback = None
    lock = threading.Lock()
    generator_count = 0

    def cv_raw_callback(self, msg: Union[Image, CompressedImage]):
        if self.generator_count < 1:
            return
        if isinstance(msg, CompressedImage):
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame_format = ""
            frame_bits = 8
            print(msg.header)
            print(np_arr.shape, np_arr.dtype)
            if "bit" in msg.header.frame_id and not "8bit" in msg.header.frame_id:
                cv_image = (decode_jai_compressedImage(msg) / 256).astype(np.uint8)

                if frame_format == "bayer":
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BayerBG2BGR)
            elif np_arr.shape[0] % (1440 * 1080) == 0:
                cv_image = np.reshape(
                    np_arr, (1080, 1440, np_arr.shape[0] // (1440 * 1080))
                ).astype(np.uint8)
                if cv_image.shape[2] == 3:
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            else:
                cv_image = cv2.imdecode(np_arr, cv2.IMREAD_ANYCOLOR)

        else:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if msg.encoding == "bayer_rggb8":
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BayerBG2BGR)
        if cv_image is None:
            return
        self.cv_image = cv_image.copy()
        if self.timestampWatermark:
            time_text = time.strftime("%H:%M:%S", time.localtime(msg.header.stamp.sec))
            cv2.putText(
                cv_image,
                time_text,
                (15, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                2,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        if self.preview_compress:
            height = cv_image.shape[0]
            width = cv_image.shape[1]
            aspect_ratio = width / height
            new_width = 256
            new_height = int(new_width / aspect_ratio)
            cv_image = cv2.resize(cv_image, (new_width, new_height))
        self.output_frame = cv_image

    def cv_ndarray_callback(self, image):
        self.output_frame = image

    def stop(self, timeStamp: str):
        pass

    def generate_preview(self, timeStamp: Union[str, None] = None, isGrayScale=False):
        # if timeStamp:
        #     self.running[timeStamp] = True
        self.generator_count += 1
        try:
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
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + bytearray(encodedImage)
                    + b"\r\n"
                )

                self.yield_time_interval = time.time() - self.yield_time
                self.yield_time = time.time()

                if self.interval_callback:
                    self.interval_callback(
                        self.topic_time_interval, self.yield_time_interval
                    )
        except GeneratorExit:
            self.generator_count -= 1
