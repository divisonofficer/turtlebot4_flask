import cv2
import threading
from cv_bridge import CvBridge
import time


class VideoStream:
    def __init__(self, create_subscriber=None):
        if create_subscriber is None:
            return
        self.subscriber = create_subscriber(self.cv_raw_callback)

    output_frame = None
    lock = threading.Lock()

    def cv_raw_callback(self, msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.output_frame = cv_image

    def cv_ndarray_callback(self, image):
        self.output_frame = image

    def generate_preview(self, isGrayScale=False):
        while True:
            with self.lock:
                if self.output_frame is None:
                    continue

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
            self.output_frame = None
            time.sleep(0.03)
