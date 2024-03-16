import cv2
import threading
from cv_bridge import CvBridge


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
        # print("Generating Frame", cv_image.shape)
        self.output_frame = cv_image

    def cv_ndarray_callback(self, image):
        # print("Generating Frame", image.shape)
        self.output_frame = image

    def generate_preview(self):
        # print("Generating preview")
        while True:
            with self.lock:
                if self.output_frame is None:
                    continue
                (flag, encodedImage) = cv2.imencode(".jpg", self.output_frame)
                if not flag:
                    continue
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + bytearray(encodedImage) + b"\r\n"
            )
