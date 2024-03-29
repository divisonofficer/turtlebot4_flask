import rclpy
from cv_bridge import CvBridge
import threading
import cv2


class VideoStream:
    def __init__(self, create_subscriber):
        self.subscriber = create_subscriber(None, self.cv_raw_callback)

    output_frame = None
    lock = threading.Lock()

    def cv_raw_callback(self, msg):
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.output_frame = cv_image

    def generate_preview(self):
        print("Generating preview")
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
