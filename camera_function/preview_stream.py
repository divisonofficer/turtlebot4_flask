import rclpy
from cv_bridge import CvBridge
import threading
import cv2
output_frame = None
lock = threading.Lock()
def cv_raw_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    global output_frame
    output_frame = cv_image

def create_ros_subscriber(controller):
    node = controller.con_subscribe_camera_preview(None, cv_raw_callback)
    # Spin in a separate thread
    def spin():
        rclpy.spin(node)

    thread = threading.Thread(target=spin)
    thread.start()


def generate_preview():
    global output_frame, lock
    print("Generating preview")
    while True:
        with lock:
            if output_frame is None:
                continue
            (flag, encodedImage) = cv2.imencode(".jpg", output_frame)
            if not flag:
                continue
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage) + b'\r\n')