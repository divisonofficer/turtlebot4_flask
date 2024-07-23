from typing import Callable
from sensor_msgs.msg import CompressedImage
from rclpy.node import Node
from rclpy.subscription import Subscription

TIMEDIFF = 0.1
MAX_QUEUE_LENGTH = 10


class StereoQueue:

    def __init__(
        self,
        node: Node,
        topic_left: str,
        topic_right: str,
        stereo_callback: Callable[[CompressedImage, CompressedImage], None],
    ):
        self.msg_right_queue: list[CompressedImage] = []
        self.msg_left_queue: list[CompressedImage] = []

        self.subscription_left = node.create_subscription(
            CompressedImage,
            topic_left,
            self.callback_left,
            1,
        )
        self.subscription_right = node.create_subscription(
            CompressedImage,
            topic_right,
            self.callback_right,
            1,
        )
        self.stereo_callback = stereo_callback

    def callback_left(self, msg: CompressedImage):
        if len(self.msg_left_queue) < MAX_QUEUE_LENGTH:
            self.msg_left_queue.append(msg)
        self.pop_queue()

    def callback_right(self, msg: CompressedImage):
        if len(self.msg_right_queue) < MAX_QUEUE_LENGTH:
            self.msg_right_queue.append(msg)

        self.pop_queue()

    def pop_queue(self):
        while len(self.msg_left_queue) > 0 and len(self.msg_right_queue) > 0:
            msg_left = self.msg_left_queue[0]
            msg_right = self.msg_right_queue[0]
            time_left = msg_left.header.stamp.sec + msg_left.header.stamp.nanosec / 1e9
            time_right = (
                msg_right.header.stamp.sec + msg_right.header.stamp.nanosec / 1e9
            )
            if abs(time_left - time_right) < TIMEDIFF:
                self.msg_left_queue.pop(0)
                self.msg_right_queue.pop(0)
                self.stereo_callback(msg_left, msg_right)
                break

            if time_left < time_right:
                self.msg_left_queue.pop(0)
            else:
                self.msg_right_queue.pop(0)
