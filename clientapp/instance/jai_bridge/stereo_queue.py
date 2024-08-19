from typing import Callable, Optional, Generic, TypeVar, Union

import cv2
from sensor_msgs.msg import CompressedImage
from rclpy.node import Node
from rclpy.subscription import Subscription
import time
from std_msgs.msg import Header

TIMEDIFF = 0.1
MAX_QUEUE_LENGTH = 10

T = TypeVar("T")


class StereoItemMerged:
    header: Header

    def __init__(
        self,
        left: Union[CompressedImage, cv2.typing.MatLike],
        right: Union[CompressedImage, cv2.typing.MatLike],
        timestamp: Optional[Header] = None,
    ):
        self.left = left
        self.right = right
        self.header = left.header if isinstance(left, CompressedImage) else timestamp  # type: ignore

    def __del__(self):
        del self.left
        del self.right


class StereoQueue(Generic[T]):

    def __init__(
        self,
        node: Node,
        topic_left: Optional[str],
        topic_right: Optional[str],
        stereo_callback: Callable[[T, T], None],
    ):
        self.msg_right_queue: list[T] = []
        self.msg_left_queue: list[T] = []
        if topic_left:
            self.subscription_left = node.create_subscription(
                CompressedImage,
                topic_left,
                self.callback_left,
                1,
            )
        if topic_right:
            self.subscription_right = node.create_subscription(
                CompressedImage,
                topic_right,
                self.callback_right,
                1,
            )
        self.stereo_callback = stereo_callback

        self.timestamp_last_merged = 0
        self.timestamp_interval_merged = 0
        self.timestamp_left_last = 0
        self.timestamp_right_last = 0

    def callback_left(self, msg: T):
        if len(self.msg_left_queue) < MAX_QUEUE_LENGTH:
            self.msg_left_queue.append(msg)
            self.timestamp_left_last = self.get_timestamp_second(msg)
        self.pop_queue()

    def callback_right(self, msg: T):
        if len(self.msg_right_queue) < MAX_QUEUE_LENGTH:
            self.msg_right_queue.append(msg)
            self.timestamp_right_last = self.get_timestamp_second(msg)

        self.pop_queue()

    def pop_queue(self):
        while len(self.msg_left_queue) > 0 and len(self.msg_right_queue) > 0:
            msg_left = self.msg_left_queue[0]
            msg_right = self.msg_right_queue[0]
            time_left = self.get_timestamp_second(msg_left)  # type: ignore
            time_right = self.get_timestamp_second(msg_right)

            if abs(time_left - time_right) < TIMEDIFF:
                self.msg_left_queue.pop(0)
                self.msg_right_queue.pop(0)
                self.stereo_callback(msg_left, msg_right)

                if self.timestamp_last_merged > 0:
                    self.timestamp_interval_merged = (
                        time_left - self.timestamp_last_merged
                    )
                self.timestamp_last_merged = time_left
                break

            if time_left < time_right:
                del self.msg_left_queue[0]
            else:
                del self.msg_right_queue[0]

    def get_timestamp_second(self, msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

    def queue_status(self):
        return {
            "left_count": len(self.msg_left_queue),
            "right_count": len(self.msg_right_queue),
            "left_last_time": (self.timestamp_left_last),
            "right_last_time": (self.timestamp_right_last),
            "merge_interval": self.timestamp_interval_merged,
            "left_last_delay": time.time() - self.timestamp_left_last,
            "right_last_delay": time.time() - self.timestamp_right_last,
            "diff": self.timestamp_right_last - self.timestamp_left_last,
        }
