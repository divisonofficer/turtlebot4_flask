from typing import Callable, Optional, Generic, TypeVar

from lucid_py_api import LucidImage
import time
from std_msgs.msg import Header

TIMEDIFF = 0.1
MAX_QUEUE_LENGTH = 10

T = TypeVar("T")
K = TypeVar("K")


class StereoItemMerged:
    header: Header

    def __init__(
        self,
        left: LucidImage,
        right: LucidImage,
        timestamp: Optional[Header] = None,
    ):
        self.left = left
        self.right = right
        self.header = left.header if hasattr(left, "header") else timestamp  # type: ignore

    def __del__(self):
        del self.left
        del self.right


class StereoQueue(Generic[T, K]):

    def __init__(
        self,
        stereo_callback: Callable[[T, K], None],
        time_diff: float = TIMEDIFF,
        max_queue_length: int = MAX_QUEUE_LENGTH,
    ):
        self.msg_left_queue: list[T] = []
        self.msg_right_queue: list[K] = []

        self.stereo_callback = stereo_callback

        self.timestamp_last_merged = 0
        self.timestamp_interval_merged = 0
        self.timestamp_left_last = 0
        self.timestamp_right_last = 0
        self.time_diff = time_diff
        self.max_queue_length = max_queue_length

    def callback_left(self, msg: T):
        if len(self.msg_left_queue) < self.max_queue_length:
            self.msg_left_queue.append(msg)
            self.timestamp_left_last = self.get_timestamp_second(msg)
        self.pop_queue()

    def callback_right(self, msg: K):
        if len(self.msg_right_queue) < self.max_queue_length:
            self.msg_right_queue.append(msg)
            self.timestamp_right_last = self.get_timestamp_second(msg)

        self.pop_queue()

    def pop_queue(self):
        while len(self.msg_left_queue) > 0 and len(self.msg_right_queue) > 0:
            msg_left = self.msg_left_queue[0]
            msg_right = self.msg_right_queue[0]
            time_left = self.get_timestamp_second(msg_left)
            time_right = self.get_timestamp_second(msg_right)

            if abs(time_left - time_right) < self.time_diff:
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
