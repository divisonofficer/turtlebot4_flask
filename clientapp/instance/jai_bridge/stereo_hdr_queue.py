from collections import deque
import time
from typing import Any, Dict, List, Optional
import threading
from std_msgs.msg import Header


class StereoHDRQueue:
    """
    HDR image들을 수집하는 queue
    여러 종류의 아이템 (header를 가지고 있는) 을 enqueue한다.
    큐가 너무 길어지면 가장 오래된 아이템을 pop한다.
    get_item을 했을 때, timestamp 이후에 들어온 모든 key 에 해당하는 아이템 하나씩을 반환한다.
    이때, timestamp 이전의 아이템은 모두 pop 한다.
    """

    def __init__(self, max_queue_size: int = 100):
        """
        초기화 메서드.

        Args:
            max_queue_size (int): 각 key 별 최대 큐 크기. 기본값은 100.
        """
        self.queues: Dict[str, deque] = {}
        self.max_queue_size = max_queue_size
        self.lock = threading.Lock()

    def register_key(self, key: str):
        """
        어떤 아이템을 수집할지 등록

        Args:
            key (str): 등록할 아이템의 키
        """
        with self.lock:
            if key not in self.queues:
                self.queues[key] = deque()
                print(f"Key '{key}' registered.")
            else:
                print(f"Key '{key}' is already registered.")

    def clear(self):
        """
        모든 큐를 비움
        """
        with self.lock:
            for key in self.queues:
                self.queues[key].clear()

    def enqueue(self, key: str, item: Any):
        """
        아이템을 큐에 넣기

        Args:
            key (str): 아이템의 키
            item (Any): 헤더를 가진 아이템 (header.stamp를 포함해야 함)
        """
        if key not in self.queues:
            raise ValueError(
                f"Key '{key}' is not registered. Please register it before enqueueing items."
            )

        with self.lock:
            queue = self.queues[key]
            # Assumes item.header.stamp is a float timestamp
            header: Header = item.header
            timestamp = header.stamp.sec + header.stamp.nanosec * 1e-9
            # Append the item maintaining the order
            queue.append(item)
            if key != "lidar":
                print(
                    f"Enqueued item with timestamp {timestamp} to key '{key}'. Queue size: {len(queue)}"
                )
            # If queue exceeds max size, pop the oldest item
            if len(queue) > self.max_queue_size:
                popped_item = queue.popleft()

    def get_item(self, timestamp: float) -> Optional[Dict[str, Any]]:
        """
        time 이후에 들어온 모든 key 에 해당하는 아이템 하나씩을 반환. 등록된 key의 모든 아이템중 하나 이상이 존재하지 않으면 None을 반환.

        Args:
            timestamp (float): 기준이 되는 타임스탬프

        Returns:
            Optional[Dict[str, Any]]: key를 기준으로 한 아이템의 딕셔너리 또는 None
        """
        with self.lock:
            result = {}
            print("Queue done :", [k for k, v in self.queues.items() if v])
            for key, queue in self.queues.items():
                # Remove items with timestamp <= given timestamp
                print(f"Checking key '{key}'...", len(queue))
                while (
                    queue
                    and queue[0].header.stamp.sec + queue[0].header.stamp.nanosec / 1e9
                    <= timestamp
                ):
                    popped = queue.popleft()
                    print(
                        f"Popped item with timestamp {popped.header.stamp} from key '{key}' as it's before or equal to {timestamp}."
                    )

                if queue:
                    # Get the first item after the timestamp
                    item = queue[0]
                    result[key] = item
                    print(
                        f"Selected item with timestamp {item.header.stamp} from key '{key}'."
                    )
                    continue
                return None

            return result if result else None
