import time
from typing import Any


class SQueue:
    class Item:
        timestamp: float
        data: Any

        def __init__(self, timestamp: float, data: Any):
            self.timestamp = timestamp
            self.data = data

    def __init__(self):
        self.queue_dict = {}
        self.max_time_interval = 0.05  # 50ms
        self.max_queue_length = 20
        self.root_key = None

    def register_synchronized_queue(self, key: str, is_root=False):
        if not key in self.queue_dict:
            self.queue_dict[key] = []
        if is_root:
            self.root_key = key

    def unregister_synchronized_queue(self, key: str):
        if key in self.queue_dict:
            del self.queue_dict

    def enqueue(self, key: str, timestamp: float, data: Any):
        if not key in self.queue_dict:
            return
        self.queue_dict[key].append(self.Item(timestamp, data))
        if len(self.queue_dict[key]) > self.max_queue_length:
            self.queue_dict[key].pop(0)

    def loop_yield_synchronized_items(self, callback):
        if not self.root_key:
            return
        while True:
            if len(self.queue_dict[self.root_key]) == 0:
                continue
            root_item = self.queue_dict[self.root_key][0]
            ret_items = {}
            ret_items[self.root_key] = root_item
            for key in self.queue_dict:
                if key == self.root_key:
                    continue
                if len(self.queue_dict[key]) == 0:
                    continue
                while len(self.queue_dict[key]) > 0:
                    item = self.queue_dict[key][0]
                    if root_item.timestamp - item.timestamp > self.max_time_interval:
                        self.queue_dict[key].pop(0)
                    else:
                        if (
                            item.timestamp - root_item.timestamp
                            < self.max_time_interval
                        ):
                            ret_items[key] = self.queue_dict[key].pop(0)
                        break
            if len(ret_items) == len(self.queue_dict):
                callback(ret_items)
            self.queue_dict[self.root_key].pop(0)
            time.sleep(0.05)
