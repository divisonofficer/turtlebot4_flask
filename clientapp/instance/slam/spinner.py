import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from typing import List

import threading
from rclpy.client import Future

from time import time, sleep
from typing import Dict, Optional


class Spinner:
    future_map: Dict[str, Future]

    def __init__(self):
        self.future_map = {}
        self.future_flag_map = {}
        self.nodes: List[Node] = []
        self.thread = None
        rclpy.init()
        self.executor = MultiThreadedExecutor()
        self.lock = threading.Lock()

    def add_node(self, node):
        self.nodes.append(node)
        return self

    def register_future(self, service_name: str, future):
        with self.lock:
            self.future_map[service_name] = future
            self.future_flag_map[service_name] = False

    def call_future_sync(self, service_name: str, future, timeout=10):
        self.register_future(service_name, future)
        time_begin = time()
        while not self.future_flag_map[service_name]:
            sleep(0.1)
            if time() - time_begin > timeout:
                break

        if self.future_flag_map[service_name]:
            result = future.result()
            with self.lock:
                del self.future_flag_map[service_name]
            return result

        return None

    def spin(self):
        while rclpy.ok():
            for node in self.nodes:
                rclpy.spin_once(node, executor=self.executor)

            for key, future in self.future_map.items():
                if future.done():
                    with self.lock:
                        self.future_flag_map[key] = True

    def spin_async(self):
        if self.thread:
            return

        self.thread = threading.Thread(target=self.spin)
        self.thread.start()
