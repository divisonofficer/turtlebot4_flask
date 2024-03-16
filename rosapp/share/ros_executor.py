from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import threading
import rclpy


class RosExecutor:
    nodes = set()
    executor = None

    def add_node_runtime(self, node):
        if node not in self.nodes:
            self.nodes.add(node)
            if self.executor:
                self.executor.add_node(node)  # Add node to the spinning executor

    def remove_node_runtime(self, node):
        if node in self.nodes:
            self.nodes.remove(node)
            if self.executor:
                self.executor.remove_node(node)

    def spin(self):
        try:
            print("Spinning")
            self.executor.spin()
        finally:
            self.executor.shutdown()

    def executor_thread(self, nodes: list[Node]):
        self.executor = MultiThreadedExecutor()

        def _spin():
            try:
                for node in nodes:
                    self.executor.add_node(node)
                self.executor.spin()
            finally:
                rclpy.shutdown()

        threading.Thread(target=_spin).start()

    def stop_and_re_spin(self):
        self.executor.shutdown()
        self.executor = MultiThreadedExecutor()
        for node in self.nodes:
            self.executor.add_node(node)
        self.thread = threading.Thread(target=self.spin)
        self.thread.start()
        return self.executor, self.thread
