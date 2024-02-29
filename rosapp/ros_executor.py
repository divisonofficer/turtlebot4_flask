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
            self.executor.add_node(node)

    def remove_node_runtime(self, node):
        if node in self.nodes:
            self.nodes.remove(node)
            self.executor.remove_node(node)

    def executor_thread(self, nodes: list[Node]):
        self.executor = MultiThreadedExecutor()
        for node in nodes:
            self.nodes.add(node)
            self.executor.add_node(node)

        def spin():
            try:
                self.executor.spin()
            finally:
                self.executor.shutdown()
                for node in nodes:
                    node.destroy_node()
                rclpy.shutdown()
            # Spin in a separate thread

        thread = threading.Thread(target=spin)
        thread.start()
        return self.executor, thread
