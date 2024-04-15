from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import rclpy


class CaptureDiagnostic(Node):

    def __init__(self):
        super().__init__("client_capture_diagnostic")

    def check_slam_availability(self) -> bool:
        # /client_slam node is running
        node_names = self.get_node_names()
        if not "/client_slam" in node_names:
            # /client_slam node is running
            return False

        # /create a single time subscription to /map topic
        # and wait for 5 seconds to receive the message
        self.map_received = False

        def map_callback(msg):
            self.map_received = True

        self.create_subscription(OccupancyGrid, "/map", map_callback, 1)

        # wait for 5 seconds

        self.get_logger().info("Waiting for /map topic")

        rclpy.spin_once(self, timeout_sec=5)

        if not self.map_received:
            return False

        return True
