import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
import threading
from jai_rosbridge.action import HDRTrigger
import time


class JaiHDRPipelineNode(Node):
    def __init__(self):
        super().__init__("jai_hdr_pipeline_node")
        self.get_logger().info("JaiHDRPipelineNode initialized")

        self.action_client_hdr_trigger = ActionClient(
            self,
            HDRTrigger,
            "jai_hdr_trigger",
        )
        self.action_client_hdr_trigger.wait_for_server()

    def trigger_hdr(self):
        self.get_logger().info("Triggering HDR")
        self.start_time = time.time()  # Start time
        goal_msg = HDRTrigger.Goal()
        goal_msg.space_id = (
            "/home/cglab/project/turtlebot4_flask/clientapp/tmp/stereo/hdr/1/"
        )
        future = self.action_client_hdr_trigger.send_goal_async(
            goal_msg, feedback_callback=self.hdr_trigger_feedback_callback
        )

        future.add_done_callback(self.hdr_trigger_done_callback)

    def hdr_trigger_feedback_callback(self, feedback_msg):
        self.get_logger().info("Received feedback: {0}".format(feedback_msg.feedback))

    def hdr_trigger_done_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        future = goal_handle.get_result_async()
        future.add_done_callback(self.hdr_trigger_result_callback)

    def hdr_trigger_result_callback(self, future):
        result = future.result().result
        end_time = time.time()  # End time
        duration = end_time - self.start_time  # Calculate duration
        self.get_logger().info("Result: {0}".format(result))
        self.get_logger().info("Action completed in {0:.2f} seconds".format(duration))


if __name__ == "__main__":
    rclpy.init()
    jai_hdr_pipeline_node = JaiHDRPipelineNode()
    jai_hdr_pipeline_node.trigger_hdr()
    rclpy.spin(jai_hdr_pipeline_node)
    jai_hdr_pipeline_node.destroy_node()
    rclpy.shutdown()
