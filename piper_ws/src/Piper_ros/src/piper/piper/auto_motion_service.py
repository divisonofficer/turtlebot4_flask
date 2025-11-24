import rclpy
from rclpy.node import Node
from piper_msgs.srv import AutoMotionToggle
from sensor_msgs.msg import JointState
from .auto_motion import generate_presets
import itertools, threading, time


class AutoMotionService(Node):
    def __init__(self):
        super().__init__('auto_motion_service')
        self.srv = self.create_service(AutoMotionToggle, 'AutoMotionToggle', self.handle_toggle)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.presets = generate_presets()
        self.iterator = itertools.cycle(self.presets)

        self.running = False
        self.thread = None
        self.stop_event = threading.Event()

    def handle_toggle(self, request, response):
        if request.enable and not self.running:
            self.running = True
            self.stop_event.clear()
            self.thread = threading.Thread(target=self.loop, daemon=True)
            self.thread.start()
            self.get_logger().info("Automatic motion started")
            response.success = True

        elif not request.enable and self.running:
            self.running = False
            self.stop_event.set()  # 즉시 종료 신호
            self.get_logger().info("Automatic motion stopped")
            response.success = True

        else:
            response.success = False

        return response

    def loop(self):
        while not self.stop_event.is_set():
            msg = JointState()
            msg.name = [f'joint{i+1}' for i in range(7)]
            msg.position = next(self.iterator)
            msg.velocity = [0.0] * 7
            msg.effort = [0.0] * 7
            self.pub.publish(msg)
            self.get_logger().info(f"Automatic preset published: {msg.position}")

            # sleep을 이벤트 기반으로 변경
            if self.stop_event.wait(timeout=10.0):
                break  # 이벤트가 set되면 즉시 종료


def main(args=None):
    rclpy.init(args=args)
    node = AutoMotionService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
