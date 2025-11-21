import rclpy
from rclpy.node import Node
from piper_msgs.srv import MoveArm
from sensor_msgs.msg import JointState
from .auto_motion import generate_presets


class MoveArmService(Node):
    def __init__(self):
        super().__init__('move_arm_service')
        self.srv = self.create_service(MoveArm, 'MoveArm', self.handle_move)
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.presets = generate_presets()

    def handle_move(self, request, response):
        idx = request.move_idx
        if idx < 0 or idx >= len(self.presets):
            self.get_logger().warn(f"Invalid move_idx: {idx}")
            response.success = False
            return response

        msg = JointState()
        msg.name = [f'joint{i+1}' for i in range(7)]
        msg.position = self.presets[idx]
        msg.velocity = [0.0] * 7
        msg.effort = [0.0] * 7

        self.pub.publish(msg)
        self.get_logger().info(f"Pose {idx} published")
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MoveArmService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
