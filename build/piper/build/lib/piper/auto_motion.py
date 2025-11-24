import math
import itertools
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# -----------------------------
# 각도 클래스
# -----------------------------
class Joint:
    def __init__(self):
        self.J1 = 0.0   #-math.pi / 2   # -90 deg
        self.J2 = 0.0
        self.J3 = 0.0
        self.J4 = 0.0
        self.J5 = -math.radians(10)
        self.J6 = math.radians(54)
        self.J7 = 0.0


# -----------------------------
# 프리셋 각도 계산 (라디안 반환)
# -----------------------------
def generate_presets():
    joints = []
    j0 = Joint()
    joints.append(j0)

    A_x_values = [5.9, 5.7, 5.3, 4.9]

    r1 = 2.55
    r2 = 2.87
    r = 6.05
    Cx, Cy = 6.05, 0.0

    for Ax in A_x_values:
        inside = r**2 - Ax**2
        if inside < 0:
            continue
        Ay = math.sqrt(inside)

        dx = Cx - Ax
        dy = Cy - Ay
        d = math.hypot(dx, dy)

        if d > r1 + r2 or d < abs(r1 - r2):
            continue

        a = (r1**2 - r2**2 + d**2) / (2 * d)
        h_sq = r1**2 - a**2
        h = math.sqrt(max(h_sq, 0.0))

        xm = Ax + a * dx / d
        ym = Ay + a * dy / d

        rx = -dy * (h / d)
        ry = dx * (h / d)

        B_candidates = [(xm + rx, ym + ry), (xm - rx, ym - ry)]
        dists = [Bx**2 + By**2 for (Bx, By) in B_candidates]
        Bx, By = B_candidates[0] if dists[0] >= dists[1] else B_candidates[1]

        # 각도 계산 (라디안으로 저장)
        v1 = (0 - Ax, 0 - Ay)
        v2 = (Bx - Ax, By - Ay)
        dot = v1[0] * v2[0] + v1[1] * v2[1]
        ang_OAB = math.radians(170) - math.acos(dot / (math.hypot(*v1) * math.hypot(*v2)))

        v1 = (Ax - Bx, Ay - By)
        v2 = (Cx - Bx, Cy - By)
        dot = v1[0] * v2[0] + v1[1] * v2[1]
        ang_ABC = math.acos(dot / (math.hypot(*v1) * math.hypot(*v2)))

        v1 = (Bx - Cx, By - Cy)
        v2 = (0 - Cx, Cy - Cy)
        dot = v1[0] * v2[0] + v1[1] * v2[1]
        ang_BCO = math.pi - math.acos(dot / (math.hypot(*v1) * math.hypot(*v2)))

        j = Joint()
        j.J5 = ang_OAB
        j.J3 = -ang_ABC
        j.J2 = ang_BCO
        joints.append(j)

    presets = [[j.J1, j.J2, j.J3, j.J4, j.J5, j.J6, j.J7] for j in joints]
    return presets


# -----------------------------
# ROS2 자동 발행 노드
# -----------------------------
class AutoMotion(Node):
    def __init__(self):
        super().__init__('auto_motion')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        self.presets_rad = generate_presets()
        self.iterator = itertools.cycle(self.presets_rad)

        self.timer = self.create_timer(7.0, self.timer_callback)

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'piper_single'
        msg.name = [f'joint{i+1}' for i in range(7)]

        rad_values = next(self.iterator)
        msg.position = rad_values
        msg.velocity = [0.0] * 7
        msg.effort = [0.0] * 7

        self.publisher_.publish(msg)
        self.get_logger().info(f'Preset sent (rad): {rad_values}')


def main(args=None):
    rclpy.init(args=args)
    node = AutoMotion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
