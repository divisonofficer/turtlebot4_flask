import math
from threading import Lock
import time

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
def generate_presets(preset_joints = []):
    joints = []
    j0 = Joint()
    joints.append(j0)

    A_x_values = [5.9, 5.7, 5.3, 4.9]

    r1 = 2.55
    r2 = 2.87
    r = 6.05
    Cx, Cy = 6.05, 0.0

    for idx, Ax in enumerate(A_x_values):
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
        if idx == 2:
            j.J3 = (-1.1728 + -1.173806) / 2
            
        if idx == 3:
            j.J3 = -1.513598

        if idx == 0:
            j.J5 = -0.0500
 

        joints.append(j)

    if len(preset_joints):
        pj1, pj4, pj6, pj7 = preset_joints
    else:
        pj1, pj4, pj6, pj7 = [0,0,-0.174,0.877]
    presets = [[pj1, j.J2, j.J3, pj4, j.J5, pj6, pj7] for j in joints]
    return presets


class PiperClient:
    """
    Piper 로봇 암 제어 클라이언트.
    - 외부 노드를 인자로 받아서 publisher/subscriber 생성
    - 프리셋 포즈로 로봇을 이동시키는 기능
    - 현재 관절 상태를 구독하여 모니터링
    - 로봇이 움직이는 중에는 새로운 명령을 거부하는 안전장치
    """
    
    # 속도 임계값 (rad/s) - 이 값보다 큰 속도가 감지되면 로봇이 움직이는 것으로 판단
    VELOCITY_THRESHOLD = 0.6
    
    def __init__(self, node: Node):
        """
        PiperClient 초기화.
        
        Args:
            node: ROS2 노드 (publisher/subscriber 생성에 사용)
        """
        self._node = node
        
        # ──────────────────────────────
        # 프리셋 포즈 생성
        # ──────────────────────────────
        self.presets = None
        
        # ──────────────────────────────
        # 퍼블리셔: 로봇 관절 제어 명령
        # ──────────────────────────────
        self.pub = self._node.create_publisher(JointState, '/joint_ctrl_single', 10)
        
        self.joint_state_subscription = self._node.create_subscription(JointState, "/joint_states_single", self.callback_joint_states, 10)
        
        # ──────────────────────────────
        # 구독자: 현재 관절 상태 읽기
        # ──────────────────────────────
        self.sub_joint = self._node.create_subscription(
            JointState, 
            '/joint_states_single', 
            self.joint_callback, 
            10
        )
        
        # ──────────────────────────────
        # 상태 변수 (스레드 세이프)
        # ──────────────────────────────
        self.lock = Lock()
        self.latest_position = [0.0] * 7
        self.latest_velocity = [10.0] * 7
        self.latest_effort = [0.0] * 7
        self.current_pose_idx: int = -1  # 현재 포즈 인덱스 (-1 = 알 수 없음)
        
        
        
        
        
    
    def get_logger(self):
        """노드의 로거 반환"""
        return self._node.get_logger()
    
    def callback_joint_states(self, state):
        if self.presets is not None:
            return
        p = state.position
        joints = [p[0], p[3],p[5],p[6]]
        self.presets = generate_presets(joints)
        self.get_logger().info("PiperClient 초기화 완료")
        self.get_logger().info(f"프리셋 포즈 개수: {len(self.presets)}")
        
        # Publisher warming up: 홈 위치(0번)로 이동
        self.piper_move_arm(0)
    
    # ──────────────────────────────
    # 콜백: 현재 관절 상태 업데이트
    # ──────────────────────────────
    def joint_callback(self, msg: JointState):
        with self.lock:
            if msg.position:
                self.latest_position = list(msg.position)
            if msg.velocity:
                self.latest_velocity = list(msg.velocity)
            if msg.effort:
                self.latest_effort = list(msg.effort)
    
    # ──────────────────────────────
    # 현재 관절 상태 조회
    # ──────────────────────────────
    def get_joint_position(self) -> list:
        """현재 관절 위치 반환 (rad)"""
        with self.lock:
            return self.latest_position.copy()
    
    def get_joint_velocity(self) -> list:
        """현재 관절 속도 반환 (rad/s)"""
        with self.lock:
            return self.latest_velocity.copy()
    
    def get_joint_effort(self) -> list:
        """현재 관절 토크 반환"""
        with self.lock:
            return self.latest_effort.copy()
    
    def get_joint_state(self) -> dict:
        """현재 관절 전체 상태 반환"""
        with self.lock:
            return {
                'position': self.latest_position.copy(),
                'velocity': self.latest_velocity.copy(),
                'effort': self.latest_effort.copy()
            }
    
    # ──────────────────────────────
    # 로봇 움직임 상태 확인
    # ──────────────────────────────
    def is_moving(self) -> bool:
        """
        로봇이 현재 움직이고 있는지 확인.
        속도 값이 임계값을 초과하면 움직이는 것으로 판단.
        """
        with self.lock:
            for vel in self.latest_velocity:
                if abs(vel) > self.VELOCITY_THRESHOLD:
                    print(f"vel : {abs(vel)} > {self.VELOCITY_THRESHOLD}")
                    return True
            return False
    
    # ──────────────────────────────
    # 로봇 암 이동 명령
    # ──────────────────────────────
    def piper_move_arm(self, move_idx: int) -> bool:
        """
        지정된 프리셋 인덱스로 로봇 암을 이동.
        
        Args:
            move_idx: 프리셋 포즈 인덱스 (0 = home, 1~4 = 촬영 위치)
        
        Returns:
            성공 여부 (True/False)
        """
        # 안전 체크: 로봇이 움직이는 중이면 거부
        # if self.is_moving():
        #     self.get_logger().warn("로봇이 움직이는 중입니다. 명령을 거부합니다.")
        #     return False
        
        # 인덱스 유효성 검사
        if move_idx < 0 or move_idx >= len(self.presets):
            self.get_logger().warn(f"유효하지 않은 move_idx: {move_idx} (범위: 0~{len(self.presets)-1})")
            return False
        
        # 이미 같은 위치에 있으면 스킵
        if self.current_pose_idx == move_idx:
            self.get_logger().info(f"Pose {move_idx} 이미 해당 위치, 스킵")
            return True
        
        # JointState 메시지 생성 및 발행
        msg = JointState()
        msg.name = [f'joint{i+1}' for i in range(7)]
        msg.position = self.presets[move_idx]
        msg.velocity = [20.0] * 7
        msg.effort = [0.0] * 7
        
        self.pub.publish(msg)
        self.current_pose_idx = move_idx  # 현재 위치 업데이트
        self.get_logger().info(f"Pose {move_idx} 발행 완료")
        return True
    
    def piper_move_to_home(self) -> bool:
        """로봇 암을 홈 위치(인덱스 0)로 이동"""
        return self.piper_move_arm(0)
    
    def piper_move_arm_force(self, move_idx: int) -> bool:
        """
        안전 체크 없이 강제로 로봇 암을 이동.
        주의: 움직이는 중에도 명령을 보냄.
        
        Args:
            move_idx: 프리셋 포즈 인덱스
        
        Returns:
            성공 여부 (True/False)
        """
        # 인덱스 유효성 검사만 수행
        if move_idx < 0 or move_idx >= len(self.presets):
            self.get_logger().warn(f"유효하지 않은 move_idx: {move_idx} (범위: 0~{len(self.presets)-1})")
            return False
        
        # JointState 메시지 생성 및 발행
        msg = JointState()
        msg.name = [f'joint{i+1}' for i in range(7)]
        msg.position = self.presets[move_idx]
        msg.velocity = [20.0] * 7
        msg.effort = [0.0] * 7
        
        self.pub.publish(msg)
        self.get_logger().info(f"Pose {move_idx} 강제 발행 완료")
        return True
    
    def get_preset_count(self) -> int:
        """사용 가능한 프리셋 포즈 개수 반환"""
        return len(self.presets)


def main(args=None):
    rclpy.init(args=args)
    node = Node('jai_piper_client')
    piper_client = PiperClient(node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
