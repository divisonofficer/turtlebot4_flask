import rclpy
from rclpy.node import Node
from flask import Flask, render_template, request, jsonify
from threading import Thread, Lock
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from piper_msgs.srv import MoveArm, AutoMotionToggle

app = Flask(__name__)

class FlaskPiperBridge(Node):
    def __init__(self):
        super().__init__('flask_piper_bridge')

        # ──────────────────────────────
        # ROS2 서비스 및 토픽 연결 설정
        # ──────────────────────────────
        self.cli_move = self.create_client(MoveArm, 'MoveArm')
        self.cli_auto = self.create_client(AutoMotionToggle, 'AutoMotionToggle')
        self.cli_enable = self.create_client(SetBool, '/enable_srv')
        self.sub_joint = self.create_subscription(JointState, '/joint_states_single', self.joint_callback, 10)

        self.lock = Lock()
        self.latest_joints = [0.0] * 7

        # 서비스 대기 (최대 3초까지만)
        self.wait_for_service(self.cli_move, 'MoveArm', timeout=3.0)
        self.wait_for_service(self.cli_auto, 'AutoMotionToggle', timeout=3.0)
        self.wait_for_service(self.cli_enable, '/enable_srv', timeout=3.0)

        self.get_logger().info("Flask <-> ROS2 인터페이스 초기화 완료")

    def wait_for_service(self, client, name, timeout=3.0):
        waited = 0.0
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"{name} 서비스 대기 중...")
            waited += 1.0
            if waited >= timeout:
                self.get_logger().warn(f"{name} 서비스가 아직 준비되지 않았습니다. Flask UI를 먼저 띄웁니다.")
                break

    # ──────────────────────────────
    # 콜백 함수: 현재 관절 상태 저장
    # ──────────────────────────────
    def joint_callback(self, msg):
        with self.lock:
            self.latest_joints = list(msg.position)

    # ──────────────────────────────
    # Flask에서 주기적으로 조회할 함수
    # ──────────────────────────────
    def get_joints(self):
        with self.lock:
            return self.latest_joints.copy()

    # ──────────────────────────────
    # 자동 제어 ON/OFF (AutoMotionToggle or /enable_srv)
    # ──────────────────────────────
    def toggle_auto(self, enable: bool):
        if self.cli_auto.service_is_ready():
            req = AutoMotionToggle.Request()
            req.enable = enable
            future = self.cli_auto.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            return future.result().success if future.result() else False
        else:
            # fallback: /enable_srv
            req = SetBool.Request()
            req.data = enable
            future = self.cli_enable.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            return future.result().success if future.result() else False

    # ──────────────────────────────
    # 수동 Pose 이동
    # ──────────────────────────────
    def move_pose(self, move_idx: int):
        if not self.cli_move.service_is_ready():
            self.get_logger().warn("MoveArm 서비스 준비 안됨")
            return False
        req = MoveArm.Request()
        req.move_idx = move_idx
        future = self.cli_move.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success if future.result() else False


# ──────────────────────────────
# Flask 라우팅
# ──────────────────────────────
rclpy.init(args=None)
bridge = FlaskPiperBridge()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/auto', methods=['POST'])
def toggle_auto():
    data = request.json
    enable = bool(data['enable'])
    success = bridge.toggle_auto(enable)
    return jsonify({'success': success})

@app.route('/move', methods=['POST'])
def move():
    data = request.json
    move_idx = int(data['move_idx'])
    success = bridge.move_pose(move_idx)
    return jsonify({'success': success})

@app.route('/joints', methods=['GET'])
def joints():
    positions = bridge.get_joints()
    return jsonify({'joints': positions})


# ──────────────────────────────
# ROS2 + Flask 동시 실행
# ──────────────────────────────
def ros_spin():
    rclpy.spin(bridge)

def run_flask():
    app.run(host='0.0.0.0', port=5000, debug=False)

if __name__ == '__main__':
    Thread(target=ros_spin, daemon=True).start()
    run_flask()
