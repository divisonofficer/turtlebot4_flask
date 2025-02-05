import math
import time
from typing import Optional
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from nav_msgs.msg import Odometry
from irobot_create_msgs.action import RotateAngle, DriveDistance
from geometry_msgs.msg import Twist
import asyncio
from rclpy.executors import MultiThreadedExecutor

RANGER_ID = "odom"


class RangerStatus:
    odom: Optional[Odometry] = None
    # 최대 각속도 설정 (라디안/초)
    max_angular_speed = 0.1  # rad/s
    max_linear_speed = 0.1  # m/s
    loop_rate = 50  # Hz


class RangerNode(Node):
    def __init__(self):
        super().__init__("ranger_agent_node")
        self.status = RangerStatus()
        self.topic_odom = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self._action_robot_angular = ActionServer(
            self, RotateAngle, "rotate_angle", self.action_robot_angular_callback
        )

        self._action_robot_drive_side = ActionServer(
            self,
            DriveDistance,
            "drive_distance_side",
            self.action_robot_drive_side_callback,
        )

        self._publisher_cmd = self.create_publisher(Twist, "/cmd_vel", 10)

    def odom_callback(self, msg):
        if msg.header.frame_id == RANGER_ID:
            self.status.odom = msg

    async def action_robot_drive_side_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("이동 목표를 수신했습니다.")
        result = DriveDistance.Result()
        goal: DriveDistance.Goal = goal_handle.request
        goal_distance = goal.distance

        # 목표 거리 설정
        distance = goal_distance
        sleep_time = 1.0 / self.status.loop_rate

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        # odometry 데이터 확인
        if self.status.odom is None:
            self.get_logger().error("Odometry 데이터가 아직 수신되지 않았습니다.")
            goal_handle.abort()
            return result

        # 시작 pose 기록
        start_pose = self.status.odom.pose.pose
        # quaternion으로부터 yaw (회전각) 계산
        q = start_pose.orientation
        start_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        # 로봇의 local y 방향 unit vector 계산 (ROS에서는 기본적으로 x: forward, y: left)
        local_y_unit = (-math.sin(start_yaw), math.cos(start_yaw))
        # 시작 위치 (global 좌표계)
        start_pos = (start_pose.position.x, start_pose.position.y)

        while rclpy.ok():
            # odometry 데이터 재확인
            if self.status.odom is None:
                self.get_logger().warn("Odometry 데이터가 갱신되지 않았습니다.")
                time.sleep(sleep_time)
                continue

            current_pose = self.status.odom.pose.pose
            current_pos = (current_pose.position.x, current_pose.position.y)
            # 시작 위치와 현재 위치 사이의 displacement vector 계산
            dx = current_pos[0] - start_pos[0]
            dy = current_pos[1] - start_pos[1]
            # scalar projection: displacement vector와 local y unit vector의 내적 계산
            traveled_distance = dx * local_y_unit[0] + dy * local_y_unit[1]
            # 남은 거리 계산
            remaining_distance = distance - traveled_distance

            # 비례 속도 제어 (local y 방향으로 제어)
            cmd.linear.y = (
                self.status.max_linear_speed
                * (1 if remaining_distance > 0 else -1)
                * min(1.0, abs(remaining_distance) / 0.1)
            )

            # 이동 명령 전송
            self._publisher_cmd.publish(cmd)

            # 목표 도달 판단 (0.01 m 이하의 오차 허용)
            if abs(remaining_distance) < 0.01:
                self.get_logger().info("목표 거리에 도달했습니다.")
                break

            # 목표 취소 요청 확인
            if goal_handle.is_cancel_requested:
                self.get_logger().info("이동 목표가 취소되었습니다.")
                goal_handle.canceled()
                cmd.linear.y = 0.0
                self._publisher_cmd.publish(cmd)
                return result

            self.get_logger().info(
                f"이동한 거리: {traveled_distance:.2f}, 목표 거리: {distance:.2f}, 남은 거리: {remaining_distance:.2f}"
            )

            time.sleep(sleep_time)

        # 이동 정지
        cmd.linear.y = 0.0
        self._publisher_cmd.publish(cmd)
        # 목표 성공 처리
        goal_handle.succeed()
        self.get_logger().info("이동이 성공적으로 완료되었습니다.")
        result.pose.pose = self.status.odom.pose.pose
        result.pose.header = self.status.odom.header
        return result

    async def action_robot_angular_callback(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("회전 목표를 수신했습니다.")
        result = RotateAngle.Result()

        goal = goal_handle.request
        angle_deg = goal.angle  # 각도가 도 단위라고 가정
        angle_rad = math.radians(angle_deg)  # 라디안으로 변환

        # 현재 위치 및 방향 기록
        if self.status.odom is None:
            self.get_logger().error("Odometry 데이터가 아직 수신되지 않았습니다.")
            goal_handle.abort()
            return result

        start_pose = self.status.odom.pose.pose
        start_yaw = self.get_yaw_from_quaternion(start_pose.orientation)

        # 목표 방향 계산 (오른쪽 회전이므로 음수 각도)
        target_yaw = start_yaw - angle_rad
        target_yaw = self.normalize_angle(target_yaw)

        sleep_time = 1.0 / self.status.loop_rate

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        while rclpy.ok():
            # 현재 방향 가져오기
            if self.status.odom is None:
                self.get_logger().warn("Odometry 데이터가 갱신되지 않았습니다.")
                time.sleep(sleep_time)
                continue

            current_pose = self.status.odom.pose.pose
            current_yaw = self.get_yaw_from_quaternion(current_pose.orientation)

            # 남은 회전 각도 계산
            delta_yaw = self.angle_difference(target_yaw, current_yaw)

            # 비례 속도 제어
            cmd.angular.z = (
                self.status.max_angular_speed
                * (-1 if delta_yaw < 0 else 1)
                * min(1.0, abs(delta_yaw) / 0.1)
            )

            # 회전 명령 전송
            self._publisher_cmd.publish(cmd)

            # 목표에 도달했는지 확인
            if abs(delta_yaw) < 0.01:  # 0.01 rad 이하이면 도달한 것으로 간주
                self.get_logger().info("목표 각도에 도달했습니다.")
                break

            # 목표 취소 요청 확인
            if goal_handle.is_cancel_requested:
                self.get_logger().info("회전 목표가 취소되었습니다.")
                goal_handle.canceled()
                cmd.angular.z = 0.0
                self._publisher_cmd.publish(cmd)
                return result

            self.get_logger().info(
                f"현재 각도: {current_yaw:.2f}, 목표 각도: {target_yaw:.2f}, 남은 각도: {delta_yaw:.2f}"
            )

            time.sleep(sleep_time)
        # 회전 정지
        cmd.angular.z = 0.0
        self._publisher_cmd.publish(cmd)
        # 목표 성공 처리
        goal_handle.succeed()
        self.get_logger().info("회전이 성공적으로 완료되었습니다.")
        result.pose.pose = self.status.odom.pose.pose
        result.pose.header = self.status.odom.header
        return result

    def get_yaw_from_quaternion(self, orientation):
        """
        쿼터니언에서 Yaw 값을 추출하는 함수
        """
        q = orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def angle_difference(self, target, current):
        """
        ��� 각도 간의 차이를 계산하여 -pi ~ pi 범위로 반환
        """
        diff = target - current
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff

    def normalize_angle(self, angle):
        """
        각도를 -pi ~ pi 범위로 정규화
        """
        return math.atan2(math.sin(angle), math.cos(angle))

    def __del__(self):
        cmd = Twist()
        self._publisher_cmd.publish(cmd)


if __name__ == "__main__":
    rclpy.init()
    node = RangerNode()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    rclpy.shutdown()
