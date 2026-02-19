"""팀장 에이전트 - 인지/판단/제어를 통합하는 메인 자율주행 노드."""

import rclpy
from rclpy.node import Node

from team_leader.perception import PerceptionModule
from team_leader.planning import PlanningModule
from team_leader.control import ControlModule


class LeaderNode(Node):
    """팀장 노드: 인지-판단-제어 파이프라인을 주기적으로 실행.

    타이머 기반으로 매 사이클마다:
    1. 인지 모듈에서 센서 데이터 결과를 수집
    2. 판단 모듈에서 주행 계획을 수립
    3. 제어 모듈에서 명령을 생성/퍼블리시
    """

    def __init__(self):
        super().__init__('team_leader')

        # 파라미터 선언
        self._declare_parameters()

        # 모듈 초기화
        self.perception = PerceptionModule(self)
        self.planning = PlanningModule(self)
        self.control = ControlModule(self)

        # 메인 루프 타이머 (주기: control_rate Hz)
        control_rate = self.get_parameter('control_rate').value
        timer_period = 1.0 / control_rate
        self._timer = self.create_timer(timer_period, self._main_loop)

        self.get_logger().info(
            f'[팀장] 노드 시작 (제어 주기: {control_rate} Hz)')

    def _declare_parameters(self):
        """노드 파라미터 선언."""
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('max_speed', 15.0)
        self.declare_parameter('safe_distance', 10.0)
        self.declare_parameter('control.speed_kp', 1.0)
        self.declare_parameter('control.speed_ki', 0.01)
        self.declare_parameter('control.speed_kd', 0.1)
        self.declare_parameter('control.steer_kp', 1.5)

        # 토픽명 파라미터
        self.declare_parameter('topics.cmd_vel', '/cmd_vel')
        self.declare_parameter('topics.camera', '/sensor/camera/front')
        self.declare_parameter('topics.lidar', '/sensor/lidar')
        self.declare_parameter('topics.gps', '/sensor/gps')
        self.declare_parameter('topics.imu', '/sensor/imu')

    def _main_loop(self):
        """인지 → 판단 → 제어 파이프라인 1사이클 실행."""
        # 1. 인지: 센서 데이터 결과 수집
        perception_result = self.perception.get_perception_result()

        # 2. 판단: 인지 결과 기반 주행 계획
        self.planning.update(perception_result)
        plan_result = self.planning.get_plan_result()

        # 3. 제어: 계획에 따른 명령 생성 및 퍼블리시
        self.control.execute(plan_result)

        self.get_logger().debug(
            f'[팀장] 모드={plan_result["mode"]}, '
            f'속도={plan_result["target_speed"]:.1f}m/s, '
            f'조향={plan_result["target_steering"]:.2f}rad')


def main(args=None):
    rclpy.init(args=args)
    node = LeaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[팀장] 노드 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
