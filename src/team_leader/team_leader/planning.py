"""판단/경로 계획 모듈 - 인지 결과를 기반으로 주행 계획 수립."""

import math
from enum import Enum, auto

from rclpy.node import Node


class DrivingMode(Enum):
    """주행 모드."""
    IDLE = auto()
    LANE_KEEPING = auto()
    LANE_CHANGE = auto()
    OBSTACLE_AVOIDANCE = auto()
    EMERGENCY_STOP = auto()


class PlanningModule:
    """인지 결과를 분석하여 경로 계획 및 주행 의사결정을 수행.

    현재 주행 모드를 관리하고, 인지 결과에 따라
    목표 속도와 조향 각도를 계산한다.
    """

    def __init__(self, node: Node):
        self._node = node
        self.current_mode = DrivingMode.IDLE
        self.target_speed = 0.0        # m/s
        self.target_steering = 0.0     # rad

        # 파라미터에서 설정 읽기
        self.max_speed = node.get_parameter('max_speed').value
        self.safe_distance = node.get_parameter('safe_distance').value

        node.get_logger().info('[판단] 모듈 초기화 완료')

    def update(self, perception_result: dict):
        """인지 결과를 바탕으로 주행 계획을 갱신.

        Args:
            perception_result: PerceptionModule.get_perception_result()의 반환값
        """
        obstacles = perception_result.get('obstacles', [])
        lane_info = perception_result.get('lane_info')

        # 긴급 정지 판단
        if self._check_emergency(obstacles):
            self.current_mode = DrivingMode.EMERGENCY_STOP
            self.target_speed = 0.0
            self.target_steering = 0.0
            self._node.get_logger().warn('[판단] 긴급 정지!')
            return

        # 장애물 회피 판단
        if self._check_obstacle_avoidance(obstacles):
            self.current_mode = DrivingMode.OBSTACLE_AVOIDANCE
            self._plan_avoidance(obstacles)
            return

        # 기본: 차선 유지
        self.current_mode = DrivingMode.LANE_KEEPING
        self._plan_lane_keeping(lane_info)

    def _check_emergency(self, obstacles: list) -> bool:
        """전방 근접 장애물 존재 시 긴급 정지 판단."""
        for obs in obstacles:
            distance = obs.get('distance', float('inf'))
            if distance < self.safe_distance * 0.5:
                return True
        return False

    def _check_obstacle_avoidance(self, obstacles: list) -> bool:
        """안전 거리 이내 장애물 존재 시 회피 기동 판단."""
        for obs in obstacles:
            distance = obs.get('distance', float('inf'))
            if distance < self.safe_distance:
                return True
        return False

    def _plan_avoidance(self, obstacles: list):
        """장애물 회피 경로 계획."""
        # TODO: 실제 회피 경로 계산 알고리즘
        self.target_speed = self.max_speed * 0.5
        self.target_steering = 0.0

    def _plan_lane_keeping(self, lane_info):
        """차선 유지 주행 계획."""
        self.target_speed = self.max_speed
        if lane_info and 'offset' in lane_info:
            # 차선 중앙으로부터의 오프셋에 비례하여 조향
            self.target_steering = -lane_info['offset'] * 0.1
        else:
            self.target_steering = 0.0

    def get_plan_result(self) -> dict:
        """현재 주행 계획 결과를 반환."""
        return {
            'mode': self.current_mode.name,
            'target_speed': self.target_speed,
            'target_steering': self.target_steering,
        }
