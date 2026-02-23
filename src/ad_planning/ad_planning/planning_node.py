"""판단/경로 계획 노드 - 인지 결과를 기반으로 주행 계획 수립.

Fields2Cover 스타일 커버리지 경로 계획(CoveragePlanner)을 통합하여
Boustrophedon, Spiral, Racetrack 3종 패턴을 지원한다.
"""

import math
from enum import Enum, auto
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header

from ad_core.coverage_planner import (
    CoveragePlanner,
    CoverageConfig,
    CoveragePattern,
    FieldBoundary,
)


class DrivingMode(Enum):
    """주행 모드."""
    IDLE = auto()
    LANE_KEEPING = auto()
    LANE_CHANGE = auto()
    OBSTACLE_AVOIDANCE = auto()
    EMERGENCY_STOP = auto()
    COVERAGE = auto()
    WAYPOINT_NAV = auto()


class PathPlanner:
    """경로 생성기 - 커버리지 패턴 및 웨이포인트 경로 생성."""

    def __init__(self, node: Node):
        self._node = node
        self._coverage_planner = CoveragePlanner()
        node.get_logger().info('[경로생성] PathPlanner 초기화 완료')

    @property
    def coverage_planner(self) -> CoveragePlanner:
        return self._coverage_planner

    def generate_coverage_path(
        self,
        field_boundary: List[Tuple[float, float]],
        swath_width: float,
        pattern: CoveragePattern = CoveragePattern.BOUSTROPHEDON,
        overlap_ratio: float = 0.0,
        headland_width: float = 0.0,
        swath_angle_deg: Optional[float] = None,
    ) -> List[Tuple[float, float]]:
        """면적 커버리지 경로 생성."""
        if len(field_boundary) < 3:
            self._node.get_logger().error(
                '[경로생성] 영역 경계 좌표가 3개 미만입니다.'
            )
            return []

        if swath_width <= 0.0:
            self._node.get_logger().error(
                '[경로생성] 작업 폭(swath_width)은 양수여야 합니다.'
            )
            return []

        waypoints = self._coverage_planner.generate_coverage_path_compat(
            field_boundary=field_boundary,
            swath_width=swath_width,
            pattern=pattern,
            overlap_ratio=overlap_ratio,
            headland_width=headland_width,
            swath_angle_deg=swath_angle_deg,
        )

        self._node.get_logger().info(
            f'[경로생성] 커버리지 경로 생성 완료: '
            f'{len(waypoints)}개 웨이포인트, '
            f'패턴={pattern.name}'
        )
        return waypoints

    def _find_x_intersections(
        self,
        polygon: List[Tuple[float, float]],
        y: float,
    ) -> List[float]:
        """다각형과 수평선(y=const)의 교차 X 좌표를 계산."""
        intersections: List[float] = []
        n = len(polygon)

        for i in range(n):
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % n]

            if (y1 <= y < y2) or (y2 <= y < y1):
                if abs(y2 - y1) > 1e-10:
                    t = (y - y1) / (y2 - y1)
                    x_intersect = x1 + t * (x2 - x1)
                    intersections.append(x_intersect)

        return sorted(intersections)

    def generate_waypoint_path(
        self,
        waypoints: List[Tuple[float, float]],
        frame_id: str = 'map',
        z: float = 0.0,
    ) -> Path:
        """웨이포인트 리스트를 Nav2 Path 메시지로 변환."""
        path = Path()
        path.header = Header()
        path.header.frame_id = frame_id
        path.header.stamp = self._node.get_clock().now().to_msg()

        for i, (x, y_coord) in enumerate(waypoints):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y_coord)
            pose.pose.position.z = float(z)

            if i < len(waypoints) - 1:
                dx = waypoints[i + 1][0] - x
                dy = waypoints[i + 1][1] - y_coord
                yaw = math.atan2(dy, dx)
            elif i > 0:
                dx = x - waypoints[i - 1][0]
                dy = y_coord - waypoints[i - 1][1]
                yaw = math.atan2(dy, dx)
            else:
                yaw = 0.0

            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            path.poses.append(pose)

        self._node.get_logger().info(
            f'[경로생성] Path 메시지 생성 완료: {len(path.poses)}개 포즈'
        )
        return path


class PlanningModule:
    """인지 결과를 분석하여 경로 계획 및 주행 의사결정을 수행."""

    def __init__(self, node: Node):
        self._node = node
        self.current_mode = DrivingMode.IDLE
        self.target_speed = 0.0
        self.target_steering = 0.0

        self.max_speed = node.get_parameter('max_speed').value
        self.safe_distance = node.get_parameter('safe_distance').value

        self._path_planner = PathPlanner(node)

        self._coverage_waypoints: List[Tuple[float, float]] = []
        self._current_waypoint_index: int = 0
        self._nav_path: Optional[Path] = None

        node.get_logger().info('[판단] 모듈 초기화 완료')

    @property
    def path_planner(self) -> PathPlanner:
        return self._path_planner

    def update(self, perception_result: dict):
        """인지 결과를 바탕으로 주행 계획을 갱신."""
        obstacles = perception_result.get('obstacles', [])
        lane_info = perception_result.get('lane_info')

        if self._check_emergency(obstacles):
            self.current_mode = DrivingMode.EMERGENCY_STOP
            self.target_speed = 0.0
            self.target_steering = 0.0
            self._node.get_logger().warn('[판단] 긴급 정지!')
            return

        if self._check_obstacle_avoidance(obstacles):
            self.current_mode = DrivingMode.OBSTACLE_AVOIDANCE
            self._plan_avoidance(obstacles)
            return

        self.current_mode = DrivingMode.LANE_KEEPING
        self._plan_lane_keeping(lane_info)

    def plan(self, mode: DrivingMode, **kwargs):
        """지정된 주행 모드에 따라 경로 계획을 수행."""
        self.current_mode = mode

        if mode == DrivingMode.COVERAGE:
            field_boundary = kwargs.get('field_boundary', [])
            swath_width = kwargs.get('swath_width', 2.0)
            pattern_name = kwargs.get('pattern', 'BOUSTROPHEDON')
            overlap_ratio = kwargs.get('overlap_ratio', 0.0)
            headland_width = kwargs.get('headland_width', 0.0)
            swath_angle_deg = kwargs.get('swath_angle_deg', None)

            if isinstance(pattern_name, str):
                try:
                    pattern = CoveragePattern[pattern_name.upper()]
                except KeyError:
                    self._node.get_logger().warn(
                        f'[판단] 미지원 패턴: {pattern_name}, '
                        f'BOUSTROPHEDON 사용'
                    )
                    pattern = CoveragePattern.BOUSTROPHEDON
            elif isinstance(pattern_name, CoveragePattern):
                pattern = pattern_name
            else:
                pattern = CoveragePattern.BOUSTROPHEDON

            self._coverage_waypoints = self._path_planner.generate_coverage_path(
                field_boundary=field_boundary,
                swath_width=swath_width,
                pattern=pattern,
                overlap_ratio=overlap_ratio,
                headland_width=headland_width,
                swath_angle_deg=swath_angle_deg,
            )
            self._current_waypoint_index = 0

            if self._coverage_waypoints:
                self._nav_path = self._path_planner.generate_waypoint_path(
                    self._coverage_waypoints
                )
                self.target_speed = self.max_speed * 0.7
                self._node.get_logger().info(
                    f'[판단] 커버리지 모드 시작: '
                    f'{len(self._coverage_waypoints)}개 웨이포인트, '
                    f'패턴={pattern.name}'
                )
            else:
                self._node.get_logger().warn(
                    '[판단] 커버리지 경로 생성 실패, IDLE 복귀'
                )
                self.current_mode = DrivingMode.IDLE
                self.target_speed = 0.0

        elif mode == DrivingMode.WAYPOINT_NAV:
            waypoints = kwargs.get('waypoints', [])

            if waypoints:
                self._coverage_waypoints = waypoints
                self._current_waypoint_index = 0
                self._nav_path = self._path_planner.generate_waypoint_path(
                    waypoints
                )
                self.target_speed = self.max_speed
                self._node.get_logger().info(
                    f'[판단] 웨이포인트 내비게이션 시작: '
                    f'{len(waypoints)}개 웨이포인트'
                )
            else:
                self._node.get_logger().warn(
                    '[판단] 웨이포인트가 비어있습니다, IDLE 복귀'
                )
                self.current_mode = DrivingMode.IDLE
                self.target_speed = 0.0

        elif mode == DrivingMode.LANE_KEEPING:
            lane_info = kwargs.get('lane_info')
            self._plan_lane_keeping(lane_info)

        elif mode == DrivingMode.OBSTACLE_AVOIDANCE:
            obstacles = kwargs.get('obstacles', [])
            self._plan_avoidance(obstacles)

        elif mode == DrivingMode.EMERGENCY_STOP:
            self.target_speed = 0.0
            self.target_steering = 0.0
            self._node.get_logger().warn('[판단] 긴급 정지 모드 설정')

        elif mode == DrivingMode.IDLE:
            self.target_speed = 0.0
            self.target_steering = 0.0

        else:
            self._node.get_logger().warn(
                f'[판단] 미지원 주행 모드: {mode.name}'
            )

    def advance_waypoint(self, current_position: Tuple[float, float],
                         tolerance: float = 1.0) -> bool:
        """현재 위치를 기준으로 다음 웨이포인트로 진행."""
        if not self._coverage_waypoints:
            return True

        if self._current_waypoint_index >= len(self._coverage_waypoints):
            self._node.get_logger().info('[판단] 모든 웨이포인트 완료')
            self.current_mode = DrivingMode.IDLE
            self.target_speed = 0.0
            return True

        target = self._coverage_waypoints[self._current_waypoint_index]
        dx = target[0] - current_position[0]
        dy = target[1] - current_position[1]
        distance = math.sqrt(dx * dx + dy * dy)

        if distance < tolerance:
            self._current_waypoint_index += 1
            remaining = len(self._coverage_waypoints) - self._current_waypoint_index
            self._node.get_logger().info(
                f'[판단] 웨이포인트 도달, '
                f'남은 웨이포인트: {remaining}개'
            )

            if self._current_waypoint_index >= len(self._coverage_waypoints):
                self._node.get_logger().info('[판단] 모든 웨이포인트 완료')
                self.current_mode = DrivingMode.IDLE
                self.target_speed = 0.0
                return True

        return False

    def get_current_target_waypoint(self) -> Optional[Tuple[float, float]]:
        """현재 목표 웨이포인트 좌표를 반환."""
        if (self._coverage_waypoints and
                self._current_waypoint_index < len(self._coverage_waypoints)):
            return self._coverage_waypoints[self._current_waypoint_index]
        return None

    def get_nav_path(self) -> Optional[Path]:
        """생성된 Nav2 Path 메시지를 반환."""
        return self._nav_path

    def _check_emergency(self, obstacles: list) -> bool:
        for obs in obstacles:
            distance = obs.get('distance', float('inf'))
            if distance < self.safe_distance * 0.5:
                return True
        return False

    def _check_obstacle_avoidance(self, obstacles: list) -> bool:
        for obs in obstacles:
            distance = obs.get('distance', float('inf'))
            if distance < self.safe_distance:
                return True
        return False

    def _plan_avoidance(self, obstacles: list):
        self.target_speed = self.max_speed * 0.5
        self.target_steering = 0.0

    def _plan_lane_keeping(self, lane_info):
        self.target_speed = self.max_speed
        if lane_info and 'offset' in lane_info:
            self.target_steering = -lane_info['offset'] * 0.1
        else:
            self.target_steering = 0.0

    def get_plan_result(self) -> dict:
        """현재 주행 계획 결과를 반환."""
        result = {
            'mode': self.current_mode.name,
            'target_speed': self.target_speed,
            'target_steering': self.target_steering,
        }

        if self.current_mode in (DrivingMode.COVERAGE, DrivingMode.WAYPOINT_NAV):
            target_wp = self.get_current_target_waypoint()
            result['current_waypoint_index'] = self._current_waypoint_index
            result['total_waypoints'] = len(self._coverage_waypoints)
            result['target_waypoint'] = target_wp
            result['has_nav_path'] = self._nav_path is not None
            result['coverage_planner_available'] = True

        return result


class PlanningNode(Node):
    """판단/경로 계획 독립 ROS2 노드."""

    def __init__(self):
        super().__init__('planning_node')

        # 파라미터 선언
        self.declare_parameter('max_speed', 3.0)
        self.declare_parameter('safe_distance', 5.0)

        # 판단 모듈 초기화
        self.planning = PlanningModule(self)

        self.get_logger().info('[판단 노드] 시작')


def main(args=None):
    rclpy.init(args=args)
    node = PlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[판단 노드] 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
