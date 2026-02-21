"""판단/경로 계획 모듈 - 인지 결과를 기반으로 주행 계획 수립."""

import math
from enum import Enum, auto
from typing import List, Tuple, Optional

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header


class DrivingMode(Enum):
    """주행 모드."""
    IDLE = auto()
    LANE_KEEPING = auto()
    LANE_CHANGE = auto()
    OBSTACLE_AVOIDANCE = auto()
    EMERGENCY_STOP = auto()
    COVERAGE = auto()          # 면적 커버리지 주행 (예: 농경지 탐사)
    WAYPOINT_NAV = auto()      # 웨이포인트 순차 내비게이션


class PathPlanner:
    """경로 생성기 - 커버리지 패턴 및 웨이포인트 경로 생성.

    면적 커버리지(boustrophedon) 패턴 생성 및
    웨이포인트 기반 Nav2 Path 메시지 생성 기능을 제공한다.
    """

    def __init__(self, node: Node):
        self._node = node
        node.get_logger().info('[경로생성] PathPlanner 초기화 완료')

    def generate_coverage_path(
        self,
        field_boundary: List[Tuple[float, float]],
        swath_width: float,
    ) -> List[Tuple[float, float]]:
        """면적 커버리지 경로 생성 (boustrophedon 패턴).

        직사각형 또는 다각형 영역을 지그재그(왕복) 패턴으로 커버하는
        웨이포인트 리스트를 생성한다.

        Args:
            field_boundary: 영역 경계 좌표 리스트 [(x1,y1), (x2,y2), ...]
                최소 3개 이상의 꼭짓점이 필요하다.
            swath_width: 작업 폭 (m) - 인접 경로 간 간격

        Returns:
            커버리지 웨이포인트 리스트 [(x, y), ...]
        """
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

        # 영역 바운딩 박스 계산
        xs = [p[0] for p in field_boundary]
        ys = [p[1] for p in field_boundary]
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)

        # Boustrophedon(왕복) 패턴 생성
        # Y축 방향으로 swath_width 간격의 스트립을 X축 방향으로 왕복
        waypoints: List[Tuple[float, float]] = []
        y = y_min + swath_width / 2.0  # 첫 번째 스트립 중심
        strip_index = 0

        while y <= y_max:
            # 현재 Y 좌표에서 영역 경계와의 교차 구간 계산
            x_intersections = self._find_x_intersections(
                field_boundary, y
            )

            if len(x_intersections) >= 2:
                # 교차 구간의 최소/최대 X 값 사용
                strip_x_min = min(x_intersections)
                strip_x_max = max(x_intersections)
            else:
                # 교차점이 부족하면 바운딩 박스 사용
                strip_x_min = x_min
                strip_x_max = x_max

            # 짝수 인덱스: 왼쪽→오른쪽, 홀수 인덱스: 오른쪽→왼쪽
            if strip_index % 2 == 0:
                waypoints.append((strip_x_min, y))
                waypoints.append((strip_x_max, y))
            else:
                waypoints.append((strip_x_max, y))
                waypoints.append((strip_x_min, y))

            y += swath_width
            strip_index += 1

        self._node.get_logger().info(
            f'[경로생성] 커버리지 경로 생성 완료: '
            f'{len(waypoints)}개 웨이포인트, '
            f'{strip_index}개 스트립'
        )
        return waypoints

    def _find_x_intersections(
        self,
        polygon: List[Tuple[float, float]],
        y: float,
    ) -> List[float]:
        """다각형과 수평선(y=const)의 교차 X 좌표를 계산.

        레이캐스팅 방식으로 다각형 변과 수평선의 교차점을 찾는다.

        Args:
            polygon: 다각형 꼭짓점 좌표 리스트
            y: 수평선의 Y 좌표

        Returns:
            교차 X 좌표 리스트
        """
        intersections: List[float] = []
        n = len(polygon)

        for i in range(n):
            # 현재 변: polygon[i] → polygon[(i+1) % n]
            x1, y1 = polygon[i]
            x2, y2 = polygon[(i + 1) % n]

            # 수평선이 변의 Y 범위 내에 있는지 확인
            if (y1 <= y < y2) or (y2 <= y < y1):
                # 선형 보간으로 교차 X 좌표 계산
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
        """웨이포인트 리스트를 Nav2 Path 메시지로 변환.

        각 웨이포인트에 대해 PoseStamped를 생성하고,
        연속된 웨이포인트 사이의 방향(yaw)을 자동으로 계산한다.

        Args:
            waypoints: 웨이포인트 좌표 리스트 [(x, y), ...]
            frame_id: 경로 프레임 ID (기본값: 'map')
            z: Z 좌표 (기본값: 0.0, 평면 주행)

        Returns:
            Nav2 호환 Path 메시지
        """
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

            # 다음 웨이포인트를 향한 방향(yaw) 계산
            if i < len(waypoints) - 1:
                dx = waypoints[i + 1][0] - x
                dy = waypoints[i + 1][1] - y_coord
                yaw = math.atan2(dy, dx)
            elif i > 0:
                # 마지막 웨이포인트: 이전 방향 유지
                dx = x - waypoints[i - 1][0]
                dy = y_coord - waypoints[i - 1][1]
                yaw = math.atan2(dy, dx)
            else:
                yaw = 0.0

            # 쿼터니언 변환 (Z축 회전만 사용, 2D 평면)
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

        # 경로 생성기 초기화
        self._path_planner = PathPlanner(node)

        # 커버리지/웨이포인트 내비게이션 상태
        self._coverage_waypoints: List[Tuple[float, float]] = []
        self._current_waypoint_index: int = 0
        self._nav_path: Optional[Path] = None

        node.get_logger().info('[판단] 모듈 초기화 완료')

    @property
    def path_planner(self) -> PathPlanner:
        """PathPlanner 인스턴스 접근자."""
        return self._path_planner

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

    def plan(self, mode: DrivingMode, **kwargs):
        """지정된 주행 모드에 따라 경로 계획을 수행.

        update() 메서드의 자동 판단과 별개로,
        외부에서 명시적으로 주행 모드와 경로를 지정할 때 사용한다.

        Args:
            mode: 목표 주행 모드
            **kwargs: 모드별 추가 파라미터
                COVERAGE 모드:
                    field_boundary: 영역 경계 좌표 리스트
                    swath_width: 작업 폭 (m)
                WAYPOINT_NAV 모드:
                    waypoints: 웨이포인트 좌표 리스트 [(x, y), ...]
                LANE_KEEPING 모드:
                    lane_info: 차선 정보 dict
        """
        self.current_mode = mode

        if mode == DrivingMode.COVERAGE:
            # 면적 커버리지 경로 생성
            field_boundary = kwargs.get('field_boundary', [])
            swath_width = kwargs.get('swath_width', 2.0)

            self._coverage_waypoints = self._path_planner.generate_coverage_path(
                field_boundary, swath_width
            )
            self._current_waypoint_index = 0

            if self._coverage_waypoints:
                # 커버리지 웨이포인트를 Nav2 Path로 변환
                self._nav_path = self._path_planner.generate_waypoint_path(
                    self._coverage_waypoints
                )
                self.target_speed = self.max_speed * 0.7  # 커버리지는 감속 주행
                self._node.get_logger().info(
                    f'[판단] 커버리지 모드 시작: '
                    f'{len(self._coverage_waypoints)}개 웨이포인트'
                )
            else:
                self._node.get_logger().warn(
                    '[판단] 커버리지 경로 생성 실패, IDLE 복귀'
                )
                self.current_mode = DrivingMode.IDLE
                self.target_speed = 0.0

        elif mode == DrivingMode.WAYPOINT_NAV:
            # 웨이포인트 순차 내비게이션
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
        """현재 위치를 기준으로 다음 웨이포인트로 진행.

        커버리지 또는 웨이포인트 내비게이션 모드에서
        현재 웨이포인트에 도달했는지 확인하고, 도달 시 다음으로 이동한다.

        Args:
            current_position: 현재 로봇 위치 (x, y)
            tolerance: 웨이포인트 도달 판정 거리 (m)

        Returns:
            True: 모든 웨이포인트를 완료함, False: 아직 진행 중
        """
        if not self._coverage_waypoints:
            return True

        if self._current_waypoint_index >= len(self._coverage_waypoints):
            self._node.get_logger().info('[판단] 모든 웨이포인트 완료')
            self.current_mode = DrivingMode.IDLE
            self.target_speed = 0.0
            return True

        # 현재 목표 웨이포인트와의 거리 계산
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
        """현재 목표 웨이포인트 좌표를 반환.

        Returns:
            현재 목표 웨이포인트 (x, y) 또는 None
        """
        if (self._coverage_waypoints and
                self._current_waypoint_index < len(self._coverage_waypoints)):
            return self._coverage_waypoints[self._current_waypoint_index]
        return None

    def get_nav_path(self) -> Optional[Path]:
        """생성된 Nav2 Path 메시지를 반환.

        Returns:
            Nav2 Path 메시지 또는 None
        """
        return self._nav_path

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
        result = {
            'mode': self.current_mode.name,
            'target_speed': self.target_speed,
            'target_steering': self.target_steering,
        }

        # 커버리지/웨이포인트 모드 추가 정보
        if self.current_mode in (DrivingMode.COVERAGE, DrivingMode.WAYPOINT_NAV):
            target_wp = self.get_current_target_waypoint()
            result['current_waypoint_index'] = self._current_waypoint_index
            result['total_waypoints'] = len(self._coverage_waypoints)
            result['target_waypoint'] = target_wp
            result['has_nav_path'] = self._nav_path is not None

        return result
