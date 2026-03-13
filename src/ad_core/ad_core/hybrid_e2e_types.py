"""Hybrid E2E 아키텍처용 데이터 타입 정의.

ARCH-004: 농업용 Hybrid E2E 아키텍처
"""

from dataclasses import dataclass, field
from typing import List, Optional, Tuple
from enum import Enum

from ad_core.datatypes import Pose2D


class TerrainClass(Enum):
    """지형 분류 (Learned Perception 출력)."""
    PAVED = "paved"
    DIRT_ROAD = "dirt_road"
    GRASS = "grass"
    GRAVEL = "gravel"
    CROP_FIELD = "crop_field"
    MUD = "mud"
    OBSTACLE = "obstacle"


@dataclass
class Line3D:
    """3D 공간의 선 (작물 행 표현용)."""
    start: Tuple[float, float, float]  # (x, y, z)
    end: Tuple[float, float, float]
    confidence: float = 1.0
    
    def to_2d(self) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        """2D 투영 (z 무시)."""
        return ((self.start[0], self.start[1]), (self.end[0], self.end[1]))


@dataclass
class Object3D:
    """3D 객체 (장애물 표현용)."""
    position: Tuple[float, float, float]  # 중심점
    size: Tuple[float, float, float]      # (width, height, depth)
    velocity: Optional[Tuple[float, float, float]] = None
    class_label: str = "unknown"
    confidence: float = 1.0
    
    @property
    def bounding_box(self) -> Tuple[float, float, float, float]:
        """2D 바운 박스 (x, y, w, h)."""
        return (
            self.position[0] - self.size[0]/2,
            self.position[1] - self.size[1]/2,
            self.size[0],
            self.size[1]
        )


@dataclass
class PerceptionFeatures:
    """Learned Perception의 출력 특징.
    
    Foundation Model (AutoSeg 스타일)의 멀티태스크 출력.
    """
    # 작물 행 정보
    crop_rows: List[Line3D] = field(default_factory=list)
    crop_row_confidence: float = 0.0
    
    # 지형 분류
    terrain_type: TerrainClass = TerrainClass.CROP_FIELD
    terrain_confidence: float = 0.0
    terrain_probs: dict = field(default_factory=dict)  # 각 클래스별 확률
    
    # 장애물 정보
    obstacles: List[Object3D] = field(default_factory=list)
    obstacle_grid: Optional[List[List[float]]] = None  # 2D 그리드 맵
    
    # 주행 가능 영역
    free_space_mask: Optional[List[List[bool]]] = None  # True = 주행 가능
    free_space_contours: List[List[Tuple[float, float]]] = field(default_factory=list)
    
    # 경사 정보
    slope_gradient: float = 0.0  # 경사도 (0~1, 1 = 45도)
    slope_direction: float = 0.0  # 경사 방향 (라디안)
    
    # 종합 신뢰도
    overall_confidence: float = 1.0
    
    # 메타데이터
    timestamp: float = 0.0
    processing_time_ms: float = 0.0
    
    def has_crop_rows(self) -> bool:
        """작물 행이 감지되었는지 확인."""
        return len(self.crop_rows) > 0 and self.crop_row_confidence > 0.5
    
    def get_nearest_crop_row(self, pose: Pose2D) -> Optional[Line3D]:
        """현재 위치에서 가장 가까운 작물 행 반환."""
        if not self.crop_rows:
            return None
        
        min_dist = float('inf')
        nearest = None
        
        for row in self.crop_rows:
            # 행의 중점과의 거리
            mid_x = (row.start[0] + row.end[0]) / 2
            mid_y = (row.start[1] + row.end[1]) / 2
            dist = ((mid_x - pose.x)**2 + (mid_y - pose.y)**2) ** 0.5
            
            if dist < min_dist:
                min_dist = dist
                nearest = row
        
        return nearest
    
    def is_safe_terrain(self) -> bool:
        """안전한 지형인지 확인."""
        unsafe_terrains = [TerrainClass.OBSTACLE, TerrainClass.MUD]
        return self.terrain_type not in unsafe_terrains


@dataclass
class TrajectoryPoint:
    """경로 상의 한 점."""
    pose: Pose2D
    velocity: float  # m/s
    acceleration: float  # m/s²
    curvature: float  # 1/m
    timestamp: float  # 상대 시간 (초)


@dataclass
class PlannedTrajectory:
    """Learned Planning의 출력 경로.
    
    Diffusion Model이 생성한 궤적.
    """
    waypoints: List[TrajectoryPoint] = field(default_factory=list)
    
    # 경로 메타데이터
    total_length: float = 0.0
    estimated_duration: float = 0.0
    confidence: float = 0.0
    
    # 대체 경로들 (multi-modal planning)
    alternative_paths: List['PlannedTrajectory'] = field(default_factory=list)
    
    # 생성 정보
    generation_method: str = "diffusion"  # "diffusion", "rule_based", "hybrid"
    processing_time_ms: float = 0.0
    
    def get_pose_at_time(self, t: float) -> Optional[Pose2D]:
        """주어진 시간에 해당하는 pose 추정 (선형 보간)."""
        if not self.waypoints:
            return None
        
        # 시간 기준으로 정렬
        sorted_points = sorted(self.waypoints, key=lambda p: p.timestamp)
        
        # 범위 체크
        if t <= sorted_points[0].timestamp:
            return sorted_points[0].pose
        if t >= sorted_points[-1].timestamp:
            return sorted_points[-1].pose
        
        # 보간
        for i in range(len(sorted_points) - 1):
            p1, p2 = sorted_points[i], sorted_points[i+1]
            if p1.timestamp <= t <= p2.timestamp:
                ratio = (t - p1.timestamp) / (p2.timestamp - p1.timestamp)
                return Pose2D(
                    x=p1.pose.x + ratio * (p2.pose.x - p1.pose.x),
                    y=p1.pose.y + ratio * (p2.pose.y - p1.pose.y),
                    yaw=p1.pose.yaw + ratio * (p2.pose.yaw - p1.pose.yaw)
                )
        
        return None
    
    def get_max_curvature(self) -> float:
        """경로의 최대 곡률 반환."""
        if not self.waypoints:
            return 0.0
        return max(abs(p.curvature) for p in self.waypoints)
    
    def get_max_speed(self) -> float:
        """경로의 최대 속도 반환."""
        if not self.waypoints:
            return 0.0
        return max(abs(p.velocity) for p in self.waypoints)


@dataclass
class GuardianDecision:
    """Safety Guardian의 결정 결과."""
    is_safe: bool
    reason: str
    safety_score: float  # 0~1, 1 = 완전 안전
    
    # 위반된 제약들
    violated_constraints: List[str] = field(default_factory=list)
    
    # 권장 조치
    recommended_action: str = "proceed"  # "proceed", "slow_down", "stop", "replan"
    recommended_speed_limit: Optional[float] = None
    
    # 대체 경로 필요 여부
    need_alternative_path: bool = False


@dataclass
class HybridE2EState:
    """Hybrid E2E 시스템의 전체 상태."""
    # 현재 단계
    current_phase: str = "normal"  # "normal", "fallback_level_1", "fallback_level_2", "emergency"
    
    # 각 모듈 상태
    perception_status: str = "active"
    planning_status: str = "active"
    guardian_status: str = "active"
    control_status: str = "active"
    
    # 현재 출력들
    latest_perception: Optional[PerceptionFeatures] = None
    latest_trajectory: Optional[PlannedTrajectory] = None
    latest_guardian_decision: Optional[GuardianDecision] = None
    
    # 성능 메트릭
    perception_latency_ms: float = 0.0
    planning_latency_ms: float = 0.0
    guardian_latency_ms: float = 0.0
    total_latency_ms: float = 0.0
    
    # fallback 카운터
    fallback_count: int = 0
    emergency_stop_count: int = 0
