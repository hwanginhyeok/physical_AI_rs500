"""Safety Guardian 모듈.

Learned Planning 출력을 검증하는 규칙 기반 안전 계층.
ARCH-004: 농업용 Hybrid E2E 아키텍처의 핵심 구성요소.

Tesla 방식의 E2E와 달리, 농업 환경에서는 작물 보호와 안전성이 최우선이므로
명시적 규칙 기반 검증 계층을 유지한다.
"""

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

from ad_core.hybrid_e2e_types import (
    PerceptionFeatures,
    PlannedTrajectory,
    GuardianDecision,
    TerrainClass,
)
from ad_core.datatypes import Pose2D
from ad_core.utils import distance_2d, EPSILON


@dataclass
class GuardianConfig:
    """Safety Guardian 설정."""
    # 경사 제한
    max_slope: float = 0.3  # tan(theta), 약 17도
    max_lateral_slope: float = 0.2  # 횡경사
    
    # 작물 거리
    min_crop_distance: float = 0.3  # m
    min_crop_distance_strict: float = 0.1  # m (엄격 모드)
    
    # 속도 제한
    max_speed: float = 1.5  # m/s (5.4 km/h)
    max_speed_slope_factor: float = 0.5  # 경사 있을 때 속도 감소율
    
    # 곡률 제한
    max_curvature: float = 0.5  # 1/m (2m 반경)
    max_lateral_acceleration: float = 1.0  # m/s²
    
    # 급정거/급가속 제한
    max_deceleration: float = 2.0  # m/s²
    max_acceleration: float = 1.5  # m/s²
    max_jerk: float = 2.0  # m/s³
    
    # 장애물
    min_obstacle_distance: float = 1.0  # m
    emergency_stop_distance: float = 0.5  # m
    
    # 지형 제한
    forbidden_terrains: List[TerrainClass] = None
    
    def __post_init__(self):
        if self.forbidden_terrains is None:
            self.forbidden_terrains = [TerrainClass.OBSTACLE, TerrainClass.MUD]


class SafetyGuardian:
    """Safety Guardian - 규칙 기반 안전 검증 계층.
    
    Learned Planning이 생성한 경로를 다양한 안전 기준으로 검증한다.
    검증 실패 시 fallback 경로를 요청하거나 emergency stop을 권장한다.
    """
    
    def __init__(self, config: Optional[GuardianConfig] = None):
        """초기화.
        
        Args:
            config: Guardian 설정. None이면 기본값 사용.
        """
        self.config = config or GuardianConfig()
        self._violation_history: List[str] = []
        self._consecutive_violations: int = 0
    
    def validate(
        self,
        trajectory: PlannedTrajectory,
        current_pose: Pose2D,
        perception: PerceptionFeatures,
        vehicle_speed: float
    ) -> GuardianDecision:
        """경로를 검증한다.
        
        Args:
            trajectory: 검증할 경로
            current_pose: 현재 차량 위치
            perception: 인지 결과
            vehicle_speed: 현재 차량 속도
            
        Returns:
            GuardianDecision: 검증 결과
        """
        violated_constraints = []
        safety_score = 1.0
        
        # 1. 경사 검증
        slope_ok, slope_msg = self._check_slope(perception)
        if not slope_ok:
            violated_constraints.append(f"SLOPE: {slope_msg}")
            safety_score *= 0.5
        
        # 2. 작물 거리 검증
        crop_ok, crop_msg = self._check_crop_distance(trajectory, perception)
        if not crop_ok:
            violated_constraints.append(f"CROP: {crop_msg}")
            safety_score *= 0.3  # 작물 손상은 심각
        
        # 3. 속도 검증
        speed_ok, speed_msg = self._check_speed(trajectory, perception)
        if not speed_ok:
            violated_constraints.append(f"SPEED: {speed_msg}")
            safety_score *= 0.7
        
        # 4. 곡률/가속도 검증
        curvature_ok, curvature_msg = self._check_curvature(trajectory)
        if not curvature_ok:
            violated_constraints.append(f"CURVATURE: {curvature_msg}")
            safety_score *= 0.8
        
        # 5. 동적 제약 (급정거/급가속)
        dynamic_ok, dynamic_msg = self._check_dynamics(trajectory)
        if not dynamic_ok:
            violated_constraints.append(f"DYNAMIC: {dynamic_msg}")
            safety_score *= 0.8
        
        # 6. 장애물 거리 검증
        obstacle_ok, obstacle_msg = self._check_obstacles(trajectory, perception)
        if not obstacle_ok:
            violated_constraints.append(f"OBSTACLE: {obstacle_msg}")
            safety_score *= 0.2  # 충돌 위험은 매우 심각
        
        # 7. 지형 적합성
        terrain_ok, terrain_msg = self._check_terrain(perception)
        if not terrain_ok:
            violated_constraints.append(f"TERRAIN: {terrain_msg}")
            safety_score *= 0.2
        
        # 결과 판정
        is_safe = len(violated_constraints) == 0
        
        # 연속 위반 카운트
        if not is_safe:
            self._consecutive_violations += 1
        else:
            self._consecutive_violations = 0
        
        # 권장 조치 결정
        recommended_action, reason = self._determine_action(
            violated_constraints,
            self._consecutive_violations,
            safety_score
        )
        
        # 속도 제한 계산
        speed_limit = self._calculate_speed_limit(
            perception, violated_constraints
        )
        
        # 대체 경로 필요 여부
        need_alternative = (
            not is_safe and 
            self._consecutive_violations >= 3 and
            safety_score < 0.5
        )
        
        decision = GuardianDecision(
            is_safe=is_safe,
            reason=reason,
            safety_score=safety_score,
            violated_constraints=violated_constraints,
            recommended_action=recommended_action,
            recommended_speed_limit=speed_limit,
            need_alternative_path=need_alternative
        )
        
        return decision
    
    def _check_slope(self, perception: PerceptionFeatures) -> Tuple[bool, str]:
        """경사도 검증."""
        if perception.slope_gradient > self.config.max_slope:
            return False, f"경사 {math.degrees(math.atan(perception.slope_gradient)):.1f}° > " \
                         f"{math.degrees(math.atan(self.config.max_slope)):.1f}°"
        return True, "OK"
    
    def _check_crop_distance(
        self,
        trajectory: PlannedTrajectory,
        perception: PerceptionFeatures
    ) -> Tuple[bool, str]:
        """작물과의 거리 검증."""
        if not perception.has_crop_rows():
            return True, "No crops detected"
        
        min_dist = float('inf')
        
        for waypoint in trajectory.waypoints:
            nearest_crop = perception.get_nearest_crop_row(waypoint.pose)
            if nearest_crop:
                # 작물 행과의 거리 계산 (단순화)
                crop_mid = (
                    (nearest_crop.start[0] + nearest_crop.end[0]) / 2,
                    (nearest_crop.start[1] + nearest_crop.end[1]) / 2
                )
                dist = distance_2d(
                    waypoint.pose.x, waypoint.pose.y,
                    crop_mid[0], crop_mid[1]
                )
                min_dist = min(min_dist, dist)
        
        if min_dist < self.config.min_crop_distance_strict:
            return False, f"Crop distance {min_dist:.2f}m < {self.config.min_crop_distance_strict}m"
        elif min_dist < self.config.min_crop_distance:
            return True, f"Warning: Crop distance {min_dist:.2f}m"
        
        return True, f"Crop distance OK ({min_dist:.2f}m)"
    
    def _check_speed(
        self,
        trajectory: PlannedTrajectory,
        perception: PerceptionFeatures
    ) -> Tuple[bool, str]:
        """속도 검증."""
        max_speed = trajectory.get_max_speed()
        
        # 경사에 따른 속도 제한
        effective_max = self.config.max_speed
        if perception.slope_gradient > 0.1:
            effective_max *= self.config.max_speed_slope_factor
        
        if max_speed > effective_max:
            return False, f"Speed {max_speed:.2f}m/s > {effective_max:.2f}m/s"
        
        return True, f"Speed OK (max {max_speed:.2f}m/s)"
    
    def _check_curvature(self, trajectory: PlannedTrajectory) -> Tuple[bool, str]:
        """곡률 및 측가속도 검증."""
        max_curvature = trajectory.get_max_curvature()
        
        if max_curvature > self.config.max_curvature:
            return False, f"Curvature {max_curvature:.2f} > {self.config.max_curvature}"
        
        # 측가속도 검증 (a = v² * curvature)
        for point in trajectory.waypoints:
            lateral_accel = (point.velocity ** 2) * abs(point.curvature)
            if lateral_accel > self.config.max_lateral_acceleration:
                return False, f"Lateral acceleration {lateral_accel:.2f} > {self.config.max_lateral_acceleration}"
        
        return True, f"Curvature OK (max {max_curvature:.2f})"
    
    def _check_dynamics(self, trajectory: PlannedTrajectory) -> Tuple[bool, str]:
        """동적 제약 검증 (가속도, jerk)."""
        if len(trajectory.waypoints) < 2:
            return True, "OK"
        
        for i in range(1, len(trajectory.waypoints)):
            p1, p2 = trajectory.waypoints[i-1], trajectory.waypoints[i]
            dt = p2.timestamp - p1.timestamp
            
            if dt < EPSILON:
                continue
            
            # 가속도
            accel = (p2.velocity - p1.velocity) / dt
            if abs(accel) > self.config.max_acceleration:
                return False, f"Acceleration {accel:.2f} exceeds limit"
            
            # Jerk (가속도 변화율)
            if i >= 2:
                p0 = trajectory.waypoints[i-2]
                dt_prev = p1.timestamp - p0.timestamp
                if dt_prev > EPSILON:
                    prev_accel = (p1.velocity - p0.velocity) / dt_prev
                    jerk = (accel - prev_accel) / dt
                    if abs(jerk) > self.config.max_jerk:
                        return False, f"Jerk {jerk:.2f} exceeds limit"
        
        return True, "Dynamics OK"
    
    def _check_obstacles(
        self,
        trajectory: PlannedTrajectory,
        perception: PerceptionFeatures
    ) -> Tuple[bool, str]:
        """장애물 거리 검증."""
        for obstacle in perception.obstacles:
            for waypoint in trajectory.waypoints:
                dist = distance_2d(
                    waypoint.pose.x, waypoint.pose.y,
                    obstacle.position[0], obstacle.position[1]
                )
                
                if dist < self.config.emergency_stop_distance:
                    return False, f"Obstacle {obstacle.class_label} too close: {dist:.2f}m"
                elif dist < self.config.min_obstacle_distance:
                    return True, f"Warning: Obstacle {dist:.2f}m away"
        
        return True, "Obstacles OK"
    
    def _check_terrain(self, perception: PerceptionFeatures) -> Tuple[bool, str]:
        """지형 적합성 검증."""
        if perception.terrain_type in self.config.forbidden_terrains:
            return False, f"Forbidden terrain: {perception.terrain_type.value}"
        
        if not perception.is_safe_terrain():
            return False, f"Unsafe terrain: {perception.terrain_type.value}"
        
        return True, f"Terrain OK: {perception.terrain_type.value}"
    
    def _determine_action(
        self,
        violations: List[str],
        consecutive_count: int,
        safety_score: float
    ) -> Tuple[str, str]:
        """위반 사항에 따른 권장 조치 결정."""
        if not violations:
            return "proceed", "All checks passed"
        
        # 심각한 위반
        critical_keywords = ["OBSTACLE", "CROP", "SLOPE", "TERRAIN"]
        has_critical = any(
            any(kw in v for kw in critical_keywords)
            for v in violations
        )
        
        if has_critical and safety_score < 0.3:
            return "stop", f"Critical violations: {violations[0]}"
        
        # 연속 위반
        if consecutive_count >= 5:
            return "stop", f"Consecutive violations: {consecutive_count}"
        
        # 일반 위반
        if safety_score < 0.5:
            return "slow_down", f"Safety score low: {safety_score:.2f}"
        
        return "replan", f"Minor violations: {len(violations)}"
    
    def _calculate_speed_limit(
        self,
        perception: PerceptionFeatures,
        violations: List[str]
    ) -> Optional[float]:
        """상황에 따른 속도 제한 계산."""
        base_limit = self.config.max_speed
        
        # 경사
        if perception.slope_gradient > 0.1:
            base_limit *= 0.7
        
        # 위반 사항에 따른 감속
        if any("CROP" in v for v in violations):
            base_limit *= 0.5
        if any("OBSTACLE" in v for v in violations):
            base_limit *= 0.3
        
        return base_limit if base_limit < self.config.max_speed else None
    
    def get_statistics(self) -> dict:
        """Guardian 통계 반환."""
        return {
            'total_violations': len(self._violation_history),
            'consecutive_violations': self._consecutive_violations,
            'recent_violations': self._violation_history[-10:]
        }
    
    def reset_statistics(self):
        """통계 초기화."""
        self._violation_history = []
        self._consecutive_violations = 0
