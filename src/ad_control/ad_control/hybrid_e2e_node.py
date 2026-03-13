"""Hybrid E2E 통합 노드.

ARCH-004: 농업용 Hybrid E2E 아키텍처의 중앙 통제 노드.

Learned Perception → Learned Planning → Safety Guardian → Traditional Control
의 파이프라인을 조율한다.
"""

import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from ad_core.hybrid_e2e_types import (
    PerceptionFeatures,
    PlannedTrajectory,
    GuardianDecision,
    HybridE2EState,
)
from ad_core.datatypes import Pose2D
from ad_control.safety_guardian import SafetyGuardian, GuardianConfig


class HybridE2ENode(Node):
    """Hybrid E2E 메인 노드.
    
    농업용 자율주행을 위한 Hybrid End-to-End 시스템의 중앙 제어 노드.
    Tesla-style Learned Planning + Autoware-style Modular Architecture +
    Agriculture-specific Safety Guardian.
    """
    
    def __init__(self):
        super().__init__('hybrid_e2e')
        
        # 파라미터 선언
        self._declare_parameters()
        
        # 상태 초기화
        self._state = HybridE2EState()
        self._guardian = SafetyGuardian(self._load_guardian_config())
        
        # 현재 차량 상태
        self._current_pose: Optional[Pose2D] = None
        self._current_speed: float = 0.0
        
        # Fallback 레벨
        self._fallback_level = 0  # 0: normal, 1: fields2cover, 2: pure_pursuit, 3: emergency
        
        # 콜백 그룹 (동시 처리용)
        self._cb_group = ReentrantCallbackGroup()
        
        # 타이머 설정
        self._control_timer = self.create_timer(
            0.05,  # 20Hz
            self._control_loop,
            callback_group=self._cb_group
        )
        
        self._monitoring_timer = self.create_timer(
            1.0,  # 1Hz
            self._monitoring_loop,
            callback_group=self._cb_group
        )
        
        self.get_logger().info('Hybrid E2E Node initialized')
        self.get_logger().info(f'Guardian config: max_slope={self._guardian.config.max_slope}, '
                              f'min_crop_dist={self._guardian.config.min_crop_distance}')
    
    def _declare_parameters(self):
        """파라미터 선언."""
        self.declare_parameter('max_speed', 1.5)
        self.declare_parameter('max_slope', 0.3)
        self.declare_parameter('min_crop_distance', 0.3)
        self.declare_parameter('use_learned_perception', True)
        self.declare_parameter('use_learned_planning', True)
        self.declare_parameter('guardian_strict_mode', False)
    
    def _load_guardian_config(self) -> GuardianConfig:
        """Guardian 설정 로드."""
        return GuardianConfig(
            max_speed=self.get_parameter('max_speed').value,
            max_slope=self.get_parameter('max_slope').value,
            min_crop_distance=self.get_parameter('min_crop_distance').value,
        )
    
    def _control_loop(self):
        """주 제어 루프 (20Hz)."""
        loop_start = time.time()
        
        try:
            # 1. Learned Perception (or传统 Perception)
            perception = self._run_perception()
            self._state.latest_perception = perception
            
            # 2. Learned Planning (with fallback)
            trajectory = self._run_planning(perception)
            self._state.latest_trajectory = trajectory
            
            # 3. Safety Guardian 검증
            if trajectory and self._current_pose:
                decision = self._guardian.validate(
                    trajectory,
                    self._current_pose,
                    perception,
                    self._current_speed
                )
                self._state.latest_guardian_decision = decision
                
                # 4. 검증 결과에 따른 처리
                if decision.is_safe:
                    self._execute_trajectory(trajectory)
                    self._state.current_phase = "normal"
                else:
                    self._handle_guardian_violation(decision, trajectory)
            else:
                self.get_logger().warn('No trajectory or pose available')
                self._enter_fallback(1)
            
            # 지연 시간 기록
            self._state.total_latency_ms = (time.time() - loop_start) * 1000
            
        except Exception as e:
            self.get_logger().error(f'Control loop error: {str(e)}')
            self._enter_fallback(2)  # 심각한 오류 시 강제 fallback
    
    def _run_perception(self) -> PerceptionFeatures:
        """인지 실행 (Learned or Traditional)."""
        use_learned = self.get_parameter('use_learned_perception').value
        
        if use_learned and self._fallback_level == 0:
            try:
                # TODO: Learned Perception 호출
                # perception = self._learned_perception.process(...)
                
                # 임시: 기존 perception 사용
                perception = self._run_traditional_perception()
                perception.processing_time_ms = 10.0  # 예상 처리 시간
                
                return perception
            except Exception as e:
                self.get_logger().warn(f'Learned perception failed: {e}, falling back')
                return self._run_traditional_perception()
        else:
            return self._run_traditional_perception()
    
    def _run_traditional_perception(self) -> PerceptionFeatures:
        """기존 인지 시스템 호출."""
        # TODO: 기존 ad_perception 모듈 연동
        # 현재는 placeholder 반환
        return PerceptionFeatures(
            crop_rows=[],
            terrain_type=TerrainClass.CROP_FIELD,
            obstacles=[],
            slope_gradient=0.0,
            overall_confidence=0.8
        )
    
    def _run_planning(self, perception: PerceptionFeatures) -> Optional[PlannedTrajectory]:
        """경로 계획 실행 (with fallback levels)."""
        use_learned = self.get_parameter('use_learned_planning').value
        
        # Level 0: Learned Planning (Diffusion)
        if self._fallback_level == 0 and use_learned:
            try:
                trajectory = self._run_learned_planning(perception)
                if trajectory:
                    trajectory.generation_method = "diffusion"
                    return trajectory
            except Exception as e:
                self.get_logger().warn(f'Learned planning failed: {e}')
        
        # Level 1: Fields2Cover (규칙 기반 커버리지)
        if self._fallback_level <= 1:
            try:
                trajectory = self._run_fields2cover_planning(perception)
                if trajectory:
                    trajectory.generation_method = "fields2cover"
                    return trajectory
            except Exception as e:
                self.get_logger().warn(f'Fields2Cover failed: {e}')
        
        # Level 2: Pure Pursuit (기존 방식)
        if self._fallback_level <= 2:
            try:
                trajectory = self._run_pure_pursuit_planning(perception)
                if trajectory:
                    trajectory.generation_method = "pure_pursuit"
                    return trajectory
            except Exception as e:
                self.get_logger().warn(f'Pure pursuit failed: {e}')
        
        # Level 3: Emergency Stop
        return None
    
    def _run_learned_planning(self, perception: PerceptionFeatures) -> Optional[PlannedTrajectory]:
        """Learned Planning (Diffusion Model) 실행."""
        # TODO: Diffusion Planner 통합
        # 현재는 미구현
        self.get_logger().debug('Learned planning not yet implemented')
        return None
    
    def _run_fields2cover_planning(self, perception: PerceptionFeatures) -> Optional[PlannedTrajectory]:
        """Fields2Cover 기반 계획."""
        # TODO: Fields2Cover 연동
        # 현재는 미구현
        return None
    
    def _run_pure_pursuit_planning(self, perception: PerceptionFeatures) -> Optional[PlannedTrajectory]:
        """기존 Pure Pursuit 방식."""
        # TODO: 기존 ad_planning 연동
        return None
    
    def _execute_trajectory(self, trajectory: PlannedTrajectory):
        """경로 실행."""
        # TODO: Control 명령 발행
        self.get_logger().debug(f'Executing {trajectory.generation_method} trajectory')
    
    def _handle_guardian_violation(self, decision: GuardianDecision, trajectory: PlannedTrajectory):
        """Guardian 위반 처리."""
        self.get_logger().warn(
            f'Guardian violation: {decision.reason}, '
            f'safety_score={decision.safety_score:.2f}, '
            f'action={decision.recommended_action}'
        )
        
        self._state.fallback_count += 1
        
        if decision.recommended_action == "stop":
            self._emergency_stop()
            self._enter_fallback(3)
        elif decision.recommended_action == "slow_down":
            self._slow_down(decision.recommended_speed_limit)
        elif decision.recommended_action == "replan":
            self._enter_fallback(min(self._fallback_level + 1, 2))
        elif decision.need_alternative_path:
            self._try_alternative_path(trajectory)
    
    def _enter_fallback(self, level: int):
        """Fallback 레벨 진입."""
        if level != self._fallback_level:
            self.get_logger().info(
                f'Entering fallback level {level} '
                f'(was {self._fallback_level})'
            )
            self._fallback_level = level
            
            if level == 0:
                self._state.current_phase = "normal"
            elif level == 1:
                self._state.current_phase = "fallback_level_1"
            elif level == 2:
                self._state.current_phase = "fallback_level_2"
            elif level >= 3:
                self._state.current_phase = "emergency"
    
    def _try_alternative_path(self, trajectory: PlannedTrajectory):
        """대체 경로 시도."""
        if trajectory.alternative_paths:
            self.get_logger().info('Trying alternative path')
            alt_trajectory = trajectory.alternative_paths[0]
            
            # 대체 경로도 검증
            if self._current_pose:
                decision = self._guardian.validate(
                    alt_trajectory,
                    self._current_pose,
                    self._state.latest_perception,
                    self._current_speed
                )
                
                if decision.is_safe:
                    self._execute_trajectory(alt_trajectory)
                    return
        
        # 대체 경로도 실패하면 fallback
        self._enter_fallback(1)
    
    def _emergency_stop(self):
        """비상 정지."""
        self.get_logger().error('EMERGENCY STOP triggered!')
        self._state.emergency_stop_count += 1
        # TODO: 비상 정지 명령 발행
    
    def _slow_down(self, speed_limit: Optional[float]):
        """감속."""
        limit = speed_limit or 0.5
        self.get_logger().info(f'Slowing down to {limit} m/s')
        # TODO: 감속 명령 발행
    
    def _monitoring_loop(self):
        """모니터링 루프 (1Hz)."""
        stats = self._guardian.get_statistics()
        
        self.get_logger().info(
            f'Status: phase={self._state.current_phase}, '
            f'fallback_level={self._fallback_level}, '
            f'fallback_count={stats["consecutive_violations"]}, '
            f'latency={self._state.total_latency_ms:.1f}ms'
        )
        
        # 연속 위반 체크 후 복구 시도
        if self._fallback_level > 0 and stats["consecutive_violations"] == 0:
            # 안정 상태가 지속되면 정상 모드로 복귀 시도
            if self._state.fallback_count > 10:  # 10번 이상 안전하면
                self._enter_fallback(max(0, self._fallback_level - 1))
                self._state.fallback_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = HybridE2ENode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
