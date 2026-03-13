"""Safety Guardian 모듈 테스트.

ARCH-004: 농업용 Hybrid E2E - Safety Guardian 검증
"""

import math
import sys
sys.path.insert(0, '/home/gint_pcd/projects/자율주행프로젝트_work/src/ad_core')
sys.path.insert(0, '/home/gint_pcd/projects/자율주행프로젝트_work/src/ad_control')

from ad_control.safety_guardian import SafetyGuardian, GuardianConfig
from ad_core.hybrid_e2e_types import (
    PerceptionFeatures,
    PlannedTrajectory,
    TerrainClass,
    Pose2D,
)


def test_basic():
    """기본 동작 테스트."""
    print("\n--- Safety Guardian 기본 테스트 ---")
    
    # Guardian 생성
    config = GuardianConfig(max_speed=1.0, min_crop_distance=0.3)
    guardian = SafetyGuardian(config)
    print(f"✅ Guardian 생성: max_speed={config.max_speed} m/s")
    
    # 안전한 경우
    perception = PerceptionFeatures(
        terrain_type=TerrainClass.CROP_FIELD,
        slope_gradient=0.05,  # 약 3도
        overall_confidence=0.9
    )
    trajectory = PlannedTrajectory(waypoints=[], confidence=1.0)
    decision = guardian.validate(trajectory, Pose2D(0, 0, 0), perception, 0.5)
    
    assert decision.is_safe, "안전한 경우 should pass"
    assert decision.safety_score == 1.0
    print(f"   안전 검증: is_safe={decision.is_safe}, score={decision.safety_score}")
    print(f"   action={decision.recommended_action}, reason={decision.reason}")
    
    # 위험한 경사 (20도)
    steep_perception = PerceptionFeatures(
        terrain_type=TerrainClass.CROP_FIELD,
        slope_gradient=math.tan(math.radians(20)),  # ~0.36
        overall_confidence=0.9
    )
    decision = guardian.validate(trajectory, Pose2D(0, 0, 0), steep_perception, 0.5)
    
    assert not decision.is_safe, "위험한 경사 should fail"
    assert any("SLOPE" in c for c in decision.violated_constraints)
    print(f"   경사 위반: is_safe={decision.is_safe}")
    print(f"   위반 항목: {decision.violated_constraints}")
    
    # 금지 지형
    mud_perception = PerceptionFeatures(
        terrain_type=TerrainClass.MUD,
        slope_gradient=0.0
    )
    decision = guardian.validate(trajectory, Pose2D(0, 0, 0), mud_perception, 0.0)
    
    assert not decision.is_safe, "진흙 지형 should fail"
    assert decision.recommended_action == "stop"
    print(f"   지형 위반: action={decision.recommended_action}")
    
    print("✅ 기본 테스트 통과!")


def test_constraint_types():
    """제약 조건 타입별 테스트."""
    print("\n--- 제약 조건 테스트 ---")
    
    guardian = SafetyGuardian(GuardianConfig(max_speed=1.0))
    
    # 속도 제약
    from ad_core.hybrid_e2e_types import TrajectoryPoint
    fast_traj = PlannedTrajectory(
        waypoints=[
            TrajectoryPoint(Pose2D(0, 0, 0), 2.0, 0, 0, 0),  # 2 m/s (과속)
        ],
        confidence=1.0
    )
    perception = PerceptionFeatures(terrain_type=TerrainClass.CROP_FIELD)
    decision = guardian.validate(fast_traj, Pose2D(0, 0, 0), perception, 2.0)
    
    assert not decision.is_safe
    assert any("SPEED" in c for c in decision.violated_constraints)
    print(f"   SPEED 제약 위반 감지: ✅")
    
    # Crop 거리 제약
    close_crop_perception = PerceptionFeatures(
        terrain_type=TerrainClass.CROP_FIELD,
        slope_gradient=0.0,
        overall_confidence=0.9,
        # min_crop_distance는 나중에 추가
    )
    # 현재는 구현에서 처리
    print(f"   CROP_DISTANCE 제약: 구현 확인 필요")
    
    print("✅ 제약 조건 테스트 통과!")


def test_safety_score_calculation():
    """안전 점수 계산 테스트."""
    print("\n--- 안전 점수 계산 테스트 ---")
    
    guardian = SafetyGuardian(GuardianConfig())
    trajectory = PlannedTrajectory(waypoints=[], confidence=1.0)
    
    # 완벽한 안전
    safe_perception = PerceptionFeatures(terrain_type=TerrainClass.CROP_FIELD)
    decision = guardian.validate(trajectory, Pose2D(0, 0, 0), safe_perception, 0.0)
    assert decision.safety_score == 1.0
    print(f"   완벽한 안전: score={decision.safety_score}")
    
    # 경미한 위반 (confidence 낮음)
    low_conf_perception = PerceptionFeatures(
        terrain_type=TerrainClass.CROP_FIELD,
        overall_confidence=0.5
    )
    decision = guardian.validate(trajectory, Pose2D(0, 0, 0), low_conf_perception, 0.0)
    print(f"   낮은 confidence: score={decision.safety_score}")
    
    # 심각한 위반
    mud_perception = PerceptionFeatures(terrain_type=TerrainClass.MUD)
    decision = guardian.validate(trajectory, Pose2D(0, 0, 0), mud_perception, 0.0)
    assert decision.safety_score < 0.5  # TERRAIN 위반은 0.4배
    print(f"   금지 지형: score={decision.safety_score}")
    
    print("✅ 안전 점수 테스트 통과!")


if __name__ == "__main__":
    print("=" * 50)
    print("Safety Guardian 통합 테스트")
    print("=" * 50)
    
    try:
        test_basic()
        test_constraint_types()
        test_safety_score_calculation()
        
        print("\n" + "=" * 50)
        print("🎉 모든 테스트 통과!")
        print("=" * 50)
        
    except AssertionError as e:
        print(f"\n❌ 테스트 실패: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
