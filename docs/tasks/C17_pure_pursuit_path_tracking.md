# C17: Pure Pursuit 경로 추종 (Phase 1)

- 분야: 알고리즘
- 담당: Claude
- 기간: 2026-02-21
- 상태: 완료

## 배경 및 목적

궤도차량용 Pure Pursuit 경로 추종 알고리즘 구현. 기본적인 lookahead 기반 경로 추종 기능.

## 변경 사항

### 주요 파일

`src/ad_core/ad_core/pure_pursuit.py` — 1002줄 (C26 슬립 보상 확장 포함)

### 핵심 클래스

**`PurePursuitConfig`** (dataclass):
- `lookahead_distance`, `min_lookahead`, `max_lookahead`
- `speed`, `angular_speed_limit`
- `goal_tolerance`, `path_end_threshold`
- `adaptive_lookahead` (속도 비례 lookahead)

**`PurePursuitTracker`**:
- `set_path(waypoints)` — 경로 설정
- `compute_command(current_pose)` → `VehicleCommand` 반환
- `_find_lookahead_point()` — 경로 상 lookahead 포인트 탐색
- `_compute_curvature()` — 곡률 계산
- `_compute_lookahead_distance()` — 적응형 lookahead 거리
- `_compute_speed()` — 곡률 기반 속도 조절
- `_twist_to_tracks()` — (v, ω) → (left, right) 변환

### 설계 결정

- 속도 비례 적응형 lookahead: `ld = base + gain * |v|`
- 곡률이 큰 구간에서 자동 감속
- `_twist_to_tracks()`를 별도 메서드로 분리하여 C26에서 오버라이드 가능하게 설계

## 테스트

- `test_slip_compensated_pursuit.py` 내에 기본 Pure Pursuit 테스트 포함
- 총 40개 테스트 (C26 슬립 보상 포함)
