# C33: 단위 테스트 추가

- 분야: 테스트
- 담당: Claude
- 기간: 2026-02-22 (추정)
- 상태: 완료 (C38에서 추가 확장)

## 배경 및 목적

ad_core 순수 알고리즘 모듈에 대한 pytest 기반 단위 테스트 작성.

## 변경 사항

### 테스트 파일 목록

**`src/ad_core/test/`** (9개 파일, 2008줄):

| 파일 | 줄수 | 테스트 수 | 대상 모듈 |
|------|------|-----------|-----------|
| test_slip_compensated_pursuit.py | 590 | 40 | pure_pursuit.py (C17, C26) |
| test_coverage_planner.py | 264 | 24 | coverage_planner.py (C28) |
| test_sensor_fusion.py | 220 | 20 | sensor_fusion.py (C31) |
| test_sensor_noise_model.py | 220 | 15 | sensor_noise_model.py (C38) |
| test_skid_steer_model.py | 189 | 20 | skid_steer_model.py (C18, C27) |
| test_track_terrain_interaction.py | 150 | 19 | track_terrain_interaction.py (C38) |
| test_vehicle_dynamics.py | 139 | 12 | vehicle_dynamics.py (C38) |
| test_lidar_processor.py | 127 | 9 | lidar_processor.py (C22) |
| test_drivetrain_model.py | 109 | 12 | drivetrain_model.py (C38) |

**`src/ad_can_bridge/test/`** (1개 파일, 236줄):

| 파일 | 줄수 | 테스트 수 | 대상 모듈 |
|------|------|-----------|-----------|
| test_ss500_codec.py | 236 | 24 | ss500_codec.py (C36) |

### 총계

- 10개 테스트 파일, 2244줄
- 195개 테스트 (C33 시점 120 + C38 추가 58 + C36 추가 24 = 현재 총계)
- 초기 C33에서 작성된 테스트는 약 83개 (task_autonomou.md 기록 기준)

## 설계 결정

- pytest 프레임워크 사용
- ROS2 독립 테스트 (ad_core 순수 알고리즘만)
- 물리 법칙 검증 (에너지 보존, 대칭성, 범위 제한 등)
- ROS2 의존 노드(ad_perception 등)의 Mock 테스트는 미작성
