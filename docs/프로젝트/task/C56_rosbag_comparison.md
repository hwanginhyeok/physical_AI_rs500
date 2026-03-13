# C56: Foxglove Rosbag 시뮬/실물 비교 체계 구축

- 분야: 인프라
- 담당: 그린
- 기간: 2026-03-01 ~ 2026-03-01
- 상태: 완료

## 배경

Foxglove 단일 인터페이스에서 시뮬레이션과 실물 로봇을 비교하고 싶다는 요구.
실물 로봇이 이미 있으므로 즉시 비교 체계가 필요.

## 결정 사항

| 결정 | 이유 | 검토한 대안 |
|------|------|-------------|
| **Method B (rosbag 오프라인 비교)** 우선 구현 | 실물 로봇 보유 → 즉시 적용 가능. 구조 변경 없이 현재 코드 그대로 사용. Foxglove 멀티소스 기능으로 두 bag을 같은 타임라인에 오버레이 | **Method A (namespace 분리 라이브 비교)**: 모든 토픽 `/sim/*`·`/real/*` 네임스페이스 적용 필요 → launch, YAML, bridge_config, waypoint_manager, foxglove_layout 전체 수정 필요. 실물 준비 완료 전까지 오버엔지니어링. C57로 등록 |

## 변경 내역

### 신규 파일

| 파일 | 역할 |
|------|------|
| `src/ad_bringup/launch/record_launch.py` | rosbag 녹화 런치. `robot:=sim\|real` 인자, zstd 압축, 타임스탬프 파일명 자동 생성 |
| `src/ad_bringup/config/foxglove_comparison_layout.json` | 4탭 비교 레이아웃: ①Navigation 비교(3D 좌/우) ②속도/제어 비교 ③GPS 경로 ④Diagnostics |

### 녹화 토픽 목록

`/tf`, `/tf_static`, `/clock`, `/robot_description`,
`/odom`, `/sensor/imu`, `/sensor/gps`, `/sensor/lidar`, `/sensor/camera/front`,
`/odometry/local`, `/odometry/global`, `/odometry/gps`,
`/cmd_vel`, `/plan`, `/local_plan`,
`/waypoint_manager/status`, `/waypoint_manager/markers`,
`/global_costmap/costmap`, `/local_costmap/costmap`, `/map`

## 검증

- 빌드: `colcon build --packages-select ad_bringup` 성공 (5.00s)
- 런치 파일 구문: Python 파싱 정상

## 사용법

```bash
# 시뮬 녹화 (시뮬 실행 중인 터미널과 별도)
ros2 launch ad_bringup record_launch.py robot:=sim
# → ~/rosbags/sim_20260301_HHMMSS/

# 실물 녹화
ros2 launch ad_bringup record_launch.py robot:=real
# → ~/rosbags/real_20260301_HHMMSS/

# Foxglove에서 비교
# Data Sources → [+] → sim bag, [+] → real bag
# Layouts → Import → foxglove_comparison_layout.json
```

## 미결 이슈

- [ ] C57: Method A (namespace 분리) 구현 시 라이브 동시 비교 가능 → 실물 로봇 연동 완료 후 착수
- [ ] rosbag 용량 모니터링 (카메라 포함 시 분당 수백MB) — 필요 시 `/sensor/camera/front` 제외 옵션 추가
