# C42: Foxglove + Gazebo + Nav2 다중 웨이포인트 네비게이션

- 분야: 시뮬레이션
- 담당: 그린
- 기간: 2026-02-28 ~ 2026-02-28
- 상태: 완료

## 배경

Foxglove Studio에서 맵 위 좌표를 클릭하면 Gazebo 시뮬레이션 내 SS500 궤도차량이
해당 좌표들을 순차적으로 방문하는 시스템이 필요했다.
기존 Nav2 설정(nav2_params.yaml)을 풀스택으로 활용하며,
집 PC(RTX 2060급)에서 Gazebo Harmonic과 연동할 계획.

## 결정 사항

| 결정 | 이유 | 검토한 대안 |
|------|------|-------------|
| SDF + URDF 이중 모델 | SDF는 Gazebo 물리/센서, URDF는 TF 발행 전용. URDF→SDF 변환 불안정 회피 | 단일 URDF (gz_ros2_control) — 변환 깨짐 위험 |
| TrackedVehicle + DiffDrive 폴백 | 궤도차량 물리 정확도 우선, API 변경 시 즉시 전환 가능 | DiffDrive만 — 궤도차량 특성 반영 불가 |
| FollowWaypoints 액션 | Nav2 내장 웨이포인트 순차 방문. 개별 실패 추적 가능 | NavigateThroughPoses — 경유점으로만 취급, 개별 도착 확인 불가 |
| Foxglove 웹 버전 | 설치 불필요, 포트 포워딩으로 WSL2 연동 | Desktop 앱 — 동일 기능, 별도 설치 필요 |
| 3탭 16패널 레이아웃 | Navigation/Diagnostics/Debug 역할 분리 | 단일 화면 — 패널 과밀 |

## 변경 내역

### 신규 파일 (12개)

| 파일 | 줄수 | 역할 |
|------|------|------|
| `ad_bringup/__init__.py` | 1 | 패키지 init |
| `ad_bringup/waypoint_manager_node.py` | 304 | 핵심 노드: /goal_pose 수집 → FollowWaypoints 전달 |
| `models/ss500/model.sdf` | 330 | Gazebo 물리 모델 + TrackedVehicle + 4센서 |
| `models/ss500/model.config` | 16 | Gazebo 메타데이터 |
| `urdf/ss500.urdf.xacro` | 194 | TF 트리 (7프레임) |
| `config/bridge_config.yaml` | 52 | ros_gz_bridge 8토픽 매핑 |
| `worlds/flat_field.sdf` | 160 | 100m×100m 평지 + 장애물 3개 |
| `maps/empty_map.yaml` | 8 | Nav2 빈 맵 설정 |
| `maps/empty_map.pgm` | — | 1000×1000 자유공간 PGM |
| `launch/simulation_launch.py` | 180 | 7-서브시스템 통합 런치 |
| `config/foxglove_layout.json` | 240 | 3탭 16패널 Foxglove 레이아웃 |
| `launch/waypoint_nav_demo_launch.py` | 48 | 데모 래퍼 |

### 수정 파일 (2개)

| 파일 | 변경 내용 |
|------|----------|
| `setup.py` | packages=['ad_bringup'], entry_points(waypoint_manager), data_files(urdf, models, worlds, maps) 추가 |
| `package.xml` | nav2_msgs, visualization_msgs, std_srvs, ros_gz_sim, ros_gz_bridge, robot_state_publisher, xacro, foxglove_bridge 의존성 추가 |

## 아키텍처

### 데이터 흐름
```
Foxglove 클릭 → /goal_pose → WaypointManager(수집)
  → /start 서비스 → FollowWaypoints 액션
  → BT Navigator → SmacPlannerLattice(경로) → MPPI Controller(추종)
  → Velocity Smoother → Collision Monitor → /cmd_vel
  → ros_gz_bridge → Gazebo TrackedVehicle → 물리 시뮬
  → OdometryPublisher → /odom → ekf_local → /odometry/local
  → ekf_global → TF(map→odom) → Nav2 costmap 갱신 → 반복
```

### TF 트리
```
map → odom → base_link → imu_link
 ^      ^               → lidar_link
 |      |               → camera_link
 |      |               → gps_link
 |      └── ekf_local    → left_track / right_track
 └── ekf_global
```

### WaypointManagerNode 인터페이스
- 구독: `/goal_pose` (PoseStamped)
- 발행: `/waypoint_manager/markers` (MarkerArray), `/waypoint_manager/status` (String/JSON)
- 서비스: `/start`, `/clear`, `/cancel`, `/remove_last` (std_srvs/Trigger)
- 액션: FollowWaypoints, NavigateThroughPoses (파라미터로 전환)

## 검증

- Python 문법 검사: 4파일 모두 py_compile 통과
- 실제 시뮬레이션 테스트: C44 (집 PC WSL2 환경 구축) 완료 후 진행

## 미결 이슈

- [ ] TrackedVehicle 플러그인 Gazebo Harmonic 호환성 (안 되면 DiffDrive 폴백)
- [ ] EKF 공분산 시뮬레이션 환경 튜닝
- [ ] GPU LiDAR + Camera 동시 실행 시 RTX 2060 성능 확인 → Hz 조절
- [ ] Foxglove 레이아웃 실제 시뮬레이션 데이터로 미세 조정
