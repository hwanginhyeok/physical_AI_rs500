# SS500 자율주행 시스템 아키텍처 개요

> **문서 성격**: Living Document — 시스템 구조가 변경될 때마다 갱신한다.
> **최종 갱신**: 2026-03-11 (C61 velocity chain 분석 반영)

---

## 1. 시스템 구성 (10개 ROS2 패키지)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        SS500 자율주행 시스템                              │
│                    (ROS2 Jazzy + Gazebo Harmonic)                        │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────┐   ┌──────────────┐   ┌──────────────┐                │
│  │ ad_interfaces│   │   ad_core     │   │ ad_can_bridge│                │
│  │ (메시지 정의) │   │ (순수 알고리즘)│   │ (CAN 하드웨어)│                │
│  └─────────────┘   └──────────────┘   └──────────────┘                │
│                           │                                             │
│         ┌─────────────────┼─────────────────┐                          │
│         ▼                 ▼                 ▼                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                 │
│  │ ad_perception │  │ ad_planning  │  │ ad_control   │                 │
│  │ (인지 노드)   │  │ (계획 노드)  │  │ (제어 노드)  │                 │
│  └──────────────┘  └──────────────┘  └──────────────┘                 │
│         │                 │                 │                          │
│         └─────────────────┼─────────────────┘                          │
│                           ▼                                             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                 │
│  │ ad_bringup   │  │ ad_simulation│  │ team_leader  │                 │
│  │ (런치/설정)   │  │ (Gazebo SDF) │  │ (통합 노드)  │                 │
│  └──────────────┘  └──────────────┘  └──────────────┘                 │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### ad_core (순수 알고리즘 — ROS 의존 없음)

| 모듈 | 역할 |
|------|------|
| `vehicle_dynamics.py` | 궤도차량 동역학 모델 |
| `skid_steer_model.py` | ICR 기반 스키드 스티어 운동학 |
| `drivetrain_model.py` | 모터/감속기 드라이브트레인 |
| `pure_pursuit.py` | 슬립 보상 Pure Pursuit 경로 추종 |
| `lidar_processor.py` | Ray Ground Filter + Euclidean Clustering |
| `camera_detector.py` | YOLO v8n 기반 객체 탐지 |
| `semantic_segmenter.py` | 7-class 지형 세그멘테이션 |
| `terrain_classifier.py` | 지형 traversability 분류 |
| `sensor_fusion.py` | Camera-LiDAR Late Fusion |
| `coverage_planner.py` | Boustrophedon 커버리지 경로 |
| `sensor_noise_model.py` | GPS/IMU/LiDAR 센서 노이즈 모델 |
| `track_terrain_interaction.py` | 궤도-지면 상호작용 |
| `can_interface.py` | CAN 프레임 인코더/디코더 |

---

## 2. 시뮬레이션 데이터 흐름 (End-to-End)

```
┌───────────────────────────── Gazebo Harmonic ─────────────────────────┐
│  SS500 Vehicle (1000kg, 궤도차량)                                      │
│  TrackedVehicle Plugin ← /cmd_vel (Twist)                             │
│  Sensors: GPS(10Hz) IMU(100Hz) LiDAR(10Hz) Camera(15Hz) Odom(50Hz)   │
└──────────────────────────────┬────────────────────────────────────────┘
                               │ ros_gz_bridge (bridge_config.yaml)
                               ▼
┌─────────────────── Localization (Dual-EKF, ARCH-002) ────────────────┐
│                                                                       │
│  /sensor/gps ──→ navsat_transform ──→ /odometry/gps ──┐             │
│                     (WGS84→UTM)                        │             │
│                                                        ▼             │
│  /odom ──────┐                                  ekf_global (10Hz)    │
│  /sensor/imu ┼──→ ekf_local (50Hz) ──→ /odometry/local              │
│              │     TF: odom→base_footprint       /odometry/global    │
│              │                                                        │
│  static_transform: map→odom (임시 고정, ARCH-001)                     │
└───────────────────────────────────────────────────────────────────────┘
                               │
                               ▼
┌─────────────────── Navigation (Nav2 Stack, ARCH-003) ────────────────┐
│                                                                       │
│  bt_navigator (BT XML) ──→ NavigateThroughPoses                      │
│       │                                                               │
│       ├──→ SmacPlannerLattice (Global) ──→ /plan                     │
│       │     5cm resolution, 0.5m turn radius                         │
│       │                                                               │
│       └──→ MPPI Controller (Local) ──→ /cmd_vel_nav                  │
│             1000 samples, 2.8s horizon, 7 critics                    │
│                                                                       │
│  Global Costmap: 100m×100m rolling, 0.1m res                        │
│  Local Costmap:  6m×6m, 0.05m res, VoxelLayer (z=16)                │
└───────────────────────────────────────────────────────────────────────┘
                               │
                               ▼
┌─────────────────── Velocity Chain ───────────────────────────────────┐
│                                                                       │
│  controller_server                                                    │
│       │ /cmd_vel_nav                                                  │
│       ▼                                                               │
│  velocity_smoother (Rate Limiter, 20Hz)                              │
│       │ /cmd_vel_smoothed                                             │
│       ▼                                                               │
│  collision_monitor (FootprintApproach, LiDAR 기반)                   │
│       │ /cmd_vel                                                      │
│       ▼                                                               │
│  ros_gz_bridge → Gazebo TrackedVehicle                               │
│                                                                       │
│  ★ C61 BLOCKER (2026-03-04):                                        │
│    velocity_smoother가 lifecycle activate 실패                        │
│    → DDS SharedMemory 포트 충돌 (FastRTPS)                            │
│    → cmd_vel_smoothed 미발행 → 차량 미이동                             │
│    → 해결: CycloneDDS 전환 또는 WSL 재시작 + shm 정리                  │
└───────────────────────────────────────────────────────────────────────┘
```

---

## 3. TF 트리 (ARCH-001)

```
/map ──(static)──→ /odom ──(ekf_local 50Hz)──→ /base_footprint
                                                      │
                                        (robot_state_publisher)
                                                      │
                                                /base_link
                                              /    |    |    \
                                             /     |    |     \
                                    /lidar  /imu  /gps  /camera  /left_track  /right_track
                                       │      │     │
                                  (static TF bridges — Gazebo scoped frame 보정, C55)
                                       │      │     │
                            /ss500/lidar_link  │  /ss500/gps_link
                             /gpu_lidar    /ss500/imu_link   /gps_sensor
                                            /imu_sensor
```

**각 프레임 발행 책임:**

| 변환 | 발행 노드 | 비고 |
|------|-----------|------|
| map → odom | `map_to_odom_static` | 임시 identity. ekf_global 전환 예정 |
| odom → base_footprint | `ekf_local` (50Hz) | IMU + 휠 오도메트리 융합 |
| base_footprint → base_link | `robot_state_publisher` | URDF 정의 |
| base_link → sensor frames | `robot_state_publisher` | URDF 정의 |
| sensor → ss500/scoped | `*_frame_bridge` (3개) | Gazebo scoped name 브릿지 |

---

## 4. Hybrid E2E 아키텍처 (ARCH-004)

```
┌──────────────── 현재: Nav2 Classical Pipeline (동작 중) ──────────────┐
│  Sensors → Costmap Layers → SmacLattice → MPPI → cmd_vel → Vehicle   │
└──────────────────────────────────────────────────────────────────────┘

┌──────────────── 목표: Hybrid E2E Pipeline (C60, 구축 중) ────────────┐
│                                                                       │
│  Sensors ──→ Learned Perception (YOLOv8 + AutoSeg)                   │
│              ├─ Terrain Segmentation (7 classes) ✅ 구현              │
│              ├─ Obstacle Detection (YOLO v8n) ✅ 구현                 │
│              ├─ Crop Row Extraction ✅ 구현                           │
│              └─ PerceptionFeatures output                            │
│                    │                                                  │
│                    ▼                                                  │
│              Learned Planning (Diffusion Model)                      │
│              ├─ MLP 기반 경량 Diffusion ✅ 구현 (미훈련)              │
│              ├─ 20 waypoints, 10s horizon                            │
│              └─ PlannedTrajectory output                             │
│                    │                                                  │
│                    ▼                                                  │
│              Safety Guardian (Rule-Based) ✅ 구현 + 테스트            │
│              ├─ 경사도 검증 (max 17°)                                │
│              ├─ 작물 거리 (min 0.3m)                                  │
│              ├─ 속도 제한 (max 1.5 m/s)                               │
│              └─ Decision: proceed/slow_down/replan/stop              │
│                    │                                                  │
│                    ▼                                                  │
│              Hybrid E2E Node (통합 제어)                              │
│              ├─ 20Hz control loop                                    │
│              └─ 4-level fallback:                                     │
│                  L0: Diffusion (⬜ 미훈련)                            │
│                  L1: Fields2Cover (⬜ stub)                           │
│                  L2: Pure Pursuit (⬜ stub)                           │
│                  L3: Emergency Stop (✅)                              │
│                                                                       │
│  ★ Phase 1 (기반 구조) 완료 — Phase 2 (모델 훈련) 미착수             │
└───────────────────────────────────────────────────────────────────────┘
```

---

## 5. 핵심 설정 파일

| 파일 | 역할 | 관련 ADR |
|------|------|----------|
| `config/nav2_params.yaml` | Nav2 전체 파라미터 (플래너/컨트롤러/costmap/smoother) | ARCH-003 |
| `config/dual_ekf.yaml` | Dual-EKF + navsat 파라미터 | ARCH-002 |
| `config/bridge_config.yaml` | Gazebo ↔ ROS2 토픽 브릿지 매핑 | — |
| `launch/simulation_launch.py` | 시뮬레이션 통합 런치 (7단계 순서) | ARCH-001 |
| `urdf/ss500.urdf.xacro` | 로봇 URDF 정의 (TF 트리 원본) | ARCH-001 |
| `models/ss500/model.sdf` | Gazebo SDF 모델 (센서/물리/플러그인) | — |
| `worlds/flat_field.sdf` | 시뮬레이션 월드 (빈 평지) | — |
| `maps/empty_map.yaml` | Nav2 빈 맵 (rolling_window 사용) | ARCH-003 |

---

## 6. 현재 알려진 이슈

### 블로커

| ID | 증상 | Root Cause | 해결 방안 | 상태 |
|----|------|-----------|-----------|------|
| C61 | 차량 미이동: cmd_vel_smoothed 미발행 | FastRTPS SharedMemory 포트 충돌 → velocity_smoother lifecycle activate 실패 | CycloneDDS 전환 또는 `sudo rm -rf /dev/shm/fastrtps*` | ❌ 미해결 |

### 임시 조치 (Workaround) — 향후 전환 예정

| ID | 현재 | 전환 대상 | 전환 조건 |
|----|------|-----------|-----------|
| C50 | map→odom static identity TF | ekf_global `publish_tf: true` | GPS datum 안정 확인 |
| C52 | global_costmap rolling_window (맵 없음) | static_layer + 사전 맵 | 농경지 맵 확보 시 |
| C61 | cmd_vel_relay (collision_monitor 우회) | collision_monitor 정상 사용 | DDS 문제 해결 후 |

### 해결 완료

| ID | 문제 | 해결일 |
|----|------|--------|
| C50 | AMCL 빈 평지 초기화 실패 | 2026-03-01 |
| C51 | GPS datum 점핑 | 2026-03-01 |
| C53 | SDF noise 포맷 오류 | 2026-03-01 |
| C54 | EKF ↔ RSP TF 부모 충돌 | 2026-03-01 |
| C55 | Gazebo scoped frame_id 불일치 | 2026-03-01 |
| C58 | bt_navigator follow_path 타임아웃 (20ms→1000ms) | 2026-03-04 |

---

## 7. ADR 인덱스 (상세 → `docs/arch/`)

| # | 제목 | 상태 | 핵심 결정 |
|---|------|------|-----------|
| ARCH-001 | TF 트리 구조 | 채택됨 | ekf_local이 odom→base_footprint 발행, map→odom은 static |
| ARCH-002 | Dual-EKF 센서 퓨전 | 채택됨 | 로컬(50Hz, IMU+Odom) + 글로벌(10Hz, GPS) 분리 |
| ARCH-003 | Nav2 스택 구성 | 채택됨 | SmacLattice + MPPI, rolling_window costmap |
| ARCH-004 | Hybrid E2E 농업 자율주행 | 채택됨 | Learned Perception + Diffusion Planning + Rule-based Guardian |

---

## 변경 이력

| 날짜 | 변경 내용 |
|------|-----------|
| 2026-03-11 | 최초 작성 (C61 velocity chain 분석, 7일 공백 후 현황 정리) |
