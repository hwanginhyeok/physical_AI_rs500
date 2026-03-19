# PROJECT MAP — 프로젝트 지도

> **관리 주체**: 팀장 (ProjectLeader)
> **최종 업데이트**: 2026-03-19
>
> 이 문서는 프로젝트의 모든 디렉토리와 파일을 설명하는 **지도**이다.
> 새로운 파일/디렉토리가 추가될 때마다 팀장이 이 문서를 갱신한다.

---

## 운영 규칙

1. **파일/디렉토리 추가 시** 반드시 이 문서에 항목을 추가한다.
2. **파일 삭제/이동 시** 해당 항목을 제거하거나 수정한다.
3. **팀장이 지시하고 확인**한다. 작업자가 코드를 추가하면 팀장이 이 문서 반영 여부를 검증한다.
4. 각 항목은 `파일명` — 한 줄 설명 형식을 따른다.
5. 디렉토리 설명은 해당 섹션 상단에 블록으로 기술한다.

---

## 프로젝트 루트

```
자율주행프로젝트/
├── src/                    # ROS2 패키지 (자율주행 코어, 7개 활성)
│   ├── ad_bringup/         # 시뮬레이션·실차 런치·설정 통합
│   ├── ad_can_bridge/      # CAN 통신 브릿지
│   ├── ad_control/         # 제어 노드 (Hybrid E2E + Safety Guardian)
│   ├── ad_core/            # 순수 알고리즘 코어 (ROS2 독립)
│   ├── ad_interfaces/      # 커스텀 메시지/서비스 (실차 통합용, 현재 미사용)
│   ├── ad_perception/      # Learned 인지 (YOLOv8, Camera-Only)
│   ├── ad_planning/        # Learned 판단 (Diffusion-based)
│   └── _archived/          # 보관 (ad_simulation — git history 참조용)
├── agents/                 # AI 에이전트 시스템 (프로젝트 관리/모델링/연구)
├── docs/                   # 기술 문서 및 선행연구 보고서
├── .claude/                # Claude Code 설정
├── README.md               # 프로젝트 개요
├── PROJECT_MAP.md          # [이 문서] 프로젝트 지도
├── docs/프로젝트/TASK.md    # 작업 현황 관리 (통합 단일 파일)
└── .gitignore              # Git 추적 제외 규칙
```

| 파일 | 설명 |
|------|------|
| `README.md` | 프로젝트 소개, 패키지 구성, 빌드/실행 방법 |
| `PROJECT_MAP.md` | 디렉토리/파일별 역할 설명 (이 문서). 팀장이 관리 |
| `docs/프로젝트/TASK.md` | 작업 현황 테이블, 완료 이력, TODO 아이디어 (통합 관리) |
| `.gitignore` | build/, install/, log/, __pycache__/ 등 제외 |

---

## src/ — ROS2 패키지

> 자율주행 시스템의 핵심 코드. ROS2 패키지 7개(활성) + 1개(보관)로 구성.
> `colcon build`로 빌드하며, 각 패키지는 독립적으로 빌드/실행 가능.

---

### src/ad_bringup/ — 시뮬레이션·런치·설정 통합

> 시뮬레이션 환경(Gazebo Harmonic), 차량 URDF, 런치, Nav2/EKF 설정을 통합 관리.
> 프로젝트의 실질적 시뮬레이션 허브.

| 파일 | 설명 |
|------|------|
| `package.xml` | 패키지 메타데이터 |
| `setup.py` | Python 패키지 빌드 설정 |

#### ad_bringup/ (코드)

| 파일 | 설명 |
|------|------|
| `__init__.py` | 패키지 초기화 |
| `cmd_vel_relay.py` | cmd_vel 릴레이 노드 |
| `waypoint_manager_node.py` | 웨이포인트 관리 노드 |

#### config/

| 파일 | 설명 |
|------|------|
| `bridge_config.yaml` | ros_gz_bridge 토픽 매핑. cmd_vel(ROS→GZ), 3대 카메라(RGB+PointCloud2+CameraInfo), GPS, IMU, odom, clock, joint_states |
| `dual_ekf.yaml` | robot_localization Dual-EKF 설정: ekf_local(50Hz, odom+IMU), ekf_global(10Hz, +GPS), navsat_transform |
| `nav2_params.yaml` | Nav2 파라미터: SmacPlannerLattice(전역), MPPI(지역, 7 critic), costmap(global 0.1m, local 0.05m) |

#### launch/

| 파일 | 설명 |
|------|------|
| `simulation_launch.py` | 통합 시뮬 런처. Gazebo 서버/GUI + 차량 스폰 + ros_gz_bridge. `world:=` 인자로 월드 선택 |
| `full_system_launch.py` | 전체 시스템 런처 (시뮬 + 네비게이션 + 제어) |
| `navigation_launch.py` | EKF(local+global) + navsat_transform + Nav2 bringup 통합 런처 |
| `waypoint_nav_demo_launch.py` | 웨이포인트 네비게이션 데모 런처 |
| `record_launch.py` | rosbag 녹화 런처 |

#### urdf/

| 파일 | 설명 |
|------|------|
| `ss500.urdf.xacro` | SS500 궤도차량 URDF (단일 소스). chassis + left/right_track, 카메라 3대(front/left/right), GPS, IMU |

#### models/ss500/

| 파일 | 설명 |
|------|------|
| `model.sdf` | SS500 차량 SDF (Gazebo용). TrackedVehicle + TrackController 플러그인, 센서(GPS, IMU, RGBD Camera 3대) |

#### worlds/

| 파일 | 설명 |
|------|------|
| `flat_field.sdf` | 평지 테스트 월드 |

#### maps/

| 파일 | 설명 |
|------|------|
| `empty_map.yaml` | 빈 맵 (Nav2 초기화용) |

---

### src/ad_core/ — 순수 알고리즘 코어

> ROS2 독립 순수 Python + numpy 알고리즘.
> adtpc(실차 배포용)로 그대로 이식 가능.

| 파일 | 설명 |
|------|------|
| `package.xml` | 패키지 메타데이터 |
| `setup.py` | Python 빌드 설정 |

#### ad_core/ (코드)

| 파일 | 설명 |
|------|------|
| `__init__.py` | 패키지 초기화. utils에서 공통 함수 re-export |
| `utils.py` | 공용 유틸: normalize_angle, clamp, lerp, distance_2d 등 |
| `datatypes.py` | 공통 데이터 타입 (Pose2D 등) |
| `hybrid_e2e_types.py` | Hybrid E2E 파이프라인 데이터 타입 (PerceptionFeatures, TerrainClass, Line3D, Object3D 등) |
| `pure_pursuit.py` | Pure Pursuit 경로 추종: 속도비례 lookahead, 원-직선 교차점 보간 |
| `skid_steer_model.py` | ICR 스키드 스티어 모델: twist↔tracks 변환, 슬립 보상, 궤적 예측 |
| `drivetrain_model.py` | 구동계 모델 |
| `vehicle_dynamics.py` | 차량 동역학 모델 |
| `track_terrain_interaction.py` | 궤도-지형 상호작용 모델 |
| `coverage_planner.py` | Boustrophedon 커버리지 경로 계획 |
| `terrain_classifier.py` | 지형 분류: RGB 색상 기반 7종(PAVED/DIRT_ROAD/GRAVEL/GRASS/CROP_FIELD/MUD/OBSTACLE) |
| `camera_detector.py` | 카메라 탐지: YOLO 래퍼 (ultralytics 미설치 시 dummy fallback) |
| `semantic_segmenter.py` | 시맨틱 세그멘테이션 모듈 |
| `sensor_noise_model.py` | 센서 노이즈 시뮬레이션 모델 |
| `can_interface.py` | CAN 인터페이스 추상화 |

#### test/

| 파일 | 설명 |
|------|------|
| `test_skid_steer_model.py` | 스키드 스티어 모델 단위 테스트 |
| `test_drivetrain_model.py` | 구동계 모델 단위 테스트 |
| `test_vehicle_dynamics.py` | 차량 동역학 단위 테스트 |
| `test_track_terrain_interaction.py` | 궤도-지형 상호작용 테스트 |
| `test_coverage_planner.py` | 커버리지 플래너 테스트 |
| `test_sensor_noise_model.py` | 센서 노이즈 모델 테스트 |
| `test_slip_compensated_pursuit.py` | 슬립 보상 Pure Pursuit 테스트 |

---

### src/ad_control/ — 제어 노드

> Hybrid E2E 통합 제어 + Safety Guardian.

| 파일 | 설명 |
|------|------|
| `package.xml` | 패키지 메타데이터. ad_core 의존 |
| `setup.py` | Python 빌드 설정 |

#### ad_control/ (코드)

| 파일 | 설명 |
|------|------|
| `__init__.py` | 패키지 초기화 |
| `hybrid_e2e_node.py` | Hybrid E2E 노드: perception→planning→control 통합 파이프라인 |
| `control_node.py` | 제어 노드: Pure Pursuit→SkidSteer 파이프라인, cmd_vel 발행 |
| `safety_guardian.py` | 안전 감시: 비상 정지, 속도 제한, 장애물 회피 판단 |

#### config/

| 파일 | 설명 |
|------|------|
| `control_params.yaml` | 제어 파라미터: PID 게인, 최대 속도/조향 등 |

#### launch/

| 파일 | 설명 |
|------|------|
| `control_launch.py` | 제어 노드 런처 |

#### test/

| 파일 | 설명 |
|------|------|
| `test_hybrid_e2e_logic.py` | Hybrid E2E 로직 테스트 |
| `test_safety_guardian.py` | Safety Guardian 테스트 |

---

### src/ad_perception/ — Learned 인지

> YOLOv8 기반 Camera-Only 인지 파이프라인 (C64: LiDAR 제거 완료).
> 지형 세그멘테이션 + 장애물 탐지 + 작물 행 추출.

| 파일 | 설명 |
|------|------|
| `package.xml` | 패키지 메타데이터. ad_core 의존 |
| `setup.py` | Python 빌드 설정. entry_points: learned_perception_node, mono_depth_node |

#### ad_perception/ (코드)

| 파일 | 설명 |
|------|------|
| `__init__.py` | 패키지 초기화 |
| `learned_perception_node.py` | Learned 인지 ROS2 노드 (메인 실행 노드) |
| `perception_manager.py` | 인지 시스템 통합 관리자: Learned/Traditional 모드 전환 |
| `perception_node.py` | 기존 인지 노드 (레거시) |
| `terrain_segmentation.py` | YOLOv8-seg 지형 세그멘테이션 |
| `obstacle_detector.py` | YOLOv8 장애물 탐지 |
| `crop_row_extractor.py` | 작물 행 추출 (Hough + 3D 변환) |
| `model_manager.py` | 모델 로딩/캐싱 관리 |
| `mono_depth_node.py` | 단안 깊이 추정 노드 (C66, 실차 테스트 대기) |

#### config/

| 파일 | 설명 |
|------|------|
| `perception_config.yaml` | Learned Perception 설정: 토픽 매핑, 모델 파라미터, 추론 설정 (MX550 최적화) |

#### launch/

| 파일 | 설명 |
|------|------|
| `perception_launch.py` | learned_perception_node 런처 |
| `perception.launch.py` | 인지 런처 (대체 형식) |

#### test/

| 파일 | 설명 |
|------|------|
| `test_learned_perception_logic.py` | Learned 인지 로직 테스트 |
| `test_perception.py` | 인지 기본 테스트 |

---

### src/ad_planning/ — Learned 판단

> Diffusion 모델 기반 경로 계획.

| 파일 | 설명 |
|------|------|
| `package.xml` | 패키지 메타데이터. ad_core, ad_control 의존 |
| `setup.py` | Python 빌드 설정 |

#### ad_planning/ (코드)

| 파일 | 설명 |
|------|------|
| `__init__.py` | 패키지 초기화 |
| `planning_node.py` | 판단 ROS2 노드: PerceptionFeatures → PlannedTrajectory. SafetyGuardian 연동 |
| `diffusion_planner.py` | Diffusion 기반 경로 계획 알고리즘 |

#### config/

| 파일 | 설명 |
|------|------|
| `planning_params.yaml` | 판단 파라미터 |

#### launch/

| 파일 | 설명 |
|------|------|
| `planning_launch.py` | 판단 노드 런처 |

---

### src/ad_can_bridge/ — CAN 통신 브릿지

> SS500 실차 CAN 통신 인터페이스.

| 파일 | 설명 |
|------|------|
| `package.xml` | 패키지 메타데이터 |
| `setup.py` | Python 빌드 설정 |

#### ad_can_bridge/ (코드)

| 파일 | 설명 |
|------|------|
| `__init__.py` | 패키지 초기화 |
| `can_bridge_node.py` | CAN 브릿지 ROS2 노드 |
| `ss500_codec.py` | SS500 CAN 프로토콜 인코더/디코더 |

#### config/

| 파일 | 설명 |
|------|------|
| `can_params.yaml` | CAN 통신 파라미터 |

#### launch/

| 파일 | 설명 |
|------|------|
| `can_bridge_launch.py` | CAN 브릿지 런처 |

#### test/

| 파일 | 설명 |
|------|------|
| `test_can_bridge_logic.py` | CAN 브릿지 로직 테스트 |
| `test_ss500_codec.py` | SS500 코덱 테스트 |

---

### src/ad_interfaces/ — 커스텀 메시지/서비스

> 실차 CAN 통합용 ROS2 인터페이스 정의.
> **시뮬레이션 파이프라인에서는 미사용.** 실차(SS500) CAN 연동 시 사용 예정.

| 파일 | 설명 |
|------|------|
| `CMakeLists.txt` | CMake 빌드 설정 (rosidl 메시지 생성) |
| `package.xml` | 패키지 메타데이터 (v0.1.0) |
| `README.md` | 패키지 상태 및 인터페이스 목록 |
| `msg/VehicleState.msg` | 차량 상태: GPS 좌표, 속도, 방향, 가속도, 기어, 상태 플래그 |
| `msg/DrivingCommand.msg` | 주행 명령: 목표 속도, 조향각, 브레이크, 주행 모드, 비상정지 |
| `srv/SetMode.srv` | 모드 변경 서비스: 요청(모드명) → 응답(성공 여부, 메시지) |

---

### src/_archived/ad_simulation/ — 보관된 시뮬레이션 패키지

> **보관 사유**: ad_bringup이 실질적 시뮬 허브로 대체 완료.
> ad_simulation의 URDF는 4륜 DiffDrive (우리 차량은 궤도차), LiDAR 포함, 차량 크기 불일치.
> git history 참조용으로 보관. 빌드 대상에서 제외됨.

---

## agents/ — AI 에이전트 시스템

> 프로젝트 관리, 시뮬레이션 모델링, 선행조사를 자동화하는 AI 에이전트.
> asyncio 기반 비동기 구조. ROS2 코드와 독립적으로 동작한다.
>
> **실행**: `python -m agents` (대화형) 또는 `python -m agents model vehicle --mass 800`

| 파일 | 설명 |
|------|------|
| `__init__.py` | 패키지 초기화 |
| `__main__.py` | `python -m agents` 진입점 |
| `main.py` | CLI 인터페이스. argparse 기반 명령어 파싱 → 에이전트 실행 |

### agents/config/

| 파일 | 설명 |
|------|------|
| `agents_config.yaml` | 에이전트 활성화/비활성화 설정, 각 에이전트별 파라미터 |

### agents/core/ — 프레임워크

| 파일 | 설명 |
|------|------|
| `base_agent.py` | `BaseAgent` 추상 클래스. `start()/stop()` 라이프사이클, `handle_task()` 추상 메서드, MessageBus 연결 |
| `task.py` | `Task` dataclass + `TaskStatus` + `TaskPriority` |
| `message_bus.py` | asyncio 기반 pub/sub 메시지 버스 |
| `task_tracker.py` | `task_autonomou.md` 파일 자동 읽기/갱신 모듈 |

### agents/project_leader/ — 팀장 에이전트

| 파일 | 설명 |
|------|------|
| `project_leader_agent.py` | 작업 생성/할당, 진행 조회, 보고서 생성, 계획→세부작업 분해 |

### agents/modeling/ — 모델링 에이전트

| 파일 | 설명 |
|------|------|
| `modeling_agent.py` | 통합 에이전트 |
| `vehicle_modeler.py` | 차량 SDF 조작 |
| `world_modeler.py` | 월드 SDF 조작 |
| `physics_modeler.py` | 물리 엔진 설정 |

### agents/research/ — 선행조사 에이전트

| 파일 | 설명 |
|------|------|
| `research_agent.py` | 통합 에이전트 |
| `web_searcher.py` | 웹/논문 검색 |
| `document_writer.py` | 마크다운 보고서 생성 |

### agents/docs/ — 에이전트 문서

| 파일 | 설명 |
|------|------|
| `README.md` | 에이전트 시스템 개요, 설치, 사용법 |
| `checklist.md` | 에이전트 기능 체크리스트 |

---

## docs/ — 기술 문서

> 프로젝트 정의서, 선행연구 보고서를 보관.

| 파일 | 설명 |
|------|------|
| `application_definition.md` | Application 정의서. RS500 시스템 개요, 적용 분야, HW/SW 아키텍처, 센서 구성, 운용 시나리오 |

### docs/research/ — 선행연구 보고서

| 파일 | 설명 |
|------|------|
| `../literature_review_dynamics_control.md` | 동역학/제어 논문 27편 |
| `path_planning_and_slam_literature_review.md` | 경로계획/SLAM 논문 40+편 |
| `perception_sensor_fusion_survey.md` | 인지/센서융합 논문 30+편 |
| `gazebo_tracked_vehicle_simulation_research.md` | 시뮬레이션 연구 |

---

## ROS2 토픽 맵

> `bridge_config.yaml` 기준. 카메라 3대 (front/left/right) RGB + PointCloud2 + CameraInfo.
> C64: LiDAR 토픽 제거 완료.

| 토픽 | 메시지 타입 | 방향 | 용도 |
|------|-----------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | ROS→GZ | 차량 속도 명령 (linear.x, angular.z) |
| `/odom` | `nav_msgs/Odometry` | GZ→ROS | 차량 오도메트리 |
| `/sensor/gps` | `sensor_msgs/NavSatFix` | GZ→ROS | GPS 위치 (WGS84) |
| `/sensor/imu` | `sensor_msgs/Imu` | GZ→ROS | IMU (가속도, 각속도, 방향) |
| `/sensor/camera/front/image` | `sensor_msgs/Image` | GZ→ROS | 전방 카메라 RGB |
| `/sensor/camera/front/points` | `sensor_msgs/PointCloud2` | GZ→ROS | 전방 카메라 PointCloud2 (Nav2 costmap 장애물 소스) |
| `/sensor/camera/front/camera_info` | `sensor_msgs/CameraInfo` | GZ→ROS | 전방 카메라 내부 파라미터 |
| `/sensor/camera/left/image` | `sensor_msgs/Image` | GZ→ROS | 좌전방 카메라 RGB |
| `/sensor/camera/left/points` | `sensor_msgs/PointCloud2` | GZ→ROS | 좌전방 카메라 PointCloud2 |
| `/sensor/camera/left/camera_info` | `sensor_msgs/CameraInfo` | GZ→ROS | 좌전방 카메라 내부 파라미터 |
| `/sensor/camera/right/image` | `sensor_msgs/Image` | GZ→ROS | 우전방 카메라 RGB |
| `/sensor/camera/right/points` | `sensor_msgs/PointCloud2` | GZ→ROS | 우전방 카메라 PointCloud2 |
| `/sensor/camera/right/camera_info` | `sensor_msgs/CameraInfo` | GZ→ROS | 우전방 카메라 내부 파라미터 |
| `/clock` | `rosgraph_msgs/Clock` | GZ→ROS | 시뮬레이션 시간 동기화 |
| `/joint_states` | `sensor_msgs/JointState` | GZ→ROS | 조인트 상태 |

---

## 의존성 그래프

```
ad_core  ←── ad_perception (hybrid_e2e_types, datatypes, camera_detector, semantic_segmenter)
         ←── ad_planning   (hybrid_e2e_types, datatypes)
         ←── ad_control    (순수 알고리즘 참조)

ad_control ←── ad_planning (safety_guardian 참조)

ad_interfaces  (현재 미사용, 실차 CAN 통합 시 활성화 예정)

ad_bringup ──→ simulation_launch.py (Gazebo + 차량 스폰)
           ──→ navigation_launch.py (EKF + Nav2)
           ──→ full_system_launch.py (전체 시스템)

ad_can_bridge ──→ SS500 CAN 통신 (실차 전용)

agents/    ──→ src/ad_bringup/models/ (vehicle_modeler가 SDF 수정)
           ──→ docs/ (research 에이전트가 보고서 생성)
```

---

## 변경 이력

| 날짜 | 변경 내용 |
|------|----------|
| 2026-03-19 | **구조 정비**: team_leader 섹션 삭제, 7개 활성 패키지 반영, ad_simulation→_archived 이동, LiDAR 토픽 제거, 카메라 3대 RGB+PointCloud2+CameraInfo 반영, 의존성 그래프 현행화 |
| 2026-02-21 | 제어/판단/인지/시뮬레이션 추가 |
| 2026-02-21 | 초판 작성 |
