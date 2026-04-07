# PROJECT MAP — 프로젝트 지도

> **관리 주체**: 팀장 (ProjectLeader)
> **최종 업데이트**: 2026-02-21
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
├── src/                    # ROS2 패키지 (자율주행 코어)
├── agents/                 # AI 에이전트 시스템 (프로젝트 관리/모델링/연구)
├── docs/                   # 기술 문서 및 선행연구 보고서
├── .claude/                # Claude Code 설정
├── README.md               # 프로젝트 개요
├── PROJECT_MAP.md          # [이 문서] 프로젝트 지도
├── task_autonomou.md       # 작업 현황 관리 (테이블 형식)
└── .gitignore              # Git 추적 제외 규칙
```

| 파일 | 설명 |
|------|------|
| `README.md` | 프로젝트 소개, 패키지 구성, 빌드/실행 방법 |
| `PROJECT_MAP.md` | 디렉토리/파일별 역할 설명 (이 문서). 팀장이 관리 |
| `task_autonomou.md` | 작업 현황 테이블, 완료 이력, TODO 아이디어 |
| `.gitignore` | build/, install/, log/, __pycache__/ 등 제외 |

---

## src/ — ROS2 패키지

> 자율주행 시스템의 핵심 코드. ROS2 패키지 3개로 구성.
> `colcon build`로 빌드하며, 각 패키지는 독립적으로 빌드/실행 가능.

### src/ad_interfaces/ — 커스텀 메시지/서비스

> 패키지 간 통신에 사용하는 ROS2 인터페이스 정의.
> 다른 패키지가 이 메시지 타입을 참조하므로 가장 먼저 빌드해야 한다.

| 파일 | 설명 |
|------|------|
| `CMakeLists.txt` | CMake 빌드 설정 (rosidl 메시지 생성) |
| `package.xml` | 패키지 메타데이터 (v0.1.0) |
| `msg/VehicleState.msg` | 차량 상태: GPS 좌표, 속도, 방향, 가속도, 기어, 상태 플래그 |
| `msg/DrivingCommand.msg` | 주행 명령: 목표 속도, 조향각, 브레이크, 주행 모드, 비상정지 |
| `srv/SetMode.srv` | 모드 변경 서비스: 요청(모드명) → 응답(성공 여부, 메시지) |

---

### src/ad_simulation/ — Gazebo 시뮬레이션

> gz-sim (Gazebo Harmonic) 기반 시뮬레이션 환경.
> 월드(지형), 차량 모델, 센서 구성, ROS 브릿지를 포함한다.
>
> **실행**: `ros2 launch ad_simulation simulation_launch.py world:=agricultural_field`

#### config/

| 파일 | 설명 |
|------|------|
| `bridge_config.yaml` | ros_gz_bridge 토픽 매핑. cmd_vel(ROS→GZ), 센서 데이터(GZ→ROS), /clock, /tf 포함 |

#### launch/

| 파일 | 설명 |
|------|------|
| `simulation_launch.py` | 통합 런처. Gazebo 서버/GUI + 차량 스폰 + ros_gz_bridge + team_leader 노드. `world:=` 인자로 월드 선택 |

#### models/tracked_vehicle/

> RS500 1톤급 궤도차량 모델. TrackedVehicle + TrackController 플러그인 사용.

| 파일 | 설명 |
|------|------|
| `model.config` | 모델 메타데이터 (이름, 버전, 저자) |
| `model.sdf` | 차량 SDF 정의: chassis + left/right_track, 센서(GPS, IMU, Camera, LiDAR), 물리 속성 |

#### worlds/ — 시뮬레이션 월드

> 3개의 월드를 제공한다. launch 파일에서 `world:=` 인자로 선택.

| 파일 | 설명 |
|------|------|
| `agricultural_field.sdf` | 농경지 월드 (200m x 200m). heightmap 지형, 고랑, 농로, 창고, 나무, 돌. 다중 마찰 영역 (포장 0.9, 밭 0.5, 진흙 0.25) |
| `agricultural_heightmap.png` | 농경지 heightmap (257x257, 16bit). 고저차 3m |
| `generate_heightmap.py` | 절차적 heightmap 생성 스크립트. 고랑(8px 간격) + 농로 패턴 |
| `visualize_agricultural.py` | matplotlib 시각화: 2D 고도맵 + 3D 표면 + 단면도 |
| `render_all_worlds.py` | 3개 월드 비교 3D 렌더링 (agricultural, yeongwol, cheongsong, cheongsong_detail) |
| `agricultural_visualization.png` | 농경지 시각화 결과 이미지 |
| `render_agricultural.png` | 농경지 3D 렌더링 결과 |
| `gazebo_screenshot_agricultural.png` | Gazebo GUI 스크린샷 |

#### worlds/real_terrain/ — 실제 지형 데이터

> SRTM DEM + OSM API로 수집한 실제 지형 기반 월드.
> `fetch_terrain_data.py`가 데이터 수집 → heightmap 생성 → SDF 월드 생성 파이프라인을 담당.

| 파일 | 설명 |
|------|------|
| **파이프라인** | |
| `fetch_terrain_data.py` | 지형 데이터 파이프라인: SRTM DEM 다운로드 → 보간 → heightmap PNG 생성 → OSM 도로/건물/수계 수집 → Cohen-Sutherland 클리핑 → 식생/바위 생성 → SDF 월드 출력 |
| `generate_orchard_world.py` | 청송 사과 과수원 월드 생성: fetch_terrain_data.py 활용 + 사과나무 2,400그루 배치 (4m x 5m 간격, 작업통로, 사과 열매) |
| `visualize_terrain.py` | matplotlib 4패널 시각화: 2D 고도 + 3D 표면 + OSM 오버레이 + 단면도 |
| **영월 신일리** | |
| `yeongwol_sinil.sdf` | 영월 월드 (500m x 500m). 고도 289-402m, 도로 38개, 수계 2개, 나무 80, 바위 25 |
| `sinil_heightmap.png` | 영월 heightmap (257x257, 16bit). SRTM 30m DEM 기반 |
| `sinil_osm_data.json` | 영월 OSM 데이터: 도로 13개, 건물 0, 수계 1(하천) |
| `terrain_visualization.png` | 영월 지형 시각화 결과 |
| **청송 과수원** | |
| `cheongsong_orchard.sdf` | 청송 월드 (300m x 300m). 고도 198-276m, 사과나무 2,400, 사과 975, 도로, 창고 |
| `cheongsong_heightmap.png` | 청송 heightmap (257x257, 16bit). SRTM 30m DEM 기반 |
| `cheongsong_osm_data.json` | 청송 OSM 데이터: 도로 7개, 건물 1(청송군청) |
| `cheongsong_visualization.png` | 청송 지형 시각화 결과 |
| **렌더링 결과** | |
| `render_yeongwol.png` | 영월 3D 렌더링 |
| `render_cheongsong.png` | 청송 3D 렌더링 |
| `render_cheongsong_detail.png` | 청송 과수원 클로즈업 (50m x 50m, 나무 간격 표시) |

#### test/ — SIL 테스트 프레임워크

> Software-in-the-Loop 테스트. pytest 기반으로 Gazebo headless 모드에서 시뮬레이션 검증.

| 파일 | 설명 |
|------|------|
| `conftest.py` | pytest fixture: headless Gazebo 프로세스, ROS2 node 생성 |
| `test_straight_drive.py` | 직진 테스트: 속도 응답, 직진 정확도(0.5m), 방향 안정성(5도) |
| `test_turn.py` | 회전 테스트: pivot turn(90/180도), curve turn(r=5m/2m), 속도 포화 |
| `sil_runner.py` | SIL 테스트 러너: Pure Pursuit 추종, RMSE/최대횡오차 메트릭, CSV 출력, --offline 모드 |

---

### src/team_leader/ — 인지-판단-제어 통합 노드

> ROS2 Python 패키지. 자율주행의 핵심 로직을 담당한다.
> perception → planning → control 순서로 매 사이클(20Hz) 실행.
>
> **실행**: `ros2 run team_leader leader_node` 또는 launch 파일에서 자동 실행.

#### 루트 파일

| 파일 | 설명 |
|------|------|
| `setup.py` | Python 패키지 빌드 설정 (entry_points로 leader_node 등록) |
| `setup.cfg` | setuptools 설정 |
| `package.xml` | ROS2 패키지 메타데이터. Nav2/robot_localization 등 13개 의존성 포함 |
| `CMakeLists.txt` | CMake 빌드 설정 |

#### config/

| 파일 | 설명 |
|------|------|
| `params.yaml` | ROS2 파라미터: 토픽 이름, 제어 주기, PID 게인, 최대 속도/조향, 주행 모드 |
| `dual_ekf.yaml` | robot_localization Dual-EKF 설정: ekf_local(50Hz, odom+IMU), ekf_global(10Hz, +GPS), navsat_transform |
| `nav2_params.yaml` | Nav2 파라미터: SmacPlannerLattice(전역), MPPI(지역, 7 critic), costmap(global 0.1m, local 0.05m) |

#### launch/

| 파일 | 설명 |
|------|------|
| `team_leader_launch.py` | 팀장 노드 단독 실행 런처 |
| `navigation_launch.py` | EKF(local+global) + navsat_transform + Nav2 bringup 통합 런처 |

#### team_leader/ (코드)

| 파일 | 설명 |
|------|------|
| `__init__.py` | 패키지 초기화 |
| `leader_node.py` | 메인 노드. 타이머 콜백으로 인지→판단→제어 파이프라인 실행, cmd_vel 발행 |
| `perception.py` | 인지 모듈. LidarProcessor+CameraDetector 통합, PointCloud2→장애물, Image→탐지결과 |
| `lidar_processor.py` | LiDAR 처리: Ray Ground Filter(360 방위각 섹터) + 적응형 Euclidean Clustering → Obstacle 리스트 |
| `camera_detector.py` | 카메라 탐지: YOLO 래퍼 (ultralytics 미설치 시 dummy fallback), Detection dataclass |
| `terrain_classifier.py` | 지형 분류: RGB 색상 기반 7종 분류 (PAVED/DIRT_ROAD/GRAVEL/GRASS/CROP_FIELD/MUD/OBSTACLE), traversability 비용 |
| `planning.py` | 판단 모듈. DrivingMode 5종(+COVERAGE, WAYPOINT_NAV), PathPlanner(Boustrophedon 커버리지), 웨이포인트 네비 |
| `pure_pursuit.py` | Pure Pursuit 경로 추종: 속도비례 lookahead(gain=0.8, 1.0~5.0m), 원-직선 교차점 보간, 골 도달 판정 |
| `skid_steer_model.py` | ICR 스키드 스티어 모델: track_width=1.4m, twist↔tracks 변환, 슬립 보상, 속도 포화, 궤적 예측 |
| `control.py` | 제어 모듈. Pure Pursuit→SkidSteer 파이프라인, 경로 없을 시 PID fallback |

#### test/

| 파일 | 설명 |
|------|------|
| `test_leader.py` | 팀장 노드 기본 테스트 (copyright, flake8, pep257) |

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

> 모든 에이전트가 공유하는 기반 코드.

| 파일 | 설명 |
|------|------|
| `base_agent.py` | `BaseAgent` 추상 클래스. `start()/stop()` 라이프사이클, `handle_task()` 추상 메서드, MessageBus 연결 |
| `task.py` | `Task` dataclass + `TaskStatus`(PENDING→IN_PROGRESS→COMPLETED/FAILED) + `TaskPriority`(LOW/MEDIUM/HIGH/CRITICAL) |
| `message_bus.py` | asyncio 기반 pub/sub 메시지 버스. 채널: task.assign, task.complete, status.report 등 |
| `task_tracker.py` | `task_autonomou.md` 파일 자동 읽기/갱신 모듈 |

### agents/project_leader/ — 팀장 에이전트

> 작업을 분해하고 하위 에이전트에 할당, 진행 추적, 보고서 생성.

| 파일 | 설명 |
|------|------|
| `project_leader_agent.py` | `create_task()` 작업 생성/할당, `check_progress()` 진행 조회, `generate_report()` 현황 보고서, `coordinate()` 계획→세부작업 분해 |

### agents/modeling/ — 모델링 에이전트

> SDF 파일(차량 모델, 월드)을 프로그래밍 방식으로 생성/수정.

| 파일 | 설명 |
|------|------|
| `modeling_agent.py` | 통합 에이전트. vehicle/world/physics modeler 조합하여 작업 수행 |
| `vehicle_modeler.py` | 차량 SDF 조작: `create_vehicle()`, `modify_physics(mass, friction)`, `add_sensor(type, position)` |
| `world_modeler.py` | 월드 SDF 조작: `create_world()`, `add_obstacle()`, `add_road()`, 표면 마찰 설정 |
| `physics_modeler.py` | 물리 엔진 설정: `set_physics_engine(step_size)`, `set_surface_friction()`, `set_gravity()` |

### agents/research/ — 선행조사 에이전트

> 웹 검색, 논문 조사, 조사 결과 문서화를 자동화.

| 파일 | 설명 |
|------|------|
| `research_agent.py` | 통합 에이전트. 검색 + 문서화 워크플로우 자동 실행 |
| `web_searcher.py` | `search(query)` 웹 검색, `search_papers(query)` 학술 논문 검색, `fetch_page(url)` 페이지 수집 |
| `document_writer.py` | `write_report(title, sections)` 마크다운 보고서 생성, `save_to_project()` docs/ 저장 |

### agents/docs/ — 에이전트 문서

| 파일 | 설명 |
|------|------|
| `README.md` | 에이전트 시스템 개요, 설치, 사용법 |
| `checklist.md` | 에이전트 기능 체크리스트 |

---

## docs/ — 기술 문서

> 프로젝트 정의서, 선행연구 보고서를 보관.
> 선행연구 보고서는 research 에이전트가 생성하며, 팀장이 검증.

| 파일 | 설명 |
|------|------|
| `application_definition.md` | Application 정의서. RS500 시스템 개요, 적용 분야(농업→군사), HW/SW 아키텍처, 센서 구성, 운용 시나리오 |

### docs/research/ — 선행연구 보고서

| 파일 | 설명 |
|------|------|
| `../literature_review_dynamics_control.md` | 동역학/제어: ICR 키네마틱, 슬립(Bekker-Wong), 조향(Diff Steering), 경로 추종(Pure Pursuit→MPC→DRL). 논문 27편 |
| `path_planning_and_slam_literature_review.md` | 경로계획/SLAM: A*→State Lattice, DWA→MPPI, Fields2Cover, LIO-SAM, Dual-EKF, Nav2 통합. 논문 40+편 |
| `perception_sensor_fusion_survey.md` | 인지/센서융합: Ray Ground Filter, YOLO11n, Late Fusion, 작물 행 인식, Traversability, Autoware. 논문 30+편 |
| `gazebo_tracked_vehicle_simulation_research.md` | 시뮬레이션: TrackedVehicle 플러그인, heightmap/DEM, 센서 노이즈, ros_gz_bridge, SIL/HIL, 오픈소스 8개. 15개 개선항목 |

---

## 월드 매핑 테이블

> `simulation_launch.py`의 `_WORLD_MAP` 딕셔너리와 일치해야 한다.

| world 인자 | SDF 경로 | 크기 | 특징 |
|------------|---------|------|------|
| `agricultural_field` | `worlds/agricultural_field.sdf` | 200m x 200m | 절차적 heightmap, 고랑, 농로, 창고 |
| `yeongwol_sinil` | `worlds/real_terrain/yeongwol_sinil.sdf` | 500m x 500m | 실제 DEM (289-402m), OSM 도로/수계 |
| `cheongsong_orchard` | `worlds/real_terrain/cheongsong_orchard.sdf` | 300m x 300m | 실제 DEM (198-276m), 사과나무 2,400 |

---

## ROS2 토픽 맵

> `bridge_config.yaml` 기준. 센서 → ROS2 토픽 매핑.

| 토픽 | 메시지 타입 | 방향 | 용도 |
|------|-----------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | ROS→GZ | 차량 속도 명령 (linear.x, angular.z) |
| `/odom` | `nav_msgs/Odometry` | GZ→ROS | 차량 오도메트리 |
| `/sensor/gps` | `sensor_msgs/NavSatFix` | GZ→ROS | GPS 위치 (WGS84) |
| `/sensor/imu` | `sensor_msgs/Imu` | GZ→ROS | IMU (가속도, 각속도, 방향) |
| `/sensor/camera/front` | `sensor_msgs/Image` | GZ→ROS | 전방 카메라 (640x480 RGB) |
| `/sensor/camera/front/camera_info` | `sensor_msgs/CameraInfo` | GZ→ROS | 카메라 내부 파라미터 |
| `/sensor/lidar` | `sensor_msgs/LaserScan` | GZ→ROS | 360도 16채널 LiDAR |
| `/sensor/lidar/points` | `sensor_msgs/PointCloud2` | GZ→ROS | LiDAR 포인트클라우드 |
| `/clock` | `rosgraph_msgs/Clock` | GZ→ROS | 시뮬레이션 시간 동기화 |
| `/tf` | `tf2_msgs/TFMessage` | GZ→ROS | 좌표 변환 |

---

## 의존성 관계

```
ad_interfaces  ←── team_leader (메시지 타입 참조)
                ←── ad_simulation (launch에서 team_leader 실행)

ad_simulation  ←── team_leader (센서 데이터 제공)
               ←── agents/modeling (SDF 파일 수정 대상)

team_leader    ──→ ad_interfaces (메시지 발행/구독)
               ──→ ad_simulation (시뮬레이션 위에서 동작)

agents/        ──→ src/ad_simulation/models/ (vehicle_modeler가 SDF 수정)
               ──→ src/ad_simulation/worlds/ (world_modeler가 SDF 수정)
               ──→ src/ad_simulation/config/ (센서 추가 시 bridge 설정 갱신)
               ──→ docs/ (research 에이전트가 보고서 생성)
               ──→ task_autonomou.md (task_tracker가 자동 갱신)
```

---

## 변경 이력

| 날짜 | 변경 내용 |
|------|----------|
| 2026-02-21 | 초판 작성. 전체 프로젝트 구조 문서화 |
| 2026-02-21 | 제어(pure_pursuit, skid_steer_model), 판단(dual_ekf, nav2_params, navigation_launch, planning 확장), 인지(lidar_processor, camera_detector, terrain_classifier), 시뮬레이션(다중마찰, SIL 테스트) 추가 |
