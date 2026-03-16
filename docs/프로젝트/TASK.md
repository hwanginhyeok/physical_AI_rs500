# TASK 관리

> 마지막 갱신: 2026-03-16 (C67 Visual SLAM 조사 완료, C68 CAN 버그 수정, C66 완료)
>
> **관리 룰**
> - 상태: `예정` → `진행` → `완료` (완료 즉시 완료 섹션 최상단으로 이동)
> - 중요도: P1(긴급) > P2(중요) > P3(향후)
> - 완료된 작업은 작업 현황에서 **즉시 삭제**하고 완료 섹션 최상단에 추가
> - 지연 강조: 3일 이상 `예정` 상태 미진행 시 `**[지연]**` 표시
> - TASK 상세 로그: 설계 결정·다중 파일 변경·미결 이슈 있을 때 `docs/프로젝트/task/{ID}.md` 작성

---

## 🔥 현재 진행 중 (세션 복귀 시 여기부터)

| # | 작업 | 담당 | 진행 상황 | 다음 할 일 |
|---|------|------|-----------|-----------|
| C41 | 집 PC Gazebo 시뮬레이션 환경 구축 | 사용자+그린 | Step 1~4 완료 (WSL2+ROS2+Gazebo+Nav2) | **Step 5~8**: git clone → colcon build → Foxglove 포트포워딩 → 시뮬레이션 E2E 테스트 ([상세](task/C44_wsl2_simulation_setup.md)) |
| C61 | 차량 물리 동작 검증 | 사용자 | 코드 완료, 집 PC Gazebo 실행 대기 | C41 완료 후 → `ros2 launch ad_bringup simulation_launch.py` → cmd_vel → 차량 이동 확인 ([상세](task/C61_velocity_chain_debug.md)) |
| C67 | Camera-Only Visual SLAM 구현 | 그린 | 조사 완료 (RTAB-Map 1순위) | C41 완료 후 → rtabmap_ros Jazzy 빌드 → Gazebo RGBD 연동 → GPS 전환 아키텍처 ([상세](../research/visual_slam_camera_only_survey.md)) |
| C60 | 농업용 Hybrid E2E 아키텍처 구축 | 그린 | Phase 1~2 완료 (구조+연결). Diffusion+PurePursuit fallback 연결됨 | Phase 3: Diffusion 모델 훈련 (Gazebo 데이터 수집 → 학습 파이프라인) |

---

## 🔍 사용자 코드 검토 필요

> 이번 세션에서 그린이 변경한 코드를 사용자가 아직 확인하지 못한 항목

| 파일 | 변경 내용 | 확인 포인트 |
|------|-----------|-------------|
| `src/ad_bringup/models/ss500/model.sdf` | **C64**: LiDAR 제거, 카메라 3대 추가 (front/left/right), `camera` → `rgbd_camera` 전환, `depth_camera` 설정 추가 | Gazebo에서 RGB + PointCloud2 정상 발행 확인 |
| `src/ad_bringup/config/bridge_config.yaml` | **C64**: LiDAR 브릿지 제거, 카메라 3대 RGB + PointCloud2 브릿지 추가 (6개 토픽) | ros2 topic list에서 /sensor/camera/*/points 확인 |
| `src/ad_bringup/config/nav2_params.yaml` | **C64**: obstacle_layer에 PointCloud2 ×3 소스 연결 (global+local), collision_monitor에 pointcloud 소스 추가 | costmap에 장애물이 정상 표시되는지 |
| `src/ad_bringup/urdf/ss500.urdf.xacro` | **C64**: LiDAR 링크/조인트 제거, 카메라 3대 링크/조인트 추가 | TF 트리에 camera_front/left/right_link 확인 |
| `src/ad_bringup/launch/simulation_launch.py` | **C64**: LiDAR frame bridge 제거, 카메라 3대 frame bridge 추가 | 프레임 ID 브릿지 정상 동작 |
| `src/ad_bringup/launch/record_launch.py` | **C64**: 토픽명 `/sensor/camera/*/image`로 변경 | — |
| `src/ad_perception/ad_perception/perception_node.py` | **C64**: LiDAR 구독/처리 전면 제거, 멀티카메라(front/left/right) 구독 전환 | 카메라 3대 이미지 수신 확인 |
| `src/ad_perception/config/perception_params.yaml` | **C64**: 토픽 파라미터 멀티카메라로 갱신 | — |

---

## ⏸️ 사용자 액션 대기 (코드 완료, 사용자 실행만 남음)

| # | 작업 | 남은 사용자 액션 |
|---|------|-----------------|
| C61 | 차량 물리 동작 검증 | 시뮬레이션 실행 검증: ① `ros2 launch ad_bringup simulation_launch.py` ② `ros2 lifecycle get /velocity_smoother` → active 확인 ③ `ros2 topic pub /cmd_vel ... --once` → Gazebo 차량 이동 확인 ④ Nav2 end-to-end 웨이포인트 검증 ([상세](task/C61_velocity_chain_debug.md)) |
| C44 | 집 PC WSL2 시뮬레이션 준비 | 체크리스트 Step 2~8 순차 진행 ([상세](task/C44_wsl2_simulation_setup.md)) |

---

## 작업 현황

| # | 분야 | 작업 | 중요도 | 담당 | 상태 | 비고 |
|---|------|------|--------|------|------|------|
| | | **── P1 긴급 ──** | | | | |
| C63 | 인프라 | STP 도면 반영 → model.sdf 물리 파라미터 + 메시 갱신 | P1 | 그린 | 예정 | 실차 하드웨어 출고 완료. STP 파일 ~3/20 수령 예정. `scripts/stp_to_sdf.py` 준비 완료 |
| C41 | 인프라 | 집 PC Gazebo 시뮬레이션 환경 구축 | P1 | 사용자+그린 | **진행** | Step 1~4 완료 (WSL2+ROS2+Gazebo+Nav2). Step 5~8 남음 (클론+빌드+Foxglove+E2E) |
| | | **── P2 중요 ──** | | | | |
| C61 | 시뮬레이션 | 차량 물리 동작 검증 (cmd_vel → Gazebo) | P2 | 사용자 | **사용자 대기** | CycloneDDS 전환 + cmd_vel_relay 수정 완료. 집 PC에서 시뮬레이션 실행 검증 대기 ([상세](task/C61_velocity_chain_debug.md)) |
| C60 | 아키텍처 | 농업용 Hybrid E2E 아키텍처 구축 | P2 | 그린 | **진행** | ARCH-004 설계, Safety Guardian 완료. C64 Camera-Only 반영 후 Learned Perception/Planning 통합 ([상세](task/C60_hybrid_e2e_architecture.md)) |
| C57 | 인프라 | 시뮬/실물 네임스페이스 분리 (라이브 동시 비교) | P2 | — | 예정 | `/sim/*` / `/real/*` 분리 |
| C67 | 인지 | Camera-Only Visual SLAM 조사 완료 — 구현 예정 | P2 | 그린 | **조사 완료** | RTAB-Map 1순위 권장. 912줄 조사 보고서 작성. 구현은 C41(시뮬 환경) 후 착수 ([상세](../research/visual_slam_camera_only_survey.md)) |
| | | **── P3 향후 ──** | | | | |
| C45 | 인지 | 농경지 작물 행 인식 (시맨틱 세그멘테이션 기반) | P3 | — | 예정 | |
| C46 | 인지 | 지형 traversability 분류 (Wild Visual Navigation 방식) | P3 | — | 예정 | |
| C47 | 인프라 | CI/CD headless 시뮬레이션 파이프라인 | P3 | — | 예정 | |
| C49 | 인프라 | CLAUDE.md 매뉴얼 트리거 기반 전환 | P3 | 그린 | 예정 | `.claude/rules/` 파일이 5개 이상으로 늘어날 때 착수 |
| C69 | 아키텍처 | 미션 관리 플러그인 설계 (농업/군사/탐사) | P3 | — | 예정 | Application Layer 교체 구조. Phase 1 안정화 후 착수 |

---

## 완료

| # | 작업 | 중요도 | 담당 | 완료일 | 상세 |
|---|------|--------|------|--------|------|
| C68 | CAN 브릿지 Twist→트랙 변환 수정 | P2 | 그린 | 2026-03-16 | angular.z 미반영 버그 수정. SkidSteerModel 연결. 실차 조향 가능하도록 |
| C66 | 실물 Mono Depth 노드 | P2 | 그린 | 2026-03-16 | mono_depth_node.py: MiDaS/DepthAnything → PointCloud2. 실차 테스트 대기 |
| C48 | ad_perception / ad_control Mock 노드 테스트 | P2 | 그린 | 2026-03-16 | 25 tests (hybrid_e2e_logic 11 + perception_logic 14). 전체 211/211 통과 |
| C65 | C64 반영 — 상위 문서 갱신 + TASK 정합성 정리 | P1 | 그린 | 2026-03-16 | App Definition, SYSTEM_OVERVIEW, TASK.md C64 Camera-Only 반영. 누락 TASK 5건(C66~C69) 등록 |
| C64 | 센서 스펙 확정 반영 — Camera-Only 전환 | P1 | 그린 | 2026-03-13 | LiDAR 전면 제거, 카메라 3대 rgbd_camera 전환, Nav2 costmap PointCloud2 연결, perception_node 멀티카메라 재구성, ARCH-005 작성. 10개 파일 수정, 테스트 29/29 통과 ([상세](task/C64_sensor_spec_camera_only.md)) |
| C62 | STP→SDF 변환 파이프라인 구축 | P3 | 그린 | 2026-03-13 | `.venv-tools/` + cadquery/trimesh 설치, `scripts/stp_to_sdf.py` 작성. STP 파일 수령 시 즉시 실행 가능 |
| C52 | Gazebo 시뮬레이션 통합 검증 (end-to-end) | P1 | 그린 | 2026-03-04 | bt_navigator 타임아웃 수정, voxel_layer 높이/z_voxels 수정, BT XML 경로 설정. Goal accepted → controller_server 응답 → cmd_vel_nav 발행 확인 ([상세](task/C52_verification_report.md)) |
| C58 | bt_navigator `follow_path` 타임아웃 해결 | P1 | 그린 | 2026-03-04 | 타임아웃 20ms→1000ms. Goal accepted 확인. C52의 하위 작업 ([상세](task/C58_bt_navigator_timeout_fix.md)) |
| C56 | Foxglove rosbag 시뮬/실물 비교 체계 구축 | P2 | 그린 | 2026-03-01 | record_launch.py (zstd 압축, robot=sim/real 인자), foxglove_comparison_layout.json (4탭: Navigation/속도/GPS/Diagnostics) ([상세](task/C56_rosbag_comparison.md)) |
| C55 | Gazebo scoped sensor frame_id → URDF TF 브릿지 추가 | P1 | 그린 | 2026-03-01 | [상세](task/C50-C55_gazebo_tf_gps_fix.md) |
| C54 | EKF ↔ robot_state_publisher TF 부모 충돌 수정 | P1 | 그린 | 2026-03-01 | [상세](task/C50-C55_gazebo_tf_gps_fix.md) |
| C53 | model.sdf 센서 noise SDF 포맷 전면 수정 | P1 | 그린 | 2026-03-01 | [상세](task/C50-C55_gazebo_tf_gps_fix.md) |
| C51 | navsat_transform GPS datum 하드코딩 고정 | P1 | 그린 | 2026-03-01 | [상세](task/C50-C55_gazebo_tf_gps_fix.md) |
| C50 | AMCL 제거 → static_transform_publisher로 map→odom 고정 | P1 | 그린 | 2026-03-01 | [상세](task/C50-C55_gazebo_tf_gps_fix.md) |
| C43 | Git 브랜치 전략 수립 (main/sim/deploy 3분기) | P2 | 그린 | 2026-02-28 | master→main rename, sim·deploy 브랜치 생성 |
| C42 | Foxglove+Gazebo+Nav2 다중 웨이포인트 네비게이션 | P1 | 그린 | 2026-02-28 | [상세](task/C42_waypoint_navigation.md) |
| C40 | 모듈별 시뮬레이션 테스트 리포트 체계 구축 | P2 | 그린 | 2026-02-24 | 4모듈 리포트 생성기 + 스킬 문서 |
| C39 | 논/밭 시나리오 배치 시뮬레이션 + 지면 피드백 수정 | P2 | 그린 | 2026-02-24 | 지형별 CTE 차이 확인 |
| C38 | Level 2 물리 시뮬레이션 업그레이드 | P1 | 그린 | 2026-02-24 | 마찰 모델, 슬립 보상, IMU 노이즈 교정 포함 |
| C37 | ad_bringup 통합 런처 | P2 | 그린 | 2026-02-23 | — |
| C36 | SS500 CAN 코덱 + CAN 브릿지 | P1 | 그린 | 2026-02-23 | — |
| C35 | ROS2 노드 패키지 분리 | P1 | 그린 | 2026-02-23 | — |
| C34 | ad_core 순수 알고리즘 분리 | P1 | 그린 | 2026-02-23 | — |
| C33 | 단위 테스트 추가 (195개) | P2 | 그린 | 2026-02-22 | — |
| C32 | WSL2 ROS2 Jazzy 환경 구축 | P1 | 그린 | 2026-02-22 | — |
| C31 | Camera + LiDAR Late Fusion | P2 | 그린 | 2026-02-21 | — |
| C30 | 시맨틱 세그멘테이션 | P2 | 그린 | 2026-02-21 | — |
| C29 | LIO-SAM SLAM 통합 | P2 | 그린 | 2026-02-21 | — |
| C28 | Fields2Cover 커버리지 경로 계획 | P2 | 그린 | 2026-02-21 | — |
| C27 | 유효 궤도 폭 보정 + 모터 비대칭 보상 | P2 | 그린 | 2026-02-21 | — |
| C26 | 슬립 보상 적응형 Pure Pursuit | P1 | 그린 | 2026-02-21 | — |
| C25 | SIL 테스트 프레임워크 | P3 | 그린 | 2026-02-21 | C34에서 재구조화 |
| C24 | 다중 마찰 영역 | P3 | 그린 | 2026-02-21 | C34에서 재구조화 |
| C23 | YOLO 카메라 객체 탐지 | P2 | 그린 | 2026-02-21 | — |
| C22 | LiDAR 장애물 감지 | P2 | 그린 | 2026-02-21 | — |
| C21 | MPPI 지역 경로 계획 | P2 | 그린 | 2026-02-21 | — |
| C20 | Nav2 SmacPlannerLattice | P2 | 그린 | 2026-02-21 | — |
| C19 | Dual-EKF 센서 퓨전 | P1 | 그린 | 2026-02-21 | — |
| C18 | ICR 기반 스키드 스티어 모델 | P1 | 그린 | 2026-02-21 | — |
| C17 | Pure Pursuit 경로 추종 | P1 | 그린 | 2026-02-21 | — |
| C16 | ROS2 프로젝트 초기 구조 | P1 | 그린 | 2026-02-21 | C34~37에서 재구조화 |
| C15 | Gazebo gz-sim 시뮬레이션 연동 | P3 | 그린 | 2026-02-21 | C34에서 재구조화 |
| C14 | AI 에이전트 시스템 | P3 | 그린 | 2026-02-21 | — |
| C13 | task.md 자동 업데이트 | P3 | 그린 | 2026-02-21 | task_tracker.py → 역할 종료 (2026-03-01) |
| C12 | 에이전트 시스템 매뉴얼 | P3 | 그린 | 2026-02-21 | — |
| C11 | Application 정의서 | P2 | 그린 | 2026-02-21 | — |
| C10 | 선행연구: Gazebo 시뮬레이션 | P3 | 그린 | 2026-02-21 | — |
| C09 | 선행연구: 인지/융합 | P3 | 그린 | 2026-02-21 | — |
| C08 | 선행연구: 경로계획/SLAM | P3 | 그린 | 2026-02-21 | — |
| C07 | 선행연구: 동역학/제어 | P3 | 그린 | 2026-02-21 | — |
| C06 | 시뮬레이션 버그 수정 | P3 | 그린 | 2026-02-21 | C34에서 재구조화 |
| C05 | 농업 하이트맵 생성 | P3 | 그린 | 2026-02-21 | C34에서 재구조화 |
| C04 | 실제 지형 파이프라인 | P3 | 그린 | 2026-02-21 | C34에서 재구조화 |
| C03 | 런치 월드 선택 | P3 | 그린 | 2026-02-21 | C34에서 재구조화 |
| C02 | 영월 신일 월드 | P3 | 그린 | 2026-02-21 | C34에서 재구조화 |
| C01 | 청송 과수원 월드 | P3 | 그린 | 2026-02-21 | C34에서 재구조화 |

---

## 확정 사항 (2026-03-13)

- **실차 하드웨어**: 출고 완료. STP 도면 ~3/20 수령 예정
- **센서 스펙**: RGBD 카메라 ×3 (front/left/right) + IMU + GPS (LiDAR 미탑재 — **확정**)
- **운용 시나리오**: 과수원 행간 주행, 논 방제, 밭 커버리지 — 3개 시나리오 전부 대응
- **집 PC 시뮬레이션**: 주말(3/15~16) 사용자+그린 공동 세팅

---

## TODO

- [ ] GitHub 기본 브랜치 master→main 변경 (Settings → Default branch) + 원격 master 삭제
- [ ] STP 도면 수령 시 `scripts/stp_to_sdf.py` 실행 → model.sdf 물리 파라미터 + 메시 갱신 (→ C63)
- [ ] 실차 데이터 확보 시 System Identification으로 물리 파라미터 튜닝
- [ ] YOLO 모델 가중치 파일 확보 및 추론 파이프라인 검증
- [ ] Drivetrain 정상상태 오차 17.65% FAIL 원인 검토 — 효율이 delta에 반복 적용되어 목표 초과 수렴. 모델 수정 vs 기록만 할지 결정 필요
- [x] ~~실물 Mono Depth 노드 개발~~ → C66 TASK로 승격
- [x] ~~LIO-SAM 실제 설치 및 실행 테스트~~ → C64: LiDAR 미탑재 확정으로 불필요. C67(Camera SLAM)으로 대체
