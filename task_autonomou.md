# 프로젝트 작업 관리

> 최종 업데이트: 2026-02-21

---

## 할 일 (TODO)

### 즉시 조치 (선행연구 도출)
- [ ] model.sdf에 TrackController 플러그인 추가 (현재 누락)
- [ ] bridge_config.yaml 보완: `/clock`, `/tf`, `camera_info` 토픽 추가
- [ ] bridge_config.yaml: LiDAR 토픽을 `LaserScan` → `PointCloud2`로 변경
- [ ] 센서 노이즈 모델 추가: LiDAR 노이즈, IMU bias, 카메라 왜곡
- [ ] launch 파일에 `use_sim_time:=true` 전역 설정 추가

### 인지 (Perception)
- [ ] LiDAR 장애물 감지: Ray Ground Filter + Euclidean Clustering + PointPillars
- [ ] 카메라 객체 탐지: YOLO11n 적용 (640x480, ~2.9ms)
- [ ] 시맨틱 세그멘테이션: ENet 또는 BiSeNetV2 적용
- [ ] Camera + LiDAR Late Fusion 구현
- [ ] 농경지 작물 행 인식 (시맨틱 세그멘테이션 기반)
- [ ] 지형 traversability 분류 (Wild Visual Navigation 방식)

### 판단/경로 계획 (Planning)
- [ ] Nav2 SmacPlannerLattice 전역 경로 계획 연동
- [ ] MPPI 지역 경로 계획 적용 (슬립 반영, Nav2 공식 지원)
- [ ] Fields2Cover 농경지 커버리지 경로 계획 통합
- [ ] robot_localization Dual-EKF 센서 퓨전 (GPS+IMU+Odom)
- [ ] LIO-SAM SLAM 통합 (GPS 가용/불가 자동 전환)

### 제어 (Control)
- [ ] Pure Pursuit 경로 추종 구현 (Phase 1)
- [ ] 슬립 보상 적응형 Pure Pursuit (Phase 2)
- [ ] ICR 기반 스키드 스티어 조향 모델 구현
- [ ] 유효 궤도 폭 보정 + 모터 비대칭 보상

### 시뮬레이션 고도화
- [ ] heightmap 기반 기복 지형 월드 생성
- [ ] 다중 마찰 영역 설정 (포장/비포장/진흙)
- [ ] SIL 테스트 프레임워크 구축 (launch_testing)
- [ ] CI/CD headless 시뮬레이션 파이프라인

### 기타
- [ ] aiohttp 설치하여 research 에이전트 웹 검색 기능 활성화
- [ ] 에이전트 시스템 단위 테스트 작성
- [ ] rich 라이브러리 적용하여 CLI 출력 개선

---

## 진행 중 (IN PROGRESS)

(현재 진행 중인 작업 없음)

---

## 완료 (DONE)

- [x] 선행연구: 궤도차량 동역학/제어 (docs/literature_review_dynamics_control.md)
- [x] 선행연구: 경로계획/SLAM (docs/research/path_planning_and_slam_literature_review.md)
- [x] 선행연구: 인지/센서융합 (docs/research/perception_sensor_fusion_survey.md)
- [x] 선행연구: Gazebo 시뮬레이션 사례 (docs/research/gazebo_tracked_vehicle_simulation_research.md)
- [x] Application 정의서 작성 (docs/application_definition.md)
- [x] 에이전트 시스템 매뉴얼/체크리스트 작성 (agents/docs/)
- [x] task.md 자동 업데이트 기능 구현 (task_tracker.py)
- [x] AI 에이전트 시스템 구현 - core/modeling/research/project_leader (7d84ca0)
  - [x] core 프레임워크 (BaseAgent, Task, MessageBus)
  - [x] modeling 에이전트 (vehicle/world/physics modeler)
  - [x] research 에이전트 (web_searcher, document_writer)
  - [x] project_leader 에이전트 (작업 분배, 진행 관리)
  - [x] CLI 진입점 (python -m agents)
- [x] Gazebo gz-sim 시뮬레이션 연동 - ad_simulation 패키지 (deecbfb)
- [x] ROS2 프로젝트 초기 구조 생성 (6ab5521)

---

## 참고

- 작업 상태: `[ ]` 미완료, `[x]` 완료
- 이 파일은 에이전트 시스템이 자동으로 관리합니다
- 선행연구 기반 TODO는 카테고리별로 분류됨
