# C70: Foxglove 레이아웃 v2.0 전면 개선

- 분야: 인프라
- 담당: 그린
- 기간: 2026-03-17
- 상태: 완료

## 배경

기존 Foxglove 레이아웃이 C64 Camera-Only 전환 이전 버전 기준으로 작성되어 있었다.
LiDAR 토픽 잔존, 카메라 1대만 표시, depth PointCloud 미시각화 등 센서 스택과 레이아웃 간 괴리가 있었고,
자율주행 업계 베스트 프랙티스 조사 결과 다음 개선 포인트를 식별했다:

- 용도별 레이아웃 분리 (Production / Dev / Debug) 부재
- 파생 메트릭 (CTE, Heading Error, Velocity Tracking Error) 미제공
- 탐지 결과 이미지 오버레이 없음
- CameraInfo 브릿지 누락으로 Foxglove 3D→이미지 투영 불가
- 녹화 토픽에 depth PointCloud / CameraInfo 누락

## 결정 사항

| 결정 | 이유 | 검토한 대안 |
|------|------|-------------|
| 4-레이아웃 체계 (Production/Dev/Debug/Comparison) | 현장 감독자·개발자·디버거의 정보 요구가 다름. 대역폭·안전성 차별화 | 단일 레이아웃에 모든 패널 → 패널 과다로 성능 저하 |
| Production에서 Teleop/CallService 제거 | 현장에서 Foxglove 원격 조작 방지 (안전) | 권한 제어로 해결 → Foxglove에 권한 시스템 없음 |
| foxglove_msgs ImageAnnotations으로 탐지 오버레이 | Foxglove Image 패널 네이티브 지원. 바운딩 박스+라벨 직접 렌더링 | visualization_msgs/MarkerArray → 3D 마커를 CameraInfo로 투영하는 간접 방식. 설정 복잡 |
| User Script로 CTE/VelError/Heading 계산 | ROS 노드 추가 없이 Foxglove 내에서 파생 메트릭 실시간 계산 가능 | ROS 노드로 퍼블리시 → 시스템 복잡도 증가, 모든 환경에 배포 필요 |
| foxglove_msgs try/except 패턴 | 선택적 의존성으로 foxglove_msgs 미설치 환경에서도 인지 노드 정상 동작 | 별도 노드 분리 → 관리 포인트 증가 |

## 레이아웃 체계

### Production (`foxglove_production_layout.json`)

| 탭 | 패널 | 대상 |
|----|------|------|
| **Monitor** | 전면 카메라(+어노테이션) · GPS 지도 · 속도 게이지 · 상태 Indicator · 웨이포인트 상태 · WARN+ 로그 | 현장 감독자 |
| **System** | 속도 추이(vx, wz) · EKF 위치 · 네비게이션 로그 | 현장 감독자 |

- 구독 토픽 최소 (PointCloud 없음, costmap 없음)
- 조작 패널 없음 (Teleop, CallService 제거)
- **패널 10개**

### Dev (`foxglove_layout.json`)

| 탭 | 패널 | 용도 |
|----|------|------|
| **Perception** | 카메라 3대 · 3D depth PointCloud 3개 · 속도 게이지 · 인지 로그 | 센서 입력 확인 |
| **Navigation** | 3D costmap/plan/URDF · Teleop · Waypoint 서비스(4개) · Indicator · 상태 · 로그 | 경로 계획 + 수동 제어 |
| **Control** | cmd vx (3계열) · cmd wz (3계열) · CTE · EKF 위치 · IMU 가속도 | 제어 추종 성능 |
| **GPS & Map** | GPS 지도 · EKF 위치 · Raw odom · Raw GPS | 위치 추정 |
| **Diagnostics** | TF 트리 · 토픽 그래프 · 로그 · 파라미터 | 시스템 건강 |

- User Script 3개 내장 (CTE, VelocityTrackingError, HeadingFromPlan)
- **패널 30개**

### Debug (`foxglove_debug_layout.json`)

| 탭 | 패널 | 용도 |
|----|------|------|
| **Raw Sensors** | 카메라 3대 · Raw IMU/GPS/Odom · IMU 가속도 · IMU 자이로 | 원시 센서 전수 점검 |
| **Depth & TF** | 3D depth 근접뷰(enableStats) · IMU Orientation(quat) · TF 트리 · Raw EKF local/global | 좌표 변환 문제 추적 |
| **Nav2 Internal** | 3D costmap+plan(enableStats) · EKF local vs global · GPS 좌표 · Raw cmd_vel · 웨이포인트 · Nav 로그 | Nav2 내부 상태 |
| **Topic & Param** | 토픽 그래프 · 파라미터 전체 · 전체 로그(DEBUG 이상) | 시스템 구조 분석 |
| **Velocity Chain** | cmd_vel→smoothed→odom→EKF 4단계 vx/wz · Raw 메시지 3개 | 속도 명령 전달 경로 추적 |

- 3D 패널 `enableStats: true` (FPS/렌더링 시간 표시)
- **패널 26개**

### Comparison (`foxglove_comparison_layout.json`)

오프라인 bag 기반 sim/real 비교. 기존 대비 Camera 비교 탭 + depth PointCloud 비교 추가.

## 변경 내역

### 신규 파일
- `config/foxglove_production_layout.json` — Production 2탭 레이아웃
- `config/foxglove_debug_layout.json` — Debug 5탭 레이아웃

### 수정 파일
- `config/foxglove_layout.json` — 3탭→5탭 전면 재설계 + User Script 3개 + [DEV] 명시
- `config/foxglove_comparison_layout.json` — LiDAR→camera points, Camera 비교 탭 추가
- `config/bridge_config.yaml` — CameraInfo ×3 브릿지 추가 (Foxglove 이미지 투영용)
- `launch/record_launch.py` — depth points ×3, camera_info ×3 녹화 토픽 추가 (6개)
- `ad_perception/perception_node.py` — Foxglove ImageAnnotations 발행 (바운딩 박스 + 라벨)
- `ad_perception/package.xml` — foxglove_msgs exec_depend 추가

## User Scripts 상세

### CrossTrackError (`/studio_script/cte`)
- 입력: `/odometry/local`, `/plan`
- 로직: `/plan` 경로의 모든 점과 차량 위치 간 최소 거리 계산
- 용도: Control 탭 CTE Plot에서 실시간 경로 추종 정확도 표시

### VelocityTrackingError (`/studio_script/vel_error`)
- 입력: `/cmd_vel`, `/odometry/local`
- 로직: 명령 속도와 실제 속도의 절대 차이 (vx, wz 각각)
- 용도: 제어 명령이 실제로 잘 추종되고 있는지 모니터링

### HeadingFromPlan (`/studio_script/heading`)
- 입력: `/odometry/local`, `/plan`
- 로직: quaternion→yaw 변환 후 경로 접선 방향과의 차이 (degrees)
- 용도: 차량이 경로를 따라 올바른 방향으로 가고 있는지 확인

## 검증

- JSON 유효성: 4개 레이아웃 전부 `json.load()` 성공
- 테스트: `ad_perception` 19 tests 전부 통과 (foxglove_msgs import는 try/except)
- 빌드 적용: `setup.py`의 `glob('config/*.json')` 패턴이 4개 파일 모두 포함

## 미결 이슈

- [ ] `foxglove_msgs` 패키지 설치 필요: `sudo apt install ros-jazzy-foxglove-msgs`
- [ ] Phase 4: C57 네임스페이스 분리 완료 후 comparison 레이아웃에 `/sim/*`, `/real/*` 자동 적용
- [ ] Gazebo에서 CameraInfo 토픽이 실제로 발행되는지 시뮬 실행 시 확인 필요
- [ ] User Script는 Foxglove Studio에서 레이아웃 Import 후 내부에서 활성화해야 동작
