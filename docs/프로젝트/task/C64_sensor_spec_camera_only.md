# C64: 센서 스펙 확정 반영 — Camera-Only 전환

- 분야: 센서/인프라
- 담당: 그린
- 기간: 2026-03-13 ~ (진행 중)
- 상태: 완료 (Stage 1~5 전체)

## 배경

실차 SS500 궤도차량의 센서 스펙이 확정됨:
- **카메라 3대** (전면 0°, 좌전방 ~60°, 우전방 ~-60°)
- **IMU** (100Hz)
- **GPS** (10Hz)
- **LiDAR 미탑재**

기존 코드베이스는 LiDAR LaserScan 기반으로 Nav2 costmap, collision_monitor, 인지 스택이 설계되어 있어 전면 재설계가 필요함.

## 결정 사항

| 결정 | 이유 | 검토한 대안 |
|------|------|-------------|
| LiDAR 전면 제거 (SDF/URDF/bridge/Nav2/recording) | 센서 미탑재 확정 — 죽은 코드 제거 | LiDAR 설정 남겨두고 비활성화 → 혼란 유발, 불필요 |
| 카메라 3대를 `rgbd_camera` 센서 타입으로 구성 | Gazebo가 RGB + Depth + PointCloud2를 직접 출력하므로 Mono Depth 신경망 불필요 (시뮬레이션) | `camera` + depth_camera 별도 센서 → 토픽 관리 복잡, 동기화 필요 |
| PointCloud2를 Nav2 costmap `obstacle_layer`에 직접 연결 | Nav2의 ObstacleLayer가 PointCloud2를 네이티브 지원. voxel_layer 불필요 | voxel_layer 사용 → rgbd_camera의 depth 범위(10m)에서는 2D obstacle_layer로 충분 |
| 실물에서는 MiDaS/DepthAnything으로 Mono Depth → PointCloud2 변환 예정 | rgbd_camera는 시뮬레이션 전용. 실물 카메라는 RGB만 출력 | 실물에 RealSense 추가 → 비용, 실차 스펙 변경 불가 |

## Stage별 진행 현황

### Stage 1 — LiDAR 제거 + 카메라 3대 추가 (완료)

**변경 파일 7개:**
- `models/ss500/model.sdf` — LiDAR link/joint/sensor 삭제, camera_front/left/right_link 추가
- `urdf/ss500.urdf.xacro` — lidar_link/joint 삭제, camera 3대 link/joint 추가
- `config/bridge_config.yaml` — LiDAR bridge 삭제, 카메라 3대 Image bridge 추가
- `launch/simulation_launch.py` — lidar_frame_bridge 삭제, 카메라 3대 frame_bridge 추가
- `config/nav2_params.yaml` — collision_monitor/costmap에서 LiDAR 소스 제거 (빈 소스로 전환)
- `launch/record_launch.py` — 녹화 토픽에서 LiDAR 제거, 카메라 토픽 추가

### Stage 2 — rgbd_camera + Nav2 costmap PointCloud2 연결 (완료)

**변경 파일 4개:**
- `models/ss500/model.sdf` — 3개 카메라 `type="camera"` → `type="rgbd_camera"`, `<depth_camera>` 설정 추가 (depth 범위 10m)
- `config/bridge_config.yaml` — RGB 토픽 `/image` 서픽스 추가, PointCloud2(`/points`) 브릿지 6개 추가
- `config/nav2_params.yaml` — global/local costmap `obstacle_layer`에 카메라 3대 PointCloud2 소스 등록, collision_monitor에 pointcloud 소스 추가
- `launch/record_launch.py` — 카메라 토픽명 `/image` 서픽스 반영

**Nav2 costmap 파라미터:**
- Global costmap: obstacle_max_range 8m, raytrace_max_range 10m
- Local costmap: obstacle_max_range 5m, raytrace_max_range 6m (근거리 집중)
- min_obstacle_height 0.1m (지면 노이즈 필터), max_obstacle_height 2.0m

### Stage 3 — perception_node.py LiDAR 제거 + 멀티카메라 전환 (완료)

**변경 파일 2개:**
- `ad_perception/perception_node.py` — 전면 재작성:
  - LidarProcessor import/초기화/구독/콜백/process_lidar/_pointcloud2_to_numpy 전체 삭제
  - SensorFusion import/초기화/_run_fusion 삭제
  - struct, PointCloud2, PointField import 삭제
  - `topics.camera` (단일) → `topics.camera_front/left/right` (멀티) 파라미터 전환
  - 3대 카메라 구독 추가: front(탐지+세그멘테이션), left/right(이미지 캐시)
  - `_latest_images` dict로 멀티카메라 최신 이미지 관리
  - `get_latest_image(camera='front')` API 추가
  - `get_perception_result()` → obstacles/fused_objects 필드 제거
- `ad_perception/config/perception_params.yaml` — 토픽명 멀티카메라로 갱신

### Stage 4 — sensor_fusion.py 독스트링 갱신 (완료)

**변경 파일 1개:**
- `ad_core/sensor_fusion.py` — 모듈 독스트링을 Camera-Only 상태 반영으로 갱신
- 라이브러리 코드 자체는 보존 (기존 테스트 29개 전체 통과)
- `lidar_processor.py`도 보존 — `Obstacle` 데이터클래스가 sensor_fusion에서 참조됨

### Stage 5 — ARCH-005 ADR 작성 + ARCH-001 TF 트리 갱신 (완료)

**신규 파일 1개:**
- `docs/arch/ARCH-005_camera_only_perception.md` — Camera-Only 인지 아키텍처 ADR

**수정 파일 2개:**
- `docs/arch/ARCH-001_tf_tree.md` — TF 트리에서 LiDAR 프레임 제거, 카메라 3대 + GPS 프레임 추가
- `docs/arch/INDEX.md` — ARCH-005 엔트리 추가

## 검증

- colcon build: **성공** (Stage 1~4, ad_core + ad_perception + ad_bringup 3개 패키지)
- 기존 테스트: **29/29 통과** (test_sensor_fusion 19개 + test_lidar_processor 10개)
- 시뮬레이션 실행 검증: **미실행** (C61 시뮬레이션 검증과 동시 진행 예정)

## 토픽 구조 변경 참조

### rgbd_camera 토픽 구조
```
/sensor/camera/front/image         → sensor_msgs/Image (RGB)
/sensor/camera/front/depth_image   → sensor_msgs/Image (Depth, float32)
/sensor/camera/front/points        → sensor_msgs/PointCloud2
```
좌/우 카메라도 동일 구조 (`left`, `right`)

### 데이터 흐름
```
Gazebo rgbd_camera
  → ros_gz_bridge (Image + PointCloud2)
    → Nav2 costmap obstacle_layer (marking + clearing)
      → inflation_layer
        → planner/controller
```

## 미결 이슈

- [ ] Gazebo Harmonic에서 `rgbd_camera` 토픽 서픽스가 `/image`, `/depth_image`, `/points` 형태인지 시뮬레이션으로 실증 필요
- [ ] 카메라 FOV 80° × 3대 = 전면 ~240° 커버리지 — 측면/후방 사각지대 존재. 후방 카메라 추가 검토 필요할 수 있음
- [ ] 실물에서 Mono Depth 추정 정확도에 따라 costmap 파라미터 튜닝 필요 (min_obstacle_height, obstacle_max_range 등)
- [ ] depth_camera far clip 10m — 농경지 직선 구간에서 부족할 수 있음. 시뮬레이션 결과 보고 조정
