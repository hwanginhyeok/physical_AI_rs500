# ARCH-005: Camera-Only 인지 아키텍처

- 상태: `채택됨`
- 결정일: 2026-03-13
- 관련 TASK: C64
- 대체: ARCH-001 TF 트리의 센서 프레임 부분 갱신

## 배경

SS500 궤도차량의 센서 스펙이 확정됨:
- **카메라 3대** (전면 0°, 좌전방 ~60°, 우전방 ~-60°), RGBD 시뮬레이션
- **IMU** (100Hz), **GPS** (10Hz)
- **LiDAR 미탑재**

기존 시스템은 LiDAR LaserScan 기반으로 설계되어 있었으나 (Nav2 costmap obstacle_layer,
collision_monitor, perception_node의 LidarProcessor, sensor_fusion Late Fusion),
실차에 LiDAR가 없으므로 Camera-Only 인지 파이프라인으로 전면 전환.

## 결정

### 1. 장애물 감지: Gazebo rgbd_camera PointCloud2 → Nav2 costmap 직접 연결

시뮬레이션에서 Gazebo `rgbd_camera` 센서가 PointCloud2를 직접 발행하고,
Nav2 costmap `obstacle_layer`가 이를 observation source로 소비한다.
별도의 인지 노드(LidarProcessor 등)를 거치지 않음.

```
[시뮬레이션]
Gazebo rgbd_camera (×3)
  → ros_gz_bridge (PointCloud2)
    → Nav2 costmap obstacle_layer (marking + clearing)

[실물 — 향후]
Camera RGB (×3)
  → MiDaS/DepthAnything Mono Depth
    → PointCloud2 변환
      → Nav2 costmap obstacle_layer
```

### 2. 인지 노드: 멀티카메라 객체 탐지 + 세그멘테이션

`perception_node.py`는 카메라 3대 RGB 이미지를 구독하고:
- 전면 카메라: YOLO 객체 탐지 + 시맨틱 세그멘테이션
- 좌/우 카메라: 이미지 캐시 (향후 멀티카메라 탐지 확장용)

### 3. 라이브러리 보존

`ad_core/lidar_processor.py`와 `ad_core/sensor_fusion.py`는 순수 알고리즘 라이브러리로서 보존.
테스트 29개가 정상 통과하며, 향후 LiDAR 탑재 차종에서 재사용 가능.

## 검토한 대안

| 대안 | 기각 이유 |
|------|-----------|
| LiDAR 설정 유지 + 비활성화 | 죽은 코드, 혼란 유발. Nav2 costmap에 빈 소스가 남아 장애물 감지 불능 |
| 카메라 Mono Depth → perception_node 내부 PointCloud 변환 | 시뮬레이션에서는 Gazebo가 depth를 직접 제공하므로 불필요한 복잡도. 실물에서만 별도 노드로 처리 |
| voxel_layer 사용 | 카메라 depth 범위(10m)에서는 2D obstacle_layer로 충분. voxel_layer의 3D 복셀은 LiDAR의 넓은 FOV에 적합 |

## 센서 커버리지

```
         좌전방 60°        전면 0°        우전방 -60°
          \              |              /
           \    80° FOV  |  80° FOV   /
            \           /|\          /
             \         / | \        /
              \       /  |  \      /
               ------    |   ------
                         |
                      SS500
```

- 각 카메라 FOV: 80° (1.396 rad)
- 3대 합산: 전면 ~240° 커버리지
- 사각지대: 측면 후방 ~120°

## 토픽 구조

| Gazebo 출력 | Bridge | ROS2 토픽 | 소비자 |
|-------------|--------|-----------|--------|
| rgbd_camera front/image | Image | `/sensor/camera/front/image` | perception_node |
| rgbd_camera front/points | PointCloud2 | `/sensor/camera/front/points` | Nav2 costmap |
| rgbd_camera left/image | Image | `/sensor/camera/left/image` | perception_node |
| rgbd_camera left/points | PointCloud2 | `/sensor/camera/left/points` | Nav2 costmap |
| rgbd_camera right/image | Image | `/sensor/camera/right/image` | perception_node |
| rgbd_camera right/points | PointCloud2 | `/sensor/camera/right/points` | Nav2 costmap |

## Nav2 costmap 파라미터

| 구분 | obstacle_max_range | raytrace_max_range | min_obstacle_height |
|------|------|------|------|
| Global costmap | 8m | 10m | 0.1m |
| Local costmap | 5m | 6m | 0.1m |
