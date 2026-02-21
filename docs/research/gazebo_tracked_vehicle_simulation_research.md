# 궤도차량(Tracked Vehicle) 자율주행 Gazebo 시뮬레이션 선행연구

> 작성일: 2026-02-21
> 대상 프로젝트: RS500 농업용 궤도차량 자율주행 (gz-sim 9+, ROS2 Humble, ODE 물리엔진)

---

## 목차

1. [Gazebo 궤도차량 시뮬레이션 사례 (TrackedVehicle 플러그인)](#1-gazebo-궤도차량-시뮬레이션-사례)
2. [비포장/농경지 지형 시뮬레이션](#2-비포장농경지-지형-시뮬레이션)
3. [센서 시뮬레이션 고도화](#3-센서-시뮬레이션-고도화)
4. [ROS2 + Gazebo 연동 베스트 프랙티스](#4-ros2--gazebo-연동-베스트-프랙티스)
5. [SIL/HIL 테스트 프레임워크 구축 사례](#5-silhil-테스트-프레임워크-구축-사례)
6. [오픈소스 농업로봇/궤도차량 시뮬레이션 프로젝트](#6-오픈소스-농업로봇궤도차량-시뮬레이션-프로젝트)
7. [현재 프로젝트 분석 및 개선 로드맵](#7-현재-프로젝트-분석-및-개선-로드맵)

---

## 1. Gazebo 궤도차량 시뮬레이션 사례

### 1.1 핵심 내용

Gazebo의 궤도차량 시뮬레이션은 CTU Prague의 Martin Pecka 연구팀이 개발한 **Contact Surface Motion** 방식을 기반으로 한다. 이 방법은 2017 IEEE/RSJ IROS에서 발표된 논문 "Fast Simulation of Vehicles with Non-deformable Tracks"에 근거하며, 물리 엔진의 접촉점(contact point)에 원하는 트랙 속도를 설정하여 마찰력 범위 내에서 트랙 구동을 시뮬레이션한다.

#### 시스템 구성 (gz-sim)

궤도차량 시뮬레이션은 두 개의 플러그인으로 구성된다:

| 플러그인 | 역할 | 파일명 |
|---------|------|--------|
| **TrackedVehicle** | 차량 레벨 컨트롤러. cmd_vel을 좌/우 트랙 속도로 변환 | `gz-sim-tracked-vehicle-system` |
| **TrackController** | 개별 트랙 레벨 컨트롤러. 접촉면 운동(contact surface motion) 관리 | `gz-sim-track-controller-system` |

#### TrackedVehicle 주요 파라미터

| 파라미터 | 설명 | 기본값 |
|---------|------|--------|
| `<left_track>` / `<right_track>` | 좌/우 트랙 링크 지정 (복수 가능) | 필수 |
| `<tracks_separation>` | 좌우 트랙 중심 간 거리(m) | 0.4 |
| `<steering_efficiency>` | 조향 효율 계수 (0.0~1.0) | 0.5 |
| `<track_mu>` / `<track_mu2>` | 트랙 마찰 계수 (1차/2차 방향) | 모델 정의 값 사용 |

#### TrackController 주요 기능

- 각 트랙 링크의 contact surface motion 파라미터를 물리 엔진에 전달
- 선회 시 순간 회전 중심(ICR)에 기반한 마찰 방향 자동 조정
- 디버그 모드(`<debug>1</debug>`)로 시각화 가능

#### 알려진 이슈 (gz-sim 8~9)

- **자체 이동(self-moving) 문제**: gz-sim 8.8.0에서 FP 한계값 수정으로 해결 ([PR #2651](https://github.com/gazebosim/gz-sim/pull/2651))
- **역방향 주행 버그**: [Issue #2008](https://github.com/gazebosim/gz-sim/issues/2008) -- 방향 반전 현상 보고
- **예제 월드 비작동**: [Issue #1662](https://github.com/gazebosim/gz-sim/issues/1662) -- Conveyor/TrackedVehicle 예제 관련

### 1.2 관련 문서 및 링크

- [TrackedVehicle Class Reference (gz-sim 7)](https://gazebosim.org/api/sim/7/classgz_1_1sim_1_1systems_1_1TrackedVehicle.html)
- [TrackController Class Reference (gz-sim 8)](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1TrackController.html)
- [TrackedVehicle PR #869 (원본 구현)](https://github.com/gazebosim/gz-sim/pull/869)
- [Fast Simulation of Vehicles with Non-deformable Tracks (논문 PDF)](https://cmp.felk.cvut.cz/~peckama2/papers/2017_IROS_Fast_Simulation_of_Vehicles_with_Non-deformable_Tracks.pdf)
- [CTU Prague - Tracked Vehicle Simulation](https://cyber.felk.cvut.cz/news/tracked-vehicle-simulation-model-for-gazebo/)
- [Gazebo Classic 예제: tracked_vehicle_simple.world](https://github.com/gazebosim/gazebo-classic/blob/gazebo11/worlds/tracked_vehicle_simple.world)

### 1.3 우리 프로젝트에 대한 적용 방안

**현재 설정 분석:**
현재 `model.sdf`에서 TrackedVehicle 플러그인은 올바르게 구성되어 있으나 **TrackController 플러그인이 누락**되어 있다. TrackedVehicle은 TrackController와 함께 사용해야 올바른 contact surface motion 기반 시뮬레이션이 가능하다.

**개선 사항:**

```xml
<!-- 좌측 트랙에 TrackController 추가 -->
<plugin
  filename="gz-sim-track-controller-system"
  name="gz::sim::systems::TrackController">
  <link>left_track</link>
  <min_velocity>-1.0</min_velocity>
  <max_velocity>1.0</max_velocity>
  <debug>0</debug>
</plugin>

<!-- 우측 트랙에 TrackController 추가 -->
<plugin
  filename="gz-sim-track-controller-system"
  name="gz::sim::systems::TrackController">
  <link>right_track</link>
  <min_velocity>-1.0</min_velocity>
  <max_velocity>1.0</max_velocity>
  <debug>0</debug>
</plugin>
```

**RS500 사양 반영 추가 권장:**

- `<tracks_separation>`: RS500 실제 트랙 간 거리로 조정 (현재 1.4m)
- `<steering_efficiency>`: 실제 차량 선회 테스트 데이터 기반 조정 (현재 0.8)
- `<track_mu>`: 논/밭 토양 마찰 계수 반영 (0.6~1.2 범위)

---

## 2. 비포장/농경지 지형 시뮬레이션

### 2.1 핵심 내용

#### 2.1.1 Heightmap 기반 지형

Gazebo는 **heightmap**(높이맵)과 **DEM(Digital Elevation Model)** 두 가지 방식으로 지형을 표현한다.

**Heightmap:**
- 회색조(grayscale) PNG 이미지를 기반으로 지형 생성
- 이미지 크기: `(2^n) + 1` 정사각형 (예: 129x129, 257x257, 513x513)
- 검정(0) = 최저점, 흰색(255) = 최고점
- 다층 텍스처 적용 가능 (높이별 토양/잔디/자갈 등)

**DEM (Digital Elevation Model):**
- 실제 지형 데이터 (GeoTIFF 등) 직접 로드
- WGS84 좌표계 기반 실세계 고도 정보 활용

#### Heightmap SDF 구성 예시

```xml
<model name="farmland_terrain">
  <static>true</static>
  <link name="terrain_link">
    <collision name="terrain_collision">
      <geometry>
        <heightmap>
          <uri>file://media/farmland_heightmap.png</uri>
          <size>200 200 5</size>   <!-- x, y 범위(m), z 최대높이(m) -->
          <pos>0 0 0</pos>
        </heightmap>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.7</mu>       <!-- 농경지 토양 마찰계수 -->
            <mu2>0.5</mu2>     <!-- 측방향 마찰계수 (낮음) -->
          </ode>
        </friction>
      </surface>
    </collision>
    <visual name="terrain_visual">
      <geometry>
        <heightmap>
          <uri>file://media/farmland_heightmap.png</uri>
          <size>200 200 5</size>
          <pos>0 0 0</pos>
          <texture>
            <diffuse>file://media/soil_diffuse.png</diffuse>
            <normal>file://media/soil_normal.png</normal>
            <size>10</size>      <!-- 텍스처 타일 크기 -->
          </texture>
          <blend>
            <min_height>2</min_height>
            <fade_dist>1</fade_dist>
          </blend>
          <texture>
            <diffuse>file://media/grass_diffuse.png</diffuse>
            <normal>file://media/grass_normal.png</normal>
            <size>10</size>
          </texture>
        </heightmap>
      </geometry>
    </visual>
  </link>
</model>
```

#### 2.1.2 지형 마찰 모델 (ODE)

ODE 물리엔진의 마찰 모델은 두 가지 방식을 지원한다:

| 모델 | 설명 |
|------|------|
| **pyramid (기본)** | 마찰 제약을 2개의 1차원 제약으로 분리하여 독립 해석. 마찰력 한계 = 법선력 x mu(무차원) |
| **box** | mu/mu2에 뉴턴(N) 단위를 사용하는 마찰 모델 |

**농경지 시뮬레이션을 위한 마찰 계수 참고값:**

| 지면 종류 | mu (종방향) | mu2 (횡방향) | 비고 |
|----------|------------|-------------|------|
| 건조 토양 | 0.8 ~ 1.0 | 0.6 ~ 0.8 | 단단한 밭 |
| 습윤 토양 | 0.4 ~ 0.6 | 0.3 ~ 0.5 | 비 온 뒤 |
| 잔디/초지 | 0.6 ~ 0.8 | 0.5 ~ 0.7 | - |
| 자갈/쇄석 | 0.7 ~ 0.9 | 0.6 ~ 0.8 | 농로 |
| 진흙 | 0.2 ~ 0.4 | 0.15 ~ 0.3 | 극단 조건 |

#### 2.1.3 변형 가능 지형 (Deformable Terrain) -- 한계와 대안

**Gazebo의 한계:** 현재 Gazebo (gz-sim 9)에서는 실시간 지형 변형(차량 통과 시 바퀴 자국, 토양 침하 등)을 네이티브로 지원하지 않는다. Heightmap은 정적(static) 지형만 제공한다.

**대안 -- Project Chrono:**

[Project Chrono](https://projectchrono.org/)는 변형 가능 지형과 궤도차량의 고충실도 시뮬레이션을 제공하는 오픈소스 물리 엔진이다:

- **SCM (Soil Contact Model)**: 정규 직교 격자 기반, 수직 변형으로 토양 모델링. 근실시간 가능
- **DEM (Discrete Element Method)**: 입자 기반 토양 해석. 높은 정확도, 높은 계산 비용
- **FEA 연속체 모델**: Drucker-Prager 파괴 기준 기반 유한요소 토양 모델
- 세그먼트형/연속 밴드형 트랙 템플릿 제공
- Gazebo와 직접 연동은 아니지만, co-simulation 인터페이스 가능

### 2.2 관련 문서 및 링크

- [Gazebo DEM Tutorial](https://classic.gazebosim.org/tutorials?tut=dem)
- [Creating Heightmaps for Gazebo](https://github.com/AS4SR/general_info/wiki/Creating-Heightmaps-for-Gazebo)
- [gazebo_terrain - Terrain Generator](https://github.com/MatthewVerbryke/gazebo_terrain)
- [terrain_generator](https://github.com/Sarath18/terrain_generator)
- [Gazebo Rendering: Heightmap API](https://gazebosim.org/api/rendering/7/heightmap.html)
- [ODE Friction Tutorial](https://classic.gazebosim.org/tutorials?tut=friction)
- [Gazebo Physics Engines](https://gazebosim.org/api/sim/9/physics.html)
- [Project Chrono - Terrain Models](https://api.projectchrono.org/vehicle_terrain.html)
- [Clearpath Agriculture Simulation](https://clearpathrobotics.com/blog/2020/07/clearpath-robots-get-new-gazebo-simulation-environments/)

### 2.3 우리 프로젝트에 대한 적용 방안

**현재 설정 분석:**
현재 `agricultural_field.sdf`에서는 **평탄한 plane 지면만 사용** 중이다. 실제 농경지의 울퉁불퉁한 지형, 경사, 논두렁 등이 전혀 반영되지 않았다.

**단계별 개선 방안:**

| 단계 | 내용 | 난이도 | 우선도 |
|------|------|--------|--------|
| Phase 1 | plane을 heightmap으로 교체 (완만한 기복 + 토양 텍스처) | 낮음 | 높음 |
| Phase 2 | 다중 마찰 영역 구현 (밭/논두렁/농로 구분) | 중간 | 높음 |
| Phase 3 | 실제 농지 DEM 데이터 활용 (국토정보플랫폼 등) | 중간 | 중간 |
| Phase 4 | 장애물 모델 추가 (돌, 고랑, 관개수로) | 중간 | 중간 |
| Phase 5 | Project Chrono co-sim 연동 (변형 지형 필요 시) | 높음 | 낮음 |

**지형 마찰 계수 개선:** 현재 ground_plane의 mu=0.8은 건조 토양에 적합하나, 다양한 조건 테스트를 위해 마찰 계수를 launch argument로 동적 변경 가능하도록 구성 권장.

---

## 3. 센서 시뮬레이션 고도화

### 3.1 핵심 내용

Gazebo는 기본적으로 센서 데이터를 **완벽하게(노이즈 없이)** 생성한다. 실제 센서의 특성을 반영하려면 노이즈 모델을 명시적으로 추가해야 한다.

#### 3.1.1 LiDAR 노이즈 모델

gz-sim의 `gpu_lidar` 센서는 가우시안 노이즈를 range에 추가할 수 있다:

```xml
<sensor name="lidar_sensor" type="gpu_lidar">
  <lidar>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>
        <resolution>1</resolution>
        <min_angle>-0.2618</min_angle>
        <max_angle>0.2618</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.3</min>
      <max>50</max>
      <resolution>0.01</resolution>
    </range>
    <!-- 거리 노이즈 추가 -->
    <noise type="gaussian">
      <mean>0.0</mean>
      <stddev>0.01</stddev>    <!-- 1cm 표준편차 -->
    </noise>
  </lidar>
</sensor>
```

**실제 LiDAR 노이즈 참고값:**

| LiDAR 모델 | Range Noise (stddev) | 비고 |
|------------|---------------------|------|
| Velodyne VLP-16 | ~0.03m | 야외 환경 |
| Ouster OS1-64 | ~0.01m | 정밀형 |
| 저가 2D LiDAR | ~0.05m | RPLiDAR 등 |

#### 3.1.2 카메라 왜곡 모델

Gazebo는 **Brown's Distortion Model** 기반 카메라 왜곡을 지원한다. 5개 왜곡 계수(k1, k2, k3, p1, p2)를 설정한다:

```xml
<sensor name="front_camera" type="camera">
  <camera>
    <horizontal_fov>1.3962634</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <!-- Brown 왜곡 모델 추가 -->
    <distortion>
      <k1>-0.25</k1>      <!-- 방사형 왜곡 계수 1 -->
      <k2>0.12</k2>       <!-- 방사형 왜곡 계수 2 -->
      <k3>0.0</k3>        <!-- 방사형 왜곡 계수 3 -->
      <p1>-0.00028</p1>   <!-- 접선 왜곡 계수 1 -->
      <p2>-0.00005</p2>   <!-- 접선 왜곡 계수 2 -->
      <center>0.5 0.5</center>  <!-- 왜곡 중심 (정규화) -->
    </distortion>
    <!-- 이미지 노이즈 추가 -->
    <noise type="gaussian">
      <mean>0.0</mean>
      <stddev>0.007</stddev>  <!-- 일반 디지털 카메라 수준 -->
    </noise>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
</sensor>
```

#### 3.1.3 GPS(NavSat) 오차 모델

현재 프로젝트의 GPS 노이즈 설정은 적절하나, 더 현실적인 시뮬레이션을 위해 다음을 고려할 수 있다:

**표준 GPS 오차 참고:**

| GPS 등급 | 수평 오차 (CEP) | 수직 오차 | 적용 |
|---------|----------------|----------|------|
| 민간 단독 | 2~5m | 5~10m | 일반 내비 |
| SBAS/DGPS | 0.5~2m | 1~3m | 중정밀 |
| RTK-GNSS | 0.01~0.02m | 0.02~0.05m | 정밀 농업 |

**GNSS Multipath 시뮬레이션:** CTU MRS 연구팀은 [가상 도시 협곡에서 GNSS 다중경로 신호 에뮬레이션을 위한 오픈소스 Gazebo 플러그인](https://arxiv.org/abs/2212.04018)을 발표하였다. Ray-tracing 기반으로 다중경로 효과를 모사하며, 위성 배치와 수신기 특성을 SDF에서 설정 가능하다.

#### 3.1.4 IMU 노이즈 모델 고도화

Gazebo IMU 센서는 **노이즈(noise)**와 **바이어스(bias)** 두 가지 왜란을 모델링한다:

```xml
<sensor name="imu_sensor" type="imu">
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.009</stddev>          <!-- rad/s -->
          <bias_mean>0.00075</bias_mean>
          <bias_stddev>0.005</bias_stddev>
          <dynamic_bias_stddev>0.00002</dynamic_bias_stddev>
          <dynamic_bias_correlation_time>400.0</dynamic_bias_correlation_time>
        </noise>
      </x>
      <!-- y, z 축 동일 설정 -->
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.021</stddev>          <!-- m/s^2 -->
          <bias_mean>0.05</bias_mean>
          <bias_stddev>0.0075</bias_stddev>
          <dynamic_bias_stddev>0.000035</dynamic_bias_stddev>
          <dynamic_bias_correlation_time>300.0</dynamic_bias_correlation_time>
        </noise>
      </x>
      <!-- y, z 축 동일 설정 -->
    </linear_acceleration>
  </imu>
</sensor>
```

**IMU 노이즈 파라미터 의미:**

| 파라미터 | 설명 | 단위 |
|---------|------|------|
| `mean` | 가우시안 노이즈 평균 | rad/s 또는 m/s^2 |
| `stddev` | 가우시안 노이즈 표준편차 | rad/s 또는 m/s^2 |
| `bias_mean` | 바이어스 평균값 (상수 드리프트) | 동일 |
| `bias_stddev` | 바이어스 표준편차 (시동 간 변동) | 동일 |
| `dynamic_bias_stddev` | 동적 바이어스 표준편차 (시간 경과에 따른 드리프트) | 동일 |
| `dynamic_bias_correlation_time` | 동적 바이어스 상관 시간 | 초(s) |

### 3.2 관련 문서 및 링크

- [Gazebo Sensor Noise Model Tutorial](https://classic.gazebosim.org/tutorials?tut=sensor_noise)
- [Gazebo Camera Distortion Tutorial](https://classic.gazebosim.org/tutorials?tut=camera_distortion)
- [gz-sensors Repository](https://github.com/gazebosim/gz-sensors)
- [Gazebo Sensors Overview (Jetty)](https://gazebosim.org/docs/latest/sensors/)
- [Nav2 Sensor Setup Guide (Gazebo)](https://docs.nav2.org/setup_guides/sensors/setup_sensors_gz.html)
- [GNSS Multipath Plugin (논문)](https://arxiv.org/abs/2212.04018)
- [CTU MRS Gazebo Plugins](https://github.com/ctu-mrs/mrs_gazebo_common_resources/blob/master/src/sensor_and_model_plugins/README.md)
- [MOGI ROS Week 5-6 Gazebo Sensors](https://github.com/MOGI-ROS/Week-5-6-Gazebo-sensors)

### 3.3 우리 프로젝트에 대한 적용 방안

**현재 설정 분석:**

| 센서 | 현재 상태 | 문제점 |
|------|----------|--------|
| GPS | 노이즈 O (H:0.5m, V:1.0m) | 바이어스/드리프트 없음. 정밀농업(RTK) 시나리오 미반영 |
| IMU | 노이즈 O (ang:0.01, acc:0.1) | **바이어스 모델 완전 누락**. 장시간 주행 시 비현실적 |
| 카메라 | 노이즈 없음, 왜곡 없음 | 완벽한 이미지 출력 -- 비현실적 |
| LiDAR | 노이즈 없음 | 완벽한 거리 측정 -- 비현실적 |

**개선 우선순위:**

1. **[높음]** LiDAR에 range 노이즈 추가 (stddev: 0.02~0.03m)
2. **[높음]** IMU에 바이어스 모델(bias_mean, dynamic_bias) 추가
3. **[중간]** 카메라에 왜곡(distortion) + 이미지 노이즈 추가
4. **[중간]** GPS 노이즈를 RTK 시나리오별로 프로파일화 (단독/DGPS/RTK)
5. **[낮음]** GNSS multipath 플러그인 연동 (농경지에서는 우선도 낮음)

---

## 4. ROS2 + Gazebo 연동 베스트 프랙티스

### 4.1 핵심 내용

#### 4.1.1 ros_gz_bridge YAML 설정

`ros_gz_bridge`는 Gazebo Transport와 ROS2 간의 양방향 메시지 변환을 제공한다. YAML 파일 기반 설정이 권장된다:

```yaml
# bridge_config.yaml 예시 (베스트 프랙티스)
- ros_topic_name: /cmd_vel
  gz_topic_name: /model/tracked_vehicle/cmd_vel
  ros_type_name: geometry_msgs/msg/Twist
  gz_type_name: gz.msgs.Twist
  direction: ROS_TO_GZ

- ros_topic_name: /sensor/lidar
  gz_topic_name: /world/agricultural_field/model/tracked_vehicle/link/chassis/sensor/lidar_sensor/scan
  ros_type_name: sensor_msgs/msg/LaserScan
  gz_type_name: gz.msgs.LaserScan
  direction: GZ_TO_ROS
  # QoS 설정 가능
  subscriber_queue: 10
  publisher_queue: 10
  lazy: true    # 구독자 있을 때만 브릿지 활성화
```

#### 4.1.2 Launch 구성 베스트 프랙티스

**Composition 모드 활용:**

```python
# 최적화된 launch 구성
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
    # Composition으로 프로세스 오버헤드 감소
    # use_composition=True,
    output='screen',
)
```

**핵심 권장 패턴:**

| 패턴 | 설명 |
|------|------|
| YAML 기반 토픽 매핑 | 유지보수 용이, 토픽 변경 시 코드 수정 불필요 |
| Composition 모드 | 여러 브릿지 노드를 하나의 프로세스로 실행하여 IPC 오버헤드 감소 |
| Lazy 구독 | 구독자가 없을 때 불필요한 브릿지 비활성화 |
| QoS 오버라이드 | 센서별 적절한 QoS 설정 (reliability, durability) |
| GUI 분리 | `gui:=false` 옵션으로 headless 실행 지원 |

#### 4.1.3 tf2 브릿지

현재 프로젝트에서 누락된 중요 요소:

```yaml
# TF 브릿지 추가 필요
- ros_topic_name: /tf
  gz_topic_name: /model/tracked_vehicle/pose
  ros_type_name: tf2_msgs/msg/TFMessage
  gz_type_name: gz.msgs.Pose_V
  direction: GZ_TO_ROS
```

또는 `ros_gz_bridge`의 `--ros-args -p qos_overrides./tf.publisher.durability:=transient_local` 설정으로 TF를 적절히 처리해야 한다.

#### 4.1.4 추가 브릿지 토픽 (누락 항목)

현재 bridge_config.yaml에서 다음 토픽들이 누락되어 있다:

| 토픽 | 용도 | 방향 |
|------|------|------|
| `/clock` | 시뮬레이션 시간 동기화 | GZ_TO_ROS |
| `/tf` | 좌표 변환 | GZ_TO_ROS |
| `/sensor/camera/front/camera_info` | 카메라 내부 파라미터 | GZ_TO_ROS |
| `/joint_states` | 트랙 조인트 상태 | GZ_TO_ROS |

### 4.2 관련 문서 및 링크

- [Use ROS 2 to interact with Gazebo (Harmonic)](https://gazebosim.org/docs/harmonic/ros2_integration/)
- [ros_gz_bridge README](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md)
- [ros_gz_bridge ROS Package](https://index.ros.org/p/ros_gz_bridge/)
- [ros_gz_bridge Documentation (Humble)](https://docs.ros.org/en/humble/p/ros_gz_bridge/)
- [Launch Gazebo from ROS 2](https://gazebosim.org/docs/latest/ros2_launch_gazebo/)
- [ROS 2 Humble Gazebo Tutorial](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [ros_gz GitHub Repository](https://github.com/gazebosim/ros_gz)
- [Gazebo ros2_control](https://control.ros.org/humble/doc/gazebo_ros2_control/doc/index.html)

### 4.3 우리 프로젝트에 대한 적용 방안

**현재 설정 분석:**
현재 `simulation_launch.py`와 `bridge_config.yaml`의 기본 구조는 양호하나 다음이 누락됨:

**개선 사항:**

1. **[높음] `/clock` 토픽 브릿지 추가** -- 시뮬레이션 시간과 ROS 노드 시간 동기화 필수
2. **[높음] `/tf` 브릿지 추가** -- Nav2, robot_localization 등과 연동에 필수
3. **[중간] `camera_info` 토픽 추가** -- 카메라 캘리브레이션 정보 전달
4. **[중간] `use_sim_time:=true` 파라미터 추가** -- 모든 ROS 노드에 적용
5. **[낮음] Composition 모드 도입** -- 성능 최적화

**Launch 파일 개선 예시:**

```python
# use_sim_time 전역 설정
SetParameter(name='use_sim_time', value=True),

# clock 브릿지 (별도 또는 YAML에 추가)
# bridge_config.yaml에 추가:
# - ros_topic_name: /clock
#   gz_topic_name: /clock
#   ros_type_name: rosgraph_msgs/msg/Clock
#   gz_type_name: gz.msgs.Clock
#   direction: GZ_TO_ROS
```

---

## 5. SIL/HIL 테스트 프레임워크 구축 사례

### 5.1 핵심 내용

#### 5.1.1 SIL (Software-in-the-Loop) 테스트

SIL 테스트는 실제 하드웨어 없이 소프트웨어만으로 알고리즘을 검증한다:

**Gazebo + ROS2 기반 SIL 구성:**

```
+------------------+     +-----------+     +----------------+
| ROS2 노드들       | <-> | ros_gz    | <-> | Gazebo gz-sim  |
| (자율주행 스택)    |     | bridge    |     | (물리 시뮬)    |
| - 경로 계획       |     +-----------+     | - 궤도차량     |
| - 제어기          |                       | - 센서 시뮬    |
| - 인지 (perception)|                      | - 환경         |
+------------------+                       +----------------+
```

**SIL 테스트 시나리오 예시:**
- 직선 주행 정밀도 테스트
- U턴/선회 동작 검증
- GPS 경로 추종 정확도
- 장애물 회피 반응 시간
- 경사지 주행 안정성

#### 5.1.2 CI/CD 파이프라인 통합

**setup-gazebo GitHub Action:**
Gazebo 공식 GitHub Action으로 CI 환경에 Gazebo를 자동 설치하여 headless 시뮬레이션 테스트를 수행할 수 있다:

```yaml
# .github/workflows/simulation_test.yml 예시
name: SIL Simulation Tests
on: [push, pull_request]

jobs:
  simulation-test:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v4

      - name: Setup ROS 2 Humble
        uses: ros-tooling/setup-ros@v0.7
        with:
          required-ros-distributions: humble

      - name: Setup Gazebo
        uses: gazebosim/setup-gazebo@v1
        with:
          required-gazebo-distributions: garden

      - name: Build workspace
        run: |
          source /opt/ros/humble/setup.bash
          colcon build

      - name: Run SIL tests (headless)
        run: |
          source install/setup.bash
          # Headless 모드로 시뮬레이션 실행
          ros2 launch ad_simulation simulation_launch.py gui:=false &
          sleep 10
          # 테스트 실행
          ros2 run ad_simulation sil_test_runner
```

#### 5.1.3 HIL (Hardware-in-the-Loop) 테스트

HIL 테스트는 실제 ECU/컨트롤러 하드웨어와 시뮬레이션을 연동한다:

```
+-------------------+     +----------+     +----------------+
| 실제 하드웨어      | <-> | CAN/     | <-> | ROS2 + Gazebo  |
| - ECU             |     | Serial   |     | 시뮬레이션      |
| - GPS 수신기      |     | Bridge   |     |                |
| - IMU             |     +----------+     +----------------+
| - 모터 드라이버   |
+-------------------+
```

**HIL 핵심 고려사항:**
- 실시간 성능: 시뮬레이션 real-time factor >= 1.0 유지 필수
- 통신 지연: CAN/Serial 인터페이스 지연 측정 및 보상
- 동기화: 하드웨어 타이밍과 시뮬레이션 시간 일치
- 안전 모니터: watchdog 타이머 기반 비상 정지

#### 5.1.4 launch_testing 기반 자동화 테스트

ROS2의 `launch_testing` 프레임워크를 활용한 시뮬레이션 테스트:

```python
# test/test_straight_drive.py
import launch_testing
import unittest

def generate_test_description():
    # Gazebo + 차량 스폰
    sim_launch = IncludeLaunchDescription(...)
    return LaunchDescription([sim_launch]), {
        'test_proc': test_proc
    }

class TestStraightDrive(unittest.TestCase):
    def test_position_accuracy(self):
        """10m 직진 후 위치 오차 0.5m 이내 확인"""
        # cmd_vel 발행 → 일정 시간 후 odom 확인
        pass

    def test_heading_stability(self):
        """직진 중 heading 변동 2도 이내 확인"""
        pass
```

### 5.2 관련 문서 및 링크

- [SIL Modeling and Simulation Framework (ResearchGate)](https://www.researchgate.net/publication/328974319_Software-in-the-Loop_Modeling_and_Simulation_Framework_for_Autonomous_Vehicles)
- [HIL Simulation for Swarm Communication (IEEE)](https://ieeexplore.ieee.org/document/9469411/)
- [Advancing Robotics with Gazebo Ionic (Intrinsic)](https://www.intrinsic.ai/blog/posts/advancing-real-world-robotics-through-simulation-with-gazebo-ionic)
- [Robotic Software Testing: ROS2, Gazebo](https://www.testriq.com/blog/post/robotic-software-testing-ros2-gazebo-and-motion-planning-validation)
- [NVIDIA Isaac Sim + ROS2 Guide](https://developer.nvidia.com/blog/a-beginners-guide-to-simulating-and-testing-robots-with-ros-2-and-nvidia-isaac-sim/)
- [PX4-ROS2-Gazebo Simulation Template](https://github.com/SathanBERNARD/PX4-ROS2-Gazebo-Drone-Simulation-Template)
- [Applied Intuition HIL Sim](https://www.appliedintuition.com/products/hil-sim)

### 5.3 우리 프로젝트에 대한 적용 방안

**현재 설정 분석:**
현재 프로젝트에는 **자동화된 테스트 프레임워크가 전혀 없다**. Launch 파일은 단순 실행만 지원하며, 결과 검증 메커니즘이 없다.

**단계별 구축 방안:**

| 단계 | 내용 | 우선도 |
|------|------|--------|
| Phase 1 | launch_testing 기반 기본 SIL 테스트 작성 (직진/선회) | 높음 |
| Phase 2 | CI/CD 파이프라인에 headless 시뮬레이션 테스트 통합 | 높음 |
| Phase 3 | 시나리오 기반 테스트 스위트 (경사지, 습윤 토양, GPS 불안정 등) | 중간 |
| Phase 4 | 성능 메트릭 자동 수집 (경로 추종 오차, 응답 시간 등) | 중간 |
| Phase 5 | HIL 인터페이스 구축 (RS500 ECU 연동) | 나중 |

---

## 6. 오픈소스 농업로봇/궤도차량 시뮬레이션 프로젝트

### 6.1 프로젝트 목록

#### 6.1.1 Fields2Cover -- 농업용 경로 계획 라이브러리

| 항목 | 내용 |
|------|------|
| **설명** | 농업 차량용 커버리지 경로 계획(CPP) 오픈소스 라이브러리 |
| **핵심 모듈** | 두렁 생성기, 작업열 생성기, 경로 계획기, 궤적 계획기 |
| **ROS2 지원** | fields2cover_ros2 패키지로 Humble/Rolling 지원 |
| **언어** | C++17, Python (SWIG) 인터페이스 |
| **라이선스** | BSD-3 |
| **GitHub** | [Fields2Cover](https://github.com/Fields2Cover/Fields2Cover) |
| **논문** | [arXiv:2210.07838](https://arxiv.org/abs/2210.07838) |
| **적용성** | RS500 농경지 자동 커버리지 경로 생성에 직접 활용 가능 |

#### 6.1.2 Smart DiffBot -- GNSS 기반 야외 내비게이션

| 항목 | 내용 |
|------|------|
| **설명** | GNSS 기반 야외 내비게이션 테스트용 diff drive 로봇 시뮬레이션 |
| **배경** | Saxion 대학 SMART 연구그룹의 RAAK-MKB 자율 농업 내비게이션 프로젝트 |
| **플랫폼** | ROS2 Humble + Gazebo Fortress |
| **GitHub** | [SaxionMechatronics/smart_diffbot](https://github.com/SaxionMechatronics/smart_diffbot) |
| **적용성** | GNSS 기반 내비게이션 아키텍처 참조, Nav2 연동 패턴 참고 |

#### 6.1.3 ROS Agriculture Organization

| 항목 | 내용 |
|------|------|
| **설명** | 농업 로봇 관련 ROS 오픈소스 생태계 |
| **범위** | 다수의 농업 로봇 관련 리포지토리 모음 |
| **GitHub** | [ros-agriculture](https://github.com/ros-agriculture) |
| **적용성** | 농업 로봇 관련 ROS 패키지, 도구, 커뮤니티 자원 |

#### 6.1.4 Tractor Simulator

| 항목 | 내용 |
|------|------|
| **설명** | Olin Robotics Lab의 자율 트랙터 Gazebo 시뮬레이션 |
| **GitHub** | [olinrobotics/tractor_sim](https://github.com/olinrobotics/tractor_sim) |
| **적용성** | 농업 차량 시뮬레이션 구조 참조 (ROS1 기반이므로 마이그레이션 필요) |

#### 6.1.5 LinoRobot2

| 항목 | 내용 |
|------|------|
| **설명** | ROS2 기반 커스텀 로봇 플랫폼 (2WD, 4WD, Mecanum) |
| **특징** | Nav2 통합, Gazebo 시뮬레이션, 실제 하드웨어 지원 |
| **GitHub** | [linorobot/linorobot2](https://github.com/linorobot/linorobot2) |
| **적용성** | ROS2 + Nav2 + Gazebo 연동 아키텍처 참조 모델 |

#### 6.1.6 AutoCarROS2

| 항목 | 내용 |
|------|------|
| **설명** | ROS 2 + Gazebo 기반 자율주행 차량 시뮬레이션 플랫폼 |
| **GitHub** | [winstxnhdw/AutoCarROS2](https://github.com/winstxnhdw/AutoCarROS2) |
| **적용성** | 자율주행 소프트웨어 스택 구조 참조 |

#### 6.1.7 gazebo_continuous_track_example

| 항목 | 내용 |
|------|------|
| **설명** | Gazebo 7/9용 연속 트랙 실시간 시뮬레이션 예제 |
| **GitHub** | [yoshito-n-students/gazebo_continuous_track_example](https://github.com/yoshito-n-students/gazebo_continuous_track_example) |
| **적용성** | 궤도(트랙) 시각화 기법 참조 (Gazebo Classic) |

#### 6.1.8 ArduPilot Gazebo

| 항목 | 내용 |
|------|------|
| **설명** | ArduPilot SITL 컨트롤러와 Gazebo Sim 연동 플러그인/모델 |
| **GitHub** | [ArduPilot/ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo) |
| **적용성** | SITL 아키텍처 및 모터 컨트롤러 시뮬레이션 참조 |

### 6.2 우리 프로젝트에 대한 적용 방안

**우선 도입 권장 프로젝트:**

1. **Fields2Cover** (높음) -- 농경지 커버리지 경로 계획. `fields2cover_ros2` 패키지로 바로 통합 가능
2. **Smart DiffBot** (높음) -- GNSS 기반 야외 내비게이션 아키텍처 참조
3. **LinoRobot2** (중간) -- ROS2 + Nav2 통합 아키텍처 참조 모델
4. **gazebo_continuous_track_example** (낮음) -- 트랙 시각화 개선 참조

---

## 7. 현재 프로젝트 분석 및 개선 로드맵

### 7.1 현재 구성 요약

**분석 대상 파일:**

| 파일 | 경로 |
|------|------|
| 차량 모델 | `src/ad_simulation/models/tracked_vehicle/model.sdf` |
| 월드 파일 | `src/ad_simulation/worlds/agricultural_field.sdf` |
| 브릿지 설정 | `src/ad_simulation/config/bridge_config.yaml` |
| Launch 파일 | `src/ad_simulation/launch/simulation_launch.py` |

### 7.2 종합 개선 사항 매트릭스

| 번호 | 개선 항목 | 현재 상태 | 개선 내용 | 우선도 | 난이도 |
|-----|---------|----------|----------|--------|--------|
| 1 | TrackController 플러그인 | 누락 | 좌/우 트랙 각각 추가 | 필수 | 낮음 |
| 2 | `/clock` 브릿지 | 누락 | bridge_config에 추가 | 필수 | 낮음 |
| 3 | `use_sim_time` | 누락 | launch에서 전역 설정 | 필수 | 낮음 |
| 4 | LiDAR 노이즈 | 없음 | gaussian stddev 0.02 추가 | 높음 | 낮음 |
| 5 | IMU 바이어스 모델 | 없음 | bias_mean/dynamic_bias 추가 | 높음 | 낮음 |
| 6 | 카메라 왜곡/노이즈 | 없음 | Brown 모델 + gaussian 추가 | 중간 | 낮음 |
| 7 | 지형 heightmap | 평탄 plane | heightmap 기반 기복 지형 | 높음 | 중간 |
| 8 | 다중 마찰 영역 | 단일 mu=0.8 | 농경지/농로/논두렁 구분 | 중간 | 중간 |
| 9 | `/tf` 브릿지 | 누락 | 좌표 변환 브릿지 추가 | 높음 | 낮음 |
| 10 | SIL 테스트 프레임워크 | 없음 | launch_testing 기반 자동화 | 높음 | 중간 |
| 11 | CI/CD 시뮬레이션 | 없음 | GitHub Actions headless 테스트 | 중간 | 중간 |
| 12 | camera_info 브릿지 | 누락 | 카메라 캘리브레이션 정보 전달 | 중간 | 낮음 |
| 13 | Fields2Cover 통합 | 없음 | 농경지 경로 계획 라이브러리 | 중간 | 중간 |
| 14 | 장애물 모델 | 마커 4개만 | 돌/고랑/수로 등 추가 | 중간 | 중간 |
| 15 | GPS RTK 프로파일 | 단일 설정 | 시나리오별 노이즈 프로파일 | 낮음 | 낮음 |

### 7.3 즉시 적용 가능한 model.sdf 개선안 (요약)

```xml
<!-- [추가 1] TrackController 플러그인 (좌측) -->
<plugin filename="gz-sim-track-controller-system"
        name="gz::sim::systems::TrackController">
  <link>left_track</link>
  <min_velocity>-1.0</min_velocity>
  <max_velocity>1.0</max_velocity>
</plugin>

<!-- [추가 2] TrackController 플러그인 (우측) -->
<plugin filename="gz-sim-track-controller-system"
        name="gz::sim::systems::TrackController">
  <link>right_track</link>
  <min_velocity>-1.0</min_velocity>
  <max_velocity>1.0</max_velocity>
</plugin>

<!-- [개선 3] LiDAR에 노이즈 추가 -->
<!-- <lidar> 섹션 내 <noise type="gaussian"><mean>0</mean><stddev>0.02</stddev></noise> -->

<!-- [개선 4] IMU에 바이어스 모델 추가 -->
<!-- angular_velocity 각 축에 bias_mean, bias_stddev, dynamic_bias 파라미터 추가 -->

<!-- [개선 5] 카메라에 왜곡 + 노이즈 추가 -->
<!-- <distortion><k1>-0.25</k1>...<center>0.5 0.5</center></distortion> -->
<!-- <noise type="gaussian"><mean>0</mean><stddev>0.007</stddev></noise> -->
```

### 7.4 즉시 적용 가능한 bridge_config.yaml 개선안

```yaml
# [추가] 시뮬레이션 시계 동기화
- ros_topic_name: /clock
  gz_topic_name: /clock
  ros_type_name: rosgraph_msgs/msg/Clock
  gz_type_name: gz.msgs.Clock
  direction: GZ_TO_ROS

# [추가] TF 변환
- ros_topic_name: /tf
  gz_topic_name: /model/tracked_vehicle/pose
  ros_type_name: tf2_msgs/msg/TFMessage
  gz_type_name: gz.msgs.Pose_V
  direction: GZ_TO_ROS

# [추가] 카메라 정보
- ros_topic_name: /sensor/camera/front/camera_info
  gz_topic_name: /world/agricultural_field/model/tracked_vehicle/link/chassis/sensor/front_camera/camera_info
  ros_type_name: sensor_msgs/msg/CameraInfo
  gz_type_name: gz.msgs.CameraInfo
  direction: GZ_TO_ROS
```

### 7.5 개선 로드맵 (권장 순서)

```
Week 1-2: 필수 수정
  ├── TrackController 플러그인 추가
  ├── /clock, /tf 브릿지 추가
  ├── use_sim_time 설정
  └── 센서 노이즈 기본 추가 (LiDAR, IMU bias)

Week 3-4: 환경 고도화
  ├── Heightmap 지형 교체
  ├── 다중 마찰 영역 구현
  ├── 카메라 왜곡/노이즈 추가
  └── 장애물 모델 배치

Week 5-6: 테스트 인프라
  ├── launch_testing 기반 SIL 테스트
  ├── CI/CD headless 시뮬레이션
  └── 성능 메트릭 수집 자동화

Week 7-8: 기능 확장
  ├── Fields2Cover 경로 계획 통합
  ├── GPS 시나리오별 프로파일
  ├── Nav2 통합 기초
  └── 다양한 기상/토양 조건 시나리오
```

---

## 참고 자료 종합

### 공식 문서
- [Gazebo Sim Documentation](https://gazebosim.org/docs/latest/sensors/)
- [Gazebo Physics Engines (gz-sim 9)](https://gazebosim.org/api/sim/9/physics.html)
- [ROS 2 Humble + Gazebo Tutorial](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [ros_gz_bridge Documentation](https://docs.ros.org/en/humble/p/ros_gz_bridge/)
- [Nav2 Sensor Setup (Gazebo)](https://docs.nav2.org/setup_guides/sensors/setup_sensors_gz.html)

### 논문
- M. Pecka, K. Zimmermann, T. Svoboda, "Fast Simulation of Vehicles with Non-deformable Tracks," IROS 2017
- Fields2Cover: Coverage Path Planning for Agricultural Vehicles, arXiv:2210.07838
- GNSS Multipath Signal Emulation in Virtual Urban Canyons, arXiv:2212.04018

### GitHub 저장소
- [gazebosim/gz-sim](https://github.com/gazebosim/gz-sim)
- [gazebosim/ros_gz](https://github.com/gazebosim/ros_gz)
- [gazebosim/gz-sensors](https://github.com/gazebosim/gz-sensors)
- [Fields2Cover/Fields2Cover](https://github.com/Fields2Cover/Fields2Cover)
- [SaxionMechatronics/smart_diffbot](https://github.com/SaxionMechatronics/smart_diffbot)
- [linorobot/linorobot2](https://github.com/linorobot/linorobot2)
- [ros-agriculture](https://github.com/ros-agriculture)
- [MatthewVerbryke/gazebo_terrain](https://github.com/MatthewVerbryke/gazebo_terrain)
- [ArduPilot/ardupilot_gazebo](https://github.com/ArduPilot/ardupilot_gazebo)
