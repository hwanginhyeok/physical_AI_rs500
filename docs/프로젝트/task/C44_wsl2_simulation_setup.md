# C44: 집 PC WSL2 다운로드 및 시뮬레이션 준비

- 분야: 인프라
- 담당: 사용자
- 기간: 2026-02-28 ~
- 상태: 예정

## 배경

C42에서 Foxglove + Gazebo + Nav2 웨이포인트 네비게이션 코드를 구현 완료했으나,
Gazebo Harmonic은 GPU 렌더링이 필요하여 WSL2 환경에서 직접 실행해야 한다.
집 PC(RTX 2060급)에 WSL2 + ROS2 + Gazebo 환경을 구축하여 시뮬레이션을 실행할 수 있도록 준비한다.

## 사전 조건

- Windows 10/11 (빌드 19041 이상)
- NVIDIA GPU + 최신 드라이버 (WSL2 GPU 지원)
- 디스크 여유 공간 ~30GB

## 설치 체크리스트

### Step 1: WSL2 설치

```powershell
# PowerShell (관리자)
wsl --install -d Ubuntu-24.04
# 재부팅 후 사용자 계정 설정
```

- [ ] WSL2 설치 완료
- [ ] Ubuntu 24.04 실행 확인
- [ ] `wsl --version`으로 WSL2 버전 확인

### Step 2: NVIDIA GPU 드라이버 (Windows 측)

- Windows용 NVIDIA 드라이버만 설치하면 WSL2에서 자동 인식
- WSL 내부에 별도 드라이버 설치 불필요

```bash
# WSL2 내부에서 확인
nvidia-smi
```

- [ ] `nvidia-smi`에서 GPU 인식 확인

### Step 3: ROS2 Jazzy 설치

```bash
# 로케일 설정
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# ROS2 저장소 추가
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 설치
sudo apt update
sudo apt install ros-jazzy-desktop
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

- [ ] `ros2 topic list` 정상 출력 확인

### Step 4: Gazebo Harmonic + Nav2 + 관련 패키지

```bash
sudo apt install \
  ros-jazzy-ros-gz \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-robot-localization \
  ros-jazzy-foxglove-bridge \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-nav2-msgs \
  ros-jazzy-visualization-msgs
```

- [ ] `gz sim --version` 확인
- [ ] `ros2 pkg list | grep nav2` 확인

### Step 5: 프로젝트 클론 + 빌드

```bash
mkdir -p ~/projects && cd ~/projects
git clone https://github.com/hwanginhyeok/physical_AI_rs500.git 자율주행프로젝트
cd 자율주행프로젝트

# colcon 빌드
sudo apt install python3-colcon-common-extensions
colcon build --packages-select ad_bringup
source install/setup.bash
```

- [ ] `colcon build` 성공
- [ ] `ros2 pkg list | grep ad_bringup` 확인

### Step 6: Foxglove Bridge 포트 포워딩

```powershell
# PowerShell (관리자) — WSL2 IP 확인 후 포워딩
$wslIp = (wsl hostname -I).Trim().Split(" ")[0]
netsh interface portproxy add v4tov4 listenport=8765 listenaddress=0.0.0.0 connectport=8765 connectaddress=$wslIp
New-NetFirewallRule -DisplayName "Foxglove Bridge" -Direction Inbound -LocalPort 8765 -Protocol TCP -Action Allow
```

- [ ] Windows 브라우저에서 `ws://localhost:8765` 접속 확인

### Step 7: 시뮬레이션 실행 테스트

```bash
# 통합 실행
ros2 launch ad_bringup simulation_launch.py

# 별도 터미널에서 확인
ros2 topic hz /sensor/lidar       # LiDAR 발행 확인
ros2 topic hz /odometry/local     # EKF 출력 확인
ros2 run tf2_tools view_frames    # TF 트리 확인
```

- [ ] Gazebo 월드 로딩 및 SS500 모델 렌더링
- [ ] LiDAR, IMU, Camera 센서 데이터 발행
- [ ] Foxglove Studio에서 3D 패널 시각화

### Step 8: 웨이포인트 네비게이션 E2E 테스트

1. Foxglove Studio → `ws://localhost:8765` 접속
2. 레이아웃 Import (`config/foxglove_layout.json`)
3. 3D 패널에서 맵 클릭 → 웨이포인트 3개 추가
4. Service Call 탭 → Start → 삼각형 경로 순차 방문 확인
5. Cancel 테스트: 이동 중 Cancel → 정지 확인

- [ ] 3점 웨이포인트 순차 방문 성공
- [ ] 장애물 자동 회피 확인
- [ ] 취소 동작 확인

## 결정 사항

| 결정 | 이유 | 검토한 대안 |
|------|------|-------------|
| WSL2 사용 (듀얼부팅 아님) | 기존 Windows 환경 유지하면서 GPU 가속 시뮬레이션 가능 | 듀얼부팅 Ubuntu — 번거로움, Docker — GPU 설정 복잡 |
| Foxglove 웹 버전 사용 | 설치 불필요, 포트 포워딩만으로 접속 | Desktop 앱 — Snap 설치 필요, 기능 동일 |

## 리스크

- **TrackedVehicle 플러그인**: Gazebo Harmonic에서 API 변경 가능 → SDF에 DiffDrive 폴백 주석 포함됨
- **WSL2 GPU 렌더링**: GPU LiDAR + 카메라 + Gazebo GUI 동시 실행 시 RTX 2060 부담 → 센서 Hz 낮추기로 대응
- **EKF 발산**: 시뮬레이션 센서 노이즈와 EKF 공분산 불일치 → dual_ekf.yaml 튜닝 필요

## 관련 파일

| 파일 | 용도 |
|------|------|
| `src/ad_bringup/launch/simulation_launch.py` | 통합 시뮬레이션 런치 |
| `src/ad_bringup/launch/waypoint_nav_demo_launch.py` | 데모 래퍼 런치 |
| `src/ad_bringup/config/foxglove_layout.json` | Foxglove 3탭 레이아웃 |
| `src/ad_bringup/config/bridge_config.yaml` | ros_gz_bridge 토픽 매핑 |
| `src/ad_bringup/config/dual_ekf.yaml` | EKF 파라미터 |
| `src/ad_bringup/config/nav2_params.yaml` | Nav2 파라미터 |
| `src/ad_bringup/models/ss500/model.sdf` | SS500 Gazebo 모델 |
| `src/ad_bringup/urdf/ss500.urdf.xacro` | SS500 URDF (TF용) |
| `src/ad_bringup/worlds/flat_field.sdf` | 테스트 월드 |
| `src/ad_bringup/maps/empty_map.yaml` | Nav2 빈 맵 |

## 미결 이슈

- [ ] TrackedVehicle 플러그인 호환성 확인 (안 되면 DiffDrive 폴백)
- [ ] Gazebo GUI 없이 headless 모드 실행 시 성능 비교
- [ ] EKF 파라미터 시뮬레이션 환경 튜닝
- [ ] GitHub 기본 브랜치 master→main 변경 (Settings → Default branch)
