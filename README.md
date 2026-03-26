# RS500 자율주행 시뮬레이션

[![Tests](https://github.com/<OWNER>/<REPO>/actions/workflows/test.yml/badge.svg)](https://github.com/<OWNER>/<REPO>/actions/workflows/test.yml)

SS500 Speed Sprayer 기반 궤도형 자율주행 차량의 물리 시뮬레이션 및 ROS2 제어 시스템.

**차량**: SS500 (궤도식, DB130-48 BLDC 3kW×2, MD2K 듀얼채널 드라이버, 48V LiFePO4)
**시뮬**: Level 2 물리 시뮬레이션 (뉴턴 역학 + Bekker 지면 모델 + 센서 노이즈)

## 아키텍처

```
src/
├── ad_core/          # 순수 Python 알고리즘 (ROS2 비의존)
│   ├── skid_steer_model.py      — ICR 기반 스키드 스티어 운동학
│   ├── vehicle_dynamics.py      — 뉴턴 역학 동역학 (질량/관성/하중전이/경사)
│   ├── drivetrain_model.py      — 모터+감속기 1차 지연 + MD2K SS/SD 가감속
│   ├── motor_protection.py      — MD2K 보호 기능 (STALL/과전류/과온도/과전압)
│   ├── md2k_codec.py            — MD2K CAN 코덱 (Tx/Rx encode/decode)
│   ├── track_terrain_interaction.py — Bekker 이론 지면 상호작용
│   ├── sensor_noise_model.py    — GPS/IMU 센서 노이즈
│   ├── sensor_fusion.py         — Dual-EKF 센서 퓨전
│   ├── pure_pursuit.py          — Pure Pursuit 경로 추종
│   ├── coverage_planner.py      — Fields2Cover 커버리지 경로 계획
│   ├── crop_row_detector.py     — 과수원 작물 행 인식 (Classical CV)
│   ├── lidar_processor.py       — LiDAR 장애물 감지
│   ├── camera_detector.py       — YOLO 카메라 객체 탐지
│   └── datatypes.py             — 공용 데이터 타입
│
├── ad_can_bridge/    # SS500 CAN 통신 코덱 + ROS2 브릿지
├── ad_control/       # ROS2 제어 노드 (cmd_vel → 트랙 속도)
├── ad_planning/      # ROS2 경로 계획 노드
├── ad_perception/    # ROS2 인지 노드 (센서 퓨전, 장애물 감지)
├── ad_bringup/       # 통합 런처 (Gazebo + Nav2 + Foxglove)
└── ad_interfaces/    # 커스텀 ROS2 메시지/서비스

tools/
├── physics_simulator.py    — 인터랙티브 L1/L2 물리 시뮬레이터 (matplotlib)
├── scenario_runner.py      — 배치 시나리오 러너 (3시나리오 × 5지형)
├── module_test_report.py   — 모듈별 파라미터 감도 분석 리포트
└── c50_simulation.py       — MD2K 기능 검증 시뮬레이션
```

## 테스트

### 순수 Python 테스트 (CI)

```bash
# ad_core (336건)
cd src/ad_core && python3 -m pytest test/ -v

# ad_can_bridge (24건)
PYTHONPATH=src/ad_can_bridge:src/ad_core python3 -m pytest src/ad_can_bridge/test/ -v
```

### ROS2 테스트 (로컬, ROS2 환경 필요)

```bash
source /opt/ros/jazzy/setup.bash
colcon build
colcon test
```

## 시뮬레이션 실행

### Gazebo + Nav2 (ROS2 환경)

```bash
source install/setup.bash
ros2 launch ad_bringup simulation_launch.py
```

### 배치 시나리오 (ROS2 불필요)

```bash
# 3시나리오 × 5지형 자동 실행
MPLBACKEND=Agg python3 tools/scenario_runner.py --no-plot --save results/

# MD2K 기능 검증
python3 tools/c50_simulation.py
```

## 최근 주요 변경

| 태스크 | 내용 | 날짜 |
|--------|------|------|
| C50 | MD2K 모터 컨트롤러 반영, SS/SD 가감속, 보호기능, CAN 코덱, 안전상태 | 2026-03-25 |
| C49 | RS500 시뮬 vs SS500 실차 36항목 교차 검증 리포트 | 2026-03-25 |
| C48 | HIH-2 실차 데이터 기반 시뮬레이션 파라미터 교정 | 2026-03-23 |
| C47 | Foxglove + YOLO + MCAP 녹화 | 2026-03-22 |
| C46 | 과수원 작물 행 인식 (Classical CV Phase 1) | 2026-03-22 |
| C38 | Level 2 물리 시뮬레이션 (뉴턴 역학 + Bekker 지면 모델) | 2026-02-24 |

전체 이력: [`docs/프로젝트/TASK.md`](docs/프로젝트/TASK.md)

## 환경

- **ROS2**: Jazzy (Ubuntu 24.04)
- **시뮬레이터**: Gazebo Harmonic 8.10.0
- **시각화**: Foxglove Studio + RViz2
- **Python**: 3.11 / 3.12
