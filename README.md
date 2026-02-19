# 자율주행 프로젝트

ROS2 기반 자율주행 시스템 프로젝트.

## 구조

```
src/
├── team_leader/     # 팀장 에이전트 (인지-판단-제어 통합 노드)
└── ad_interfaces/   # 커스텀 메시지/서비스 정의
```

## 패키지 설명

### team_leader
인지(Perception), 판단(Planning), 제어(Control) 파이프라인을 통합 실행하는 메인 자율주행 노드.

- **인지**: 카메라, 라이다, GPS, IMU 센서 데이터 수집 및 처리
- **판단**: 장애물 회피, 차선 유지 등 주행 모드 결정 및 경로 계획
- **제어**: PID 기반 속도/조향 명령 생성

### ad_interfaces
- `VehicleState.msg` - 차량 상태 (위치, 속도, 방향 등)
- `DrivingCommand.msg` - 주행 제어 명령
- `SetMode.srv` - 주행 모드 변경 서비스

## 빌드

```bash
colcon build
source install/setup.bash
```

## 실행

```bash
ros2 launch team_leader team_leader_launch.py
```
