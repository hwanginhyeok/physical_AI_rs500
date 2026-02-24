# C10: 선행연구 — Gazebo 시뮬레이션 사례

- 분야: 연구
- 담당: Claude
- 기간: 2026-02-21
- 상태: 완료

## 배경 및 목적

Gazebo gz-sim에서 궤도차량 시뮬레이션 구축 사례 및 베스트 프랙티스 조사.

## 산출물

`docs/research/gazebo_tracked_vehicle_simulation_research.md` — 903줄

### 주요 내용
- TrackedVehicle 플러그인 (접촉면 운동 모델)
- Heightmap 기반 지형 모델링
- 센서 노이즈 모델 (LiDAR, 카메라 왜곡, IMU 바이어스)
- ros_gz_bridge 토픽 매핑 베스트 프랙티스
- SIL/HIL 테스트 프레임워크 설계
- 오픈소스 궤도차량 시뮬레이션 프로젝트 현황

## 활용

C1~C6 시뮬레이션 월드 구축 및 C24(다중 마찰 영역), C25(SIL 테스트) 구현에 참조됨.
