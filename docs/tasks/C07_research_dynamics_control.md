# C7: 선행연구 — 궤도차량 동역학/제어

- 분야: 연구
- 담당: Claude
- 기간: 2026-02-21
- 상태: 완료

## 배경 및 목적

궤도차량 자율주행에 필요한 동역학 모델링, 슬립 모델, 경로 추종 제어 기법의 문헌 조사.

## 산출물

`docs/literature_review_dynamics_control.md` — 594줄

### 주요 내용
- 궤도차량 운동학 모델링 (ICR 기반)
- 슬립 모델링 (Bekker-Wong 지반역학)
- 차동 조향 메커니즘
- 경로 추종 제어: Pure Pursuit, Stanley, MPC
- ROS2 + Gazebo 시뮬레이션 선행 사례

## 활용

이 조사 결과는 이후 C18(스키드 스티어 모델), C26(슬립 보상 Pure Pursuit), C38(차량 동역학) 구현의 이론적 근거로 사용됨.
