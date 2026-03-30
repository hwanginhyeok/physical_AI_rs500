# 야간작업 브리핑 - 2026-03-30

## 작업 요약

C52 카메라 온리 과수원 행 추종 (Reactive) -- ROS 배선 완성

## 변경 내역

### 커밋 1: 인지 노드 crop_row 퍼블리셔 + 파라미터 버그 수정
- `perception_node.py`: declare_parameter -> get_parameter (중복 선언 방지)
- `perception_node.py`: /perception/crop_row Float32MultiArray 퍼블리셔 + 10Hz 타이머
- `perception_params.yaml`: crop_row 파라미터 블록 추가 (enabled, orchard_type, process_every_n_frames, no_detect_limit)
- `test_perception_module.py`: process_every_n_frames=1 추가로 첫 호출 감지 보장

### 커밋 2: 판단 노드 crop_row 구독 + cmd_vel 퍼블리셔
- `planning_node.py`: /perception/crop_row 구독 + _crop_row_callback
- `planning_node.py`: /cmd_vel Twist 퍼블리셔 (target_speed/target_steering 발행)
- end_detected=True && row_detected=False 시 정지 신호 보존 로직

### 커밋 3: Foxglove 검증용 경량 런치 파일
- `crop_row_test_launch.py`: Gazebo + 카메라 브릿지 + perception_node + foxglove_bridge
- Nav2/EKF 없이 crop_row 감지 토픽 검증 가능

## 데이터 흐름 (완성된 파이프라인)

```
카메라 → perception_node (crop_row_detector)
    → /perception/crop_row [Float32MultiArray]
    → planning_node (_crop_row_callback)
    → /cmd_vel [Twist]
    → can_bridge → 모터
```

## 테스트 결과

- **346건 통과** / 9건 실패 (기존 실패, 오늘 변경과 무관)
- 실패 항목: `test_vehicle_dynamics.py` -- VehicleDynamics 안전 상태 관련 미구현 메서드
  - test_default_values (mass 800 vs 200)
  - TestVehicleDynamicsSafety 8건 (is_operational, set_emergency_stop 등 미구현)

## 코드 리뷰

- CRITICAL 이슈: 없음
- Float32MultiArray 빈 데이터 가드 절 있음
- 타입 변환 처리 완료
- 기존 코드 패턴(한국어 주석, docstring)과 일관

## 미결 사항

- T05 카메라 피드 타임아웃 안전 정지: 예정 상태 유지
- test_vehicle_dynamics.py 9건 실패: C50에서 파라미터 변경 후 미갱신 상태, 별도 작업 필요
- 실차 검증: Foxglove 런치 파일로 crop_row 토픽 확인 후 튜닝 필요

## WTF 지표

- 파일 수정 횟수: 0 (기존 변경사항 커밋만 수행)
- 재시도: 0
- 롤백: 0
