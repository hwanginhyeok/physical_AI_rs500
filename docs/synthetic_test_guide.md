# 합성 이미지 기반 Perception 테스트 가이드

> Gazebo/GPU 없이 crop_row 파이프라인을 검증하는 방법.

## 개요

합성 이미지 퍼블리셔가 가짜 과수원 이미지를 `/sensor/camera/front`에 발행하면,
perception_node가 이를 처리하여 `/perception/crop_row`에 결과를 내보낸다.
Foxglove Studio에서 실시간으로 시각 확인 가능.

```
[synthetic_image_publisher]  →  /sensor/camera/front (Image, 5Hz)
                                       ↓
                              [perception_node]
                                       ↓
                              /perception/crop_row (Float32MultiArray, 10Hz)
                                       ↓
                              [Foxglove Studio]  ws://localhost:8765
```

## 빠른 시작

```bash
# 1. 빌드
colcon build --packages-select ad_perception ad_bringup

# 2. 소스
source install/setup.bash

# 3. 실행
ros2 launch ad_bringup synthetic_test_launch.py
```

Foxglove Studio → Open connection → `ws://localhost:8765`

## 시나리오

| 시나리오 | 설명 | 기대 결과 |
|----------|------|-----------|
| `center` | 두 행 사이 중앙 | row=2, offset≈0 |
| `offset_left` | 로봇이 왼쪽으로 치우침 | row=2, offset>0 (우측 조향) |
| `offset_right` | 로봇이 오른쪽으로 치우침 | row=2, offset<0 (좌측 조향) |
| `no_rows` | 빈 필드 (나무 없음) | row=0 |
| `end_of_row` | 행 끝 (나무 사라짐) | row=0 → end_detected 트리거 |
| `single_row` | 행 1개만 | row=1 |

## 실행 옵션

```bash
# 특정 시나리오
ros2 launch ad_bringup synthetic_test_launch.py scenario:=offset_left

# 자동 순환 (5초마다 시나리오 전환)
ros2 launch ad_bringup synthetic_test_launch.py auto_cycle:=true

# 순환 간격 변경 (10초)
ros2 launch ad_bringup synthetic_test_launch.py auto_cycle:=true cycle_interval:=10.0

# 발행 주기 변경 (10Hz)
ros2 launch ad_bringup synthetic_test_launch.py publish_rate:=10.0

# Foxglove 없이 (토픽만 확인)
ros2 launch ad_bringup synthetic_test_launch.py use_foxglove:=false
```

## 노드 단독 실행

런치 파일 없이 노드만 따로 실행할 수도 있다.

```bash
# 퍼블리셔만
ros2 run ad_perception synthetic_image_publisher --ros-args -p scenario:=center

# perception_node 별도 실행
ros2 run ad_perception perception_node --ros-args --params-file src/ad_perception/config/perception_params.yaml
```

## 토픽 확인 (CLI)

```bash
# 카메라 이미지 발행 확인
ros2 topic hz /sensor/camera/front

# crop_row 결과 실시간 확인
ros2 topic echo /perception/crop_row
# data: [row_detected, steering_offset, heading_error, end_detected]
# 예: [1.0, 0.43, 0.0, 0.0]  → 행 감지, 우측으로 0.43 조향 필요
```

## crop_row 메시지 해석

`/perception/crop_row` (Float32MultiArray.data):

| 인덱스 | 필드 | 범위 | 설명 |
|--------|------|------|------|
| 0 | row_detected | 0.0 / 1.0 | 행 감지 여부 |
| 1 | steering_offset | -1.0 ~ +1.0 | 좌(-) / 우(+) 조향 오프셋 |
| 2 | heading_error | deg | 행 기준 방향 오차 |
| 3 | end_detected | 0.0 / 1.0 | 행 끝 감지 (2초 미감지) |

## 테스트

```bash
# 합성 이미지 + crop_row 통합 테스트 (28건)
source /opt/ros/jazzy/setup.bash
PYTHONPATH="src/ad_perception:src/ad_core:$PYTHONPATH" \
  python3 -m pytest src/ad_perception/test/test_synthetic_image.py -v
```

## Foxglove 레이아웃 설정

Foxglove에서 아래 패널을 추가하면 편하다:

1. **Image** 패널 → 토픽: `/sensor/camera/front` — 합성 이미지 확인
2. **Plot** 패널 → `/perception/crop_row.data[0]` (row_detected), `.data[1]` (steering_offset)
3. **Log** 패널 → 필터: `perception_node` — 행 감지 로그

## 파일 위치

| 파일 | 경로 |
|------|------|
| 합성 이미지 퍼블리셔 | `src/ad_perception/ad_perception/synthetic_image_publisher.py` |
| 경량 런치 파일 | `src/ad_bringup/launch/synthetic_test_launch.py` |
| 테스트 | `src/ad_perception/test/test_synthetic_image.py` |
| perception 파라미터 | `src/ad_perception/config/perception_params.yaml` |
