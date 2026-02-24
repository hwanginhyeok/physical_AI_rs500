# C35: ROS2 노드 패키지 분리

- 분야: 리팩토링
- 담당: Claude
- 기간: 2026-02-23
- 상태: 완료
- 커밋: 6c4ff96

## 배경 및 목적

ROS2 노드를 기능별 패키지(ad_perception, ad_planning, ad_control)로 분리. 각 패키지가 독립적으로 빌드/배포 가능.

## 변경 사항

### ad_perception (인지)

| 파일 | 줄수 | 역할 |
|------|------|------|
| perception_node.py | 341 | LiDAR/카메라 인지 ROS2 노드 |
| localization_node.py | 802 | GPS/SLAM 위치 추정 노드 |

Entry points: `perception_node`, `localization_node`
의존: rclpy, sensor_msgs, geometry_msgs, nav_msgs, ad_core

### ad_planning (판단)

| 파일 | 줄수 | 역할 |
|------|------|------|
| planning_node.py | 408 | 경로 계획 ROS2 노드 |

Entry point: `planning_node`
의존: rclpy, geometry_msgs, nav_msgs, ad_core, ad_interfaces

### ad_control (제어)

| 파일 | 줄수 | 역할 |
|------|------|------|
| control_node.py | 399 | 제어 명령 생성 ROS2 노드 |

Entry point: `control_node`
의존: rclpy, geometry_msgs, ad_core, ad_interfaces

### 설계 결정

- Perception → Planning → Control 파이프라인 각각 독립 패키지
- ad_core를 import하여 알고리즘 호출 (로직 중복 없음)
- 각 노드는 ROS2 토픽/서비스만 담당, 연산은 ad_core에 위임
