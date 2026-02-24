# C16: ROS2 프로젝트 초기 구조 생성

- 분야: 인프라
- 담당: Claude
- 기간: 2026-02-21 (추정)
- 상태: 완료 (C34~C37에서 대폭 재구조화됨)

## 배경 및 목적

ROS2 Jazzy 기반 자율주행 프로젝트의 초기 패키지 구조 및 빌드 시스템 생성.

## 현재 상태

초기 구조는 C34~C37 재구조화를 거쳐 7개 패키지 구조로 변경됨. 현재 package.xml 확인 결과:

| 패키지 | 버전 | 빌드 타입 | 라이선스 | 주요 의존성 |
|--------|------|-----------|----------|-------------|
| ad_core | 0.1.0 | ament_python | Apache-2.0 | numpy (ROS2 독립) |
| ad_interfaces | 0.1.0 | ament_cmake | Apache-2.0 | std_msgs, rosidl |
| ad_perception | 0.1.0 | ament_python | MIT | rclpy, sensor_msgs, ad_core |
| ad_planning | 0.1.0 | ament_python | MIT | rclpy, nav_msgs, ad_core, ad_interfaces |
| ad_control | 0.1.0 | ament_python | MIT | rclpy, ad_core, ad_interfaces |
| ad_can_bridge | 0.1.0 | ament_python | MIT | rclpy, ad_core |
| ad_bringup | 0.1.0 | ament_python | MIT | robot_localization, nav2_bringup |

## 설계 결정

- ad_core만 Apache-2.0 (adtpc 이식 대상, ROS2 독립)
- ad_interfaces만 ament_cmake (rosidl 메시지 생성 필요)
- 나머지는 ament_python + MIT

## 비고

초기 C16에서 만든 원래 구조의 상세는 재구조화로 인해 불명. 현재 구조는 C34~C37의 결과물.
