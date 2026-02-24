# C34: ad_core 순수 알고리즘 분리

- 분야: 리팩토링
- 담당: Claude
- 기간: 2026-02-23
- 상태: 완료
- 커밋: 6c4ff96

## 배경 및 목적

모든 알고리즘을 ROS2 독립적인 순수 Python 패키지(`ad_core`)로 분리. 이식성(adtpc 등) 확보 및 테스트 용이성 향상.

## 변경 사항

### ad_core 패키지 구조

`src/ad_core/ad_core/` 하위 모듈 (numpy만 의존):

| 파일 | 줄수 | 역할 | 원래 태스크 |
|------|------|------|-------------|
| datatypes.py | — | Pose2D, VehicleCommand, VehicleFeedback | — |
| skid_steer_model.py | 834 | 스키드 스티어 운동학 | C18, C27 |
| pure_pursuit.py | 1002 | Pure Pursuit + 슬립 보상 | C17, C26 |
| coverage_planner.py | 783 | 커버리지 경로 계획 | C28 |
| terrain_classifier.py | — | 지면 분류 | — |
| lidar_processor.py | 278 | LiDAR 장애물 감지 | C22 |
| camera_detector.py | 184 | 카메라 객체 탐지 | C23 |
| sensor_fusion.py | 540 | Camera+LiDAR 융합 | C31 |
| semantic_segmenter.py | 1127 | 시맨틱 세그멘테이션 | C30 |
| can_interface.py | — | VehicleInterface ABC | C36 |

### 삭제된 패키지

- `ad_simulation/` — Gazebo 시뮬레이션 패키지 전체 삭제 (C1~C6, C15, C24, C25 관련 파일)
- `team_leader/` — 이전 에이전트 시스템 삭제

### 설계 결정

- Layer 0 (ad_core): ROS2 완전 독립, numpy만 의존
- Layer 1 (ad_interfaces): ROS2 메시지 정의
- Layer 2 (나머지): ROS2 노드, ad_core를 import하여 사용
- `setup.py`에 `install_requires=['setuptools', 'numpy']`만 명시

### 빌드 설정

```python
# ad_core/setup.py (19줄)
packages = find_packages(exclude=['test'])
install_requires = ['setuptools', 'numpy']
# console_scripts 없음 (라이브러리 전용)
```
