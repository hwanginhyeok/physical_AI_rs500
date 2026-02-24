# C37: ad_bringup 통합 런처

- 분야: 인프라
- 담당: Claude
- 기간: 2026-02-23
- 상태: 완료

## 배경 및 목적

전체 시스템 및 네비게이션 스택을 한 번에 실행하는 ROS2 런치 파일 패키지.

## 변경 사항

### 런치 파일

**`src/ad_bringup/launch/full_system_launch.py`** — 61줄:
- 전체 시스템 통합 실행
- perception, planning, control, can_bridge 노드 일괄 기동

**`src/ad_bringup/launch/navigation_launch.py`** — 197줄:
- Nav2 네비게이션 스택 실행
- robot_localization (Dual-EKF) 설정 포함
- SmacPlannerLattice + MPPI 컨트롤러

### 설정 파일

| 파일 | 줄수 | 역할 |
|------|------|------|
| config/dual_ekf.yaml | 190 | Dual-EKF 센서 퓨전 (C19) |
| config/nav2_params.yaml | 489 | Nav2 파라미터 (C20, C21) |
| config/lio_sam_params.yaml | — | LIO-SAM SLAM (C29) |

### 패키지 설정

```python
# setup.py (26줄)
packages = []  # Python 패키지 없음, 데이터 전용
data_files: config/*.yaml, launch/*.py
# Entry points 없음
```

### 설계 결정

- 데이터 전용 패키지 (Python 코드 없음)
- 모든 설정을 config/ 디렉토리에 집중
- full_system vs navigation 두 가지 런치 프로파일 제공
