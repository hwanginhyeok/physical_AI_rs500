# C22: LiDAR 장애물 감지

- 분야: 알고리즘
- 담당: Claude
- 기간: 2026-02-21
- 상태: 완료

## 배경 및 목적

2D/3D LiDAR 포인트 클라우드에서 장애물을 감지하고 클러스터링하는 알고리즘.

## 변경 사항

### 주요 파일

`src/ad_core/ad_core/lidar_processor.py` — 278줄

### 핵심 클래스

**`Obstacle`** (dataclass):
- `center: Tuple[float, float]` — 중심 좌표
- `radius: float` — 바운딩 반경
- `points: np.ndarray` — 원시 포인트
- `confidence: float` — 감지 신뢰도

**`LidarProcessor`**:
- `process_scan(ranges, angles)` → `List[Obstacle]`
- 거리 필터링 (min/max range)
- 클러스터링 (DBSCAN 기반 또는 간단한 거리 기반)
- 장애물 바운딩 계산

### 설계 결정

- numpy만 의존 (ad_core 순수 알고리즘 원칙)
- ROS2 메시지 변환은 ad_perception의 perception_node에서 처리

## 테스트

- `src/ad_core/test/test_lidar_processor.py` — 127줄, 9개 테스트
