# C23: YOLO 카메라 객체 탐지

- 분야: 알고리즘
- 담당: Claude
- 기간: 2026-02-21
- 상태: 완료

## 배경 및 목적

카메라 영상에서 YOLO 기반 객체 탐지를 수행하는 모듈. 사람, 차량, 장애물 등의 감지.

## 변경 사항

### 주요 파일

`src/ad_core/ad_core/camera_detector.py` — 184줄

### 핵심 클래스

**`Detection`** (dataclass):
- `class_id: int` — 객체 클래스
- `confidence: float` — 감지 신뢰도
- `bbox: Tuple[float, float, float, float]` — 바운딩 박스

**`CameraDetector`**:
- 객체 탐지 인터페이스
- YOLO 모델 래핑 (실제 추론은 외부 모델 의존)

### 설계 결정

- ad_core 순수 알고리즘 레이어에 인터페이스 정의
- 실제 YOLO 모델 로딩/추론은 ad_perception 노드에서 처리
- numpy만 의존

## 비고

실제 YOLO 모델 가중치 파일은 프로젝트에 포함되지 않음. 배포 시 별도 다운로드 필요.
