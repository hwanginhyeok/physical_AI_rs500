# C28: Fields2Cover 커버리지 경로 계획

- 분야: 알고리즘
- 담당: Claude
- 기간: 2026-02-21
- 상태: 완료

## 배경 및 목적

농업용 커버리지 경로 계획. 필드 전체를 빠짐없이 커버하는 경로를 생성하는 Fields2Cover 스타일 알고리즘.

## 변경 사항

### 주요 파일

`src/ad_core/ad_core/coverage_planner.py` — 783줄

### 핵심 클래스

- `CoveragePlannerConfig` — 작업 폭, 오버랩, 패턴 설정
- `CoveragePlanner` — 메인 플래너
- `FieldDecomposer` — 필드 분할
- 패턴 유형: Boustrophedon(왕복), Spiral(나선), Lands(구획)

### 주요 기능

- 볼록/비볼록 필드 지원
- 헤드랜드(선회 영역) 자동 생성
- 작업 폭 기반 스트립 분할
- 오버랩 비율 설정
- 경로 최적화 (최소 선회)

### 설계 결정

- numpy만 의존 (순수 알고리즘)
- Spiral 패턴 무한루프 버그 수정됨 (commit f021ebb)

## 테스트

- `src/ad_core/test/test_coverage_planner.py` — 264줄, 24개 테스트
