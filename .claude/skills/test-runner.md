# 통합 테스트 실행 스킬 (test-runner)

> **트리거**: 아래 표현이 나오면 이 스킬을 즉시 실행한다.
> - "테스트 돌려줘" / "테스트 실행" / "pytest"
> - "전체 테스트" / "단위 테스트" / "test"
> - 코드 수정 후 테스트 필요 시 자동 제안

---

## 목적

테스트 실행 → 결과 수집 → 실패 분석 → TASK.md 기록을 하나의 흐름으로 통합한다.

---

## 테스트 디렉토리

| 패키지 | 테스트 경로 | 비고 |
|--------|------------|------|
| `ad_core` | `src/ad_core/test/` | 공통 유틸 테스트 |
| `ad_control` | `src/ad_control/test/` | 제어기 테스트 |
| `ad_perception` | `src/ad_perception/test/` | 인지 모듈 테스트 |
| `ad_planning` | `src/ad_planning/test/` | 경로 계획 테스트 |
| `ad_can_bridge` | `src/ad_can_bridge/test/` | CAN 통신 테스트 |

---

## 실행 순서 (순서 준수 필수)

### STEP 1 — 테스트 범위 결정

사용자 요청에 따라 범위를 결정한다:

| 요청 | 범위 |
|------|------|
| "전체 테스트" | 모든 패키지 |
| "ad_control 테스트" | 지정 패키지만 |
| "변경된 패키지 테스트" | git diff로 변경 감지된 패키지 |
| "pytest" (구체적 파일 지정 없음) | 변경된 패키지 우선, 없으면 전체 |

변경 패키지 탐지는 `build-smart` 스킬의 STEP 1~2와 동일한 방법 사용.

### STEP 2 — colcon test 실행 (확인 후)

사용자에게 테스트 대상을 보여주고 확인한다:

```
🧪 테스트 대상: ad_control, ad_core
   실행할 명령: colcon test --packages-select ad_control ad_core
   실행할까요? [Y/n]
```

```bash
cd /home/gint_pcd/projects/자율주행프로젝트
colcon test --packages-select {패키지들} --event-handlers console_cohesion+
```

전체 테스트:
```bash
colcon test --event-handlers console_cohesion+
```

### STEP 3 — 결과 수집

```bash
colcon test-result --verbose
```

출력에서 추출:
- 총 테스트 수
- 성공/실패/에러/건너뛰기 수
- 실패한 테스트 이름 + 에러 메시지

### STEP 4 — 실패 분석

실패한 테스트가 있으면:
1. 해당 테스트 파일을 읽고 실패 원인 분석
2. 관련 소스 코드 확인
3. 수정 제안 (자동 수정은 하지 않음)

### STEP 5 — 결과 보고

```
🧪 테스트 결과 — {날짜}

| 패키지 | 총 | ✅ | ❌ | ⏭️ | 결과 |
|--------|-----|-----|-----|-----|------|
| ad_control | 12 | 11 | 1 | 0 | FAIL |
| ad_core | 8 | 8 | 0 | 0 | PASS |

❌ 실패 테스트:
- ad_control::test_pid_controller — AssertionError: expected 0.5, got 0.48
  원인 추정: PID 게인 변경 후 기대값 미갱신
  수정 제안: test 기대값을 0.48로 업데이트 (또는 허용 오차 확대)
```

### STEP 6 — TASK.md 기록

테스트 결과를 TASK.md에 기록한다:
- 전체 PASS → "현재 진행 중" 항목에 "✅ 테스트 통과 (N건)" 한 줄
- FAIL 존재 → 실패 내용 요약 + 수정 필요 여부 기록

---

## 판단 규칙

| 상황 | 행동 |
|------|------|
| 전체 PASS | 성공 요약 한 줄 |
| FAIL 1~2건 | 실패 분석 + 수정 제안 |
| FAIL 다수 | 패키지별 요약 + 공통 원인 분석 |
| 빌드 에러로 테스트 불가 | `build-smart` 스킬 실행 권고 |
| 테스트 파일 없음 | "테스트 미작성" 알림 |

---

## 주의사항

- `colcon test`는 `colcon build` 이후에만 유효하다. 빌드 미완료 시 `build-smart` 먼저 실행
- ROS2 통합 테스트(launch_testing)는 환경 의존성이 있어 실패할 수 있음 — 단위 테스트와 구분
- 테스트 결과 로그는 `build/*/test_results/`에 XML로 저장됨
- 커버리지 리포트는 `pytest-cov` 설치 시에만 가능 (`--pytest-args --cov`)
