# 모듈별 배치 시뮬레이션 스킬 (sim-batch)

> **트리거**: 아래 표현이 나오면 이 스킬을 즉시 실행한다.
> - "모듈 테스트 전체" / "배치 시뮬레이션" / "배치 테스트"
> - "물리 검증 전체" / "전체 모듈 시뮬"
> - "regression 테스트" / "회귀 테스트"

---

## 목적

4개 물리 시뮬레이션 모듈(drivetrain, terrain, dynamics, sensor)을 순차 실행하고, 각 모듈의 결과를 `simulation-report.md` 형식으로 리포트 생성하며, 이전 결과와 비교하여 regression을 감지한다.

---

## 관련 도구

| 파일 | 역할 |
|------|------|
| `tools/module_test_report.py` | 4개 모듈 순차 실행 + 리포트 생성 |
| `tools/scenario_runner.py` | 시나리오 기반 시뮬 실행 |
| `tools/physics_simulator.py` | 물리 시뮬레이션 엔진 |
| `tools/path_follower_sim.py` | 경로 추종 시뮬레이션 |

---

## 실행 순서 (순서 준수 필수)

### STEP 1 — 실행 범위 확인

사용자에게 실행 범위를 확인한다:

| 요청 | 범위 |
|------|------|
| "전체" / "배치" | 4개 모듈 전체 |
| "drivetrain만" | 지정 모듈만 |
| "regression" | 전체 실행 + 이전 결과 비교 |

### STEP 2 — 사전 조건 확인

```bash
# tools 디렉토리 확인
ls /home/gint_pcd/projects/자율주행프로젝트/tools/module_test_report.py

# 최신 빌드 확인 (build 디렉토리 존재 여부)
ls /home/gint_pcd/projects/자율주행프로젝트/build/ 2>/dev/null
```

빌드가 안 되어 있으면 → `build-smart` 스킬 먼저 실행 권고.

### STEP 3 — 배치 실행 (확인 후)

```
🔄 배치 시뮬레이션 실행
   대상: drivetrain, terrain, dynamics, sensor
   예상 소요: ~5분
   결과 저장: results/reports/

   실행할까요? [Y/n]
```

```bash
cd /home/gint_pcd/projects/자율주행프로젝트
python tools/module_test_report.py
```

스크립트가 개별 모듈 실행을 지원하는 경우:
```bash
python tools/module_test_report.py --module drivetrain
```

### STEP 4 — 리포트 생성

각 모듈의 결과를 `simulation-report.md` 스킬의 형식으로 리포트를 생성한다.

저장 경로:
```
results/reports/YYMMDD_{모듈명}_report.md
results/reports/figures/YYMMDD_{모듈명}_*.png
```

### STEP 5 — Regression 비교

이전 리포트와 비교:

```bash
ls -lt /home/gint_pcd/projects/자율주행프로젝트/results/reports/*_report.md | head -20
```

동일 모듈의 이전 리포트에서 핵심 지표를 추출하여 비교:

```
📊 Regression 비교 — {날짜}

| 모듈 | 지표 | 이전값 | 현재값 | 변화 | 판정 |
|------|------|--------|--------|------|------|
| drivetrain | RMSE | 0.023 | 0.025 | +8.7% | ⚠️ |
| terrain | 안정성 | PASS | PASS | - | ✅ |
| dynamics | 에너지오차 | 0.1% | 0.1% | - | ✅ |
| sensor | 노이즈 | 0.05 | 0.04 | -20% | ✅ |
```

### STEP 6 — 종합 보고

```
🏭 배치 시뮬레이션 종합 — {날짜}

| 모듈 | 결과 | 소요시간 | Regression |
|------|------|---------|------------|
| drivetrain | ✅ PASS | 45s | ⚠️ RMSE +8.7% |
| terrain | ✅ PASS | 62s | ✅ |
| dynamics | ✅ PASS | 38s | ✅ |
| sensor | ✅ PASS | 51s | ✅ |

종합: 4/4 PASS, Regression 경고 1건
리포트: results/reports/260313_*.md
```

TASK.md에 결과 한 줄 기록.

---

## 판단 규칙

| 상황 | 행동 |
|------|------|
| 모듈 FAIL | 에러 분석 + 수정 제안 |
| Regression 10% 이상 | ⚠️ 경고 + 변경 원인 추적 |
| Regression 30% 이상 | ❌ 심각 → 최근 커밋 변경사항 확인 |
| 이전 리포트 없음 | 비교 건너뛰기, "첫 베이스라인으로 기록" |
| 스크립트 실행 오류 | 에러 메시지 분석 + 의존성 확인 |

---

## 주의사항

- `module_test_report.py`가 최신 코드와 호환되는지 먼저 확인 (import 에러 가능)
- 물리 시뮬레이션은 CPU 집약적 — 다른 무거운 프로세스(Gazebo 등)와 동시 실행 자제
- 리포트 파일은 git에 커밋 가능 (결과 추적용)
- 그래프/figure 생성 시 matplotlib 필요 — 설치 여부 확인
