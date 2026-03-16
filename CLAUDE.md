# 프로젝트 규칙

## 세션 시작 프로토콜

세션이 시작되면 반드시 다음 순서를 따른다:

1. **규칙 로딩** — `.claude/rules/` 하위 모든 `.md` 파일을 읽는다 (건너뛰지 않는다)
2. **Task 확인** — `docs/프로젝트/TASK.md` 점검 (진행 중, 지연, 블로커)
3. **미팅** — 사용자에게 현재 상황 브리핑
4. **방향성 논의** — 오늘 작업의 우선순위와 방향을 함께 결정

이 프로토콜을 생략하지 않는다. 1번은 다른 어떤 행동보다 먼저 수행한다.

---

## 핵심 원칙

- **비판적 사고 파트너** — 새 기능·아키텍처·기술 선택은 먼저 질문하고 논의. 버그 수정·합의된 구현은 바로 실행.
- **TASK 실시간 관리** — `docs/프로젝트/TASK.md` 단일 파일에서 통합 관리. 착수/완료/발견 시 즉시 갱신.
- **아키텍처 결정 기록** — 구조·기술 선택 결정은 `docs/arch/` ADR에 기록. 인덱스: `docs/arch/INDEX.md`.

---

## 매뉴얼 체계

### Rules (`.claude/rules/`) — 세션 시작 시 전체 로딩

모든 세션에 적용되는 행동 규칙. 세션 시작 프로토콜 1번에서 강제 로딩.

### Skills (`.claude/skills/`) — 작업 시점에 선택 로딩

특정 작업 수행 시 필요한 절차/체크리스트. 해당 작업 착수 전에 읽는다.
스킬 파일에 `트리거` 섹션이 있으면 해당 조건에서 반드시 로딩한다.

| 스킬 파일 | 트리거 표현 | 역할 |
|-----------|-------------|------|
| `task-wrap.md` | "task 정리해줘", "브리핑해줘", "마무리해줘", "문서화해줘" | TASK.md 갱신 → 문서화(task 로그/ADR) → 빌드 확인 → 브리핑 |
| `simulation-report.md` | 모듈 시뮬레이션 결과 정리 시 | 표준 리포트 형식 작성 |
| `task-sync.md` | "TASK 동기화", "이슈 동기화", "업무 동기화" | PLUVA Robotics 팀 TASK 이슈 관리 DB → 개인 통합 업무 관리 DB 동기화 |
| `build-smart.md` | "빌드해줘", "빌드", "build" | git diff → 변경 패키지 추출 → 의존성 전파 → 선택 빌드 |
| `launch-preflight.md` | "런치 검증", "시뮬 체크", "프리플라이트" | YAML 문법·필수키·TF·Launch 참조 사전 검증 |
| `dds-troubleshoot.md` | "DDS 초기화", "SharedMemory 충돌", "activate 실패" | SharedMemory 정리 → lifecycle 상태 → 토픽 체인 검증 |
| `test-runner.md` | "테스트 돌려줘", "pytest", "전체 테스트" | colcon test → 결과 수집 → 실패 분석 → TASK.md 기록 |
| `rosbag-workflow.md` | "로스백 녹화", "rosbag 분석", "비교 분석" | Rosbag 녹화/분석/sim-vs-real 비교 |
| `sim-batch.md` | "배치 시뮬레이션", "모듈 테스트 전체", "regression 테스트" | 4개 모듈 순차 실행 → 리포트 → regression 비교 |
| `param-tuning-log.md` | "파라미터 튜닝", "nav2 조정", "costmap 조정" | before/after 스냅샷 → 사유 → 테스트 결과 → ADR 판단 |
| `tf-check.md` | "TF 확인", "TF 트리 검증", URDF 변경 시 | xacro 정적 분석 → ARCH-001 대조 → 런타임 TF → 문서 동기화 |
