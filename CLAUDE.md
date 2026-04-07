# 자율주행 RS500 — 프로젝트 규칙

> ROS2 기반 과수원 자율주행 궤도 차량(RS500) 소프트웨어 프로젝트.
> 인지(카메라)→판단(행 추종)→제어(CAN/모터) 파이프라인.

## Tasks

- [CURRENT_TASK.md](CURRENT_TASK.md) | [PREPARED_TASK.md](PREPARED_TASK.md) | [FINISHED_TASK.md](FINISHED_TASK.md)
- [DIFFICULTY.md](DIFFICULTY.md) — 어려웠던 문제 & 노하우

---

## 세션 시작 프로토콜

1. **Task 확인** — `CURRENT_TASK.md` 점검 (진행 중, blocked, 블로커)
2. **미팅** — 사용자에게 현재 상황 브리핑
3. **방향성 논의** — 오늘 작업의 우선순위와 방향을 함께 결정

---

## 프로젝트 구조

```
physical_AI_rs500/
├── src/
│   ├── ad_bringup/       # 런치 파일, 파라미터 YAML
│   ├── ad_can_bridge/    # CAN 통신 (MD2K 모터 컨트롤러)
│   ├── ad_control/       # 모터 제어, 안전 정지
│   ├── ad_core/          # 공통 유틸, 상수
│   ├── ad_interfaces/    # ROS2 커스텀 메시지/서비스
│   ├── ad_perception/    # 카메라 인지 (crop_row 검출)
│   └── ad_planning/      # 경로 판단, 행 추종
├── TASK.md               # 태스크 인덱스
├── CURRENT_TASK.md       # 진행 중
├── PREPARED_TASK.md      # 예정 + TODO
├── FINISHED_TASK.md      # 완료 (당월)
├── TASK_ARCHIVE/         # 월별 완료 아카이브
├── DIFFICULTY.md         # 어려웠던 문제 & 노하우
├── docs/프로젝트/
│   └── task/             # 태스크 상세 로그
└── .claude/
    ├── rules/            # 세션 규칙 (자동 로딩)
    └── skills/           # 작업별 스킬 (필요 시 로딩)
```

## Commands

```bash
# 빌드
colcon build

# 테스트 전체 실행
colcon test && colcon test-result --verbose

# 특정 패키지 테스트
colcon test --packages-select ad_control
PYTHONPATH="src/ad_control:$PYTHONPATH" python3 -m pytest src/ad_control/test/ -v

# 런치
ros2 launch ad_bringup rs500.launch.py
```

## Tech Stack

- **ROS2** (Humble) — 노드 간 통신, 런치, 파라미터
- **Python 3** — 모든 노드 구현
- **CAN** — MD2K 모터 컨트롤러 제어
- **GitHub Actions** — CI 파이프라인 (colcon build + test)

---

## 핵심 원칙

- **비판적 사고 파트너** — 새 기능/아키텍처/기술 선택은 먼저 질문하고 논의. 버그 수정/합의된 구현은 바로 실행. 상세: `.claude/rules/critical-thinking.md`
- **TASK 실시간 관리** — `CURRENT_TASK.md` / `PREPARED_TASK.md` / `FINISHED_TASK.md` 3파일 체제. 착수/완료/발견 시 즉시 갱신

---

## Rules (자동 로딩)

| 파일 | 내용 |
|------|------|
| `critical-thinking.md` | 바로 실행 vs 논의 기준, 질문 패턴 |

## Skills (작업 시 참조)

| 파일 | 트리거 |
|------|--------|
| `simulation-report.md` | 시뮬레이션 테스트 리포트 작성 시 |
