# C13: task.md 자동 업데이트 기능

- 분야: 인프라
- 담당: Claude
- 기간: 2026-02-21
- 상태: 완료

## 배경 및 목적

에이전트가 작업을 수행할 때 task_autonomou.md 파일을 자동으로 갱신하는 기능.

## 변경 사항

### 신규 파일
- `agents/core/task_tracker.py` — 238줄

### 주요 클래스/함수
- `on_task_start()` — 작업 시작 시 호출, IN PROGRESS 섹션에 추가
- `on_task_complete()` — 작업 완료 시 호출, DONE 섹션으로 이동
- `on_task_fail()` — 작업 실패 시 호출
- `add_todo()` — TODO 항목 추가
- `_read_task_md()`, `_write_task_md()` — 파일 읽기/쓰기
- `_parse_sections()`, `_build_content()` — 섹션 파싱/재구성
