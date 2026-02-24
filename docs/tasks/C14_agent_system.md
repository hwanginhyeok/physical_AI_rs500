# C14: AI 에이전트 시스템 (core/modeling/research/leader)

- 분야: 인프라
- 담당: Claude
- 기간: 2026-02-21
- 상태: 완료

## 배경 및 목적

프로젝트 관리, 모델링, 연구를 자동화하는 멀티에이전트 시스템 구축.

## 변경 사항

### 신규 파일

**코어 프레임워크:**
- `agents/core/base_agent.py` — 112줄, BaseAgent ABC (start/stop/execute_task)
- `agents/core/task.py` — 82줄, Task dataclass (TaskStatus, TaskPriority)
- `agents/core/task_tracker.py` — 238줄 (C13과 동일)
- `agents/core/message_bus.py` — 에이전트 간 통신

**에이전트 구현:**
- `agents/modeling/modeling_agent.py` — 모델링 에이전트
- `agents/modeling/vehicle_modeler.py` — 차량 모델링
- `agents/modeling/physics_modeler.py` — 물리 모델링
- `agents/modeling/world_modeler.py` — 환경 모델링
- `agents/research/research_agent.py` — 연구 에이전트
- `agents/research/web_searcher.py` — 웹 검색
- `agents/research/document_writer.py` — 문서 작성
- `agents/project_leader/project_leader_agent.py` — 프로젝트 리더

**진입점:**
- `agents/main.py` — 397줄, CLI 인터페이스 (argparse)
- `agents/__main__.py` — 모듈 실행 진입점

### 설계 결정

- 메시지 버스 기반 에이전트 간 통신 (느슨한 결합)
- 서브커맨드 CLI: `model`, `research`, `status`, `coordinate`
- 대화형 모드 (`agents>` 프롬프트) 지원

## 향후 과제

- aiohttp 설치하여 research 에이전트 웹 검색 활성화
- 에이전트 시스템 단위 테스트 미작성
