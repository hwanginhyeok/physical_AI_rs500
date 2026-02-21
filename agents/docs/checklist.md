# 프로젝트 시작 전 체크리스트

## 1. 환경 점검

### 필수
- [ ] Python 3.10+ 설치
- [ ] PyYAML 설치 (`pip install pyyaml`)
- [ ] 프로젝트 루트에서 `python -m agents --help` 실행 확인

### 선택 (기능별)
- [ ] aiohttp 설치 (`pip install aiohttp`) — 웹/논문 검색
- [ ] rich 설치 (`pip install rich`) — CLI 출력 개선
- [ ] ROS2 Humble 설치 — 시뮬레이션 실행
- [ ] Gazebo gz-sim 설치 — 시뮬레이션 실행

---

## 2. 파일 무결성 점검

### 시뮬레이션 모델
- [ ] `src/ad_simulation/models/tracked_vehicle/model.sdf` 존재 및 XML 유효
- [ ] `src/ad_simulation/worlds/agricultural_field.sdf` 존재 및 XML 유효
- [ ] `src/ad_simulation/config/bridge_config.yaml` 존재

### 에이전트 시스템
- [ ] `agents/core/` — base_agent.py, task.py, message_bus.py
- [ ] `agents/modeling/` — modeling_agent.py, vehicle_modeler.py, world_modeler.py, physics_modeler.py
- [ ] `agents/research/` — research_agent.py, web_searcher.py, document_writer.py
- [ ] `agents/project_leader/` — project_leader_agent.py
- [ ] `agents/main.py` + `agents/__main__.py`

### 점검 명령
```bash
python -c "from agents.core import BaseAgent, Task, MessageBus; print('core OK')"
python -c "from agents.modeling import ModelingAgent; print('modeling OK')"
python -c "from agents.research import ResearchAgent; print('research OK')"
python -c "from agents.project_leader import ProjectLeaderAgent; print('leader OK')"
```

---

## 3. 기능 동작 점검

### 모델링 에이전트
- [ ] `python -m agents model vehicle --info` → 차량 정보 출력
- [ ] `python -m agents model world --info` → 월드 정보 출력
- [ ] `python -m agents model physics --info` → 물리 설정 출력

### 선행조사 에이전트 (aiohttp 필요)
- [ ] `python -m agents research "ROS2 Gazebo" --web` → 웹 검색 결과
- [ ] `python -m agents research "tracked vehicle" --papers` → 논문 검색 결과

### 프로젝트 관리
- [ ] `python -m agents status` → 현황 보고서 출력

---

## 4. SDF 수정 전 확인

- [ ] 백업 파일 존재 확인 (수정 시 자동 생성되지만 수동 확인 권장)
- [ ] 수정할 파라미터 현재 값 확인 (`--info` 명령)
- [ ] 수정 후 Gazebo 재시작 필요 여부 인지

---

## 5. 조사 시작 전 확인

- [ ] 인터넷 연결 상태
- [ ] aiohttp 설치 여부
- [ ] 조사 주제/키워드 준비
- [ ] `docs/` 폴더 쓰기 권한
