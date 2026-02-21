# 프로젝트 작업 관리

> 최종 업데이트: 2026-02-21

---

## 할 일 (TODO)

- [ ] aiohttp 설치하여 research 에이전트 웹 검색 기능 활성화
- [ ] 카메라 이미지 처리 파이프라인 구현 (perception.py)
- [ ] LiDAR 포인트클라우드 기반 장애물 감지 구현 (perception.py)
- [ ] 실제 경로 계획 알고리즘 구현 (planning.py)
- [ ] 좌/우회전 조향 로직 구현 (control.py)
- [ ] 에이전트 시스템 단위 테스트 작성
- [ ] Gazebo 시뮬레이션과 에이전트 시스템 연동 테스트
- [ ] rich 라이브러리 적용하여 CLI 출력 개선

---

## 진행 중 (IN PROGRESS)

(현재 진행 중인 작업 없음)

---

## 완료 (DONE)

- [x] 에이전트 시스템 매뉴얼/체크리스트 작성 (agents/docs/)
- [x] task.md 자동 업데이트 기능 구현 (task_tracker.py)
- [x] AI 에이전트 시스템 구현 - core/modeling/research/project_leader (7d84ca0)
  - [x] core 프레임워크 (BaseAgent, Task, MessageBus)
  - [x] modeling 에이전트 (vehicle/world/physics modeler)
  - [x] research 에이전트 (web_searcher, document_writer)
  - [x] project_leader 에이전트 (작업 분배, 진행 관리)
  - [x] CLI 진입점 (python -m agents)
- [x] Gazebo gz-sim 시뮬레이션 연동 - ad_simulation 패키지 (deecbfb)
- [x] ROS2 프로젝트 초기 구조 생성 (6ab5521)

---

## 참고

- 작업 상태: `[ ]` 미완료, `[x]` 완료
- 이 파일은 에이전트 시스템이 자동으로 관리합니다
