# 에이전트 시스템 매뉴얼

## 시작 전 체크리스트

### 환경 확인
- [ ] Python 3.10+ 설치 확인: `python --version`
- [ ] PyYAML 설치 확인: `python -c "import yaml"`
- [ ] (선택) aiohttp 설치: `pip install aiohttp` — research 에이전트 웹 검색에 필요
- [ ] (선택) rich 설치: `pip install rich` — CLI 출력 포맷팅

### 프로젝트 파일 확인
- [ ] `src/ad_simulation/models/tracked_vehicle/model.sdf` 존재
- [ ] `src/ad_simulation/worlds/agricultural_field.sdf` 존재
- [ ] `src/ad_simulation/config/bridge_config.yaml` 존재

### 실행 확인
```bash
python -m agents --help          # CLI 도움말
python -m agents model vehicle --info  # 차량 정보 조회 (파일 파싱 테스트)
python -m agents status          # 에이전트 상태 확인
```

---

## 에이전트별 스킬 매뉴얼

### 1. Modeling Agent (모델링)

SDF 파일을 직접 파싱/수정하여 시뮬레이션 모델을 조작한다.
**모든 수정 전 자동 백업** (.sdf.bak)

#### 차량 모델 (vehicle)
| 명령 | CLI | 설명 |
|------|-----|------|
| vehicle_info | `model vehicle --info` | 차량 정보 조회 (질량, 크기, 센서) |
| modify_physics | `model vehicle --mass 800` | 차체 질량 수정 |
| modify_physics | `model vehicle --friction 0.9` | 트랙 마찰계수 수정 |
| add_sensor | `model vehicle --add-sensor camera` | 센서 추가 (camera/lidar/imu/gps) |
| list_sensors | `model vehicle --list-sensors` | 장착된 센서 목록 |
| create_vehicle | (코드 내) | 새 차량 모델 SDF 생성 |

#### 월드 (world)
| 명령 | CLI | 설명 |
|------|-----|------|
| world_info | `model world --info` | 월드 정보 조회 |
| add_obstacle | `model world --add-obstacle box` | 장애물 추가 (box/cylinder/sphere) |
| add_road | (코드 내) | 도로 세그먼트 추가 |
| create_world | (코드 내) | 새 월드 SDF 생성 |

#### 물리 (physics)
| 명령 | CLI | 설명 |
|------|-----|------|
| physics_info | `model physics --info` | 물리 엔진 설정 조회 |
| set_physics_engine | `model physics --step-size 0.002` | 시뮬레이션 스텝 크기 |
| set_physics_engine | `model physics --rtf 2.0` | 실시간 배속 |
| set_surface_friction | `model physics --ground-friction 0.6` | 지면 마찰계수 |
| set_gravity | `model physics --gravity-z -9.81` | 중력 설정 |

#### 주의사항
- SDF 수정 후 Gazebo 재시작 필요
- 센서 추가 시 bridge_config.yaml 자동 갱신됨
- 백업 파일(.sdf.bak)로 복원 가능

---

### 2. Research Agent (선행조사)

웹/논문 검색 후 마크다운 보고서를 생성한다.
**aiohttp 필수** (`pip install aiohttp`)

| 명령 | CLI | 설명 |
|------|-----|------|
| research | `research "궤도차량 자율주행"` | 통합 조사 (웹+논문+보고서) |
| search | `research "주제" --web` | 웹 검색만 (DuckDuckGo) |
| search_papers | `research "주제" --papers` | 논문 검색만 (arXiv) |
| list_documents | `research --list-docs` | 저장된 문서 목록 |

#### 조사 워크플로우
1. DuckDuckGo Lite로 웹 검색 (5건)
2. arXiv API로 논문 검색 (5건)
3. 결과를 마크다운 보고서로 생성
4. `docs/` 폴더에 자동 저장

#### 추천 검색 키워드
- autonomous driving tracked vehicle
- agricultural robot ROS2 Gazebo
- 궤도차량 자율주행 경로 계획
- tracked vehicle dynamics simulation

---

### 3. Project Leader Agent (팀장)

작업을 생성하고 하위 에이전트에 배분, 진행 상황을 추적한다.

| 명령 | CLI | 설명 |
|------|-----|------|
| status | `status` | 프로젝트 현황 보고서 |
| coordinate | `coordinate "계획 텍스트"` | 계획을 작업으로 분해/배분 |
| create_task | (코드 내) | 작업 생성 및 할당 |
| check_progress | (코드 내) | 진행률 조회 |
| generate_report | (코드 내) | 마크다운 보고서 생성 |

#### coordinate 키워드 매핑
- "모델", "SDF", "차량", "센서", "물리", "월드" → modeling 에이전트
- "조사", "검색", "논문", "연구", "자료" → research 에이전트
- 해당 없음 → 기본으로 research 에이전트

---

## 대화형 모드

```bash
python -m agents
```

프롬프트 `agents>` 에서 명령 입력:
```
agents> model vehicle --info
agents> model vehicle --mass 500
agents> model world --add-obstacle box
agents> research 궤도차량 자율주행
agents> status
agents> help
agents> quit
```

---

## 에이전트 간 통신 흐름

```
사용자 (CLI)
    ↓ 명령
팀장 (ProjectLeader)
    ├── Task 생성 → MessageBus "task.assign"
    ├── → modeling 에이전트 (SDF 수정)
    └── → research 에이전트 (웹 조사)
         ↓ MessageBus "task.complete" / "task.failed"
팀장 ← 결과 취합 → 보고서 생성
```

---

## 문제 해결

| 증상 | 원인 | 해결 |
|------|------|------|
| `aiohttp가 설치되지 않았습니다` | research 의존성 미설치 | `pip install aiohttp` |
| `chassis 링크를 찾을 수 없습니다` | model.sdf 구조 변경됨 | .sdf.bak에서 복원 |
| `physics 요소를 찾을 수 없습니다` | world SDF 손상 | .sdf.bak에서 복원 |
| CLI 한글 깨짐 | 터미널 인코딩 | `chcp 65001` (Windows) |
