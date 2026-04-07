# Difficulties & Know-how

## D-001: CAN 브릿지 Twist→트랙 속도 변환 버그 — 조향 불가

- **날짜**: 2026-03-16
- **상황**: CAN 브릿지 노드가 ROS2 `cmd_vel` (Twist)을 받아 좌/우 트랙 모터에 CAN 명령 전송
- **이슈**: `_periodic_send()`에서 좌/우 트랙에 `linear.x * 3.6`을 동일하게 할당. `angular.z`(회전)가 완전히 무시되어 **실차에서 직진만 가능, 조향 불가** 상태
- **삽질**: 처음에 CAN 프로토콜 문제로 의심하여 DBC 파일 재분석. 실제로는 ROS→CAN 변환 레이어의 단순 로직 오류
- **해결**: `SkidSteerModel.twist_to_tracks()`로 Twist → 좌/우 차등 속도 변환 구현. 직진(L=R), 좌회전(L<R), 우회전(L>R), 피봇(L=-R)
- **대안**: CAN 메시지 레벨에서 angular를 별도 필드로 전송 → 기존 MD2K 프로토콜과 불일치, 기각
- **노하우**: 스키드 스티어 차량은 Twist를 바로 쓰면 안 된다. **반드시 twist_to_tracks() 변환**을 거쳐야 함. 단위 테스트에 직진/후진/좌회전/우회전/피봇/정지 6가지 케이스 필수
- **회고**: CAN 브릿지 초기 구현 시 직진 테스트만 했음. 회전 테스트를 초기부터 포함했으면 즉시 발견 가능했다
- **관련 파일**: `src/ad_can_bridge/ad_can_bridge/can_bridge_node.py`, `src/ad_can_bridge/test/test_can_bridge_logic.py`

## D-002: Curtis→MD2K 모터 컨트롤러 오인 — 시뮬 전면 재작업

- **날짜**: 2026-03-25
- **상황**: C49 교차 검증 리포트 작성 중, 모터 컨트롤러를 Curtis 1226BL로 기술
- **이슈**: 실차는 **MDROBOT MD2K** (듀얼채널 DC 모터 드라이버)를 사용. Curtis 기반으로 작성한 제어 특성, 보호 기능, CAN 프로토콜이 전부 틀림
- **삽질**: Curtis 1226BL 데이터시트 기반으로 시뮬 파라미터를 정교하게 맞춤 → 근본적으로 잘못된 컨트롤러 기준
- **해결**: C49 리포트 5개소 전면 정정 + C50에서 MD2K 기반 재구현 (SS/SD 가감속, 보호기능, CAN 코덱, 열 모델)
- **대안**: Curtis 파라미터 위에 MD2K 보정값만 덮어쓰기 → 아키텍처가 달라서 불가능, 처음부터 재작성
- **노하우**: **하드웨어 컴포넌트 확인은 데이터시트보다 실차 현물 확인이 먼저**. 이름이 비슷한 부품이라도 제조사·모델이 다르면 제어 특성이 완전히 다름. HIH-2 프로젝트 자료를 볼 때 파일명이 아니라 내용의 컨텍스트(CAN ID, 파라미터 범위)로 실제 장착 부품을 교차 확인할 것
- **회고**: 처음에 인혁에게 "모터 컨트롤러 뭐 쓰세요?" 한마디만 물었으면 하루 절약 가능했다. 데이터시트만 보고 추정하지 말 것
- **관련 파일**: `docs/프로젝트/task/C49_simulation_vs_actual_comparison.md`, `docs/프로젝트/task/C50_md2k_motor_controller.md`

## D-003: Gazebo SDF IMU noise type 속성 누락 — 8곳 문법 오류

- **날짜**: 2026-03-22
- **상황**: 집 PC(WSL2)에서 Gazebo Harmonic 환경 구축 후 시뮬레이션 기동
- **이슈**: model.sdf의 IMU/GPS 센서 noise 태그에 `type` 속성이 빠져있어 Gazebo가 센서를 로드하지 못함. 에러 메시지가 모호하여 원인 특정 어려움
- **삽질**: Gazebo 버전 호환성 문제로 의심, 플러그인 재설치 시도. Nav2 파라미터 문제도 동시에 발생하여 혼선 (SmacPlannerLattice → SmacPlannerHybrid 교체 필요)
- **해결**: SDF 파일 8곳에 `type="gaussian"` 또는 `type="gaussian_quantized"` 속성 추가. Nav2도 동시 수정
- **대안**: 없음, SDF 스펙 준수 필수
- **노하우**: Gazebo SDF 센서 noise 태그는 반드시 `<noise type="gaussian">` 형태여야 함. `<noise>` 단독 사용 불가. SDF 검증은 `gz sdf -k model.sdf`로 사전 체크
- **회고**: SDF 파일을 수동 편집할 때 XML 스키마 검증을 먼저 돌렸어야 함. CI에 SDF 검증 단계를 추가하면 재발 방지 가능
- **관련 파일**: `src/ad_bringup/models/ss500/model.sdf`, `src/ad_bringup/config/nav2_params.yaml`

## D-004: ROS2 파라미터 중복선언 — declare vs get 혼동

- **날짜**: 2026-03-30
- **상황**: PerceptionNode에 crop_row 퍼블리셔 추가 중
- **이슈**: YAML 파라미터 파일에서 이미 선언된 파라미터를 코드에서 `declare_parameter()`로 다시 선언 → `ParameterAlreadyDeclaredException` 발생
- **삽질**: 파라미터 이름 오타로 의심하여 YAML과 코드 이름 대조. 실제로는 선언 방식 자체의 문제
- **해결**: `declare_parameter()` → `get_parameter()`로 변경. YAML에서 선언하고, 코드에서는 읽기만
- **대안**: YAML에서 파라미터를 제거하고 코드에서만 declare → YAML 중앙 관리 원칙에 위배
- **노하우**: ROS2 파라미터 규칙: **YAML에서 선언 → 코드에서 `get_parameter()`로 읽기**. 코드에서 `declare_parameter()`는 YAML에 없는 파라미터의 디폴트값 정의 시에만 사용. 혼용하면 중복선언 에러
- **회고**: 파라미터 관리 방침을 프로젝트 초기에 정해두었으면 (YAML-first or code-first) 이런 혼선 없었을 것
- **관련 파일**: `src/ad_perception/ad_perception/perception_node.py`, `src/ad_perception/config/perception_params.yaml`

## D-005: 시뮬레이션 파라미터 4배 오차 — 추정값만으로 개발

- **날짜**: 2026-03-23
- **상황**: 2월부터 시뮬레이션을 추정값(mass=200kg, gear_ratio=30, RPM=3000)으로 돌리고 있었음
- **이슈**: HIH-2 실차 데이터(DB130-48 데이터시트, CAN DBC, BLACKTAN 시험)와 교차 검증 시 **차량 질량 4배(200→800kg), 감속비 1.5배(30→20), 모터 RPM 1.5배(3000→2000)** 오차 발견. 시뮬레이션 결과 전체의 신뢰도 의심
- **삽질**: 추정값 기반으로 1개월간 제어 파라미터 튜닝, 경로 계획 테스트 진행. 결과가 "그럴듯해 보여서" 오차를 인지 못함
- **해결**: C48에서 6개 파일 전면 교정(VehicleDynamicsConfig, DrivetrainConfig, SensorNoiseModel, SDF). 218건 테스트 전체 재통과 확인
- **대안**: 파라미터를 범위(min/max)로 설정하고 Monte Carlo 시뮬레이션 → 과도한 복잡성, 기각
- **노하우**: **시뮬레이션 초기부터 실차 데이터와 교차 검증**. "대충 맞겠지"로 추정값 사용하면 나중에 전면 재작업. 특히 질량, 감속비, 모터 정격은 시뮬 결과에 비선형적 영향. 데이터시트를 먼저 확보하고, 없으면 TODO로 명시적 추적
- **회고**: 프로젝트 Day 1에 HIH-2 데이터를 먼저 분석했어야 했다. 시뮬-실차 교차 검증을 마일스톤이 아니라 지속적 프로세스로 운영할 것
- **관련 파일**: `src/ad_core/ad_core/vehicle_dynamics.py`, `src/ad_core/ad_core/drivetrain_model.py`, `src/ad_bringup/models/ss500/model.sdf`, `docs/프로젝트/task/C48_hih2_parameter_update.md`
