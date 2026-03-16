# DDS/SharedMemory 트러블슈팅 스킬 (dds-troubleshoot)

> **트리거**: 아래 표현이 나오면 이 스킬을 즉시 실행한다.
> - "DDS 초기화" / "DDS 문제" / "DDS 정리"
> - "공유 메모리 정리" / "SharedMemory 충돌"
> - "velocity_smoother 안됨" / "activate 실패"
> - lifecycle 노드 activate/configure 실패 시

---

## 배경

C61 블로커: FastRTPS SharedMemory 포트 충돌 → `velocity_smoother` activate 실패.
CycloneDDS 전환 예정이지만, 전환 전/후 모두 트러블슈팅 절차가 필요하다.

참조: `docs/프로젝트/task/C61_velocity_chain_debug.md`

---

## 실행 순서 (순서 준수 필수)

### STEP 1 — 현재 DDS 구현 확인

```bash
echo $RMW_IMPLEMENTATION
```

| 값 | 의미 |
|----|------|
| `rmw_fastrtps_cpp` (또는 미설정) | FastRTPS (기본값, SharedMemory 문제 가능) |
| `rmw_cyclonedds_cpp` | CycloneDDS (SharedMemory 미사용) |

### STEP 2 — SharedMemory 잔여 세그먼트 확인

FastRTPS 사용 시에만 해당:

```bash
ls -la /dev/shm/fastrtps* 2>/dev/null
ls -la /dev/shm/ | grep -i fast 2>/dev/null
```

잔여 파일이 있으면 → STEP 3으로 진행.
없으면 → "SharedMemory 잔여 없음" 알림 후 STEP 4로 진행.

### STEP 3 — SharedMemory 정리 (확인 후 실행)

사용자에게 정리 대상을 보여주고 확인한다:

```
⚠️ SharedMemory 잔여 세그먼트 발견:
   /dev/shm/fastrtps_xxx (크기, 수정일)
   /dev/shm/fastrtps_yyy (크기, 수정일)

   ROS2 프로세스가 실행 중이지 않은지 먼저 확인합니다.
```

```bash
# ROS2 프로세스 확인
ps aux | grep -E "(ros2|nav2|gazebo)" | grep -v grep

# 프로세스 없으면 정리
rm /dev/shm/fastrtps*
```

**주의**: ROS2 프로세스가 실행 중이면 정리하지 않는다. 먼저 프로세스를 종료하도록 안내.

### STEP 4 — Lifecycle 노드 상태 확인

ROS2가 실행 중일 때만 수행:

```bash
# velocity_smoother 노드 상태
ros2 lifecycle get /velocity_smoother

# 전체 lifecycle 노드 목록
ros2 lifecycle list
```

| 현재 상태 | 의미 | 행동 |
|-----------|------|------|
| `unconfigured` | 초기 상태 | configure → activate 순서 시도 |
| `inactive` | configure 완료 | activate 시도 |
| `active` | 정상 동작 | 문제 없음, 토픽 체인 확인 (STEP 5) |
| `finalized` | 종료됨 | 노드 재시작 필요 |
| (노드 없음) | 노드 미시작 | launch 파일 확인 |

### STEP 5 — Manual Activate 시도

노드가 `inactive` 상태면:

```bash
ros2 lifecycle set /velocity_smoother activate
```

실패 시 에러 메시지를 캡처하여 원인 분석:
- `SharedMemory error` → STEP 3 재실행
- `transition not allowed` → 현재 상태 확인 후 올바른 전이 시도
- `timeout` → DDS 통신 문제, RMW 변경 검토

### STEP 6 — 토픽 체인 검증

velocity_smoother가 active 상태일 때, 명령 속도 토픽 체인이 정상인지 확인:

```bash
# 토픽 발행 확인
ros2 topic list | grep cmd_vel
ros2 topic info /cmd_vel_nav
ros2 topic info /cmd_vel_smoothed
ros2 topic info /cmd_vel

# 발행자/구독자 확인
ros2 topic info /cmd_vel_nav --verbose
ros2 topic info /cmd_vel --verbose
```

**정상 토픽 체인:**
```
Nav2 controller → /cmd_vel_nav → velocity_smoother → /cmd_vel_smoothed → cmd_vel_relay → /cmd_vel
```

### STEP 7 — 결과 요약 및 기록

```
🔧 DDS 트러블슈팅 — {날짜}

RMW: {rmw_fastrtps_cpp | rmw_cyclonedds_cpp}
SharedMemory: {정리됨 | 잔여 없음 | 정리 불필요(CycloneDDS)}
velocity_smoother: {active | inactive | unconfigured | 노드 없음}
토픽 체인: {정상 | /cmd_vel_nav 발행 없음 | ...}

조치: {수행한 작업 요약}
```

TASK.md의 C61 항목에 결과를 기록한다.

---

## CycloneDDS 전환 가이드

CycloneDDS로 전환할 때 참고:

### 전환 방법
```bash
# 패키지 설치
sudo apt install ros-jazzy-rmw-cyclonedds-cpp

# 환경변수 설정 (bashrc 또는 launch 파일)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 전환 후 확인사항
- SharedMemory 관련 문제 해소 여부
- 토픽 통신 지연(latency) 변화
- `velocity_smoother` activate 성공 여부
- Nav2 전체 lifecycle 정상 전환 여부

---

## 판단 규칙

| 상황 | 행동 |
|------|------|
| SharedMemory 충돌 반복 | CycloneDDS 전환 권고 |
| activate 실패 + SharedMemory 에러 | STEP 3 정리 후 재시도 |
| activate 실패 + 다른 에러 | 에러 메시지 분석 후 대응 |
| 토픽 체인 끊김 | 해당 노드 상태 확인 → launch 파일 점검 |
| CycloneDDS에서도 실패 | Nav2 파라미터 검증 (launch-preflight 스킬) |

---

## 주의사항

- `/dev/shm/` 파일 삭제는 **반드시** ROS2 프로세스 종료 후 수행
- `ros2 lifecycle` 명령은 ROS2 런타임이 실행 중일 때만 사용 가능
- CycloneDDS 전환은 아키텍처 결정 → ADR 업데이트 필요 (ARCH-003)
- 이 스킬은 진단·정리만 수행한다. Nav2 파라미터 자체 수정은 `param-tuning-log` 스킬 관할
