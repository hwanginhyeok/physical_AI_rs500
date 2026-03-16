# 지능형 선택 빌드 스킬 (build-smart)

> **트리거**: 아래 표현이 나오면 이 스킬을 즉시 실행한다.
> - "빌드해줘" / "빌드" / "build"
> - "변경사항 빌드" / "패키지 빌드"
> - 코드 수정 후 빌드 필요 시 자동 제안 가능
> - `task-wrap.md` STEP 5에서 위임받아 실행

---

## 목적

변경된 파일에서 패키지를 자동 추출하고, 의존성 그래프를 분석하여 영향받는 패키지까지 포함한 최소 빌드를 수행한다.

---

## 패키지 목록

| 패키지 | 빌드 시스템 | 비고 |
|--------|-------------|------|
| `ad_bringup` | ament_python | 런치 + 설정 |
| `ad_can_bridge` | ament_python | CAN 통신 |
| `ad_control` | ament_python | 제어기 |
| `ad_core` | ament_python | 공통 유틸 |
| `ad_interfaces` | ament_cmake | msg/srv 정의 |
| `ad_perception` | ament_python | 인지 |
| `ad_planning` | ament_python | 경로 계획 |
| `ad_simulation` | ament_cmake | Gazebo 모델 + URDF |

---

## 실행 순서 (순서 준수 필수)

### STEP 1 — 변경 파일 탐지

`git diff --name-only` (unstaged + staged)로 변경된 파일 목록을 수집한다.

```bash
git diff --name-only HEAD
```

커밋 이후 변경이 없으면 마지막 커밋의 변경 파일을 확인:
```bash
git diff --name-only HEAD~1
```

### STEP 2 — 패키지 매핑

변경된 파일 경로에서 패키지명을 추출한다:

| 파일 경로 패턴 | 패키지 |
|---------------|--------|
| `src/ad_bringup/**` | `ad_bringup` |
| `src/ad_control/**` | `ad_control` |
| `src/ad_core/**` | `ad_core` |
| `src/ad_interfaces/**` | `ad_interfaces` |
| `src/ad_perception/**` | `ad_perception` |
| `src/ad_planning/**` | `ad_planning` |
| `src/ad_simulation/**` | `ad_simulation` |
| `src/ad_can_bridge/**` | `ad_can_bridge` |

### STEP 3 — 의존성 전파

`ad_interfaces`가 변경되면 → 해당 패키지에 의존하는 모든 패키지를 빌드 대상에 추가.
각 패키지의 `package.xml`에서 `<depend>`, `<exec_depend>`, `<build_depend>` 태그를 확인하여 역방향 의존성을 추출한다.

**알려진 의존성 체인:**
- `ad_interfaces` 변경 → `ad_core`, `ad_control`, `ad_perception`, `ad_planning`, `ad_bringup` 전체 빌드
- `ad_core` 변경 → `ad_control`, `ad_perception`, `ad_planning` 빌드
- 설정 파일만 변경 (`config/*.yaml`) → 빌드 불필요, "설정 변경만 감지됨" 알림

### STEP 4 — 빌드 실행 확인

사용자에게 빌드 대상을 보여주고 확인한다:

```
📦 빌드 대상: ad_control, ad_bringup
   변경 감지: ad_control/control_node.py (직접 변경)
              ad_bringup/launch/... (ad_control 의존)
   실행할 명령: colcon build --packages-select ad_control ad_bringup

   실행할까요? [Y/n]
```

### STEP 5 — 빌드 실행

```bash
cd /home/gint_pcd/projects/자율주행프로젝트
colcon build --packages-select {패키지1} {패키지2} ...
```

### STEP 6 — 결과 보고

빌드 결과를 요약한다:

```
✅ 빌드 완료 — {N}개 패키지
   ad_control: ✅ 성공 (0 warnings)
   ad_bringup: ✅ 성공 (2 warnings)
   소요 시간: 12.3s
```

실패 시:
```
❌ 빌드 실패 — ad_control
   에러: {핵심 에러 메시지 1줄 요약}
   파일: src/ad_control/...py:42
```

빌드 결과(성공/실패)를 TASK.md 현재 진행 중 항목에 한 줄 기록.

---

## 판단 규칙

| 상황 | 행동 |
|------|------|
| 설정 파일(.yaml)만 변경 | "설정 변경만 감지됨, 빌드 불필요" 알림 |
| URDF/SDF/xacro 변경 | `ad_simulation` 빌드 |
| msg/srv 변경 | `ad_interfaces` + 전체 의존 패키지 빌드 |
| launch 파일만 변경 | `ad_bringup` 빌드 (Python이라 install 필요) |
| 변경 파일 없음 | "변경 사항 없음" 알림, 강제 빌드 여부 확인 |
| 빌드 실패 시 | 에러 요약 출력, 수정 제안 (재빌드는 사용자 요청 시) |

---

## 주의사항

- `source install/setup.bash`는 빌드 후 자동 실행하지 않음 (새 터미널에서 수동 source)
- `--symlink-install`은 Python 패키지에서만 유효. CMake 패키지(ad_interfaces, ad_simulation)에는 불필요
- 전체 빌드(`colcon build` 인자 없음)는 사용자가 명시적으로 요청할 때만 실행
