"""task_autonomou.md 파일 자동 업데이트 모듈.

[비활성] 2026-03-01 — task_autonomou.md 폐기로 이 모듈의 역할 종료.
공식 TASK 관리는 docs/프로젝트/TASK.md에서 수기로 운영한다. (C13 완료 참조)
"""

from __future__ import annotations

import logging
import re
from datetime import datetime
from pathlib import Path

logger = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
_TASK_MD = _PROJECT_ROOT / "task_autonomou.md"

# 섹션 헤더 패턴
_TODO_HEADER = "## 할 일 (TODO)"
_PROGRESS_HEADER = "## 진행 중 (IN PROGRESS)"
_DONE_HEADER = "## 완료 (DONE)"
_SEPARATOR = "---"


def _read_task_md() -> str:
    if not _TASK_MD.exists():
        return _create_default()
    return _TASK_MD.read_text(encoding="utf-8")


def _write_task_md(content: str) -> None:
    # 최종 업데이트 타임스탬프 갱신
    now = datetime.now().strftime("%Y-%m-%d %H:%M")
    content = re.sub(
        r"> 최종 업데이트: .+",
        f"> 최종 업데이트: {now}",
        content,
    )
    _TASK_MD.write_text(content, encoding="utf-8")


def _create_default() -> str:
    now = datetime.now().strftime("%Y-%m-%d")
    content = f"""# 프로젝트 작업 관리

> 최종 업데이트: {now}

---

{_TODO_HEADER}

(할 일 없음)

---

{_PROGRESS_HEADER}

(현재 진행 중인 작업 없음)

---

{_DONE_HEADER}

(완료된 작업 없음)

---

## 참고

- 작업 상태: `[ ]` 미완료, `[x]` 완료
- 이 파일은 에이전트 시스템이 자동으로 관리합니다
"""
    _TASK_MD.write_text(content, encoding="utf-8")
    return content


def _parse_sections(content: str) -> dict[str, list[str]]:
    """task.md를 TODO / IN PROGRESS / DONE 섹션으로 분리."""
    sections: dict[str, list[str]] = {"todo": [], "progress": [], "done": []}
    current = None

    for line in content.splitlines():
        stripped = line.strip()
        if stripped == _TODO_HEADER:
            current = "todo"
            continue
        elif stripped == _PROGRESS_HEADER:
            current = "progress"
            continue
        elif stripped == _DONE_HEADER:
            current = "done"
            continue
        elif stripped == _SEPARATOR or stripped.startswith("## 참고"):
            current = None
            continue

        if current and stripped and stripped != "(현재 진행 중인 작업 없음)" and stripped != "(할 일 없음)" and stripped != "(완료된 작업 없음)":
            sections[current].append(line)

    return sections


def _build_content(sections: dict[str, list[str]]) -> str:
    """섹션 데이터를 task.md 형식으로 재구성."""
    now = datetime.now().strftime("%Y-%m-%d %H:%M")

    lines = [
        "# 프로젝트 작업 관리",
        "",
        f"> 최종 업데이트: {now}",
        "",
        _SEPARATOR,
        "",
        _TODO_HEADER,
        "",
    ]

    if sections["todo"]:
        lines.extend(sections["todo"])
    else:
        lines.append("(할 일 없음)")
    lines.extend(["", _SEPARATOR, "", _PROGRESS_HEADER, ""])

    if sections["progress"]:
        lines.extend(sections["progress"])
    else:
        lines.append("(현재 진행 중인 작업 없음)")
    lines.extend(["", _SEPARATOR, "", _DONE_HEADER, ""])

    if sections["done"]:
        lines.extend(sections["done"])
    else:
        lines.append("(완료된 작업 없음)")

    lines.extend([
        "",
        _SEPARATOR,
        "",
        "## 참고",
        "",
        "- 작업 상태: `[ ]` 미완료, `[x]` 완료",
        "- 이 파일은 에이전트 시스템이 자동으로 관리합니다",
        "",
    ])

    return "\n".join(lines)


def _find_and_remove(items: list[str], title: str) -> bool:
    """리스트에서 title이 포함된 항목을 찾아 제거. 반환: 찾았는지 여부."""
    for i, line in enumerate(items):
        if title in line:
            items.pop(i)
            return True
    return False


def on_task_start(title: str, agent_name: str) -> None:
    """작업 시작 시 호출: TODO → IN PROGRESS로 이동 또는 새로 추가."""
    try:
        content = _read_task_md()
        sections = _parse_sections(content)

        entry = f"- [ ] {title} — {agent_name}"

        # TODO에 있으면 제거
        _find_and_remove(sections["todo"], title)

        # 이미 진행 중에 있으면 중복 추가 안함
        if not any(title in line for line in sections["progress"]):
            sections["progress"].append(entry)

        _write_task_md(_build_content(sections))
        logger.info("[task.md] 시작: %s (%s)", title, agent_name)
    except Exception as e:
        logger.warning("[task.md] 업데이트 실패 (start): %s", e)


def on_task_complete(title: str, agent_name: str) -> None:
    """작업 완료 시 호출: IN PROGRESS → DONE으로 이동."""
    try:
        content = _read_task_md()
        sections = _parse_sections(content)

        now = datetime.now().strftime("%Y-%m-%d %H:%M")
        entry = f"- [x] {title} — {agent_name} ({now})"

        # TODO/진행 중에서 제거
        _find_and_remove(sections["todo"], title)
        _find_and_remove(sections["progress"], title)

        # DONE에 추가 (중복 방지)
        if not any(title in line for line in sections["done"]):
            sections["done"].insert(0, entry)

        _write_task_md(_build_content(sections))
        logger.info("[task.md] 완료: %s (%s)", title, agent_name)
    except Exception as e:
        logger.warning("[task.md] 업데이트 실패 (complete): %s", e)


def on_task_fail(title: str, agent_name: str, error: str) -> None:
    """작업 실패 시 호출: IN PROGRESS → TODO로 되돌림 (실패 표시)."""
    try:
        content = _read_task_md()
        sections = _parse_sections(content)

        now = datetime.now().strftime("%Y-%m-%d %H:%M")
        entry = f"- [ ] {title} — {agent_name} (실패: {error[:50]}, {now})"

        # 진행 중에서 제거
        _find_and_remove(sections["progress"], title)

        # TODO에 실패 표시로 추가
        _find_and_remove(sections["todo"], title)
        sections["todo"].insert(0, entry)

        _write_task_md(_build_content(sections))
        logger.info("[task.md] 실패: %s (%s) - %s", title, agent_name, error[:50])
    except Exception as e:
        logger.warning("[task.md] 업데이트 실패 (fail): %s", e)


def add_todo(title: str) -> None:
    """수동으로 TODO 항목 추가."""
    try:
        content = _read_task_md()
        sections = _parse_sections(content)

        entry = f"- [ ] {title}"
        if not any(title in line for line in sections["todo"]):
            sections["todo"].append(entry)

        _write_task_md(_build_content(sections))
        logger.info("[task.md] TODO 추가: %s", title)
    except Exception as e:
        logger.warning("[task.md] 업데이트 실패 (add_todo): %s", e)
