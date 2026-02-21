"""작업 단위 데이터 클래스."""

from __future__ import annotations

import uuid
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from typing import Any


class TaskStatus(Enum):
    """작업 상태."""

    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"


class TaskPriority(Enum):
    """작업 우선순위."""

    LOW = 1
    NORMAL = 2
    HIGH = 3
    URGENT = 4


@dataclass
class Task:
    """에이전트가 처리하는 작업 단위."""

    title: str
    description: str = ""
    assignee: str = ""
    priority: TaskPriority = TaskPriority.NORMAL
    id: str = field(default_factory=lambda: uuid.uuid4().hex[:8])
    status: TaskStatus = TaskStatus.PENDING
    result: Any = None
    error: str = ""
    created_at: datetime = field(default_factory=datetime.now)
    completed_at: datetime | None = None
    dependencies: list[str] = field(default_factory=list)

    def start(self) -> None:
        """작업 시작."""
        self.status = TaskStatus.IN_PROGRESS

    def complete(self, result: Any = None) -> None:
        """작업 완료."""
        self.status = TaskStatus.COMPLETED
        self.result = result
        self.completed_at = datetime.now()

    def fail(self, error: str) -> None:
        """작업 실패."""
        self.status = TaskStatus.FAILED
        self.error = error
        self.completed_at = datetime.now()

    @property
    def is_done(self) -> bool:
        return self.status in (TaskStatus.COMPLETED, TaskStatus.FAILED)

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "title": self.title,
            "description": self.description,
            "assignee": self.assignee,
            "status": self.status.value,
            "priority": self.priority.name,
            "result": str(self.result) if self.result else None,
            "error": self.error,
            "created_at": self.created_at.isoformat(),
            "completed_at": self.completed_at.isoformat() if self.completed_at else None,
        }

    def __str__(self) -> str:
        return f"[{self.status.value}] {self.title} ({self.id})"
