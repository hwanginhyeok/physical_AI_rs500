"""모든 에이전트의 기반 추상 클래스."""

from __future__ import annotations

import asyncio
import logging
from abc import ABC, abstractmethod
from datetime import datetime

from .message_bus import Message, MessageBus
from .task import Task, TaskStatus

logger = logging.getLogger(__name__)


class BaseAgent(ABC):
    """에이전트 기반 클래스.

    모든 에이전트는 이 클래스를 상속하고 handle_task()를 구현해야 한다.
    """

    def __init__(self, name: str, bus: MessageBus) -> None:
        self.name = name
        self.bus = bus
        self._running = False
        self._current_task: Task | None = None
        self._completed_tasks: list[Task] = []
        self._started_at: datetime | None = None

    async def start(self) -> None:
        """에이전트 시작. 메시지 버스 구독 등록."""
        self._running = True
        self._started_at = datetime.now()
        self.bus.subscribe("task.assign", self._on_task_assigned)
        logger.info("[%s] 에이전트 시작", self.name)

    async def stop(self) -> None:
        """에이전트 정지."""
        self._running = False
        self.bus.unsubscribe("task.assign", self._on_task_assigned)
        logger.info("[%s] 에이전트 정지", self.name)

    async def _on_task_assigned(self, msg: Message) -> None:
        """작업 할당 메시지 수신 처리."""
        task: Task = msg.payload
        if task.assignee != self.name:
            return

        logger.info("[%s] 작업 수신: %s", self.name, task.title)
        self._current_task = task
        task.start()

        try:
            result = await self.handle_task(task)
            task.complete(result)
            await self.bus.publish("task.complete", self.name, task)
        except Exception as e:
            task.fail(str(e))
            await self.bus.publish("task.failed", self.name, task)
            logger.error("[%s] 작업 실패: %s - %s", self.name, task.title, e)
        finally:
            self._completed_tasks.append(task)
            self._current_task = None

    @abstractmethod
    async def handle_task(self, task: Task) -> object:
        """작업 처리 (하위 클래스에서 구현).

        Returns:
            작업 결과 객체
        """

    def report_status(self) -> dict:
        """에이전트 상태 보고."""
        return {
            "name": self.name,
            "running": self._running,
            "current_task": str(self._current_task) if self._current_task else None,
            "completed": len(self._completed_tasks),
            "success": sum(
                1 for t in self._completed_tasks if t.status == TaskStatus.COMPLETED
            ),
            "failed": sum(
                1 for t in self._completed_tasks if t.status == TaskStatus.FAILED
            ),
            "uptime": str(datetime.now() - self._started_at) if self._started_at else "N/A",
        }

    async def execute_task(self, task: Task) -> Task:
        """작업을 직접 실행 (메시지 버스를 거치지 않고 직접 호출)."""
        self._current_task = task
        task.start()

        try:
            result = await self.handle_task(task)
            task.complete(result)
        except Exception as e:
            task.fail(str(e))
            logger.error("[%s] 작업 실패: %s - %s", self.name, task.title, e)
        finally:
            self._completed_tasks.append(task)
            self._current_task = None

        return task
