"""프로젝트 팀장 에이전트 - 작업 분배, 진행 관리, 보고서 생성."""

from __future__ import annotations

import asyncio
import logging
from datetime import datetime

from ..core.base_agent import BaseAgent
from ..core.message_bus import Message, MessageBus
from ..core.task import Task, TaskPriority, TaskStatus

logger = logging.getLogger(__name__)


class ProjectLeaderAgent(BaseAgent):
    """프로젝트 리딩을 담당하는 팀장 에이전트.

    - 작업 생성 및 하위 에이전트에 할당
    - 진행 상황 추적
    - 프로젝트 현황 보고서 생성
    - 계획을 세부 작업으로 분해
    """

    def __init__(self, bus: MessageBus) -> None:
        super().__init__("project_leader", bus)
        self._task_registry: dict[str, Task] = {}  # id -> Task
        self._agents: dict[str, BaseAgent] = {}  # name -> agent

    async def start(self) -> None:
        await super().start()
        self.bus.subscribe("task.complete", self._on_task_complete)
        self.bus.subscribe("task.failed", self._on_task_failed)

    async def stop(self) -> None:
        self.bus.unsubscribe("task.complete", self._on_task_complete)
        self.bus.unsubscribe("task.failed", self._on_task_failed)
        await super().stop()

    def register_agent(self, agent: BaseAgent) -> None:
        """하위 에이전트 등록."""
        self._agents[agent.name] = agent
        logger.info("[project_leader] 에이전트 등록: %s", agent.name)

    async def _on_task_complete(self, msg: Message) -> None:
        task: Task = msg.payload
        if task.id in self._task_registry:
            self._task_registry[task.id] = task
            logger.info("[project_leader] 작업 완료: %s (%s)", task.title, msg.sender)

    async def _on_task_failed(self, msg: Message) -> None:
        task: Task = msg.payload
        if task.id in self._task_registry:
            self._task_registry[task.id] = task
            logger.warning("[project_leader] 작업 실패: %s - %s", task.title, task.error)

    async def handle_task(self, task: Task) -> object:
        params = task.result if isinstance(task.result, dict) else {}
        command = params.pop("command", None) or task.title

        dispatch = {
            "create_task": lambda: self._create_and_assign(**params),
            "check_progress": lambda: self.check_progress(),
            "generate_report": lambda: self.generate_report(),
            "coordinate": lambda: self._coordinate(params.get("plan", task.description)),
            "status": lambda: self._all_status(),
        }

        handler = dispatch.get(command)
        if handler is None:
            raise ValueError(
                f"알 수 없는 팀장 명령: {command}\n"
                f"사용 가능: {', '.join(sorted(dispatch.keys()))}"
            )

        result = handler()
        if asyncio.iscoroutine(result):
            return await result
        return result

    async def create_task(
        self,
        title: str,
        description: str = "",
        assignee: str = "",
        priority: TaskPriority = TaskPriority.NORMAL,
        params: dict | None = None,
    ) -> Task:
        """작업 생성 및 에이전트에 할당.

        Args:
            title: 작업 제목 (명령어 역할도 함)
            description: 작업 설명
            assignee: 할당 대상 에이전트 이름
            priority: 우선순위
            params: 추가 파라미터

        Returns:
            생성된 Task
        """
        task = Task(
            title=title,
            description=description,
            assignee=assignee,
            priority=priority,
        )
        if params:
            task.result = params

        self._task_registry[task.id] = task
        logger.info("[project_leader] 작업 생성: %s -> %s", title, assignee or "미할당")

        if assignee:
            await self.bus.publish("task.assign", self.name, task)

        return task

    async def _create_and_assign(
        self,
        title: str = "",
        description: str = "",
        assignee: str = "",
        **kwargs: object,
    ) -> dict:
        task = await self.create_task(title, description, assignee, params=kwargs)
        return task.to_dict()

    def check_progress(self) -> dict:
        """전체 진행 상황 조회."""
        total = len(self._task_registry)
        by_status = {}
        for task in self._task_registry.values():
            status = task.status.value
            by_status.setdefault(status, []).append(task.to_dict())

        return {
            "total_tasks": total,
            "completed": len(by_status.get("completed", [])),
            "in_progress": len(by_status.get("in_progress", [])),
            "pending": len(by_status.get("pending", [])),
            "failed": len(by_status.get("failed", [])),
            "tasks_by_status": by_status,
        }

    def generate_report(self) -> str:
        """프로젝트 현황 보고서 (마크다운)."""
        progress = self.check_progress()
        now = datetime.now().strftime("%Y-%m-%d %H:%M")

        lines = [
            "# 프로젝트 현황 보고서",
            "",
            f"> 생성: {now}",
            "",
            "## 요약",
            "",
            f"- 전체 작업: {progress['total_tasks']}건",
            f"- 완료: {progress['completed']}건",
            f"- 진행 중: {progress['in_progress']}건",
            f"- 대기 중: {progress['pending']}건",
            f"- 실패: {progress['failed']}건",
            "",
        ]

        # 에이전트 상태
        if self._agents:
            lines.append("## 에이전트 상태")
            lines.append("")
            for name, agent in self._agents.items():
                status = agent.report_status()
                lines.append(f"### {name}")
                lines.append(f"- 실행 중: {status['running']}")
                lines.append(f"- 현재 작업: {status['current_task'] or '없음'}")
                lines.append(f"- 완료: {status['success']}건 / 실패: {status['failed']}건")
                lines.append("")

        # 작업 상세
        for status_name, label in [
            ("in_progress", "진행 중 작업"),
            ("pending", "대기 중 작업"),
            ("completed", "완료된 작업"),
            ("failed", "실패한 작업"),
        ]:
            tasks = progress["tasks_by_status"].get(status_name, [])
            if tasks:
                lines.append(f"## {label}")
                lines.append("")
                for t in tasks:
                    lines.append(f"- [{t['id']}] {t['title']} (할당: {t['assignee'] or '미할당'})")
                lines.append("")

        return "\n".join(lines)

    async def _coordinate(self, plan: str) -> dict:
        """계획을 세부 작업으로 분해하여 에이전트에 배분.

        Args:
            plan: 자연어 계획 텍스트

        Returns:
            생성된 작업 목록
        """
        tasks_created = []

        # 간단한 키워드 기반 분배
        plan_lower = plan.lower()

        if any(kw in plan_lower for kw in ["모델", "sdf", "차량", "센서", "물리", "월드"]):
            task = await self.create_task(
                title="modeling_task",
                description=plan,
                assignee="modeling",
            )
            tasks_created.append(task.to_dict())

        if any(kw in plan_lower for kw in ["조사", "검색", "논문", "연구", "자료", "리서치"]):
            task = await self.create_task(
                title="research",
                description=plan,
                assignee="research",
                params={"command": "research", "query": plan},
            )
            tasks_created.append(task.to_dict())

        if not tasks_created:
            # 기본: 연구 작업으로 할당
            task = await self.create_task(
                title="research",
                description=plan,
                assignee="research",
                params={"command": "research", "query": plan},
            )
            tasks_created.append(task.to_dict())

        return {
            "plan": plan,
            "tasks_created": len(tasks_created),
            "tasks": tasks_created,
        }

    def _all_status(self) -> dict:
        """팀장 + 모든 에이전트 상태."""
        return {
            "leader": self.report_status(),
            "agents": {
                name: agent.report_status()
                for name, agent in self._agents.items()
            },
            "progress": self.check_progress(),
        }
