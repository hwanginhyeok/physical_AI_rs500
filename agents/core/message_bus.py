"""에이전트 간 비동기 pub/sub 메시지 버스."""

from __future__ import annotations

import asyncio
import logging
from collections import defaultdict
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Callable, Coroutine

logger = logging.getLogger(__name__)

Callback = Callable[["Message"], Coroutine[Any, Any, None]]


@dataclass
class Message:
    """메시지 버스를 통해 전달되는 메시지."""

    channel: str
    sender: str
    payload: Any
    timestamp: datetime = field(default_factory=datetime.now)


class MessageBus:
    """asyncio 기반 에이전트 간 pub/sub 메시지 버스.

    채널 예시:
        task.assign    - 작업 할당
        task.complete  - 작업 완료 보고
        task.failed    - 작업 실패 보고
        status.report  - 상태 보고
        command.*      - 명령 전달
    """

    def __init__(self) -> None:
        self._subscribers: dict[str, list[Callback]] = defaultdict(list)
        self._history: list[Message] = []

    def subscribe(self, channel: str, callback: Callback) -> None:
        """채널 구독."""
        self._subscribers[channel].append(callback)
        logger.debug("Subscribed to '%s'", channel)

    def unsubscribe(self, channel: str, callback: Callback) -> None:
        """채널 구독 해제."""
        if callback in self._subscribers[channel]:
            self._subscribers[channel].remove(callback)

    async def publish(self, channel: str, sender: str, payload: Any) -> None:
        """채널에 메시지 발행."""
        msg = Message(channel=channel, sender=sender, payload=payload)
        self._history.append(msg)
        logger.debug("[%s] %s -> %s", channel, sender, type(payload).__name__)

        # 정확한 채널 매칭 + 와일드카드 매칭
        callbacks: list[Callback] = []
        for pattern, subs in self._subscribers.items():
            if pattern == channel or (pattern.endswith(".*") and channel.startswith(pattern[:-1])):
                callbacks.extend(subs)

        await asyncio.gather(
            *(cb(msg) for cb in callbacks),
            return_exceptions=True,
        )

    def get_history(self, channel: str | None = None, limit: int = 50) -> list[Message]:
        """메시지 이력 조회."""
        msgs = self._history
        if channel:
            msgs = [m for m in msgs if m.channel == channel]
        return msgs[-limit:]
