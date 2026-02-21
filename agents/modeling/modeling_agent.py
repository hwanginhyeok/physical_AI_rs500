"""모델링 통합 에이전트."""

from __future__ import annotations

import logging

from ..core.base_agent import BaseAgent
from ..core.message_bus import MessageBus
from ..core.task import Task
from . import physics_modeler, vehicle_modeler, world_modeler

logger = logging.getLogger(__name__)


class ModelingAgent(BaseAgent):
    """SDF 모델/월드/물리 파라미터를 조작하는 에이전트."""

    # handle_task에서 디스패치할 명령 매핑
    COMMANDS = {
        "create_vehicle",
        "modify_physics",
        "add_sensor",
        "list_sensors",
        "vehicle_info",
        "create_world",
        "add_obstacle",
        "add_road",
        "world_info",
        "set_physics_engine",
        "set_surface_friction",
        "set_gravity",
        "physics_info",
    }

    def __init__(self, bus: MessageBus) -> None:
        super().__init__("modeling", bus)

    async def handle_task(self, task: Task) -> object:
        """작업 처리. task.description에서 command와 params를 파싱한다.

        task.description 형식:
            command: <명령어>
            또는 자유 텍스트 (간단 파싱)

        task.result 예상:
            dict 형태의 params (CLI에서 전달)
        """
        params = task.result if isinstance(task.result, dict) else {}
        command = params.pop("command", None) or task.title

        dispatch = {
            # 차량 모델
            "create_vehicle": lambda: vehicle_modeler.create_vehicle(**params),
            "modify_physics": lambda: vehicle_modeler.modify_physics(**params),
            "add_sensor": lambda: vehicle_modeler.add_sensor(**params),
            "list_sensors": lambda: vehicle_modeler.list_sensors(),
            "vehicle_info": lambda: vehicle_modeler.get_model_info(),
            # 월드
            "create_world": lambda: world_modeler.create_world(**params),
            "add_obstacle": lambda: world_modeler.add_obstacle(**params),
            "add_road": lambda: world_modeler.add_road(**params),
            "world_info": lambda: world_modeler.get_world_info(),
            # 물리
            "set_physics_engine": lambda: physics_modeler.set_physics_engine(**params),
            "set_surface_friction": lambda: physics_modeler.set_surface_friction(**params),
            "set_gravity": lambda: physics_modeler.set_gravity(**params),
            "physics_info": lambda: physics_modeler.get_physics_info(),
        }

        handler = dispatch.get(command)
        if handler is None:
            raise ValueError(
                f"알 수 없는 모델링 명령: {command}\n"
                f"사용 가능한 명령: {', '.join(sorted(dispatch.keys()))}"
            )

        logger.info("[modeling] 명령 실행: %s", command)
        return handler()
