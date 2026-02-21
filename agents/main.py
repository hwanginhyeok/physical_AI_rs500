"""에이전트 시스템 CLI 진입점."""

from __future__ import annotations

import argparse
import asyncio
import logging
import sys

from .core.message_bus import MessageBus
from .core.task import Task, TaskPriority
from .modeling.modeling_agent import ModelingAgent
from .project_leader.project_leader_agent import ProjectLeaderAgent
from .research.research_agent import ResearchAgent

logger = logging.getLogger("agents")


def _setup_logging(level: str = "INFO") -> None:
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )


class AgentSystem:
    """에이전트 시스템 전체 관리."""

    def __init__(self) -> None:
        self.bus = MessageBus()
        self.leader = ProjectLeaderAgent(self.bus)
        self.modeling = ModelingAgent(self.bus)
        self.research = ResearchAgent(self.bus)

        self.leader.register_agent(self.modeling)
        self.leader.register_agent(self.research)

    async def start(self) -> None:
        await self.leader.start()
        await self.modeling.start()
        await self.research.start()

    async def stop(self) -> None:
        await self.research.stop()
        await self.modeling.stop()
        await self.leader.stop()


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="agents",
        description="자율주행 프로젝트 AI 에이전트 시스템",
    )
    parser.add_argument("--log-level", default="INFO", help="로그 레벨")
    sub = parser.add_subparsers(dest="command")

    # model 서브커맨드
    model_parser = sub.add_parser("model", help="모델링 작업")
    model_sub = model_parser.add_subparsers(dest="target")

    # model vehicle
    vehicle = model_sub.add_parser("vehicle", help="차량 모델 조작")
    vehicle.add_argument("--mass", type=float, help="차체 질량 (kg)")
    vehicle.add_argument("--friction", type=float, help="트랙 마찰계수 (mu)")
    vehicle.add_argument("--friction2", type=float, help="트랙 마찰계수 (mu2)")
    vehicle.add_argument("--add-sensor", choices=["camera", "lidar", "imu", "gps"], help="센서 추가")
    vehicle.add_argument("--sensor-name", help="센서 이름")
    vehicle.add_argument("--sensor-pos", nargs=3, type=float, metavar=("X", "Y", "Z"), help="센서 위치")
    vehicle.add_argument("--info", action="store_true", help="차량 정보 조회")
    vehicle.add_argument("--list-sensors", action="store_true", help="센서 목록")

    # model world
    world = model_sub.add_parser("world", help="월드 조작")
    world.add_argument("--add-obstacle", choices=["box", "cylinder", "sphere"], help="장애물 추가")
    world.add_argument("--obstacle-name", help="장애물 이름")
    world.add_argument("--obstacle-pos", nargs=3, type=float, metavar=("X", "Y", "Z"), help="장애물 위치")
    world.add_argument("--obstacle-size", nargs=3, type=float, metavar=("W", "H", "D"), help="장애물 크기")
    world.add_argument("--info", action="store_true", help="월드 정보 조회")

    # model physics
    physics = model_sub.add_parser("physics", help="물리 파라미터")
    physics.add_argument("--step-size", type=float, help="시뮬레이션 스텝 크기")
    physics.add_argument("--rtf", type=float, help="실시간 팩터")
    physics.add_argument("--ground-friction", type=float, help="지면 마찰계수")
    physics.add_argument("--gravity-z", type=float, help="중력 Z 값")
    physics.add_argument("--info", action="store_true", help="물리 설정 조회")

    # research 서브커맨드
    research_parser = sub.add_parser("research", help="선행조사")
    research_parser.add_argument("query", nargs="?", help="조사 주제")
    research_parser.add_argument("--papers", action="store_true", help="논문만 검색")
    research_parser.add_argument("--web", action="store_true", help="웹만 검색")
    research_parser.add_argument("--no-save", action="store_true", help="보고서 저장 안함")
    research_parser.add_argument("--list-docs", action="store_true", help="기존 문서 목록")

    # status 서브커맨드
    sub.add_parser("status", help="프로젝트 현황")

    # coordinate 서브커맨드
    coord_parser = sub.add_parser("coordinate", help="계획을 작업으로 분해")
    coord_parser.add_argument("plan", help="계획 텍스트")

    return parser


def _print_result(result: object) -> None:
    """결과를 보기 좋게 출력."""
    if isinstance(result, str):
        print(result)
    elif isinstance(result, dict):
        _print_dict(result)
    elif isinstance(result, list):
        for item in result:
            if isinstance(item, dict):
                _print_dict(item)
                print()
            else:
                print(f"  - {item}")
    else:
        print(result)


def _print_dict(d: dict, indent: int = 0) -> None:
    prefix = "  " * indent
    for key, value in d.items():
        if isinstance(value, dict):
            print(f"{prefix}{key}:")
            _print_dict(value, indent + 1)
        elif isinstance(value, list):
            print(f"{prefix}{key}:")
            for item in value:
                if isinstance(item, dict):
                    _print_dict(item, indent + 1)
                    print()
                else:
                    print(f"{prefix}  - {item}")
        else:
            print(f"{prefix}{key}: {value}")


async def _run_model_vehicle(system: AgentSystem, args: argparse.Namespace) -> None:
    if args.info:
        task = Task(title="vehicle_info", assignee="modeling")
        task.result = {"command": "vehicle_info"}
        result = await system.modeling.execute_task(task)
        _print_result(result.result)
        return

    if args.list_sensors:
        task = Task(title="list_sensors", assignee="modeling")
        task.result = {"command": "list_sensors"}
        result = await system.modeling.execute_task(task)
        _print_result(result.result)
        return

    if args.add_sensor:
        params: dict = {"command": "add_sensor", "sensor_type": args.add_sensor}
        if args.sensor_name:
            params["name"] = args.sensor_name
        if args.sensor_pos:
            params["position"] = tuple(args.sensor_pos)
        task = Task(title="add_sensor", assignee="modeling")
        task.result = params
        result = await system.modeling.execute_task(task)
        print(f"센서 추가 완료: {result.result}")
        return

    if args.mass is not None or args.friction is not None or args.friction2 is not None:
        params = {"command": "modify_physics"}
        if args.mass is not None:
            params["mass"] = args.mass
        if args.friction is not None:
            params["friction_mu"] = args.friction
        if args.friction2 is not None:
            params["friction_mu2"] = args.friction2
        task = Task(title="modify_physics", assignee="modeling")
        task.result = params
        result = await system.modeling.execute_task(task)
        print("물리 파라미터 수정 완료:")
        _print_result(result.result)
        return

    print("옵션을 지정하세요. --help로 도움말을 확인하세요.")


async def _run_model_world(system: AgentSystem, args: argparse.Namespace) -> None:
    if args.info:
        task = Task(title="world_info", assignee="modeling")
        task.result = {"command": "world_info"}
        result = await system.modeling.execute_task(task)
        _print_result(result.result)
        return

    if args.add_obstacle:
        params: dict = {"command": "add_obstacle", "obstacle_type": args.add_obstacle}
        if args.obstacle_name:
            params["name"] = args.obstacle_name
        if args.obstacle_pos:
            params["position"] = tuple(args.obstacle_pos)
        if args.obstacle_size:
            params["size"] = tuple(args.obstacle_size)
        task = Task(title="add_obstacle", assignee="modeling")
        task.result = params
        result = await system.modeling.execute_task(task)
        print(f"장애물 추가 완료: {result.result}")
        return

    print("옵션을 지정하세요. --help로 도움말을 확인하세요.")


async def _run_model_physics(system: AgentSystem, args: argparse.Namespace) -> None:
    if args.info:
        task = Task(title="physics_info", assignee="modeling")
        task.result = {"command": "physics_info"}
        result = await system.modeling.execute_task(task)
        _print_result(result.result)
        return

    has_change = False

    if args.step_size is not None or args.rtf is not None:
        params: dict = {"command": "set_physics_engine"}
        if args.step_size is not None:
            params["step_size"] = args.step_size
        if args.rtf is not None:
            params["real_time_factor"] = args.rtf
        task = Task(title="set_physics_engine", assignee="modeling")
        task.result = params
        result = await system.modeling.execute_task(task)
        print("물리 엔진 설정 변경:")
        _print_result(result.result)
        has_change = True

    if args.ground_friction is not None:
        task = Task(title="set_surface_friction", assignee="modeling")
        task.result = {"command": "set_surface_friction", "mu": args.ground_friction, "mu2": args.ground_friction}
        result = await system.modeling.execute_task(task)
        print("지면 마찰계수 변경:")
        _print_result(result.result)
        has_change = True

    if args.gravity_z is not None:
        task = Task(title="set_gravity", assignee="modeling")
        task.result = {"command": "set_gravity", "z": args.gravity_z}
        result = await system.modeling.execute_task(task)
        print("중력 설정 변경:")
        _print_result(result.result)
        has_change = True

    if not has_change:
        print("옵션을 지정하세요. --help로 도움말을 확인하세요.")


async def _run_research(system: AgentSystem, args: argparse.Namespace) -> None:
    if args.list_docs:
        task = Task(title="list_documents", assignee="research")
        task.result = {"command": "list_documents"}
        result = await system.research.execute_task(task)
        docs = result.result
        if docs:
            print("저장된 문서:")
            _print_result(docs)
        else:
            print("저장된 문서가 없습니다.")
        return

    if not args.query:
        print("조사 주제를 입력하세요: python -m agents research \"주제\"")
        return

    if args.papers:
        task = Task(title="search_papers", description=args.query, assignee="research")
        task.result = {"command": "search_papers", "query": args.query}
        result = await system.research.execute_task(task)
        _print_result(result.result)
    elif args.web:
        task = Task(title="search", description=args.query, assignee="research")
        task.result = {"command": "search", "query": args.query}
        result = await system.research.execute_task(task)
        _print_result(result.result)
    else:
        task = Task(title="research", description=args.query, assignee="research")
        task.result = {"command": "research", "query": args.query, "save": not args.no_save}
        result = await system.research.execute_task(task)
        _print_result(result.result)


async def _run_interactive(system: AgentSystem) -> None:
    """대화형 모드."""
    print("=" * 50)
    print("  자율주행 프로젝트 에이전트 시스템")
    print("=" * 50)
    print()
    print("사용 가능한 명령:")
    print("  model vehicle --info          차량 정보")
    print("  model vehicle --mass <kg>     차체 질량 수정")
    print("  model world --info            월드 정보")
    print("  model physics --info          물리 설정")
    print("  research <주제>               선행 조사")
    print("  status                        프로젝트 현황")
    print("  coordinate <계획>             작업 분배")
    print("  help                          도움말")
    print("  quit                          종료")
    print()

    while True:
        try:
            line = input("agents> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n종료합니다.")
            break

        if not line:
            continue
        if line in ("quit", "exit", "q"):
            print("종료합니다.")
            break
        if line == "help":
            print("  model vehicle --info / --mass <kg> / --add-sensor <type>")
            print("  model world --info / --add-obstacle <type>")
            print("  model physics --info / --step-size <s> / --rtf <f>")
            print("  research <주제> / --papers / --web")
            print("  status")
            print("  coordinate <계획>")
            continue

        # 커맨드 파싱
        parts = line.split()
        try:
            parser = _build_parser()
            ns = parser.parse_args(parts)
            await _dispatch(system, ns)
        except SystemExit:
            pass
        except Exception as e:
            print(f"오류: {e}")


async def _dispatch(system: AgentSystem, args: argparse.Namespace) -> None:
    """파싱된 args에 따라 실행."""
    if args.command == "model":
        if args.target == "vehicle":
            await _run_model_vehicle(system, args)
        elif args.target == "world":
            await _run_model_world(system, args)
        elif args.target == "physics":
            await _run_model_physics(system, args)
        else:
            print("대상을 지정하세요: vehicle, world, physics")

    elif args.command == "research":
        await _run_research(system, args)

    elif args.command == "status":
        report = system.leader.generate_report()
        print(report)

    elif args.command == "coordinate":
        task = Task(title="coordinate", description=args.plan, assignee="project_leader")
        task.result = {"command": "coordinate", "plan": args.plan}
        result = await system.leader.execute_task(task)
        _print_result(result.result)

    else:
        print("명령을 지정하세요. --help로 도움말을 확인하세요.")


async def _async_main() -> None:
    parser = _build_parser()
    args = parser.parse_args()

    _setup_logging(args.log_level)

    system = AgentSystem()
    await system.start()

    try:
        if args.command:
            await _dispatch(system, args)
        else:
            await _run_interactive(system)
    finally:
        await system.stop()


def main() -> None:
    try:
        asyncio.run(_async_main())
    except KeyboardInterrupt:
        print("\n중단됨.")
        sys.exit(0)


if __name__ == "__main__":
    main()
