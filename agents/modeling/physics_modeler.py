"""물리 엔진 파라미터 조정 모듈."""

from __future__ import annotations

import logging
import shutil
from pathlib import Path
from xml.etree import ElementTree as ET

logger = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
_WORLD_SDF = _PROJECT_ROOT / "src" / "ad_simulation" / "worlds" / "agricultural_field.sdf"
_MODEL_SDF = _PROJECT_ROOT / "src" / "ad_simulation" / "models" / "tracked_vehicle" / "model.sdf"


def _backup(path: Path) -> None:
    backup = path.with_suffix(".sdf.bak")
    if not backup.exists():
        shutil.copy2(path, backup)


def set_physics_engine(
    step_size: float | None = None,
    real_time_factor: float | None = None,
    real_time_update_rate: float | None = None,
    world_path: Path | None = None,
) -> dict[str, float]:
    """월드 물리 엔진 파라미터 설정.

    Returns:
        변경된 파라미터
    """
    path = world_path or _WORLD_SDF
    _backup(path)
    tree = ET.parse(path)
    physics = tree.find(".//physics")
    if physics is None:
        raise ValueError("physics 요소를 찾을 수 없습니다")

    changes: dict[str, float] = {}

    if step_size is not None:
        elem = physics.find("max_step_size")
        if elem is not None:
            elem.text = str(step_size)
            changes["max_step_size"] = step_size

    if real_time_factor is not None:
        elem = physics.find("real_time_factor")
        if elem is not None:
            elem.text = str(real_time_factor)
            changes["real_time_factor"] = real_time_factor

    if real_time_update_rate is not None:
        elem = physics.find("real_time_update_rate")
        if elem is not None:
            elem.text = str(real_time_update_rate)
            changes["real_time_update_rate"] = real_time_update_rate

    ET.indent(tree, space="  ")
    tree.write(path, encoding="unicode", xml_declaration=True)
    logger.info("물리 엔진 설정 변경: %s", changes)
    return changes


def set_surface_friction(
    model_name: str = "ground_plane",
    mu: float | None = None,
    mu2: float | None = None,
    world_path: Path | None = None,
) -> dict[str, float]:
    """월드 내 모델의 표면 마찰계수 설정.

    Returns:
        변경된 파라미터
    """
    path = world_path or _WORLD_SDF
    _backup(path)
    tree = ET.parse(path)

    changes: dict[str, float] = {}
    for model in tree.iter("model"):
        if model.get("name") != model_name:
            continue
        for collision in model.iter("collision"):
            if mu is not None:
                mu_elem = collision.find(".//mu")
                if mu_elem is not None:
                    mu_elem.text = str(mu)
                    changes["mu"] = mu
            if mu2 is not None:
                mu2_elem = collision.find(".//mu2")
                if mu2_elem is not None:
                    mu2_elem.text = str(mu2)
                    changes["mu2"] = mu2

    if not changes:
        logger.warning("모델 '%s'에서 마찰 설정을 찾을 수 없습니다", model_name)
        return changes

    ET.indent(tree, space="  ")
    tree.write(path, encoding="unicode", xml_declaration=True)
    logger.info("표면 마찰 설정 변경 (%s): %s", model_name, changes)
    return changes


def set_gravity(
    x: float = 0,
    y: float = 0,
    z: float = -9.81,
    world_path: Path | None = None,
) -> dict[str, float]:
    """월드 중력 설정.

    Returns:
        설정된 중력 값
    """
    path = world_path or _WORLD_SDF
    _backup(path)
    tree = ET.parse(path)
    world = tree.find(".//world")
    if world is None:
        raise ValueError("world 요소를 찾을 수 없습니다")

    gravity = world.find("gravity")
    if gravity is None:
        gravity = ET.SubElement(world, "gravity")
    gravity.text = f"{x} {y} {z}"

    ET.indent(tree, space="  ")
    tree.write(path, encoding="unicode", xml_declaration=True)
    logger.info("중력 설정: (%s, %s, %s)", x, y, z)
    return {"x": x, "y": y, "z": z}


def get_physics_info(world_path: Path | None = None) -> dict:
    """현재 물리 설정 조회."""
    path = world_path or _WORLD_SDF
    tree = ET.parse(path)
    physics = tree.find(".//physics")
    world = tree.find(".//world")

    gravity_elem = world.find("gravity") if world is not None else None
    gravity = gravity_elem.text if gravity_elem is not None else "0 0 -9.81"

    info = {"gravity": gravity}
    if physics is not None:
        info.update({
            "engine": physics.get("type", ""),
            "max_step_size": physics.findtext("max_step_size", ""),
            "real_time_factor": physics.findtext("real_time_factor", ""),
            "real_time_update_rate": physics.findtext("real_time_update_rate", ""),
        })
    return info
