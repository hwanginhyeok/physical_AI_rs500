"""월드 SDF 생성/수정 모듈."""

from __future__ import annotations

import logging
import shutil
from pathlib import Path
from xml.etree import ElementTree as ET

logger = logging.getLogger(__name__)

_PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
_WORLD_SDF = _PROJECT_ROOT / "src" / "ad_simulation" / "worlds" / "agricultural_field.sdf"


def _backup(path: Path) -> Path:
    backup = path.with_suffix(".sdf.bak")
    shutil.copy2(path, backup)
    logger.info("백업 생성: %s", backup)
    return backup


def _load_world(path: Path | None = None) -> ET.ElementTree:
    return ET.parse(path or _WORLD_SDF)


def _save_world(tree: ET.ElementTree, path: Path | None = None) -> None:
    path = path or _WORLD_SDF
    ET.indent(tree, space="  ")
    tree.write(path, encoding="unicode", xml_declaration=True)
    logger.info("월드 SDF 저장: %s", path)


def _find_world(tree: ET.ElementTree) -> ET.Element:
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError("world 요소를 찾을 수 없습니다")
    return world


def create_world(
    name: str = "custom_world",
    ground_size: tuple[float, float] = (200, 200),
    latitude: float = 36.35,
    longitude: float = 127.38,
    elevation: float = 50,
) -> str:
    """새 월드 SDF 파일 생성.

    Returns:
        생성된 파일 경로
    """
    w, h = ground_size
    sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{name}">
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{latitude}</latitude_deg>
      <longitude_deg>{longitude}</longitude_deg>
      <elevation>{elevation}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.5 0.3 -1.0</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="ground_link">
        <collision name="ground_collision">
          <geometry><plane><normal>0 0 1</normal><size>{w} {h}</size></plane></geometry>
          <surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface>
        </collision>
        <visual name="ground_visual">
          <geometry><plane><normal>0 0 1</normal><size>{w} {h}</size></plane></geometry>
          <material>
            <ambient>0.45 0.35 0.2 1</ambient>
            <diffuse>0.55 0.42 0.25 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
  </world>
</sdf>"""

    worlds_dir = _WORLD_SDF.parent
    worlds_dir.mkdir(parents=True, exist_ok=True)
    output_path = worlds_dir / f"{name}.sdf"
    output_path.write_text(sdf, encoding="utf-8")
    logger.info("월드 생성: %s", output_path)
    return str(output_path)


def add_obstacle(
    obstacle_type: str = "box",
    name: str | None = None,
    position: tuple[float, float, float] = (10, 0, 0.5),
    size: tuple[float, float, float] = (1, 1, 1),
    color: tuple[float, float, float, float] = (0.5, 0.5, 0.5, 1),
) -> str:
    """월드에 장애물 추가.

    Args:
        obstacle_type: box, cylinder, sphere
        name: 장애물 이름 (자동 생성)
        position: (x, y, z) 위치
        size: (w, h, d) 또는 (radius, length, _)
        color: (r, g, b, a) 색상

    Returns:
        추가된 장애물 이름
    """
    _backup(_WORLD_SDF)
    tree = _load_world()
    world = _find_world(tree)

    # 기존 장애물 개수로 이름 생성
    if name is None:
        count = sum(1 for m in world.iter("model") if m.get("name", "").startswith("obstacle_"))
        name = f"obstacle_{count}"

    x, y, z = position

    if obstacle_type == "box":
        geom = f"<box><size>{size[0]} {size[1]} {size[2]}</size></box>"
    elif obstacle_type == "cylinder":
        geom = f"<cylinder><radius>{size[0]}</radius><length>{size[1]}</length></cylinder>"
    elif obstacle_type == "sphere":
        geom = f"<sphere><radius>{size[0]}</radius></sphere>"
    else:
        raise ValueError(f"지원하지 않는 장애물 타입: {obstacle_type}")

    r, g, b, a = color
    model_xml = f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>{geom}</geometry>
        </collision>
        <visual name="visual">
          <geometry>{geom}</geometry>
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
          </material>
        </visual>
      </link>
    </model>"""

    model_elem = ET.fromstring(model_xml.strip())
    world.append(model_elem)
    _save_world(tree)
    logger.info("장애물 추가: %s (%s) at (%s, %s, %s)", name, obstacle_type, x, y, z)
    return name


def add_road(
    waypoints: list[tuple[float, float]],
    width: float = 4.0,
    name: str | None = None,
) -> str:
    """월드에 도로(시각적 표시) 추가.

    Args:
        waypoints: [(x1,y1), (x2,y2), ...] 경로점
        width: 도로 폭 (m)
        name: 도로 이름

    Returns:
        추가된 도로 이름
    """
    if len(waypoints) < 2:
        raise ValueError("최소 2개의 경로점이 필요합니다")

    _backup(_WORLD_SDF)
    tree = _load_world()
    world = _find_world(tree)

    if name is None:
        count = sum(1 for m in world.iter("model") if m.get("name", "").startswith("road_"))
        name = f"road_{count}"

    import math

    for i in range(len(waypoints) - 1):
        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i + 1]
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        length = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        yaw = math.atan2(y2 - y1, x2 - x1)

        seg_name = f"{name}_seg{i}"
        seg_xml = f"""
        <model name="{seg_name}">
          <static>true</static>
          <pose>{cx} {cy} 0.01 0 0 {yaw}</pose>
          <link name="link">
            <visual name="visual">
              <geometry><box><size>{length} {width} 0.02</size></box></geometry>
              <material>
                <ambient>0.3 0.3 0.3 1</ambient>
                <diffuse>0.4 0.4 0.4 1</diffuse>
              </material>
            </visual>
          </link>
        </model>"""

        seg_elem = ET.fromstring(seg_xml.strip())
        world.append(seg_elem)

    _save_world(tree)
    logger.info("도로 추가: %s (%d 세그먼트)", name, len(waypoints) - 1)
    return name


def list_models() -> list[dict[str, str]]:
    """월드 내 모델 목록 조회."""
    tree = _load_world()
    world = _find_world(tree)
    models = []
    for model in world.findall("model"):
        pose = model.findtext("pose", "0 0 0 0 0 0")
        models.append({
            "name": model.get("name", ""),
            "static": model.findtext("static", "false"),
            "pose": pose,
        })
    return models


def get_world_info() -> dict:
    """월드 정보 조회."""
    tree = _load_world()
    world = _find_world(tree)
    physics = world.find("physics")
    coords = world.find("spherical_coordinates")

    return {
        "name": world.get("name", ""),
        "physics": {
            "type": physics.get("type", "") if physics is not None else "",
            "step_size": physics.findtext("max_step_size", "") if physics is not None else "",
            "real_time_factor": physics.findtext("real_time_factor", "") if physics is not None else "",
        },
        "coordinates": {
            "latitude": coords.findtext("latitude_deg", "") if coords is not None else "",
            "longitude": coords.findtext("longitude_deg", "") if coords is not None else "",
            "elevation": coords.findtext("elevation", "") if coords is not None else "",
        },
        "models": list_models(),
        "sdf_path": str(_WORLD_SDF),
    }
