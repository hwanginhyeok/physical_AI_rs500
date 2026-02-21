"""차체 SDF 모델 생성/수정 모듈."""

from __future__ import annotations

import logging
import shutil
from pathlib import Path
from xml.etree import ElementTree as ET

logger = logging.getLogger(__name__)

# 프로젝트 루트 기준 SDF 경로
_PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
_MODEL_SDF = _PROJECT_ROOT / "src" / "ad_simulation" / "models" / "tracked_vehicle" / "model.sdf"
_BRIDGE_CONFIG = _PROJECT_ROOT / "src" / "ad_simulation" / "config" / "bridge_config.yaml"

# 센서 SDF 템플릿
_SENSOR_TEMPLATES: dict[str, str] = {
    "camera": """
      <sensor name="{name}" type="camera">
        <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
        <always_on>true</always_on>
        <update_rate>{rate}</update_rate>
        <topic>{topic}</topic>
        <camera>
          <horizontal_fov>{fov}</horizontal_fov>
          <image>
            <width>{width}</width>
            <height>{height}</height>
            <format>R8G8B8</format>
          </image>
          <clip><near>0.1</near><far>100</far></clip>
        </camera>
      </sensor>""",
    "lidar": """
      <sensor name="{name}" type="gpu_lidar">
        <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
        <always_on>true</always_on>
        <update_rate>{rate}</update_rate>
        <topic>{topic}</topic>
        <lidar>
          <scan>
            <horizontal>
              <samples>{h_samples}</samples>
              <resolution>1</resolution>
              <min_angle>-3.14159</min_angle>
              <max_angle>3.14159</max_angle>
            </horizontal>
            <vertical>
              <samples>{v_samples}</samples>
              <resolution>1</resolution>
              <min_angle>-0.2618</min_angle>
              <max_angle>0.2618</max_angle>
            </vertical>
          </scan>
          <range>
            <min>{range_min}</min>
            <max>{range_max}</max>
            <resolution>0.01</resolution>
          </range>
        </lidar>
      </sensor>""",
    "imu": """
      <sensor name="{name}" type="imu">
        <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
        <always_on>true</always_on>
        <update_rate>{rate}</update_rate>
        <topic>{topic}</topic>
        <imu>
          <angular_velocity>
            <x><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.01</stddev></noise></z>
          </angular_velocity>
          <linear_acceleration>
            <x><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></x>
            <y><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></y>
            <z><noise type="gaussian"><mean>0</mean><stddev>0.1</stddev></noise></z>
          </linear_acceleration>
        </imu>
      </sensor>""",
    "gps": """
      <sensor name="{name}" type="navsat">
        <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
        <always_on>true</always_on>
        <update_rate>{rate}</update_rate>
        <topic>{topic}</topic>
        <navsat>
          <position_sensing>
            <horizontal>
              <noise type="gaussian"><mean>0</mean><stddev>{h_noise}</stddev></noise>
            </horizontal>
            <vertical>
              <noise type="gaussian"><mean>0</mean><stddev>{v_noise}</stddev></noise>
            </vertical>
          </position_sensing>
        </navsat>
      </sensor>""",
}

# 센서 타입별 기본 브릿지 매핑
_BRIDGE_TEMPLATES: dict[str, dict] = {
    "camera": {
        "ros_type_name": "sensor_msgs/msg/Image",
        "gz_type_name": "gz.msgs.Image",
        "direction": "GZ_TO_ROS",
    },
    "lidar": {
        "ros_type_name": "sensor_msgs/msg/LaserScan",
        "gz_type_name": "gz.msgs.LaserScan",
        "direction": "GZ_TO_ROS",
    },
    "imu": {
        "ros_type_name": "sensor_msgs/msg/Imu",
        "gz_type_name": "gz.msgs.IMU",
        "direction": "GZ_TO_ROS",
    },
    "gps": {
        "ros_type_name": "sensor_msgs/msg/NavSatFix",
        "gz_type_name": "gz.msgs.NavSat",
        "direction": "GZ_TO_ROS",
    },
}


def _backup(path: Path) -> Path:
    """SDF 파일 백업."""
    backup = path.with_suffix(".sdf.bak")
    shutil.copy2(path, backup)
    logger.info("백업 생성: %s", backup)
    return backup


def _load_sdf(path: Path | None = None) -> ET.ElementTree:
    path = path or _MODEL_SDF
    return ET.parse(path)


def _save_sdf(tree: ET.ElementTree, path: Path | None = None) -> None:
    path = path or _MODEL_SDF
    ET.indent(tree, space="  ")
    tree.write(path, encoding="unicode", xml_declaration=True)
    logger.info("SDF 저장: %s", path)


def _find_chassis(tree: ET.ElementTree) -> ET.Element:
    """chassis 링크 요소를 찾는다."""
    for link in tree.iter("link"):
        if link.get("name") == "chassis":
            return link
    raise ValueError("chassis 링크를 찾을 수 없습니다")


def create_vehicle(
    name: str = "tracked_vehicle",
    mass: float = 1000.0,
    size: tuple[float, float, float] = (2.0, 1.2, 0.8),
    track_mass: float = 50.0,
) -> str:
    """새 차량 모델 SDF 파일 생성.

    Returns:
        생성된 파일 경로
    """
    length, width, height = size
    track_width = 0.3
    track_height = 0.4
    track_y = width / 2 + track_width / 2

    sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>false</static>
    <link name="chassis">
      <pose>0 0 {height / 2} 0 0 0</pose>
      <inertial>
        <mass>{mass}</mass>
      </inertial>
      <collision name="chassis_collision">
        <geometry><box><size>{length} {width} {height}</size></box></geometry>
      </collision>
      <visual name="chassis_visual">
        <geometry><box><size>{length} {width} {height}</size></box></geometry>
        <material>
          <ambient>0.2 0.4 0.1 1</ambient>
          <diffuse>0.3 0.5 0.15 1</diffuse>
        </material>
      </visual>
    </link>
    <link name="left_track">
      <pose>0 {track_y} {track_height / 2} 0 0 0</pose>
      <inertial><mass>{track_mass}</mass></inertial>
      <collision name="left_track_collision">
        <geometry><box><size>{length} {track_width} {track_height}</size></box></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
      <visual name="left_track_visual">
        <geometry><box><size>{length} {track_width} {track_height}</size></box></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
      </visual>
    </link>
    <joint name="left_track_joint" type="revolute">
      <parent>chassis</parent><child>left_track</child>
      <axis><xyz>1 0 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
    </joint>
    <link name="right_track">
      <pose>0 -{track_y} {track_height / 2} 0 0 0</pose>
      <inertial><mass>{track_mass}</mass></inertial>
      <collision name="right_track_collision">
        <geometry><box><size>{length} {track_width} {track_height}</size></box></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
      <visual name="right_track_visual">
        <geometry><box><size>{length} {track_width} {track_height}</size></box></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
      </visual>
    </link>
    <joint name="right_track_joint" type="revolute">
      <parent>chassis</parent><child>right_track</child>
      <axis><xyz>1 0 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper></limit></axis>
    </joint>
    <plugin filename="gz-sim-tracked-vehicle-system" name="gz::sim::systems::TrackedVehicle">
      <left_track><link>left_track</link></left_track>
      <right_track><link>right_track</link></right_track>
      <tracks_separation>{track_y * 2}</tracks_separation>
      <track_surface_radius>{track_height / 2}</track_surface_radius>
      <steering_efficiency>0.8</steering_efficiency>
      <topic>cmd_vel</topic>
    </plugin>
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_topic>odometry</odom_topic>
      <odom_frame>odom</odom_frame>
      <robot_base_frame>chassis</robot_base_frame>
      <dimensions>3</dimensions>
    </plugin>
  </model>
</sdf>"""

    output_dir = _MODEL_SDF.parent
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir.parent / name / "model.sdf"
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(sdf, encoding="utf-8")
    logger.info("차량 모델 생성: %s", output_path)
    return str(output_path)


def modify_physics(
    mass: float | None = None,
    friction_mu: float | None = None,
    friction_mu2: float | None = None,
) -> dict[str, object]:
    """기존 tracked_vehicle 모델의 물리 파라미터 수정.

    Returns:
        변경된 파라미터 딕셔너리
    """
    _backup(_MODEL_SDF)
    tree = _load_sdf()
    changes: dict[str, object] = {}

    if mass is not None:
        chassis = _find_chassis(tree)
        mass_elem = chassis.find(".//mass")
        if mass_elem is not None:
            mass_elem.text = str(mass)
            changes["chassis_mass"] = mass
            logger.info("차체 질량 변경: %s kg", mass)

    if friction_mu is not None or friction_mu2 is not None:
        for link in tree.iter("link"):
            if link.get("name") in ("left_track", "right_track"):
                if friction_mu is not None:
                    mu = link.find(".//mu")
                    if mu is not None:
                        mu.text = str(friction_mu)
                if friction_mu2 is not None:
                    mu2 = link.find(".//mu2")
                    if mu2 is not None:
                        mu2.text = str(friction_mu2)
        if friction_mu is not None:
            changes["track_friction_mu"] = friction_mu
        if friction_mu2 is not None:
            changes["track_friction_mu2"] = friction_mu2
        logger.info("트랙 마찰계수 변경: mu=%s, mu2=%s", friction_mu, friction_mu2)

    _save_sdf(tree)
    return changes


def add_sensor(
    sensor_type: str,
    name: str | None = None,
    position: tuple[float, float, float] = (0, 0, 0),
    rotation: tuple[float, float, float] = (0, 0, 0),
    config: dict | None = None,
) -> str:
    """차량에 센서 추가.

    Args:
        sensor_type: camera, lidar, imu, gps
        name: 센서 이름 (미지정 시 자동 생성)
        position: (x, y, z) 위치
        rotation: (roll, pitch, yaw) 회전
        config: 센서별 추가 설정

    Returns:
        추가된 센서 이름
    """
    if sensor_type not in _SENSOR_TEMPLATES:
        raise ValueError(f"지원하지 않는 센서 타입: {sensor_type}. 지원: {list(_SENSOR_TEMPLATES.keys())}")

    config = config or {}
    name = name or f"{sensor_type}_sensor_{len(list(_load_sdf().iter('sensor')))}"
    topic = config.get("topic", name)

    defaults = {
        "camera": {"rate": 15, "fov": 1.3962634, "width": 640, "height": 480},
        "lidar": {"rate": 10, "h_samples": 360, "v_samples": 16, "range_min": 0.3, "range_max": 50},
        "imu": {"rate": 100},
        "gps": {"rate": 10, "h_noise": 0.5, "v_noise": 1.0},
    }

    params = {**defaults.get(sensor_type, {}), **config}
    params.update(
        name=name, topic=topic,
        x=position[0], y=position[1], z=position[2],
        roll=rotation[0], pitch=rotation[1], yaw=rotation[2],
    )

    sensor_xml = _SENSOR_TEMPLATES[sensor_type].format(**params)

    _backup(_MODEL_SDF)
    tree = _load_sdf()
    chassis = _find_chassis(tree)
    sensor_elem = ET.fromstring(sensor_xml.strip())
    chassis.append(sensor_elem)
    _save_sdf(tree)

    _update_bridge_config(sensor_type, name, topic)
    logger.info("센서 추가: %s (%s)", name, sensor_type)
    return name


def _update_bridge_config(sensor_type: str, sensor_name: str, topic: str) -> None:
    """센서 추가 시 bridge_config.yaml에 매핑 추가."""
    if sensor_type not in _BRIDGE_TEMPLATES:
        return

    bridge = _BRIDGE_TEMPLATES[sensor_type]
    gz_topic = (
        f"/world/agricultural_field/model/tracked_vehicle"
        f"/link/chassis/sensor/{sensor_name}/{topic}"
    )
    ros_topic = f"/sensor/{topic}"

    entry = (
        f"\n- ros_topic_name: {ros_topic}\n"
        f"  ros_type_name: {bridge['ros_type_name']}\n"
        f"  gz_type_name: {bridge['gz_type_name']}\n"
        f"  direction: {bridge['direction']}\n"
        f"  gz_topic_name: {gz_topic}\n"
    )

    existing = _BRIDGE_CONFIG.read_text(encoding="utf-8")
    if ros_topic in existing:
        logger.info("브릿지 설정에 이미 존재: %s", ros_topic)
        return

    _backup(_BRIDGE_CONFIG.with_suffix("")) if not _BRIDGE_CONFIG.with_suffix(".bak").exists() else None
    with open(_BRIDGE_CONFIG, "a", encoding="utf-8") as f:
        f.write(entry)
    logger.info("브릿지 설정 추가: %s -> %s", ros_topic, gz_topic)


def list_sensors() -> list[dict[str, str]]:
    """현재 모델의 센서 목록 조회."""
    tree = _load_sdf()
    sensors = []
    for sensor in tree.iter("sensor"):
        sensors.append({
            "name": sensor.get("name", ""),
            "type": sensor.get("type", ""),
            "topic": sensor.findtext("topic", ""),
        })
    return sensors


def get_model_info() -> dict:
    """현재 모델 정보 조회."""
    tree = _load_sdf()
    model = tree.getroot().find("model")
    chassis = _find_chassis(tree)

    mass = chassis.findtext(".//mass", "0")
    size = chassis.findtext(".//geometry/box/size", "0 0 0")

    tracks = {}
    for link in tree.iter("link"):
        name = link.get("name", "")
        if "track" in name:
            tracks[name] = {
                "mass": link.findtext(".//mass", "0"),
                "mu": link.findtext(".//mu", "0"),
                "mu2": link.findtext(".//mu2", "0"),
            }

    return {
        "name": model.get("name") if model is not None else "unknown",
        "chassis_mass": float(mass),
        "chassis_size": size,
        "tracks": tracks,
        "sensors": list_sensors(),
        "sdf_path": str(_MODEL_SDF),
    }
