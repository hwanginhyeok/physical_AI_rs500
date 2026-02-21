"""경북 청송군 현서면 사과 과수원 지형 기반 Gazebo 월드 생성.

대상: 경북 청송군 현서면 사과 과수원 지역
좌표: 약 36.436N, 129.057E
영역: 300m x 300m

데이터 소스 (API 키 불필요):
    1. Open Topo Data API -> SRTM 30m DEM
    2. Overpass API -> OSM 도로/건물/토지이용 데이터

사용법:
    pip install requests matplotlib numpy Pillow
    python generate_orchard_world.py
"""

from __future__ import annotations

import json
import math
import random
import sys
from pathlib import Path

try:
    import requests  # noqa: F401 – fetch_terrain_data 내부에서 사용
except ImportError:
    print("requests 필요: pip install requests")
    sys.exit(1)

# 기존 모듈에서 재사용 가능한 함수 import
from fetch_terrain_data import (
    fetch_dem_grid,
    interpolate_grid,
    dem_to_heightmap_png,
    fetch_osm_data,
    get_bounding_box,
    _clip_segment,
    _point_in_area,
    meters_to_degrees_lat,
    meters_to_degrees_lon,
)

# ==================== 설정 ====================

# 청송군 현서면 사과 과수원 중심 좌표 (WGS84)
CENTER_LAT = 36.436
CENTER_LON = 129.057

# 영역 크기 (미터)
AREA_SIZE = 300  # 300m x 300m

# Heightmap 크기 (2^n + 1)
HEIGHTMAP_SIZE = 257

# 출력 디렉토리
OUTPUT_DIR = Path(__file__).parent

# ==================== 과수나무 배치 설정 ====================

ROW_SPACING = 5.0       # 행간 거리 (남북 방향, m)
COL_SPACING = 4.0       # 주간 거리 (동서 방향, m)
TREE_HEIGHT_MIN = 3.0   # 나무 높이 최소 (m)
TREE_HEIGHT_MAX = 4.5   # 나무 높이 최대 (m)
TRUNK_RADIUS_MIN = 0.08 # 줄기 반경 최소 (m)
TRUNK_RADIUS_MAX = 0.15 # 줄기 반경 최대 (m)
CANOPY_RADIUS_MIN = 1.2 # 수관 반경 최소 (m)
CANOPY_RADIUS_MAX = 2.0 # 수관 반경 최대 (m)
ORCHARD_RATIO = 0.60    # 전체 영역 대비 과수원 비율
ROAD_BUFFER = 6.0       # 도로에서 최소 이격 거리 (m)
OFFSET_JITTER = 0.3     # 랜덤 오프셋 범위 (+/-, m)
WORK_LANE_INTERVAL = 10 # 매 N행마다 작업 통로
WORK_LANE_WIDTH = 3.0   # 작업 통로 폭 (m)

# 사과 과일 설정
APPLE_TREE_RATIO = 10   # N그루당 1그루에 사과 배치
APPLE_COUNT_MIN = 3     # 사과 개수 최소
APPLE_COUNT_MAX = 5     # 사과 개수 최대
APPLE_RADIUS = 0.04     # 사과 구 반경 (m)


# ==================== 과수나무 배치 ====================

def generate_orchard_trees(
    osm: dict,
    area_size: float,
    seed: int = 42,
) -> tuple[list[dict], list[dict]]:
    """실제 사과 과수원 패턴으로 나무를 배치.

    Returns:
        (trees, apples) - 나무 목록, 사과 목록
    """
    rng = random.Random(seed)
    half = area_size / 2

    # 과수원 영역: 전체의 60% (중앙부)
    orchard_half = half * math.sqrt(ORCHARD_RATIO)

    # 도로 좌표 수집 (배제 영역)
    road_points: list[tuple[float, float]] = []
    for road in osm.get("roads", []):
        for coord in road["coords"]:
            if _point_in_area(coord[0], coord[1], half):
                road_points.append((coord[0], coord[1]))

    # 건물 좌표 수집
    building_zones: list[tuple[float, float, float]] = []
    for bld in osm.get("buildings", []):
        if bld["coords"]:
            cx = sum(p[0] for p in bld["coords"]) / len(bld["coords"])
            cy = sum(p[1] for p in bld["coords"]) / len(bld["coords"])
            building_zones.append((cx, cy, 12.0))

    def near_road(x: float, y: float) -> bool:
        """도로 포인트에서 ROAD_BUFFER 이내인지 확인."""
        for rx, ry in road_points:
            if math.sqrt((x - rx) ** 2 + (y - ry) ** 2) < ROAD_BUFFER:
                return True
        return False

    def near_building(x: float, y: float) -> bool:
        """건물에서 일정 거리 이내인지 확인."""
        for bx, by, br in building_zones:
            if math.sqrt((x - bx) ** 2 + (y - by) ** 2) < br:
                return True
        return False

    trees: list[dict] = []
    apples: list[dict] = []
    tree_idx = 0

    # 격자 기반 배치: 남북 방향 (y) = 행, 동서 방향 (x) = 열
    # 과수원 영역 내에서 격자 생성
    y_start = -orchard_half
    y_end = orchard_half
    x_start = -orchard_half
    x_end = orchard_half

    row_number = 0
    y = y_start
    while y <= y_end:
        row_number += 1

        # 작업 통로: 매 WORK_LANE_INTERVAL행마다 한 행 비움
        if row_number % WORK_LANE_INTERVAL == 0:
            y += WORK_LANE_WIDTH
            continue

        x = x_start
        while x <= x_end:
            # 랜덤 오프셋으로 자연스러움 추가
            jx = x + rng.uniform(-OFFSET_JITTER, OFFSET_JITTER)
            jy = y + rng.uniform(-OFFSET_JITTER, OFFSET_JITTER)

            # 영역 내 확인
            if not _point_in_area(jx, jy, half * 0.95):
                x += COL_SPACING
                continue

            # 도로 회피
            if near_road(jx, jy):
                x += COL_SPACING
                continue

            # 건물 회피
            if near_building(jx, jy):
                x += COL_SPACING
                continue

            # 나무 속성 랜덤 생성
            tree_h = rng.uniform(TREE_HEIGHT_MIN, TREE_HEIGHT_MAX)
            trunk_r = rng.uniform(TRUNK_RADIUS_MIN, TRUNK_RADIUS_MAX)
            canopy_r = rng.uniform(CANOPY_RADIUS_MIN, CANOPY_RADIUS_MAX)

            tree = {
                "idx": tree_idx,
                "x": jx,
                "y": jy,
                "height": tree_h,
                "trunk_r": trunk_r,
                "canopy_r": canopy_r,
            }
            trees.append(tree)

            # 사과 배치 (APPLE_TREE_RATIO 그루당 1그루)
            if tree_idx % APPLE_TREE_RATIO == 0:
                n_apples = rng.randint(APPLE_COUNT_MIN, APPLE_COUNT_MAX)
                for ai in range(n_apples):
                    # 수관 내 랜덤 위치
                    angle = rng.uniform(0, 2 * math.pi)
                    dist = rng.uniform(0.3, canopy_r * 0.8)
                    az = jx + dist * math.cos(angle)
                    ay = jy + dist * math.sin(angle)
                    # 높이: 줄기 상단 + 수관 중심 근처
                    trunk_top = tree_h * 0.65  # 줄기는 나무 높이의 ~65%
                    apple_z = trunk_top + rng.uniform(-canopy_r * 0.4, canopy_r * 0.3)
                    apples.append({
                        "tree_idx": tree_idx,
                        "apple_idx": ai,
                        "x": az,
                        "y": ay,
                        "z": apple_z,
                    })

            tree_idx += 1
            x += COL_SPACING

        y += ROW_SPACING

    print(f"  과수나무: {len(trees)}그루 배치")
    print(f"  사과: {len(apples)}개 배치 ({len(apples) // max(1, len([t for t in trees if t['idx'] % APPLE_TREE_RATIO == 0]))}개/나무 평균)")
    return trees, apples


# ==================== SDF 월드 생성 ====================

def generate_orchard_sdf(
    dem_stats: tuple[float, float, float],
    osm: dict,
    trees: list[dict],
    apples: list[dict],
    area_size: float,
    center_lat: float,
    center_lon: float,
    output_path: Path,
) -> None:
    """사과 과수원 Gazebo 월드 SDF 생성."""
    min_elev, max_elev, height_range = dem_stats
    base_elevation = min_elev
    half = area_size / 2

    # ---- 도로 SDF 생성 (클리핑 적용) ----
    roads_sdf = ""
    road_idx = 0
    for road in osm.get("roads", []):
        coords = road["coords"]
        tags = road.get("tags", {})

        highway_type = tags.get("highway", "")
        width_map = {
            "motorway": 7.0, "trunk": 6.0, "primary": 5.0,
            "secondary": 4.0, "tertiary": 3.5, "residential": 3.0,
            "unclassified": 3.0, "track": 2.5, "path": 1.5, "footway": 1.0,
        }
        road_width = width_map.get(highway_type, 3.0)
        surface = tags.get("surface", "")

        if surface == "asphalt":
            ambient = "0.25 0.25 0.25 1"
            diffuse = "0.35 0.35 0.35 1"
        elif surface == "concrete":
            ambient = "0.5 0.48 0.45 1"
            diffuse = "0.6 0.58 0.55 1"
        else:
            ambient = "0.4 0.35 0.28 1"
            diffuse = "0.5 0.45 0.38 1"

        for j in range(len(coords) - 1):
            x1, y1 = coords[j]
            x2, y2 = coords[j + 1]

            clipped = _clip_segment(x1, y1, x2, y2, half)
            if clipped is None:
                continue
            x1, y1, x2, y2 = clipped

            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            length = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            if length < 0.5:
                continue
            yaw = math.atan2(y2 - y1, x2 - x1)

            roads_sdf += f"""
    <model name="road_{road_idx}">
      <static>true</static>
      <pose>{cx:.1f} {cy:.1f} 0.02 0 0 {yaw:.4f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{length:.1f} {road_width:.1f} 0.04</size></box></geometry>
          <surface>
            <friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>{length:.1f} {road_width:.1f} 0.04</size></box></geometry>
          <material>
            <ambient>{ambient}</ambient>
            <diffuse>{diffuse}</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""
            road_idx += 1

    # ---- 과수나무 SDF 생성 ----
    trees_sdf = ""
    for t in trees:
        idx = t["idx"]
        tx, ty = t["x"], t["y"]
        th = t["height"]
        tr = t["trunk_r"]
        cr = t["canopy_r"]

        # 줄기 높이: 나무 전체 높이의 약 65%
        trunk_h = th * 0.65
        # 수관 중심 높이: 줄기 상단 + 수관 반경
        canopy_z = trunk_h + cr

        trees_sdf += f"""
    <model name="apple_tree_{idx}">
      <static>true</static>
      <pose>{tx:.2f} {ty:.2f} 0 0 0 0</pose>
      <link name="link">
        <!-- 줄기 (원기둥) -->
        <collision name="trunk_col">
          <pose>0 0 {trunk_h / 2:.2f} 0 0 0</pose>
          <geometry><cylinder><radius>{tr:.3f}</radius><length>{trunk_h:.2f}</length></cylinder></geometry>
        </collision>
        <visual name="trunk_vis">
          <pose>0 0 {trunk_h / 2:.2f} 0 0 0</pose>
          <geometry><cylinder><radius>{tr:.3f}</radius><length>{trunk_h:.2f}</length></cylinder></geometry>
          <material>
            <ambient>0.35 0.22 0.08 1</ambient>
            <diffuse>0.45 0.30 0.12 1</diffuse>
          </material>
        </visual>
        <!-- 수관 (구형 - 사과나무 특성) -->
        <collision name="canopy_col">
          <pose>0 0 {canopy_z:.2f} 0 0 0</pose>
          <geometry><sphere><radius>{cr:.2f}</radius></sphere></geometry>
        </collision>
        <visual name="canopy_vis">
          <pose>0 0 {canopy_z:.2f} 0 0 0</pose>
          <geometry><sphere><radius>{cr:.2f}</radius></sphere></geometry>
          <material>
            <ambient>0.04 0.28 0.06 1</ambient>
            <diffuse>0.06 0.38 0.10 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

    # ---- 사과 (빨간 구) SDF 생성 ----
    apples_sdf = ""
    for a in apples:
        tidx = a["tree_idx"]
        aidx = a["apple_idx"]
        ax, ay, az = a["x"], a["y"], a["z"]

        apples_sdf += f"""
    <model name="apple_{tidx}_{aidx}">
      <static>true</static>
      <pose>{ax:.2f} {ay:.2f} {az:.2f} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><sphere><radius>{APPLE_RADIUS}</radius></sphere></geometry>
          <material>
            <ambient>0.7 0.05 0.02 1</ambient>
            <diffuse>0.9 0.10 0.05 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

    # ---- 농업 창고 (과수원 가장자리) ----
    barn_x = half * 0.85
    barn_y = -half * 0.80
    barn_w = 12.0
    barn_d = 8.0
    barn_h = 5.0

    barn_sdf = f"""
    <model name="farm_barn">
      <static>true</static>
      <pose>{barn_x:.1f} {barn_y:.1f} {barn_h / 2:.1f} 0 0 0.15</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{barn_w} {barn_d} {barn_h}</size></box></geometry>
        </collision>
        <visual name="walls">
          <geometry><box><size>{barn_w} {barn_d} {barn_h}</size></box></geometry>
          <material>
            <ambient>0.55 0.45 0.30 1</ambient>
            <diffuse>0.65 0.55 0.40 1</diffuse>
          </material>
        </visual>
        <!-- 지붕 (간이) -->
        <visual name="roof">
          <pose>0 0 {barn_h / 2 + 0.8:.1f} 0 0 0</pose>
          <geometry><box><size>{barn_w + 1.0} {barn_d + 1.0} 0.3</size></box></geometry>
          <material>
            <ambient>0.3 0.15 0.08 1</ambient>
            <diffuse>0.4 0.2 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

    # ---- 건물 SDF (OSM) ----
    buildings_sdf = ""
    bld_count = 0
    for bld in osm.get("buildings", []):
        if not bld["coords"]:
            continue
        cx = sum(p[0] for p in bld["coords"]) / len(bld["coords"])
        cy = sum(p[1] for p in bld["coords"]) / len(bld["coords"])
        if not _point_in_area(cx, cy, half):
            continue
        xs = [p[0] for p in bld["coords"]]
        ys = [p[1] for p in bld["coords"]]
        w = max(max(xs) - min(xs), 3.0)
        h = max(max(ys) - min(ys), 3.0)
        bld_height = 4.0
        tags = bld.get("tags", {})
        try:
            bld_height = float(tags.get("building:levels", "1")) * 3.0
        except ValueError:
            pass

        buildings_sdf += f"""
    <model name="building_{bld_count}">
      <static>true</static>
      <pose>{cx:.1f} {cy:.1f} {bld_height / 2:.1f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{w:.1f} {h:.1f} {bld_height:.1f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{w:.1f} {h:.1f} {bld_height:.1f}</size></box></geometry>
          <material>
            <ambient>0.6 0.55 0.5 1</ambient>
            <diffuse>0.7 0.65 0.6 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""
        bld_count += 1

    # ---- 수계 SDF ----
    waterways_sdf = ""
    water_idx = 0
    for ww in osm.get("waterways", []):
        coords = ww["coords"]
        tags = ww.get("tags", {})
        ww_type = tags.get("waterway", "stream")
        ww_width = {"river": 8.0, "stream": 3.0, "ditch": 1.5}.get(ww_type, 3.0)

        for j in range(len(coords) - 1):
            x1, y1 = coords[j]
            x2, y2 = coords[j + 1]
            clipped = _clip_segment(x1, y1, x2, y2, half)
            if clipped is None:
                continue
            x1, y1, x2, y2 = clipped
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            length = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            if length < 1.0:
                continue
            yaw = math.atan2(y2 - y1, x2 - x1)

            waterways_sdf += f"""
    <model name="water_{water_idx}">
      <static>true</static>
      <pose>{cx:.1f} {cy:.1f} -0.15 0 0 {yaw:.4f}</pose>
      <link name="link">
        <visual name="visual">
          <geometry><box><size>{length:.1f} {ww_width:.1f} 0.02</size></box></geometry>
          <material>
            <ambient>0.05 0.15 0.4 0.9</ambient>
            <diffuse>0.1 0.25 0.55 0.9</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""
            water_idx += 1

    print(f"  건물: {bld_count}, 도로: {road_idx} 세그먼트, "
          f"수계: {water_idx} 세그먼트")
    print(f"  농업 창고: 1개 ({barn_x:.0f}, {barn_y:.0f})")

    # ---- 전체 SDF 조합 ----
    sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="cheongsong_orchard">

    <!-- 물리 엔진 설정 -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <gravity>0 0 -9.81</gravity>

    <!-- GPS 좌표 기준: 경북 청송군 현서면 사과 과수원 -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{center_lat}</latitude_deg>
      <longitude_deg>{center_lon}</longitude_deg>
      <elevation>{base_elevation:.1f}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <!-- 태양광 (한국 남부 위도 기준) -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.5 0.3 -1.0</direction>
    </light>

    <!-- 환경광 (과수원 분위기) -->
    <light type="directional" name="ambient_light">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 50 0 0 0</pose>
      <diffuse>0.35 0.35 0.3 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
      <direction>0.2 -0.3 -1.0</direction>
    </light>

    <!-- ==================== 실제 지형 (SRTM DEM 기반) ==================== -->
    <!-- 위치: 경북 청송군 현서면 사과 과수원 -->
    <!-- 고도: {min_elev:.0f}m ~ {max_elev:.0f}m (고저차 {height_range:.0f}m) -->
    <model name="real_terrain">
      <static>true</static>
      <link name="terrain_link">
        <collision name="terrain_collision">
          <geometry>
            <heightmap>
              <uri>cheongsong_heightmap.png</uri>
              <size>{area_size} {area_size} {height_range:.1f}</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
          <surface>
            <friction>
              <ode><mu>0.6</mu><mu2>0.6</mu2></ode>
            </friction>
          </surface>
        </collision>
        <visual name="terrain_visual">
          <geometry>
            <heightmap>
              <uri>cheongsong_heightmap.png</uri>
              <size>{area_size} {area_size} {height_range:.1f}</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
          <material>
            <ambient>0.30 0.35 0.15 1</ambient>
            <diffuse>0.40 0.45 0.20 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- ==================== OSM 건물 ({bld_count}개) ==================== -->
{buildings_sdf}
    <!-- ==================== OSM 도로 ({road_idx} 세그먼트) ==================== -->
{roads_sdf}
    <!-- ==================== 수계 ({water_idx} 세그먼트) ==================== -->
{waterways_sdf}
    <!-- ==================== 사과 과수나무 ({len(trees)}그루) ==================== -->
    <!-- 행간: {ROW_SPACING}m, 주간: {COL_SPACING}m -->
    <!-- 작업 통로: 매 {WORK_LANE_INTERVAL}행마다 {WORK_LANE_WIDTH}m -->
{trees_sdf}
    <!-- ==================== 사과 과일 ({len(apples)}개) ==================== -->
{apples_sdf}
    <!-- ==================== 농업 창고 ==================== -->
{barn_sdf}
    <!-- ==================== 경계 표시 ==================== -->
    <model name="marker_ne">
      <static>true</static>
      <pose>{half} {half} 1.0 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>0.15</radius><length>2.0</length></cylinder></geometry>
          <material><ambient>1 0 0 1</ambient><diffuse>1 0.2 0.2 1</diffuse></material>
        </visual>
      </link>
    </model>
    <model name="marker_nw">
      <static>true</static>
      <pose>{half} -{half} 1.0 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>0.15</radius><length>2.0</length></cylinder></geometry>
          <material><ambient>1 0 0 1</ambient><diffuse>1 0.2 0.2 1</diffuse></material>
        </visual>
      </link>
    </model>
    <model name="marker_se">
      <static>true</static>
      <pose>-{half} {half} 1.0 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>0.15</radius><length>2.0</length></cylinder></geometry>
          <material><ambient>1 0 0 1</ambient><diffuse>1 0.2 0.2 1</diffuse></material>
        </visual>
      </link>
    </model>
    <model name="marker_sw">
      <static>true</static>
      <pose>-{half} -{half} 1.0 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><cylinder><radius>0.15</radius><length>2.0</length></cylinder></geometry>
          <material><ambient>1 0 0 1</ambient><diffuse>1 0.2 0.2 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- ==================== Gazebo 플러그인 ==================== -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

  </world>
</sdf>
"""

    output_path.write_text(sdf, encoding="utf-8")
    print(f"  월드 SDF 생성: {output_path}")


# ==================== 시각화 ====================

def generate_visualization(
    trees: list[dict],
    apples: list[dict],
    osm: dict,
    area_size: float,
    heightmap_path: Path,
    dem_stats: tuple[float, float, float],
    output_path: Path,
) -> None:
    """과수원 배치를 matplotlib으로 시각화."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from PIL import Image

    half = area_size / 2
    min_elev, max_elev, height_range = dem_stats

    # Heightmap 읽기
    img = Image.open(heightmap_path)
    hmap = np.array(img, dtype=np.float64)
    hmap_meters = min_elev + (hmap / 65535.0) * height_range

    fig = plt.figure(figsize=(20, 16))
    fig.suptitle(
        f"Cheongsong Apple Orchard ({CENTER_LAT}N, {CENTER_LON}E)\n"
        f"SRTM 30m DEM + OSM Data | {AREA_SIZE}m x {AREA_SIZE}m | "
        f"Trees: {len(trees)}, Apples: {len(apples)}",
        fontsize=14, fontweight="bold",
    )

    # ---- 1. 2D 고도맵 + 나무 위치 ----
    ax1 = fig.add_subplot(2, 2, 1)
    im = ax1.imshow(
        hmap_meters, extent=[-half, half, -half, half],
        cmap="terrain", origin="lower",
    )
    plt.colorbar(im, ax=ax1, label="Elevation (m)", shrink=0.8)

    # 도로 표시
    for road in osm.get("roads", []):
        coords = road["coords"]
        if len(coords) < 2:
            continue
        xs = [c[0] for c in coords if -half <= c[0] <= half and -half <= c[1] <= half]
        ys = [c[1] for c in coords if -half <= c[0] <= half and -half <= c[1] <= half]
        if xs:
            ax1.plot(xs, ys, "w-", linewidth=1.5, alpha=0.8)

    # 수계 표시
    for ww in osm.get("waterways", []):
        coords = ww["coords"]
        xs = [c[0] for c in coords if -half <= c[0] <= half and -half <= c[1] <= half]
        ys = [c[1] for c in coords if -half <= c[0] <= half and -half <= c[1] <= half]
        if xs:
            ax1.plot(xs, ys, "c-", linewidth=2, alpha=0.9)

    # 나무 위치
    if trees:
        txs = [t["x"] for t in trees]
        tys = [t["y"] for t in trees]
        ax1.scatter(txs, tys, c="darkgreen", s=8, alpha=0.5, marker="o")

    ax1.set_title("2D Elevation + Tree Positions")
    ax1.set_xlabel("East-West (m)")
    ax1.set_ylabel("North-South (m)")
    ax1.set_xlim(-half, half)
    ax1.set_ylim(-half, half)
    ax1.set_aspect("equal")

    ax1.plot([], [], "w-", linewidth=1.5, label="Roads")
    ax1.plot([], [], "c-", linewidth=2, label="Waterways")
    ax1.scatter([], [], c="darkgreen", s=8, marker="o", label=f"Trees ({len(trees)})")
    ax1.legend(loc="upper left", fontsize=8)

    # ---- 2. 나무 배치 상세도 (중앙 100m x 100m 확대) ----
    ax2 = fig.add_subplot(2, 2, 2)
    zoom_half = 50.0

    # 배경: 연한 녹색
    ax2.set_facecolor("#e8f0e0")

    # 도로 (확대 영역 내)
    for road in osm.get("roads", []):
        coords = road["coords"]
        if len(coords) < 2:
            continue
        xs = [c[0] for c in coords]
        ys = [c[1] for c in coords]
        ax2.plot(xs, ys, "-", color="#8B7355", linewidth=3, alpha=0.8)

    # 과수원 영역 표시
    orchard_half = half * math.sqrt(ORCHARD_RATIO)
    from matplotlib.patches import Rectangle
    rect = Rectangle(
        (-orchard_half, -orchard_half), 2 * orchard_half, 2 * orchard_half,
        linewidth=1.5, edgecolor="green", facecolor="none", linestyle="--",
        label="Orchard area",
    )
    ax2.add_patch(rect)

    # 나무 (확대 영역)
    zoom_trees = [t for t in trees
                  if -zoom_half <= t["x"] <= zoom_half and -zoom_half <= t["y"] <= zoom_half]
    if zoom_trees:
        for t in zoom_trees:
            # 수관 원
            circle = plt.Circle(
                (t["x"], t["y"]), t["canopy_r"],
                color="#2d6a2d", alpha=0.35,
            )
            ax2.add_patch(circle)
            # 줄기 점
            ax2.plot(t["x"], t["y"], "o", color="#5c3317", markersize=2)

    # 사과 (확대 영역)
    zoom_apples = [a for a in apples
                   if -zoom_half <= a["x"] <= zoom_half and -zoom_half <= a["y"] <= zoom_half]
    if zoom_apples:
        axs_ = [a["x"] for a in zoom_apples]
        ays_ = [a["y"] for a in zoom_apples]
        ax2.scatter(axs_, ays_, c="red", s=12, marker="o", alpha=0.8, zorder=5)

    ax2.set_title(f"Detail View (center {int(zoom_half * 2)}m x {int(zoom_half * 2)}m)")
    ax2.set_xlabel("East-West (m)")
    ax2.set_ylabel("North-South (m)")
    ax2.set_xlim(-zoom_half, zoom_half)
    ax2.set_ylim(-zoom_half, zoom_half)
    ax2.set_aspect("equal")
    ax2.grid(True, alpha=0.2)

    ax2.scatter([], [], c="#2d6a2d", s=30, marker="o", alpha=0.5, label=f"Trees ({len(zoom_trees)})")
    ax2.scatter([], [], c="red", s=12, marker="o", label=f"Apples ({len(zoom_apples)})")
    ax2.legend(loc="upper left", fontsize=8)

    # ---- 3. 전체 Object Layout ----
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.imshow(
        hmap_meters, extent=[-half, half, -half, half],
        cmap="Greens", origin="lower", alpha=0.3,
    )

    # 도로
    for road in osm.get("roads", []):
        coords = road["coords"]
        xs = [c[0] for c in coords if -half <= c[0] <= half and -half <= c[1] <= half]
        ys = [c[1] for c in coords if -half <= c[0] <= half and -half <= c[1] <= half]
        if xs:
            ax3.plot(xs, ys, "-", color="#8B7355", linewidth=2, alpha=0.9)

    # 수계
    for ww in osm.get("waterways", []):
        coords = ww["coords"]
        xs = [c[0] for c in coords if -half <= c[0] <= half and -half <= c[1] <= half]
        ys = [c[1] for c in coords if -half <= c[0] <= half and -half <= c[1] <= half]
        if xs:
            ax3.plot(xs, ys, "-", color="#4488CC", linewidth=3, alpha=0.8)

    # 나무 전체
    if trees:
        txs = [t["x"] for t in trees]
        tys = [t["y"] for t in trees]
        ax3.scatter(txs, tys, c="#228B22", s=12, marker="^", alpha=0.6,
                    label=f"Apple Trees ({len(trees)})")

    # 사과
    if apples:
        axs_all = [a["x"] for a in apples]
        ays_all = [a["y"] for a in apples]
        ax3.scatter(axs_all, ays_all, c="red", s=6, marker="o", alpha=0.7,
                    label=f"Apples ({len(apples)})")

    # 농업 창고
    barn_x = half * 0.85
    barn_y = -half * 0.80
    ax3.plot(barn_x, barn_y, "s", color="#8B4513", markersize=10,
             label="Farm barn")

    # 경계
    ax3.plot([half, half, -half, -half, half],
             [half, -half, -half, half, half],
             "r--", linewidth=1, alpha=0.5, label="Area boundary")

    # 과수원 영역
    rect2 = Rectangle(
        (-orchard_half, -orchard_half), 2 * orchard_half, 2 * orchard_half,
        linewidth=1.5, edgecolor="green", facecolor="none", linestyle="--",
    )
    ax3.add_patch(rect2)

    ax3.set_title("Full Object Layout")
    ax3.set_xlabel("East-West (m)")
    ax3.set_ylabel("North-South (m)")
    ax3.set_xlim(-half - 10, half + 10)
    ax3.set_ylim(-half - 10, half + 10)
    ax3.set_aspect("equal")
    ax3.legend(loc="upper left", fontsize=7)

    # ---- 4. 통계 + 행 패턴 확인 ----
    ax4 = fig.add_subplot(2, 2, 4)

    # y 좌표 히스토그램 (행 패턴 확인)
    if trees:
        tys_all = [t["y"] for t in trees]
        ax4.hist(tys_all, bins=80, orientation="horizontal",
                 color="#228B22", alpha=0.6, edgecolor="none")

        ax4.set_title("Row Distribution (Y-axis)")
        ax4.set_xlabel("Tree count per bin")
        ax4.set_ylabel("North-South position (m)")

        # 작업 통로 위치 표시
        orchard_half_val = half * math.sqrt(ORCHARD_RATIO)
        y_pos = -orchard_half_val
        row_num = 0
        while y_pos <= orchard_half_val:
            row_num += 1
            if row_num % WORK_LANE_INTERVAL == 0:
                ax4.axhline(y=y_pos, color="orange", linewidth=1.5, alpha=0.7,
                            linestyle="--")
                y_pos += WORK_LANE_WIDTH
                continue
            y_pos += ROW_SPACING

        ax4.axhline(y=0, color="gray", linewidth=0.5, alpha=0.3)
        ax4.plot([], [], "--", color="orange", label="Work lanes")
        ax4.legend(loc="upper right", fontsize=8)

    # 통계 텍스트
    stats_text = (
        f"Area: {AREA_SIZE}m x {AREA_SIZE}m\n"
        f"Elevation: {min_elev:.0f}m - {max_elev:.0f}m (range {height_range:.0f}m)\n"
        f"Row spacing: {ROW_SPACING}m | Col spacing: {COL_SPACING}m\n"
        f"Trees: {len(trees)} | Apples: {len(apples)}\n"
        f"Roads: {len(osm.get('roads', []))} | "
        f"Waterways: {len(osm.get('waterways', []))}\n"
        f"Work lanes: every {WORK_LANE_INTERVAL} rows ({WORK_LANE_WIDTH}m wide)"
    )
    ax4.text(
        0.98, 0.02, stats_text, transform=ax4.transAxes,
        fontsize=8, verticalalignment="bottom", horizontalalignment="right",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8),
    )

    plt.tight_layout()
    plt.savefig(str(output_path), dpi=150, bbox_inches="tight")
    print(f"  시각화 저장: {output_path}")
    plt.close()


# ==================== 메인 ====================

def main() -> None:
    print("=" * 60)
    print("  청송 사과 과수원 Gazebo 월드 생성")
    print(f"  위치: 경북 청송군 현서면 ({CENTER_LAT}N, {CENTER_LON}E)")
    print(f"  영역: {AREA_SIZE}m x {AREA_SIZE}m")
    print("=" * 60)

    output_dir = OUTPUT_DIR
    output_dir.mkdir(parents=True, exist_ok=True)

    # 1. DEM 수집
    print("\n[1/5] DEM 고도 데이터 수집 (SRTM 30m)...")
    grid_size = 11  # 11x11 = 121 포인트 (2 API 요청, 300m 영역에 적합)
    dem = fetch_dem_grid(CENTER_LAT, CENTER_LON, AREA_SIZE, grid_size)

    flat = [v for row in dem for v in row]
    print(f"  수집 완료: {grid_size}x{grid_size} 격자")
    print(f"  고도 범위: {min(flat):.1f}m ~ {max(flat):.1f}m")

    # 2. Heightmap 생성
    print(f"\n[2/5] Heightmap PNG 생성 ({HEIGHTMAP_SIZE}x{HEIGHTMAP_SIZE})...")
    heightmap_path = output_dir / "cheongsong_heightmap.png"
    dem_stats = dem_to_heightmap_png(dem, heightmap_path, HEIGHTMAP_SIZE)
    print(f"  저장: {heightmap_path}")
    print(f"  고저차: {dem_stats[2]:.1f}m ({dem_stats[0]:.1f}m ~ {dem_stats[1]:.1f}m)")

    # 3. OSM 데이터 수집
    print("\n[3/5] OSM 데이터 수집 (도로, 건물, 토지이용)...")
    osm = fetch_osm_data(CENTER_LAT, CENTER_LON, AREA_SIZE)

    # OSM 데이터 저장
    osm_json_path = output_dir / "cheongsong_osm_data.json"
    osm_serializable = {}
    for key, items in osm.items():
        osm_serializable[key] = [
            {"tags": item["tags"], "coords": [[c[0], c[1]] for c in item["coords"]]}
            for item in items
        ]
    osm_json_path.write_text(
        json.dumps(osm_serializable, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(f"  OSM 데이터 저장: {osm_json_path}")

    # 4. 과수나무 배치 + SDF 생성
    print("\n[4/5] 과수나무 배치 및 월드 SDF 생성...")
    trees, apples = generate_orchard_trees(osm, AREA_SIZE)

    sdf_path = output_dir / "cheongsong_orchard.sdf"
    generate_orchard_sdf(
        dem_stats, osm, trees, apples,
        AREA_SIZE, CENTER_LAT, CENTER_LON, sdf_path,
    )

    # 5. 시각화
    print("\n[5/5] 시각화 이미지 생성...")
    vis_path = output_dir / "cheongsong_visualization.png"
    generate_visualization(
        trees, apples, osm, AREA_SIZE,
        heightmap_path, dem_stats, vis_path,
    )

    # 요약
    print("\n" + "=" * 60)
    print("  생성 완료!")
    print(f"  - Heightmap: {heightmap_path}")
    print(f"  - OSM 데이터: {osm_json_path}")
    print(f"  - 월드 SDF: {sdf_path}")
    print(f"  - 시각화: {vis_path}")
    print()
    print(f"  과수나무: {len(trees)}그루")
    print(f"  사과: {len(apples)}개")
    print(f"  행간: {ROW_SPACING}m, 주간: {COL_SPACING}m")
    print(f"  작업 통로: 매 {WORK_LANE_INTERVAL}행마다")
    print()
    print("  시뮬레이션 실행:")
    print(f"    gz sim -r {sdf_path}")
    print("=" * 60)


if __name__ == "__main__":
    main()
