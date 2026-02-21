"""실제 지형 데이터 수집 및 Gazebo 월드 생성 파이프라인.

대상: 강원특별자치도 영월군 주천면 신일리 산435-1
좌표: 약 37.26°N, 128.24°E (신일리 중심부)

데이터 소스 (API 키 불필요):
    1. Open Topo Data API → SRTM 30m DEM (격자 고도 데이터)
    2. Overpass API → OSM 도로/건물/토지이용 데이터

사용법:
    pip install requests aiohttp
    python fetch_terrain_data.py
"""

from __future__ import annotations

import json
import math
import struct
import sys
import time
import zlib
from pathlib import Path

try:
    import requests
except ImportError:
    print("requests 필요: pip install requests")
    sys.exit(1)

# ==================== 설정 ====================

# 신일리 중심 좌표 (WGS84)
CENTER_LAT = 37.2599
CENTER_LON = 128.2404

# 영역 크기 (미터)
AREA_SIZE = 500  # 500m x 500m

# DEM 격자 해상도 (미터)
DEM_RESOLUTION = 30  # SRTM 30m

# 출력 디렉토리
OUTPUT_DIR = Path(__file__).parent

# Heightmap 크기 (2^n + 1)
HEIGHTMAP_SIZE = 257


# ==================== 좌표 변환 ====================

def meters_to_degrees_lat(meters: float) -> float:
    """미터를 위도 변화량으로 변환."""
    return meters / 111320.0


def meters_to_degrees_lon(meters: float, lat: float) -> float:
    """미터를 경도 변화량으로 변환 (위도 보정)."""
    return meters / (111320.0 * math.cos(math.radians(lat)))


def get_bounding_box(
    center_lat: float, center_lon: float, size_m: float
) -> tuple[float, float, float, float]:
    """중심 좌표와 크기로 바운딩 박스 계산.

    Returns:
        (south, west, north, east) in degrees
    """
    half = size_m / 2
    dlat = meters_to_degrees_lat(half)
    dlon = meters_to_degrees_lon(half, center_lat)
    return (
        center_lat - dlat,
        center_lon - dlon,
        center_lat + dlat,
        center_lon + dlon,
    )


# ==================== DEM 데이터 수집 ====================

def fetch_dem_grid(
    center_lat: float,
    center_lon: float,
    size_m: float,
    grid_size: int = 17,
) -> list[list[float]]:
    """Open Topo Data API로 DEM 격자 데이터 수집.

    Args:
        center_lat: 중심 위도
        center_lon: 중심 경도
        size_m: 영역 크기 (미터)
        grid_size: 격자 크기 (한 변의 포인트 수, API 제한 고려)

    Returns:
        2D 고도 배열 [row][col] (남→북, 서→동)
    """
    south, west, north, east = get_bounding_box(center_lat, center_lon, size_m)

    lats = [south + (north - south) * i / (grid_size - 1) for i in range(grid_size)]
    lons = [west + (east - west) * j / (grid_size - 1) for j in range(grid_size)]

    # Open Topo Data API: 최대 100개 좌표/요청
    all_points = []
    for lat in lats:
        for lon in lons:
            all_points.append((lat, lon))

    elevations = []
    batch_size = 100
    for i in range(0, len(all_points), batch_size):
        batch = all_points[i : i + batch_size]
        locations = "|".join(f"{lat},{lon}" for lat, lon in batch)
        url = f"https://api.opentopodata.org/v1/srtm30m?locations={locations}"

        print(f"  DEM 요청 {i // batch_size + 1}/{math.ceil(len(all_points) / batch_size)}...")
        resp = requests.get(url, timeout=30)
        if resp.status_code != 200:
            print(f"  경고: HTTP {resp.status_code}, 기본값 사용")
            elevations.extend([300.0] * len(batch))
            continue

        data = resp.json()
        for result in data.get("results", []):
            elev = result.get("elevation")
            elevations.append(elev if elev is not None else 300.0)

        # API rate limit 대응
        if i + batch_size < len(all_points):
            time.sleep(1.1)

    # 2D 배열로 변환
    grid = []
    for r in range(grid_size):
        row = []
        for c in range(grid_size):
            row.append(elevations[r * grid_size + c])
        grid.append(row)

    return grid


def interpolate_grid(grid: list[list[float]], target_size: int) -> list[list[float]]:
    """바이리니어 보간으로 격자를 target_size x target_size로 확대."""
    src_h = len(grid)
    src_w = len(grid[0])
    result = [[0.0] * target_size for _ in range(target_size)]

    for y in range(target_size):
        for x in range(target_size):
            # 원본 좌표 매핑
            src_y = y / (target_size - 1) * (src_h - 1)
            src_x = x / (target_size - 1) * (src_w - 1)

            y0 = int(src_y)
            x0 = int(src_x)
            y1 = min(y0 + 1, src_h - 1)
            x1 = min(x0 + 1, src_w - 1)

            fy = src_y - y0
            fx = src_x - x0

            v00 = grid[y0][x0]
            v01 = grid[y0][x1]
            v10 = grid[y1][x0]
            v11 = grid[y1][x1]

            val = (
                v00 * (1 - fx) * (1 - fy)
                + v01 * fx * (1 - fy)
                + v10 * (1 - fx) * fy
                + v11 * fx * fy
            )
            result[y][x] = val

    return result


# ==================== OSM 데이터 수집 ====================

def fetch_osm_data(
    center_lat: float, center_lon: float, size_m: float
) -> dict:
    """Overpass API로 OSM 데이터 수집 (도로, 건물, 토지이용, 수계).

    Returns:
        {"roads": [...], "buildings": [...], "landuse": [...], "waterways": [...]}
    """
    south, west, north, east = get_bounding_box(center_lat, center_lon, size_m)
    bbox = f"{south},{west},{north},{east}"

    query = f"""
    [out:json][timeout:60];
    (
      way["highway"]({bbox});
      way["building"]({bbox});
      way["landuse"]({bbox});
      way["waterway"]({bbox});
      way["natural"="wood"]({bbox});
      way["natural"="water"]({bbox});
    );
    out body;
    >;
    out skel qt;
    """

    print("  OSM 데이터 요청 중...")
    url = "https://overpass-api.de/api/interpreter"
    resp = requests.post(url, data={"data": query}, timeout=60)

    if resp.status_code != 200:
        print(f"  경고: Overpass API HTTP {resp.status_code}")
        return {"roads": [], "buildings": [], "landuse": [], "waterways": []}

    data = resp.json()

    # 노드 딕셔너리 구축
    nodes: dict[int, tuple[float, float]] = {}
    for elem in data.get("elements", []):
        if elem["type"] == "node":
            nodes[elem["id"]] = (elem["lat"], elem["lon"])

    # Way 분류
    roads = []
    buildings = []
    landuse = []
    waterways = []

    for elem in data.get("elements", []):
        if elem["type"] != "way":
            continue
        tags = elem.get("tags", {})
        coords = []
        for nid in elem.get("nodes", []):
            if nid in nodes:
                lat, lon = nodes[nid]
                # 로컬 좌표로 변환 (중심 기준 미터)
                dy = (lat - center_lat) * 111320.0
                dx = (lon - center_lon) * 111320.0 * math.cos(math.radians(center_lat))
                coords.append((dx, dy))

        if not coords:
            continue

        entry = {"tags": tags, "coords": coords}

        if "highway" in tags:
            roads.append(entry)
        elif "building" in tags:
            buildings.append(entry)
        elif "landuse" in tags:
            landuse.append(entry)
        elif "waterway" in tags:
            waterways.append(entry)
        elif tags.get("natural") in ("wood", "water"):
            landuse.append(entry)

    print(f"  OSM: 도로 {len(roads)}, 건물 {len(buildings)}, "
          f"토지 {len(landuse)}, 수계 {len(waterways)}")

    return {
        "roads": roads,
        "buildings": buildings,
        "landuse": landuse,
        "waterways": waterways,
    }


# ==================== Heightmap PNG 생성 ====================

def dem_to_heightmap_png(
    dem: list[list[float]],
    output_path: Path,
    size: int = HEIGHTMAP_SIZE,
) -> tuple[float, float, float]:
    """DEM 데이터를 16bit PNG heightmap으로 변환.

    Returns:
        (min_elevation, max_elevation, height_range) in meters
    """
    # 보간
    if len(dem) != size or len(dem[0]) != size:
        dem = interpolate_grid(dem, size)

    flat = [v for row in dem for v in row]
    min_elev = min(flat)
    max_elev = max(flat)
    height_range = max_elev - min_elev

    if height_range < 0.01:
        height_range = 1.0

    # 정규화 (0 ~ 65535)
    pixels = []
    for row in dem:
        prow = []
        for val in row:
            normalized = (val - min_elev) / height_range
            prow.append(int(normalized * 65535))
        pixels.append(prow)

    # PNG 저장
    png_data = _make_png_16bit(size, size, pixels)
    output_path.write_bytes(png_data)

    return min_elev, max_elev, height_range


def _make_png_16bit(width: int, height: int, pixels: list[list[int]]) -> bytes:
    """16bit grayscale PNG 생성 (순수 Python)."""

    def chunk(chunk_type: bytes, data: bytes) -> bytes:
        c = chunk_type + data
        return (
            struct.pack(">I", len(data))
            + c
            + struct.pack(">I", zlib.crc32(c) & 0xFFFFFFFF)
        )

    sig = b"\x89PNG\r\n\x1a\n"
    ihdr = chunk(
        b"IHDR", struct.pack(">IIBBBBB", width, height, 16, 0, 0, 0, 0)
    )

    raw = b""
    for row in pixels:
        raw += b"\x00"
        for val in row:
            raw += struct.pack(">H", min(max(val, 0), 65535))

    idat = chunk(b"IDAT", zlib.compress(raw, 9))
    iend = chunk(b"IEND", b"")

    return sig + ihdr + idat + iend


# ==================== 좌표 클리핑 ====================

def _clip_segment(
    x1: float, y1: float, x2: float, y2: float, half: float
) -> tuple[float, float, float, float] | None:
    """선분을 바운딩 박스 [-half, half] 내로 클리핑 (Cohen-Sutherland).

    Returns:
        클리핑된 (x1, y1, x2, y2) 또는 영역 밖이면 None
    """
    INSIDE, LEFT, RIGHT, BOTTOM, TOP = 0, 1, 2, 4, 8

    def code(x: float, y: float) -> int:
        c = INSIDE
        if x < -half: c |= LEFT
        elif x > half: c |= RIGHT
        if y < -half: c |= BOTTOM
        elif y > half: c |= TOP
        return c

    c1, c2 = code(x1, y1), code(x2, y2)
    for _ in range(20):
        if not (c1 | c2):
            return (x1, y1, x2, y2)
        if c1 & c2:
            return None
        c = c1 or c2
        if c & TOP:
            x = x1 + (x2 - x1) * (half - y1) / (y2 - y1)
            y = half
        elif c & BOTTOM:
            x = x1 + (x2 - x1) * (-half - y1) / (y2 - y1)
            y = -half
        elif c & RIGHT:
            y = y1 + (y2 - y1) * (half - x1) / (x2 - x1)
            x = half
        elif c & LEFT:
            y = y1 + (y2 - y1) * (-half - x1) / (x2 - x1)
            x = -half
        if c == c1:
            x1, y1 = x, y
            c1 = code(x1, y1)
        else:
            x2, y2 = x, y
            c2 = code(x2, y2)
    return None


def _point_in_area(x: float, y: float, half: float) -> bool:
    """점이 영역 내에 있는지 확인."""
    return -half <= x <= half and -half <= y <= half


# ==================== 식생 배치 ====================

def _generate_vegetation(
    dem: list[list[float]],
    osm: dict,
    area_size: float,
    heightmap_size: int,
    seed: int = 42,
) -> list[dict]:
    """DEM 기반으로 도로/건물을 피해 나무를 배치.

    산악 지역 특성: 경사면과 능선에 나무가 밀집, 도로/하천 주변은 제외.
    """
    import random
    rng = random.Random(seed)
    half = area_size / 2

    # 도로/건물 좌표를 수집하여 배제 영역 생성
    exclusion_zones: list[tuple[float, float, float]] = []  # (x, y, radius)

    for road in osm.get("roads", []):
        for coord in road["coords"]:
            if _point_in_area(coord[0], coord[1], half):
                exclusion_zones.append((coord[0], coord[1], 8.0))

    for bld in osm.get("buildings", []):
        if bld["coords"]:
            cx = sum(p[0] for p in bld["coords"]) / len(bld["coords"])
            cy = sum(p[1] for p in bld["coords"]) / len(bld["coords"])
            exclusion_zones.append((cx, cy, 15.0))

    for ww in osm.get("waterways", []):
        for coord in ww["coords"]:
            if _point_in_area(coord[0], coord[1], half):
                exclusion_zones.append((coord[0], coord[1], 6.0))

    trees = []
    num_trees = 80  # 산악 지역에 적절한 나무 수

    for _ in range(num_trees * 5):  # 충분한 후보 생성
        if len(trees) >= num_trees:
            break

        x = rng.uniform(-half * 0.9, half * 0.9)
        y = rng.uniform(-half * 0.9, half * 0.9)

        # 배제 영역 확인
        excluded = False
        for ex, ey, er in exclusion_zones:
            if math.sqrt((x - ex) ** 2 + (y - ey) ** 2) < er:
                excluded = True
                break
        if excluded:
            continue

        # 다른 나무와의 최소 간격 (5m)
        too_close = False
        for t in trees:
            if math.sqrt((x - t["x"]) ** 2 + (y - t["y"]) ** 2) < 5.0:
                too_close = True
                break
        if too_close:
            continue

        # DEM에서 높이 추정 (heightmap 픽셀 좌표로 변환)
        px = int((x + half) / area_size * (heightmap_size - 1))
        py = int((y + half) / area_size * (heightmap_size - 1))
        px = max(0, min(px, heightmap_size - 1))
        py = max(0, min(py, heightmap_size - 1))

        # 나무 종류 결정 (침엽수/활엽수 혼합 - 강원도 특성)
        tree_type = rng.choice(["conifer", "deciduous", "conifer"])
        trunk_h = rng.uniform(3.0, 6.0)
        trunk_r = rng.uniform(0.12, 0.25)

        if tree_type == "conifer":
            canopy_h = rng.uniform(2.0, 4.0)
            canopy_r = rng.uniform(1.0, 2.0)
        else:
            canopy_h = 0.0  # 구형 캐노피
            canopy_r = rng.uniform(1.5, 3.0)

        trees.append({
            "x": x, "y": y,
            "type": tree_type,
            "trunk_h": trunk_h, "trunk_r": trunk_r,
            "canopy_h": canopy_h, "canopy_r": canopy_r,
            "yaw": rng.uniform(0, 2 * math.pi),
        })

    return trees


# ==================== SDF 월드 생성 ====================

def generate_world_sdf(
    dem_stats: tuple[float, float, float],
    osm: dict,
    area_size: float,
    center_lat: float,
    center_lon: float,
    output_path: Path,
    dem: list[list[float]] | None = None,
) -> None:
    """실제 데이터 기반 Gazebo 월드 SDF 생성."""
    min_elev, max_elev, height_range = dem_stats
    base_elevation = min_elev
    half = area_size / 2

    # 건물 SDF 생성
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

    # 도로 SDF 생성 (클리핑 적용)
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
        # 도로 재질 색상
        if surface == "asphalt":
            ambient = "0.25 0.25 0.25 1"
            diffuse = "0.35 0.35 0.35 1"
        elif surface == "concrete":
            ambient = "0.5 0.48 0.45 1"
            diffuse = "0.6 0.58 0.55 1"
        else:  # gravel, dirt, etc.
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

    # 수계 (하천) SDF - 클리핑 적용
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

    # 식생 (나무) SDF 생성
    vegetation_sdf = ""
    if dem is not None:
        trees = _generate_vegetation(dem, osm, area_size, len(dem))
    else:
        trees = _generate_vegetation([[0]], osm, area_size, HEIGHTMAP_SIZE)

    for i, t in enumerate(trees):
        if t["type"] == "conifer":
            # 침엽수: 원기둥 줄기 + 원뿔 캐노피
            vegetation_sdf += f"""
    <model name="tree_{i}">
      <static>true</static>
      <pose>{t['x']:.1f} {t['y']:.1f} {t['trunk_h'] / 2:.1f} 0 0 {t['yaw']:.2f}</pose>
      <link name="link">
        <collision name="trunk_col">
          <geometry><cylinder><radius>{t['trunk_r']:.2f}</radius><length>{t['trunk_h']:.1f}</length></cylinder></geometry>
        </collision>
        <visual name="trunk">
          <geometry><cylinder><radius>{t['trunk_r']:.2f}</radius><length>{t['trunk_h']:.1f}</length></cylinder></geometry>
          <material><ambient>0.35 0.2 0.08 1</ambient><diffuse>0.45 0.28 0.12 1</diffuse></material>
        </visual>
        <visual name="canopy">
          <pose>0 0 {t['trunk_h'] / 2 + t['canopy_h'] / 2:.1f} 0 0 0</pose>
          <geometry><cylinder><radius>{t['canopy_r']:.1f}</radius><length>{t['canopy_h']:.1f}</length></cylinder></geometry>
          <material><ambient>0.05 0.3 0.08 1</ambient><diffuse>0.08 0.4 0.12 1</diffuse></material>
        </visual>
      </link>
    </model>
"""
        else:
            # 활엽수: 원기둥 줄기 + 구형 캐노피
            vegetation_sdf += f"""
    <model name="tree_{i}">
      <static>true</static>
      <pose>{t['x']:.1f} {t['y']:.1f} {t['trunk_h'] / 2:.1f} 0 0 {t['yaw']:.2f}</pose>
      <link name="link">
        <collision name="trunk_col">
          <geometry><cylinder><radius>{t['trunk_r']:.2f}</radius><length>{t['trunk_h']:.1f}</length></cylinder></geometry>
        </collision>
        <visual name="trunk">
          <geometry><cylinder><radius>{t['trunk_r']:.2f}</radius><length>{t['trunk_h']:.1f}</length></cylinder></geometry>
          <material><ambient>0.35 0.2 0.08 1</ambient><diffuse>0.45 0.28 0.12 1</diffuse></material>
        </visual>
        <visual name="canopy">
          <pose>0 0 {t['trunk_h'] / 2 + t['canopy_r']:.1f} 0 0 0</pose>
          <geometry><sphere><radius>{t['canopy_r']:.1f}</radius></sphere></geometry>
          <material><ambient>0.08 0.35 0.1 1</ambient><diffuse>0.12 0.45 0.15 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

    # 돌/바위 (산악 지역 특성)
    rocks_sdf = ""
    import random
    rng = random.Random(123)
    rock_count = 25
    for i in range(rock_count):
        rx = rng.uniform(-half * 0.85, half * 0.85)
        ry = rng.uniform(-half * 0.85, half * 0.85)

        # 도로 근처 제외
        near_road = False
        for road in osm.get("roads", []):
            for coord in road["coords"]:
                if math.sqrt((rx - coord[0]) ** 2 + (ry - coord[1]) ** 2) < 5.0:
                    near_road = True
                    break
            if near_road:
                break
        if near_road:
            continue

        rw = rng.uniform(0.3, 1.2)
        rh = rng.uniform(0.2, 0.8)
        rd = rng.uniform(0.3, 1.0)
        ryaw = rng.uniform(0, 3.14)
        rpitch = rng.uniform(-0.3, 0.3)

        rocks_sdf += f"""
    <model name="rock_{i}">
      <static>true</static>
      <pose>{rx:.1f} {ry:.1f} {rh / 2:.2f} {rpitch:.2f} 0 {ryaw:.2f}</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{rw:.2f} {rd:.2f} {rh:.2f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{rw:.2f} {rd:.2f} {rh:.2f}</size></box></geometry>
          <material><ambient>0.45 0.43 0.4 1</ambient><diffuse>0.55 0.53 0.5 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

    tree_count = len(trees)
    print(f"  건물: {bld_count}, 도로: {road_idx} 세그먼트, "
          f"수계: {water_idx} 세그먼트, 나무: {tree_count}, 바위: {rock_count}")

    sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="yeongwol_sinil">

    <!-- 물리 엔진 설정 -->
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <gravity>0 0 -9.81</gravity>

    <!-- GPS 좌표 기준: 영월군 주천면 신일리 -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>{center_lat}</latitude_deg>
      <longitude_deg>{center_lon}</longitude_deg>
      <elevation>{base_elevation:.1f}</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <!-- 태양광 (한국 중부 위도 기준 남쪽 방향) -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.5 0.3 -1.0</direction>
    </light>

    <!-- 환경광 (숲 속 분위기) -->
    <light type="directional" name="ambient_light">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 50 0 0 0</pose>
      <diffuse>0.3 0.3 0.35 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
      <direction>0.2 -0.3 -1.0</direction>
    </light>

    <!-- ==================== 실제 지형 (SRTM DEM 기반) ==================== -->
    <!-- 위치: 영월군 주천면 신일리 산435-1 -->
    <!-- 고도: {min_elev:.0f}m ~ {max_elev:.0f}m (고저차 {height_range:.0f}m) -->
    <model name="real_terrain">
      <static>true</static>
      <link name="terrain_link">
        <collision name="terrain_collision">
          <geometry>
            <heightmap>
              <uri>sinil_heightmap.png</uri>
              <size>{area_size} {area_size} {height_range:.1f}</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
          <surface>
            <friction>
              <ode><mu>0.7</mu><mu2>0.7</mu2></ode>
            </friction>
          </surface>
        </collision>
        <visual name="terrain_visual">
          <geometry>
            <heightmap>
              <uri>sinil_heightmap.png</uri>
              <size>{area_size} {area_size} {height_range:.1f}</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
          <material>
            <ambient>0.35 0.32 0.18 1</ambient>
            <diffuse>0.45 0.4 0.22 1</diffuse>
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
    <!-- ==================== 식생 ({tree_count}그루) ==================== -->
{vegetation_sdf}
    <!-- ==================== 바위/돌 ==================== -->
{rocks_sdf}
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


# ==================== 메인 ====================

def main() -> None:
    print("=" * 60)
    print("  실제 지형 데이터 기반 Gazebo 월드 생성")
    print(f"  위치: 영월군 주천면 신일리 ({CENTER_LAT}°N, {CENTER_LON}°E)")
    print(f"  영역: {AREA_SIZE}m x {AREA_SIZE}m")
    print("=" * 60)

    output_dir = OUTPUT_DIR
    output_dir.mkdir(parents=True, exist_ok=True)

    # 1. DEM 수집
    print("\n[1/4] DEM 고도 데이터 수집 (SRTM 30m)...")
    grid_size = 17  # 17x17 = 289 포인트 (3 API 요청)
    dem = fetch_dem_grid(CENTER_LAT, CENTER_LON, AREA_SIZE, grid_size)

    flat = [v for row in dem for v in row]
    print(f"  수집 완료: {grid_size}x{grid_size} 격자")
    print(f"  고도 범위: {min(flat):.1f}m ~ {max(flat):.1f}m")

    # 2. Heightmap 생성
    print(f"\n[2/4] Heightmap PNG 생성 ({HEIGHTMAP_SIZE}x{HEIGHTMAP_SIZE})...")
    heightmap_path = output_dir / "sinil_heightmap.png"
    dem_stats = dem_to_heightmap_png(dem, heightmap_path, HEIGHTMAP_SIZE)
    print(f"  저장: {heightmap_path}")
    print(f"  고저차: {dem_stats[2]:.1f}m ({dem_stats[0]:.1f}m ~ {dem_stats[1]:.1f}m)")

    # 3. OSM 데이터 수집
    print("\n[3/4] OSM 데이터 수집 (도로, 건물, 토지이용)...")
    osm = fetch_osm_data(CENTER_LAT, CENTER_LON, AREA_SIZE)

    # 데이터 저장 (디버깅/참조용)
    osm_json_path = output_dir / "sinil_osm_data.json"
    # coords를 직렬화 가능하게 변환
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

    # 4. 월드 SDF 생성
    print("\n[4/4] Gazebo 월드 SDF 생성...")
    sdf_path = output_dir / "yeongwol_sinil.sdf"
    # 보간된 DEM을 전달하여 식생 배치에 사용
    dem_interp = interpolate_grid(dem, HEIGHTMAP_SIZE)
    generate_world_sdf(dem_stats, osm, AREA_SIZE, CENTER_LAT, CENTER_LON, sdf_path, dem_interp)

    # 요약
    print("\n" + "=" * 60)
    print("  생성 완료!")
    print(f"  - Heightmap: {heightmap_path}")
    print(f"  - OSM 데이터: {osm_json_path}")
    print(f"  - 월드 SDF: {sdf_path}")
    print()
    print("  시뮬레이션 실행:")
    print(f"    gz sim -r {sdf_path}")
    print("=" * 60)


if __name__ == "__main__":
    main()
