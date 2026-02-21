"""실제 지형 데이터 시각화.

heightmap PNG + OSM 데이터를 2D/3D로 시각화하여 PNG 이미지로 저장.
"""

import json
import struct
import zlib
from pathlib import Path

import matplotlib
matplotlib.use('Agg')  # headless
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


def read_png_16bit(path: Path) -> np.ndarray:
    """16bit grayscale PNG를 numpy 배열로 읽기."""
    from PIL import Image
    img = Image.open(path)
    return np.array(img, dtype=np.float64)


def main():
    base_dir = Path(__file__).parent
    heightmap_path = base_dir / "sinil_heightmap.png"
    osm_path = base_dir / "sinil_osm_data.json"
    output_path = base_dir / "terrain_visualization.png"

    # Heightmap 읽기
    hmap = read_png_16bit(heightmap_path)
    size = hmap.shape[0]

    # 정규화: 실제 고도로 변환 (289m ~ 402m)
    min_elev, max_elev = 289.0, 402.0
    hmap_meters = min_elev + (hmap / 65535.0) * (max_elev - min_elev)

    # OSM 데이터 읽기
    with open(osm_path, 'r', encoding='utf-8') as f:
        osm = json.load(f)

    area_size = 500.0
    half = area_size / 2

    # 좌표 범위
    x = np.linspace(-half, half, size)
    y = np.linspace(-half, half, size)
    X, Y = np.meshgrid(x, y)

    # ==================== Figure 생성 ====================
    fig = plt.figure(figsize=(20, 16))
    fig.suptitle(
        'Yeongwol Sinil-ri Real Terrain (37.2599N, 128.2404E)\n'
        'SRTM 30m DEM + OSM Data | 500m x 500m',
        fontsize=16, fontweight='bold',
    )

    # --- 1. 2D Heightmap (Top-down) ---
    ax1 = fig.add_subplot(2, 2, 1)
    im = ax1.imshow(
        hmap_meters, extent=[-half, half, -half, half],
        cmap='terrain', origin='lower',
    )
    plt.colorbar(im, ax=ax1, label='Elevation (m)', shrink=0.8)

    # 도로 표시
    for road in osm.get('roads', []):
        coords = road['coords']
        if len(coords) < 2:
            continue
        xs = [c[0] for c in coords if -half <= c[0] <= half and -half <= c[1] <= half]
        ys = [c[1] for c in coords if -half <= c[0] <= half and -half <= c[1] <= half]
        if xs:
            ax1.plot(xs, ys, 'w-', linewidth=1.5, alpha=0.8)

    # 수계 표시
    for ww in osm.get('waterways', []):
        coords = ww['coords']
        xs = [c[0] for c in coords if -half <= c[0] <= half and -half <= c[1] <= half]
        ys = [c[1] for c in coords if -half <= c[0] <= half and -half <= c[1] <= half]
        if xs:
            ax1.plot(xs, ys, 'c-', linewidth=2, alpha=0.9)

    ax1.set_title('2D Elevation Map + OSM')
    ax1.set_xlabel('East-West (m)')
    ax1.set_ylabel('North-South (m)')
    ax1.set_xlim(-half, half)
    ax1.set_ylim(-half, half)
    ax1.set_aspect('equal')

    # 범례
    ax1.plot([], [], 'w-', linewidth=1.5, label='Roads')
    ax1.plot([], [], 'c-', linewidth=2, label='Waterways')
    ax1.legend(loc='upper left', fontsize=8)

    # --- 2. 3D Surface ---
    ax2 = fig.add_subplot(2, 2, 2, projection='3d')
    stride = max(1, size // 64)
    ax2.plot_surface(
        X[::stride, ::stride], Y[::stride, ::stride],
        hmap_meters[::stride, ::stride],
        cmap='terrain', alpha=0.85,
        linewidth=0, antialiased=True,
    )
    ax2.set_title('3D Terrain Surface')
    ax2.set_xlabel('E-W (m)')
    ax2.set_ylabel('N-S (m)')
    ax2.set_zlabel('Elevation (m)')
    ax2.view_init(elev=35, azim=-60)

    # --- 3. SDF Object Layout ---
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.imshow(
        hmap_meters, extent=[-half, half, -half, half],
        cmap='Greens', origin='lower', alpha=0.4,
    )

    # 도로
    for road in osm.get('roads', []):
        coords = road['coords']
        xs_full = [c[0] for c in coords]
        ys_full = [c[1] for c in coords]
        # 클리핑된 부분만
        xs = [c[0] for c in coords if -half <= c[0] <= half and -half <= c[1] <= half]
        ys = [c[1] for c in coords if -half <= c[0] <= half and -half <= c[1] <= half]
        if xs:
            ax3.plot(xs, ys, '-', color='#8B7355', linewidth=2, alpha=0.9)

    # 수계
    for ww in osm.get('waterways', []):
        coords = ww['coords']
        xs = [c[0] for c in coords if -half <= c[0] <= half and -half <= c[1] <= half]
        ys = [c[1] for c in coords if -half <= c[0] <= half and -half <= c[1] <= half]
        if xs:
            ax3.plot(xs, ys, '-', color='#4488CC', linewidth=3, alpha=0.8)

    # 나무 위치 (SDF에서 추출하지 않고 동일 시드로 재생성)
    import random
    rng = random.Random(42)
    tree_positions = []
    exclusion = []
    for road in osm.get('roads', []):
        for c in road['coords']:
            if -half <= c[0] <= half and -half <= c[1] <= half:
                exclusion.append((c[0], c[1], 8.0))
    for ww in osm.get('waterways', []):
        for c in ww['coords']:
            if -half <= c[0] <= half and -half <= c[1] <= half:
                exclusion.append((c[0], c[1], 6.0))

    import math
    for _ in range(400):
        if len(tree_positions) >= 80:
            break
        tx = rng.uniform(-half * 0.9, half * 0.9)
        ty = rng.uniform(-half * 0.9, half * 0.9)
        excluded = False
        for ex, ey, er in exclusion:
            if math.sqrt((tx - ex)**2 + (ty - ey)**2) < er:
                excluded = True
                break
        if excluded:
            continue
        too_close = False
        for pt in tree_positions:
            if math.sqrt((tx - pt[0])**2 + (ty - pt[1])**2) < 5.0:
                too_close = True
                break
        if too_close:
            continue
        tree_positions.append((tx, ty))

    if tree_positions:
        txs, tys = zip(*tree_positions)
        ax3.scatter(txs, tys, c='#228B22', s=25, marker='^', alpha=0.7, label=f'Trees ({len(tree_positions)})')

    # 바위
    rng2 = random.Random(123)
    rock_positions = []
    for _ in range(25):
        rx = rng2.uniform(-half * 0.85, half * 0.85)
        ry = rng2.uniform(-half * 0.85, half * 0.85)
        near_road = False
        for road in osm.get('roads', []):
            for c in road['coords']:
                if math.sqrt((rx - c[0])**2 + (ry - c[1])**2) < 5.0:
                    near_road = True
                    break
            if near_road:
                break
        if not near_road:
            rock_positions.append((rx, ry))

    if rock_positions:
        rxs, rys = zip(*rock_positions)
        ax3.scatter(rxs, rys, c='#888888', s=18, marker='s', alpha=0.7, label=f'Rocks ({len(rock_positions)})')

    # 경계 마커
    ax3.plot([half, half, -half, -half, half],
             [half, -half, -half, half, half],
             'r--', linewidth=1, alpha=0.5, label='Area boundary')

    ax3.set_title('Object Layout (Trees, Rocks, Roads)')
    ax3.set_xlabel('East-West (m)')
    ax3.set_ylabel('North-South (m)')
    ax3.set_xlim(-half - 10, half + 10)
    ax3.set_ylim(-half - 10, half + 10)
    ax3.set_aspect('equal')
    ax3.legend(loc='upper left', fontsize=7)

    # --- 4. Elevation Profile (Cross-section) ---
    ax4 = fig.add_subplot(2, 2, 4)
    mid = size // 2
    profile_ew = hmap_meters[mid, :]
    profile_ns = hmap_meters[:, mid]

    ax4.plot(x, profile_ew, 'b-', linewidth=1.5, label='E-W (y=0)')
    ax4.plot(y, profile_ns, 'r-', linewidth=1.5, label='N-S (x=0)')
    ax4.fill_between(x, min_elev, profile_ew, alpha=0.15, color='blue')
    ax4.fill_between(y, min_elev, profile_ns, alpha=0.15, color='red')
    ax4.set_title('Elevation Cross-sections')
    ax4.set_xlabel('Distance from center (m)')
    ax4.set_ylabel('Elevation (m)')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    # 통계 텍스트
    stats_text = (
        f'Min: {min_elev:.0f}m | Max: {max_elev:.0f}m | Range: {max_elev - min_elev:.0f}m\n'
        f'Roads: {len(osm.get("roads", []))} | '
        f'Waterways: {len(osm.get("waterways", []))} | '
        f'Trees: {len(tree_positions)} | '
        f'Rocks: {len(rock_positions)}'
    )
    ax4.text(
        0.02, 0.98, stats_text, transform=ax4.transAxes,
        fontsize=8, verticalalignment='top',
        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
    )

    plt.tight_layout()
    plt.savefig(str(output_path), dpi=150, bbox_inches='tight')
    print(f"Visualization saved: {output_path}")
    plt.close()


if __name__ == "__main__":
    main()
