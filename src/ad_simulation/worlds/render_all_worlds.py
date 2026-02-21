"""3개 월드를 고품질 3D로 렌더링하여 비교 이미지 생성."""
import json
import math
import random
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
from PIL import Image


BASE = Path(__file__).parent


def load_heightmap(png_path, area_size, min_elev, max_elev):
    """PNG heightmap을 고도 배열로 변환."""
    img = Image.open(png_path)
    hmap = np.array(img, dtype=np.float64)
    hmap_m = min_elev + (hmap / 65535.0) * (max_elev - min_elev)
    size = hmap.shape[0]
    half = area_size / 2
    x = np.linspace(-half, half, size)
    y = np.linspace(-half, half, size)
    return np.meshgrid(x, y), hmap_m


def render_agricultural():
    """농경지 월드 3D 렌더링."""
    png = BASE / "agricultural_heightmap.png"
    (X, Y), Z = load_heightmap(png, 200, 0, 3.0)

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')

    stride = 4
    ax.plot_surface(X[::stride, ::stride], Y[::stride, ::stride], Z[::stride, ::stride],
                    cmap='YlGn', alpha=0.9, linewidth=0, antialiased=True)

    # 농로 (십자형)
    road_x = np.linspace(-100, 100, 50)
    ax.plot(road_x, [0]*50, [1.15]*50, color='#8B7355', linewidth=3, alpha=0.8)
    ax.plot([0]*50, road_x, [1.15]*50, color='#8B7355', linewidth=3, alpha=0.8)

    # 창고
    ax.bar3d(-43, -2.5, 0, 8, 5, 3, color='#8B4513', alpha=0.8)

    # 나무
    for tx, ty in [(15, 20), (25, -15)]:
        ax.plot([tx, tx], [ty, ty], [0, 4], color='#654321', linewidth=3)
        u = np.linspace(0, 2*np.pi, 12)
        v = np.linspace(0, np.pi, 8)
        sx = tx + 1.5 * np.outer(np.cos(u), np.sin(v))
        sy = ty + 1.5 * np.outer(np.sin(u), np.sin(v))
        sz = 4.5 + 1.5 * np.outer(np.ones_like(u), np.cos(v))
        ax.plot_surface(sx, sy, sz, color='#228B22', alpha=0.7)

    # 돌
    ax.scatter([10, -5], [-8, 12], [0.3, 0.2], c='gray', s=100, marker='o', alpha=0.8)

    # 경계 마커
    for mx, my in [(50, 50), (50, -50), (-50, 50), (-50, -50)]:
        ax.plot([mx, mx], [my, my], [0, 1.5], color='red', linewidth=2)

    ax.set_title('1. Agricultural Field\n200m x 200m | Furrows + Farm Roads',
                 fontsize=13, fontweight='bold', pad=15)
    ax.set_xlabel('E-W (m)', fontsize=9)
    ax.set_ylabel('N-S (m)', fontsize=9)
    ax.set_zlabel('Height (m)', fontsize=9)
    ax.view_init(elev=25, azim=-45)
    ax.set_box_aspect([2, 2, 0.3])

    return fig


def render_yeongwol():
    """영월 신일리 3D 렌더링."""
    png = BASE / "real_terrain" / "sinil_heightmap.png"
    osm_path = BASE / "real_terrain" / "sinil_osm_data.json"
    (X, Y), Z = load_heightmap(png, 500, 289, 402)

    with open(osm_path, 'r', encoding='utf-8') as f:
        osm = json.load(f)

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')

    stride = 4
    ax.plot_surface(X[::stride, ::stride], Y[::stride, ::stride], Z[::stride, ::stride],
                    cmap='terrain', alpha=0.9, linewidth=0, antialiased=True)

    # 도로
    half = 250
    for road in osm.get('roads', []):
        coords = road['coords']
        xs = [c[0] for c in coords if -half <= c[0] <= half and -half <= c[1] <= half]
        ys = [c[1] for c in coords if -half <= c[0] <= half and -half <= c[1] <= half]
        if len(xs) >= 2:
            zs = [340] * len(xs)  # 대략적 도로 높이
            ax.plot(xs, ys, zs, color='#D2B48C', linewidth=2, alpha=0.8)

    # 나무 (시드 동일)
    rng = random.Random(42)
    for _ in range(40):
        tx = rng.uniform(-200, 200)
        ty = rng.uniform(-200, 200)
        tz = 330 + rng.uniform(-20, 40)
        ax.plot([tx, tx], [ty, ty], [tz-3, tz], color='#654321', linewidth=1.5, alpha=0.5)
        ax.scatter([tx], [ty], [tz+2], c='#228B22', s=40, marker='^', alpha=0.6)

    ax.set_title('2. Yeongwol Sinil-ri (Real DEM)\n500m x 500m | Elevation 289-402m',
                 fontsize=13, fontweight='bold', pad=15)
    ax.set_xlabel('E-W (m)', fontsize=9)
    ax.set_ylabel('N-S (m)', fontsize=9)
    ax.set_zlabel('Elevation (m)', fontsize=9)
    ax.view_init(elev=30, azim=-55)
    ax.set_box_aspect([2, 2, 0.8])

    return fig


def render_cheongsong():
    """청송 사과 과수원 3D 렌더링."""
    png = BASE / "real_terrain" / "cheongsong_heightmap.png"
    osm_path = BASE / "real_terrain" / "cheongsong_osm_data.json"
    (X, Y), Z = load_heightmap(png, 300, 198, 276)

    with open(osm_path, 'r', encoding='utf-8') as f:
        osm = json.load(f)

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')

    stride = 4
    ax.plot_surface(X[::stride, ::stride], Y[::stride, ::stride], Z[::stride, ::stride],
                    cmap='YlGn', alpha=0.7, linewidth=0, antialiased=True)

    # 도로
    half = 150
    for road in osm.get('roads', []):
        coords = road['coords']
        xs = [c[0] for c in coords if -half <= c[0] <= half and -half <= c[1] <= half]
        ys = [c[1] for c in coords if -half <= c[0] <= half and -half <= c[1] <= half]
        if len(xs) >= 2:
            zs = [240] * len(xs)
            ax.plot(xs, ys, zs, color='#8B7355', linewidth=2, alpha=0.7)

    # 과수원 영역의 사과나무 배치 (실제 generate_orchard_world.py 로직 재현)
    orchard_half = half * 0.6  # 60% 영역
    row_spacing = 5.0
    col_spacing = 4.0
    rng = random.Random(42)

    tree_xs, tree_ys = [], []
    apple_xs, apple_ys, apple_zs = [], [], []

    row_idx = 0
    y_pos = -orchard_half
    while y_pos <= orchard_half:
        if row_idx % 10 == 9:  # 작업 통로
            y_pos += row_spacing
            row_idx += 1
            continue
        x_pos = -orchard_half
        while x_pos <= orchard_half:
            tx = x_pos + rng.uniform(-0.3, 0.3)
            ty = y_pos + rng.uniform(-0.3, 0.3)
            tree_xs.append(tx)
            tree_ys.append(ty)

            # 일부 나무에 사과
            if rng.random() < 0.1:
                for _ in range(rng.randint(3, 5)):
                    ax_off = rng.uniform(-1.2, 1.2)
                    ay_off = rng.uniform(-1.2, 1.2)
                    apple_xs.append(tx + ax_off)
                    apple_ys.append(ty + ay_off)
                    apple_zs.append(242 + rng.uniform(-0.5, 0.5))

            x_pos += col_spacing
        y_pos += row_spacing
        row_idx += 1

    # 나무 (초록 점으로 표현)
    tree_zs = [243] * len(tree_xs)
    ax.scatter(tree_xs, tree_ys, tree_zs, c='#1B7A1B', s=12, marker='o', alpha=0.6,
               label=f'Apple trees ({len(tree_xs)})')

    # 사과 (빨간 점)
    if apple_xs:
        ax.scatter(apple_xs, apple_ys, apple_zs, c='red', s=5, marker='o', alpha=0.8,
                   label=f'Apples ({len(apple_xs)})')

    # 창고
    ax.bar3d(120, -115, 235, 12, 8, 5, color='#8B4513', alpha=0.8)

    ax.set_title('3. Cheongsong Apple Orchard (Real DEM)\n300m x 300m | 2,400 Trees + Apples',
                 fontsize=13, fontweight='bold', pad=15)
    ax.set_xlabel('E-W (m)', fontsize=9)
    ax.set_ylabel('N-S (m)', fontsize=9)
    ax.set_zlabel('Elevation (m)', fontsize=9)
    ax.view_init(elev=35, azim=-50)
    ax.set_box_aspect([2, 2, 0.6])
    ax.legend(loc='upper left', fontsize=8)

    return fig


def render_cheongsong_detail():
    """청송 과수원 클로즈업 (50m x 50m)."""
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111)

    rng = random.Random(42)
    orchard_half = 90  # 150 * 0.6

    row_idx = 0
    y_pos = -orchard_half
    while y_pos <= orchard_half:
        if row_idx % 10 == 9:
            # 작업 통로 표시
            if -25 <= y_pos <= 25:
                ax.axhspan(y_pos - 1.5, y_pos + 1.5, color='#D2B48C', alpha=0.3)
                ax.text(26, y_pos, 'work lane', fontsize=7, color='brown', va='center')
            y_pos += 5.0
            row_idx += 1
            continue

        x_pos = -orchard_half
        while x_pos <= orchard_half:
            tx = x_pos + rng.uniform(-0.3, 0.3)
            ty = y_pos + rng.uniform(-0.3, 0.3)

            if -25 <= tx <= 25 and -25 <= ty <= 25:
                # 수관 원
                circle = plt.Circle((tx, ty), 1.5, color='#2E8B2E', alpha=0.35)
                ax.add_patch(circle)
                # 줄기
                ax.plot(tx, ty, '.', color='#654321', markersize=3)

                # 사과
                if rng.random() < 0.1:
                    for _ in range(rng.randint(3, 5)):
                        ax_off = rng.uniform(-1.0, 1.0)
                        ay_off = rng.uniform(-1.0, 1.0)
                        ax.plot(tx + ax_off, ty + ay_off, '.', color='red', markersize=5)
                else:
                    rng.random()  # keep sync
            else:
                # 영역 밖은 건너뛰되 rng 동기화
                if rng.random() < 0.1:
                    for _ in range(rng.randint(3, 5)):
                        rng.uniform(-1, 1)
                        rng.uniform(-1, 1)
                else:
                    rng.random()

            x_pos += 4.0
        y_pos += 5.0
        row_idx += 1

    # 행/열 간격 표시
    ax.annotate('', xy=(4, -24), xytext=(0, -24),
                arrowprops=dict(arrowstyle='<->', color='blue', lw=1.5))
    ax.text(2, -23, '4m', fontsize=9, color='blue', ha='center', fontweight='bold')

    ax.annotate('', xy=(-24, 5), xytext=(-24, 0),
                arrowprops=dict(arrowstyle='<->', color='blue', lw=1.5))
    ax.text(-23, 2.5, '5m', fontsize=9, color='blue', ha='center', fontweight='bold')

    ax.set_xlim(-25, 28)
    ax.set_ylim(-25, 25)
    ax.set_aspect('equal')
    ax.set_title('Cheongsong Orchard Detail (50m x 50m)\nTree spacing: 4m x 5m | Red dots = Apples',
                 fontsize=13, fontweight='bold')
    ax.set_xlabel('E-W (m)')
    ax.set_ylabel('N-S (m)')
    ax.grid(True, alpha=0.15)

    # 범례
    ax.plot([], [], 'o', color='#2E8B2E', markersize=10, alpha=0.35, label='Tree canopy (r=1.5m)')
    ax.plot([], [], '.', color='red', markersize=8, label='Apples')
    ax.plot([], [], '.', color='#654321', markersize=5, label='Trunk')
    ax.legend(loc='upper right', fontsize=8)

    return fig


def main():
    print("3개 월드 렌더링 중...")

    # 각 월드 개별 렌더링
    worlds = [
        ("agricultural", render_agricultural),
        ("yeongwol", render_yeongwol),
        ("cheongsong", render_cheongsong),
        ("cheongsong_detail", render_cheongsong_detail),
    ]

    for name, render_fn in worlds:
        print(f"  {name}...")
        fig = render_fn()
        out = BASE / f"render_{name}.png"
        fig.savefig(str(out), dpi=150, bbox_inches='tight', facecolor='white')
        plt.close(fig)
        print(f"    -> {out}")

    print("완료!")


if __name__ == "__main__":
    main()
