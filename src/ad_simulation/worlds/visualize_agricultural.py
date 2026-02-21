"""농경지 heightmap 시각화."""
import struct
import zlib
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image


def main():
    base = Path(__file__).parent
    hmap_path = base / "agricultural_heightmap.png"
    out_path = base / "agricultural_visualization.png"

    img = Image.open(hmap_path)
    hmap = np.array(img, dtype=np.float64)
    size = hmap.shape[0]

    # agricultural_field.sdf: size 200x200x3, base_height=32768
    # heightmap 범위를 실제 미터로 변환
    h_min, h_max = hmap.min(), hmap.max()
    # SDF에서 heightmap size z=3 이므로 고저차 3m
    hmap_m = (hmap - h_min) / (h_max - h_min) * 3.0 if h_max > h_min else hmap * 0

    half = 100.0  # 200m / 2
    x = np.linspace(-half, half, size)
    y = np.linspace(-half, half, size)
    X, Y = np.meshgrid(x, y)

    fig = plt.figure(figsize=(18, 7))
    fig.suptitle(
        'Agricultural Field (Procedural Heightmap)\n'
        'Daejeon area (36.35N, 127.38E) | 200m x 200m | Furrows + Farm Roads',
        fontsize=14, fontweight='bold',
    )

    # 1. 2D
    ax1 = fig.add_subplot(1, 3, 1)
    im = ax1.imshow(hmap_m, extent=[-half, half, -half, half], cmap='terrain', origin='lower')
    plt.colorbar(im, ax=ax1, label='Height (m)', shrink=0.8)
    # 농로 표시 (십자형, 중앙)
    ax1.axhline(0, color='white', linewidth=1.5, alpha=0.5, linestyle='--', label='Farm roads')
    ax1.axvline(0, color='white', linewidth=1.5, alpha=0.5, linestyle='--')
    # 구조물
    ax1.plot(-40, 0, 's', color='brown', markersize=10, label='Barn')
    ax1.plot(15, 20, '^', color='green', markersize=8, label='Trees')
    ax1.plot(25, -15, '^', color='green', markersize=8)
    ax1.plot(10, -8, 'o', color='gray', markersize=6, label='Rocks')
    ax1.plot(-5, 12, 'o', color='gray', markersize=6)
    # 경계
    ax1.plot([50, 50, -50, -50, 50], [50, -50, -50, 50, 50], 'r--', linewidth=1, alpha=0.5)
    ax1.set_title('2D Elevation + Objects')
    ax1.set_xlabel('E-W (m)')
    ax1.set_ylabel('N-S (m)')
    ax1.legend(loc='upper left', fontsize=7)
    ax1.set_aspect('equal')

    # 2. 3D
    ax2 = fig.add_subplot(1, 3, 2, projection='3d')
    stride = max(1, size // 64)
    ax2.plot_surface(
        X[::stride, ::stride], Y[::stride, ::stride],
        hmap_m[::stride, ::stride],
        cmap='terrain', alpha=0.85, linewidth=0, antialiased=True,
    )
    ax2.set_title('3D Surface')
    ax2.set_xlabel('E-W (m)')
    ax2.set_ylabel('N-S (m)')
    ax2.set_zlabel('Height (m)')
    ax2.view_init(elev=30, azim=-50)

    # 3. Cross-section
    ax3 = fig.add_subplot(1, 3, 3)
    mid = size // 2
    # 고랑 영역 단면 (밭 중앙부)
    profile = hmap_m[mid + 20, :]  # 농로에서 약간 떨어진 곳
    ax3.plot(x, profile, 'g-', linewidth=1, label='E-W profile (furrow area)')
    ax3.fill_between(x, 0, profile, alpha=0.2, color='green')

    profile_road = hmap_m[mid, :]
    ax3.plot(x, profile_road, 'brown', linewidth=1.5, label='E-W profile (farm road)')

    ax3.set_title('Elevation Cross-section')
    ax3.set_xlabel('E-W (m)')
    ax3.set_ylabel('Height (m)')
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)

    stats = (
        f'Size: 200m x 200m | Height range: {hmap_m.max():.2f}m\n'
        f'Features: Furrows (8px spacing), Cross roads, Barn, 2 Trees, 2 Rocks'
    )
    ax3.text(0.02, 0.98, stats, transform=ax3.transAxes, fontsize=7,
             verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

    plt.tight_layout()
    plt.savefig(str(out_path), dpi=150, bbox_inches='tight')
    print(f"Saved: {out_path}")


if __name__ == "__main__":
    main()
