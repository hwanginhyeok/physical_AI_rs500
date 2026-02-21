"""농경지 heightmap 이미지 생성기.

Gazebo heightmap에 사용할 (2^n)+1 크기의 16bit 회색조 PNG를 생성한다.
농경지 특성: 완만한 기복 + 밭고랑 패턴 + 농로.

사용법:
    python generate_heightmap.py
    -> agricultural_heightmap.png 생성 (257x257, 16bit)
"""

import math
import struct
import zlib
from pathlib import Path


def _make_png_16bit(width: int, height: int, pixels: list[list[int]]) -> bytes:
    """16bit grayscale PNG를 순수 Python으로 생성."""

    def chunk(chunk_type: bytes, data: bytes) -> bytes:
        c = chunk_type + data
        return struct.pack('>I', len(data)) + c + struct.pack('>I', zlib.crc32(c) & 0xFFFFFFFF)

    # PNG signature
    sig = b'\x89PNG\r\n\x1a\n'

    # IHDR: width, height, bit_depth=16, color_type=0(grayscale)
    ihdr = chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 16, 0, 0, 0, 0))

    # IDAT: raw image data
    raw = b''
    for row in pixels:
        raw += b'\x00'  # filter byte (None)
        for val in row:
            raw += struct.pack('>H', min(max(val, 0), 65535))

    idat = chunk(b'IDAT', zlib.compress(raw, 9))

    # IEND
    iend = chunk(b'IEND', b'')

    return sig + ihdr + idat + iend


def generate_agricultural_heightmap(
    size: int = 257,
    max_height_value: int = 3000,
    base_height: int = 32768,
) -> list[list[int]]:
    """농경지 heightmap 생성.

    Args:
        size: 이미지 크기 (2^n+1, 예: 257)
        max_height_value: 최대 고저차 (16bit 범위 내, ~1.5m에 해당)
        base_height: 기본 높이 (32768 = 중간값)

    Returns:
        2D 픽셀 배열 (16bit 값)
    """
    pixels = [[base_height] * size for _ in range(size)]

    # 1. 완만한 기복 (대규모 Perlin-like 사인파 조합)
    for y in range(size):
        for x in range(size):
            # 저주파 기복 (전체 지형 경사)
            h1 = math.sin(x / size * 2 * math.pi * 0.5) * 0.3
            h2 = math.sin(y / size * 2 * math.pi * 0.7) * 0.2
            # 중주파 기복 (언덕/골짜기)
            h3 = math.sin(x / size * 2 * math.pi * 2.0 + 0.5) * 0.15
            h4 = math.sin(y / size * 2 * math.pi * 1.5 + 1.2) * 0.1
            # 고주파 노이즈 (지면 울퉁불퉁)
            h5 = math.sin(x * 0.8 + y * 1.3) * 0.03
            h6 = math.sin(x * 1.7 - y * 0.9) * 0.02

            total = h1 + h2 + h3 + h4 + h5 + h6
            pixels[y][x] = base_height + int(total * max_height_value)

    # 2. 밭고랑 패턴 (y방향 주기적 홈)
    furrow_spacing = 8  # 픽셀 간격 (~3m 간격)
    furrow_depth = 200  # 고랑 깊이 (~0.1m)
    # 고랑 영역: 중앙 60% 구역
    margin = int(size * 0.2)
    for y in range(margin, size - margin):
        for x in range(margin, size - margin):
            # x 방향으로 고랑 (y축 따라 평행)
            phase = (x - margin) % furrow_spacing
            if phase < 2:
                pixels[y][x] -= furrow_depth

    # 3. 농로 (동서 방향 + 남북 방향 십자형)
    road_width = 5  # 픽셀 (~2m)
    road_height_bump = 150  # 약간 높게 (다진 땅)
    center = size // 2

    # 동서 농로
    for y in range(center - road_width, center + road_width):
        for x in range(size):
            if 0 <= y < size:
                pixels[y][x] = base_height + road_height_bump

    # 남북 농로
    for y in range(size):
        for x in range(center - road_width, center + road_width):
            if 0 <= x < size:
                pixels[y][x] = base_height + road_height_bump

    # 4. 경계 정리 (가장자리를 부드럽게)
    fade = 15
    for y in range(size):
        for x in range(size):
            dist_to_edge = min(x, y, size - 1 - x, size - 1 - y)
            if dist_to_edge < fade:
                factor = dist_to_edge / fade
                pixels[y][x] = int(base_height + (pixels[y][x] - base_height) * factor)

    return pixels


def main() -> None:
    size = 257  # (2^8)+1
    print(f"농경지 heightmap 생성 중... ({size}x{size}, 16bit)")

    pixels = generate_agricultural_heightmap(size)

    # 통계
    flat = [p for row in pixels for p in row]
    min_v, max_v = min(flat), max(flat)
    print(f"  높이 범위: {min_v} ~ {max_v} (16bit)")
    print(f"  고저차: ~{(max_v - min_v) / 65535 * 10:.2f}m (10m 스케일 기준)")

    # PNG 저장
    png_data = _make_png_16bit(size, size, pixels)
    output = Path(__file__).parent / "agricultural_heightmap.png"
    output.write_bytes(png_data)
    print(f"  저장: {output}")
    print("완료!")


if __name__ == "__main__":
    main()
