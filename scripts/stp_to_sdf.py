#!/usr/bin/env python3
"""
STP → SDF 변환 파이프라인

STP(STEP) CAD 파일에서 물리 파라미터를 추출하고,
STL/DAE 메시를 생성한 뒤, model.sdf를 자동 갱신한다.

사용법:
    # venv 활성화 필요
    source .venv-tools/bin/activate

    # 기본 사용 (model.sdf 자동 갱신)
    python scripts/stp_to_sdf.py path/to/ss500.stp

    # 메시만 생성 (SDF 갱신 안 함)
    python scripts/stp_to_sdf.py path/to/ss500.stp --mesh-only

    # 밀도 지정 (기본: 7850 kg/m³ = 강철)
    python scripts/stp_to_sdf.py path/to/ss500.stp --density 2700

    # 출력 디렉토리 지정
    python scripts/stp_to_sdf.py path/to/ss500.stp -o src/ad_bringup/models/ss500/meshes

의존성:
    pip install cadquery trimesh numpy
"""

import argparse
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import cadquery as cq
import numpy as np
import trimesh


# ── 기본값 ──────────────────────────────────────────────────
DEFAULT_SDF_PATH = "src/ad_bringup/models/ss500/model.sdf"
DEFAULT_MESH_DIR = "src/ad_bringup/models/ss500/meshes"
DEFAULT_DENSITY = 7850.0  # kg/m³ (강철)
COLLISION_FACE_COUNT = 1000   # 충돌 메시 해상도
VISUAL_FACE_COUNT = 10000     # 시각 메시 해상도


def load_step(stp_path: str) -> cq.Workplane:
    """STP 파일을 로드한다. 항상 Workplane으로 반환."""
    path = Path(stp_path)
    if not path.exists():
        raise FileNotFoundError(f"STP 파일을 찾을 수 없습니다: {path}")

    result = cq.importers.importStep(str(path))
    return result


def extract_physics(shape, density: float) -> dict:
    """CadQuery Shape에서 물리 파라미터를 추출한다.

    Returns:
        dict with keys: mass, center_of_mass, inertia_tensor, volume, bbox
    """
    # CadQuery Workplane → OCP Shape
    if isinstance(shape, cq.Workplane):
        occ_shape = shape.val()
    else:
        occ_shape = shape

    # 볼륨 (mm³ → m³ 변환: STEP은 보통 mm 단위)
    # CadQuery는 mm 단위 기본
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp

    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(occ_shape.wrapped, props)

    volume_mm3 = props.Mass()  # GProp의 Mass()는 실제로는 볼륨
    volume_m3 = volume_mm3 * 1e-9  # mm³ → m³

    mass = density * volume_m3

    # 무게중심 (mm → m)
    com = props.CentreOfMass()
    center_of_mass = [com.X() * 1e-3, com.Y() * 1e-3, com.Z() * 1e-3]

    # 관성 텐서 (mm 단위 → SI 단위 변환)
    # GProp 관성은 볼륨 기준이므로 density를 곱해야 함
    mat = props.MatrixOfInertia()
    # 3x3 행렬 추출
    inertia_mm = np.array([
        [mat.Value(1, 1), mat.Value(1, 2), mat.Value(1, 3)],
        [mat.Value(2, 1), mat.Value(2, 2), mat.Value(2, 3)],
        [mat.Value(3, 1), mat.Value(3, 2), mat.Value(3, 3)],
    ])
    # mm⁵ → m⁵ (길이^5), 그 다음 density 곱하기
    # 관성 모멘트 = density * ∫r²dV → 단위: kg/m³ * m⁵ = kg·m²
    inertia_si = inertia_mm * 1e-15 * density  # mm⁵ → m⁵ = *1e-15

    # 바운딩 박스 (mm → m)
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib

    bbox = Bnd_Box()
    BRepBndLib.Add_s(occ_shape.wrapped, bbox)
    xmin, ymin, zmin, xmax, ymax, zmax = bbox.Get()
    bbox_m = {
        "min": [xmin * 1e-3, ymin * 1e-3, zmin * 1e-3],
        "max": [xmax * 1e-3, ymax * 1e-3, zmax * 1e-3],
        "size": [
            (xmax - xmin) * 1e-3,
            (ymax - ymin) * 1e-3,
            (zmax - zmin) * 1e-3,
        ],
    }

    return {
        "mass": round(mass, 3),
        "volume_m3": round(volume_m3, 6),
        "center_of_mass": [round(v, 4) for v in center_of_mass],
        "inertia": {
            "ixx": round(inertia_si[0, 0], 4),
            "iyy": round(inertia_si[1, 1], 4),
            "izz": round(inertia_si[2, 2], 4),
            "ixy": round(inertia_si[0, 1], 4),
            "ixz": round(inertia_si[0, 2], 4),
            "iyz": round(inertia_si[1, 2], 4),
        },
        "bbox": bbox_m,
    }


def export_meshes(shape, output_dir: Path, name: str = "body") -> dict:
    """CadQuery Shape → STL(충돌용) + DAE(시각용) 메시 파일 생성.

    Returns:
        dict: {"collision": path, "visual": path}
    """
    output_dir.mkdir(parents=True, exist_ok=True)

    # STL 내보내기 (CadQuery 내장)
    collision_path = output_dir / f"{name}_collision.stl"
    visual_path = output_dir / f"{name}_visual.stl"

    # 고해상도 STL (시각용)
    cq.exporters.export(shape, str(visual_path), exportType="STL",
                        tolerance=0.1, angularTolerance=0.1)

    # 저해상도 STL (충돌용)
    cq.exporters.export(shape, str(collision_path), exportType="STL",
                        tolerance=1.0, angularTolerance=0.5)

    # trimesh로 면 수 줄이기
    _simplify_mesh(collision_path, COLLISION_FACE_COUNT)
    _simplify_mesh(visual_path, VISUAL_FACE_COUNT)

    # mm → m 스케일 변환
    for mesh_path in [collision_path, visual_path]:
        mesh = trimesh.load(str(mesh_path))
        mesh.apply_scale(0.001)  # mm → m
        mesh.export(str(mesh_path))

    return {
        "collision": str(collision_path),
        "visual": str(visual_path),
    }


def _simplify_mesh(path: Path, target_faces: int):
    """trimesh로 메시 면 수를 줄인다."""
    mesh = trimesh.load(str(path))
    if len(mesh.faces) > target_faces:
        ratio = 1.0 - (target_faces / len(mesh.faces))
        try:
            simplified = mesh.simplify_quadric_decimation(target_faces)
        except (ValueError, TypeError):
            # fast_simplification API: target_reduction (0~1 비율)
            import fast_simplification
            verts, faces = fast_simplification.simplify(
                mesh.vertices, mesh.faces, target_reduction=ratio
            )
            simplified = trimesh.Trimesh(vertices=verts, faces=faces)
        simplified.export(str(path))


def update_sdf(sdf_path: str, physics: dict, mesh_paths: dict | None = None):
    """model.sdf의 base_link <inertial>, <collision>, <visual>을 정규식으로 갱신한다.

    원본 파일의 주석, 포맷, 다른 링크/플러그인을 그대로 보존한다.
    """
    content = Path(sdf_path).read_text(encoding="utf-8")

    # ── inertial 블록 교체 ──
    i = physics["inertia"]
    com = physics["center_of_mass"]
    new_inertial = f"""      <inertial>
        <mass>{physics['mass']}</mass>
        <pose>{com[0]} {com[1]} {com[2]} 0 0 0</pose>
        <inertia>
          <ixx>{i['ixx']}</ixx>
          <iyy>{i['iyy']}</iyy>
          <izz>{i['izz']}</izz>
          <ixy>{i['ixy']}</ixy>
          <ixz>{i['ixz']}</ixz>
          <iyz>{i['iyz']}</iyz>
        </inertia>
      </inertial>"""

    # base_link 내 <inertial>...</inertial> 블록 매칭 (들여쓰기 보존)
    content = re.sub(
        r'      <inertial>.*?</inertial>',             # 첫 번째 inertial 블록 (base_link)
        new_inertial,
        content,
        count=1,
        flags=re.DOTALL,
    )

    # ── collision 메시 교체 ──
    if mesh_paths:
        sdf_dir = Path(sdf_path).parent
        collision_rel = str(Path(mesh_paths["collision"]).relative_to(sdf_dir))
        visual_rel = str(Path(mesh_paths["visual"]).relative_to(sdf_dir))

        # body_collision geometry 내부만 교체
        content = re.sub(
            r'(<collision name="body_collision">\s*<geometry>)\s*'
            r'<box><size>[^<]*</size></box>'
            r'(\s*</geometry>)',
            rf'\1\n          <mesh><uri>{collision_rel}</uri></mesh>\2',
            content,
            count=1,
        )

        # body_visual geometry 내부만 교체
        content = re.sub(
            r'(<visual name="body_visual">\s*<geometry>)\s*'
            r'<box><size>[^<]*</size></box>'
            r'(\s*</geometry>)',
            rf'\1\n          <mesh><uri>{visual_rel}</uri></mesh>\2',
            content,
            count=1,
        )

    Path(sdf_path).write_text(content, encoding="utf-8")
    print(f"✅ SDF 갱신 완료: {sdf_path}")


def print_report(physics: dict, mesh_paths: dict | None = None):
    """추출된 물리 파라미터를 보기 좋게 출력한다."""
    print("\n" + "=" * 60)
    print("  STP 물리 파라미터 추출 결과")
    print("=" * 60)
    print(f"  질량:      {physics['mass']:.3f} kg")
    print(f"  부피:      {physics['volume_m3']:.6f} m³")
    com = physics["center_of_mass"]
    print(f"  무게중심:  ({com[0]:.4f}, {com[1]:.4f}, {com[2]:.4f}) m")
    print()
    print("  관성 텐서 (kg·m²):")
    i = physics["inertia"]
    print(f"    Ixx = {i['ixx']:>10.4f}    Ixy = {i['ixy']:>10.4f}")
    print(f"    Iyy = {i['iyy']:>10.4f}    Ixz = {i['ixz']:>10.4f}")
    print(f"    Izz = {i['izz']:>10.4f}    Iyz = {i['iyz']:>10.4f}")
    print()
    bbox = physics["bbox"]
    print(f"  바운딩 박스: {bbox['size'][0]:.3f} x {bbox['size'][1]:.3f} x {bbox['size'][2]:.3f} m")
    print()

    if mesh_paths:
        print(f"  충돌 메시: {mesh_paths['collision']}")
        print(f"  시각 메시: {mesh_paths['visual']}")

    print("=" * 60)

    # 기존 SDF 값과 비교
    print("\n  [비교] 기존 model.sdf 값 (수기 입력):")
    print(f"    질량: 1000.0 kg / 차체 크기: 2.0 x 1.2 x 0.8 m")
    print(f"    Ixx=106.67, Iyy=153.33, Izz=113.33 (박스 근사)")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="STP → SDF 변환 파이프라인 (SS500 궤도차량)"
    )
    parser.add_argument("stp_file", help="STP(STEP) 파일 경로")
    parser.add_argument(
        "-o", "--output-dir",
        default=DEFAULT_MESH_DIR,
        help=f"메시 출력 디렉토리 (기본: {DEFAULT_MESH_DIR})",
    )
    parser.add_argument(
        "--sdf",
        default=DEFAULT_SDF_PATH,
        help=f"갱신할 model.sdf 경로 (기본: {DEFAULT_SDF_PATH})",
    )
    parser.add_argument(
        "--density",
        type=float,
        default=DEFAULT_DENSITY,
        help=f"재질 밀도 kg/m³ (기본: {DEFAULT_DENSITY} = 강철)",
    )
    parser.add_argument(
        "--mesh-only",
        action="store_true",
        help="메시만 생성하고 SDF는 갱신하지 않음",
    )
    parser.add_argument(
        "--no-mesh",
        action="store_true",
        help="메시 생성 없이 물리 파라미터만 추출 (SDF inertial만 갱신)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="파라미터 추출만 하고 파일 변경 없음",
    )
    parser.add_argument(
        "--unit",
        choices=["mm", "m"],
        default="mm",
        help="STP 파일의 단위 (기본: mm)",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="결과를 JSON으로 출력",
    )

    args = parser.parse_args()

    # 1. STP 로드
    print(f"📂 STP 파일 로드 중: {args.stp_file}")
    shape = load_step(args.stp_file)
    print("   로드 완료")

    # 2. 물리 파라미터 추출
    print(f"🔧 물리 파라미터 추출 중 (밀도: {args.density} kg/m³)...")
    physics = extract_physics(shape, args.density)

    # 단위가 m인 경우 스케일 보정 불필요 (기본 mm 가정)
    if args.unit == "m":
        # extract_physics 내부에서 mm→m 변환을 하므로,
        # m 단위 STP라면 변환을 되돌려야 함
        physics["mass"] *= 1e9  # 볼륨 보정
        physics["volume_m3"] *= 1e9
        for key in ["ixx", "iyy", "izz", "ixy", "ixz", "iyz"]:
            physics["inertia"][key] *= 1e15
        for i in range(3):
            physics["center_of_mass"][i] *= 1e3
            physics["bbox"]["min"][i] *= 1e3
            physics["bbox"]["max"][i] *= 1e3
            physics["bbox"]["size"][i] *= 1e3

    # 3. 리포트 출력
    if args.json:
        print(json.dumps(physics, indent=2, ensure_ascii=False))
    else:
        mesh_paths = None
        if not args.no_mesh and not args.dry_run:
            pass  # 아래에서 생성 후 출력
        print_report(physics)

    if args.dry_run:
        print("🔍 dry-run 모드: 파일 변경 없음")
        return

    # 4. 메시 생성
    mesh_paths = None
    if not args.no_mesh:
        print(f"📐 메시 생성 중...")
        output_dir = Path(args.output_dir)
        mesh_paths = export_meshes(shape, output_dir)
        print(f"   충돌 메시: {mesh_paths['collision']}")
        print(f"   시각 메시: {mesh_paths['visual']}")

    # 5. SDF 갱신
    if not args.mesh_only:
        print(f"📝 SDF 갱신 중: {args.sdf}")
        update_sdf(args.sdf, physics, mesh_paths)

    print("\n✅ 완료!")


if __name__ == "__main__":
    main()
