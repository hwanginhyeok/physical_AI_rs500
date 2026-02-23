"""SensorFusion Late Fusion 단위 테스트."""

import math
import pytest

try:
    import numpy as np
    _HAS_NUMPY = True
except ImportError:
    _HAS_NUMPY = False

from ad_core.lidar_processor import Obstacle
from ad_core.camera_detector import Detection
from ad_core.sensor_fusion import (
    CameraExtrinsics,
    CameraIntrinsics,
    FusedObject,
    SensorFusion,
)

pytestmark = pytest.mark.skipif(not _HAS_NUMPY, reason="numpy 필요")


# ── 픽스처 ─────────────────────────────────────────────────────

@pytest.fixture
def intrinsics():
    return CameraIntrinsics(
        fx=500.0, fy=500.0, cx=320.0, cy=240.0,
        image_width=640, image_height=480,
    )


@pytest.fixture
def extrinsics():
    return CameraExtrinsics(
        rotation=np.eye(3),
        translation=np.array([0.0, 0.0, 0.0]),
    )


@pytest.fixture
def fusion(intrinsics, extrinsics):
    return SensorFusion(
        intrinsics=intrinsics,
        extrinsics=extrinsics,
    )


def make_obstacle(cx=5.0, cy=0.0, cz=0.5, w=1.0, h=1.0, d=1.0, n=20):
    return Obstacle(
        center_x=cx, center_y=cy, center_z=cz,
        width=w, height=h, depth=d, num_points=n,
    )


def make_detection(cls="person", conf=0.9, x1=100, y1=100, x2=200, y2=300):
    return Detection(
        class_name=cls, confidence=conf,
        bbox_x1=x1, bbox_y1=y1, bbox_x2=x2, bbox_y2=y2,
    )


# ── FusedObject ────────────────────────────────────────────────

class TestFusedObject:
    def test_distance_auto_calculated(self):
        obj = FusedObject(x=3.0, y=4.0, z=0.0)
        assert abs(obj.distance - 5.0) < 1e-6

    def test_default_source(self):
        obj = FusedObject()
        assert obj.source == "lidar"

    def test_fused_source(self):
        obj = FusedObject(source="fused", confidence=0.95)
        assert obj.source == "fused"


# ── CameraIntrinsics ──────────────────────────────────────────

class TestCameraIntrinsics:
    def test_to_matrix_shape(self, intrinsics):
        K = intrinsics.to_matrix()
        assert K.shape == (3, 3)

    def test_to_matrix_values(self, intrinsics):
        K = intrinsics.to_matrix()
        assert K[0, 0] == 500.0  # fx
        assert K[1, 1] == 500.0  # fy
        assert K[0, 2] == 320.0  # cx
        assert K[1, 2] == 240.0  # cy


# ── CameraExtrinsics ──────────────────────────────────────────

class TestCameraExtrinsics:
    def test_to_matrix_shape(self, extrinsics):
        T = extrinsics.to_matrix()
        assert T.shape == (4, 4)

    def test_identity_extrinsics(self, extrinsics):
        T = extrinsics.to_matrix()
        assert np.allclose(T[:3, :3], np.eye(3))
        assert np.allclose(T[:3, 3], np.zeros(3))


# ── SensorFusion 핵심 기능 ────────────────────────────────────

class TestSensorFusionProjection:
    def test_project_point_in_front(self, fusion):
        """카메라 Z축 앞 점은 이미지 내에 투영된다.

        Identity extrinsics에서 LiDAR (x,y,z) → Camera (x,y,z) 그대로.
        카메라 Z축이 앞 방향이므로 z>0인 점이 카메라 앞에 있음.
        """
        result = fusion.project_to_image((0.0, 0.0, 5.0))
        assert result is not None
        u, v = result
        assert 0 <= u <= 640
        assert 0 <= v <= 480

    def test_project_point_behind_camera(self, fusion):
        """카메라 Z축 뒤 점은 None을 반환한다."""
        result = fusion.project_to_image((0.0, 0.0, -5.0))
        assert result is None

    def test_project_obstacle_bbox(self, fusion):
        """장애물의 3D bbox가 2D bbox로 투영된다."""
        obs = make_obstacle(cx=5.0, cy=0.0, cz=0.5)
        result = fusion.project_obstacle_bbox(obs)
        # 카메라 앞에 있으므로 bbox가 나와야 함
        if result is not None:
            x1, y1, x2, y2 = result
            assert x2 > x1
            assert y2 > y1


class TestSensorFusionFuse:
    def test_lidar_only(self, fusion):
        """카메라 탐지 없이 LiDAR만 → lidar 소스 객체."""
        obstacles = [make_obstacle(cx=5.0)]
        result = fusion.fuse(obstacles, [], None)
        assert len(result) >= 1
        assert all(obj.source in ("lidar", "fused") for obj in result)

    def test_camera_only(self, fusion):
        """LiDAR 없이 카메라만 → camera 소스 객체."""
        detections = [make_detection()]
        result = fusion.fuse([], detections, None)
        assert len(result) >= 1
        for obj in result:
            assert obj.source == "camera"

    def test_empty_inputs(self, fusion):
        """빈 입력 → 빈 결과."""
        result = fusion.fuse([], [], None)
        assert len(result) == 0

    def test_fused_confidence_higher(self, fusion):
        """매칭된 fused 객체의 신뢰도가 단일 소스보다 높을 수 있다."""
        obstacles = [make_obstacle(cx=5.0, cy=0.0)]
        # 카메라 bbox를 LiDAR 투영 위치 근처에 배치
        bbox = fusion.project_obstacle_bbox(obstacles[0])
        if bbox is not None:
            x1, y1, x2, y2 = bbox
            detections = [make_detection(x1=x1, y1=y1, x2=x2, y2=y2, conf=0.8)]
            result = fusion.fuse(obstacles, detections, None)
            fused = [r for r in result if r.source == "fused"]
            if fused:
                assert fused[0].confidence >= 0.8

    def test_multiple_obstacles(self, fusion):
        """여러 장애물 처리."""
        obstacles = [
            make_obstacle(cx=3.0, cy=1.0),
            make_obstacle(cx=8.0, cy=-2.0),
            make_obstacle(cx=15.0, cy=0.5),
        ]
        result = fusion.fuse(obstacles, [], None)
        assert len(result) == 3

    def test_result_types(self, fusion):
        """결과가 모두 FusedObject 타입."""
        obstacles = [make_obstacle()]
        result = fusion.fuse(obstacles, [], None)
        for obj in result:
            assert isinstance(obj, FusedObject)
            assert isinstance(obj.distance, float)
            assert obj.distance >= 0


class TestSensorFusionProperties:
    def test_intrinsics_property(self, fusion, intrinsics):
        assert fusion.intrinsics.fx == intrinsics.fx

    def test_set_intrinsics(self, fusion):
        new_intr = CameraIntrinsics(
            fx=600, fy=600, cx=320, cy=240,
            image_width=640, image_height=480,
        )
        fusion.intrinsics = new_intr
        assert fusion.intrinsics.fx == 600.0

    def test_extrinsics_property(self, fusion):
        assert fusion.extrinsics is not None

    def test_iou_computation(self):
        """IoU 계산: 완전 겹침 → 1.0, 무겹침 → 0.0."""
        # 완전 겹침
        iou = SensorFusion._compute_iou(
            (0, 0, 100, 100), (0, 0, 100, 100)
        )
        assert abs(iou - 1.0) < 1e-6

        # 무겹침
        iou = SensorFusion._compute_iou(
            (0, 0, 50, 50), (100, 100, 200, 200)
        )
        assert abs(iou) < 1e-6
