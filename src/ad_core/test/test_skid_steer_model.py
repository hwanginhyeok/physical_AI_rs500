"""SkidSteerModel / CalibratedSkidSteerModel 단위 테스트."""

import math
import os
import json
import tempfile

import pytest

from ad_core.skid_steer_model import (
    CalibratedSkidSteerModel,
    CalibrationData,
    Pose2D,
    SkidSteerModel,
)


# ── SkidSteerModel 기본 동작 ───────────────────────────────────

class TestSkidSteerModel:
    @pytest.fixture
    def model(self):
        return SkidSteerModel(track_width=1.4, max_speed=2.0, steering_efficiency=0.8)

    def test_straight_forward(self, model):
        """동일한 트랙 속도 → 직진 (angular=0)."""
        v_left, v_right = model.twist_to_tracks(linear=1.0, angular=0.0)
        assert abs(v_left - v_right) < 1e-6
        assert abs(v_left - 1.0) < 1e-6

    def test_pure_rotation(self, model):
        """linear=0, angular>0 → 좌/우 트랙이 반대 방향."""
        v_left, v_right = model.twist_to_tracks(linear=0.0, angular=1.0)
        assert v_left < 0  # 좌 트랙 후진
        assert v_right > 0  # 우 트랙 전진

    def test_twist_tracks_inverse(self, model):
        """twist_to_tracks → tracks_to_twist 왕복 변환이 일관적."""
        lin_in, ang_in = 0.5, 0.3
        v_l, v_r = model.twist_to_tracks(lin_in, ang_in)
        lin_out, ang_out = model.tracks_to_twist(v_l, v_r)
        assert abs(lin_out - lin_in) < 1e-4
        assert abs(ang_out - ang_in) < 1e-4

    def test_speed_saturation(self, model):
        """트랙 속도가 max_speed를 초과하지 않는다."""
        v_left, v_right = model.twist_to_tracks(linear=10.0, angular=10.0)
        assert abs(v_left) <= model.max_speed + 1e-6
        assert abs(v_right) <= model.max_speed + 1e-6

    def test_predict_pose_straight(self, model):
        """동일 트랙 속도로 직진하면 y=0, yaw=0 유지."""
        pose = Pose2D(x=0, y=0, yaw=0)
        new_pose = model.predict_pose(pose, v_left=1.0, v_right=1.0, dt=1.0)
        assert new_pose.x > 0  # 전진
        assert abs(new_pose.y) < 1e-6
        assert abs(new_pose.yaw) < 1e-6

    def test_predict_pose_rotation(self, model):
        """반대 트랙 속도 → yaw 변화."""
        pose = Pose2D(x=0, y=0, yaw=0)
        new_pose = model.predict_pose(pose, v_left=-0.5, v_right=0.5, dt=1.0)
        assert abs(new_pose.yaw) > 0.01  # 회전 발생

    def test_predict_pose_zero_dt(self, model):
        """dt=0이면 위치 변화 없음."""
        pose = Pose2D(x=5.0, y=3.0, yaw=1.0)
        new_pose = model.predict_pose(pose, v_left=1.0, v_right=0.5, dt=0.0)
        assert abs(new_pose.x - pose.x) < 1e-6
        assert abs(new_pose.y - pose.y) < 1e-6
        assert abs(new_pose.yaw - pose.yaw) < 1e-6

    def test_estimate_icr_straight(self, model):
        """동일 속도 → ICR 무한대 (직진)."""
        icr = model.estimate_icr(v_left=1.0, v_right=1.0)
        assert abs(icr) > 1000  # 매우 큰 값 (직진)

    def test_estimate_icr_rotation(self, model):
        """반대 속도 → ICR ≈ 0."""
        icr = model.estimate_icr(v_left=-1.0, v_right=1.0)
        assert abs(icr) < 0.1


# ── CalibratedSkidSteerModel ──────────────────────────────────

class TestCalibratedSkidSteerModel:
    @pytest.fixture
    def cal_model(self, tmp_path):
        return CalibratedSkidSteerModel(
            track_width=1.4,
            max_speed=2.0,
            steering_efficiency=0.8,
            calibration_file=str(tmp_path / "cal.json"),
            calibration_enabled=True,
        )

    def test_inherits_base_model(self, cal_model):
        assert isinstance(cal_model, SkidSteerModel)

    def test_initial_effective_width_equals_track_width(self, cal_model):
        assert abs(cal_model.effective_track_width - 1.4) < 1e-6

    def test_track_width_correction_updates(self, cal_model):
        """commanded vs measured angular 차이로 보정값이 변한다."""
        initial_width = cal_model.effective_track_width
        for _ in range(20):
            cal_model.update_track_width_correction(
                commanded_angular=1.0, measured_angular=0.8
            )
        assert cal_model.effective_track_width != initial_width

    def test_motor_asymmetry_updates(self, cal_model):
        """yaw rate 비대칭이 감지되면 게인이 변한다."""
        initial_left = cal_model.get_calibration_status()["left_motor_gain"]
        initial_right = cal_model.get_calibration_status()["right_motor_gain"]
        # 직진 명령 (linear > threshold, angular ≈ 0)이면서 yaw drift 발생
        for _ in range(50):
            cal_model.update_motor_asymmetry(
                commanded_linear=2.0,
                commanded_angular=0.0,
                measured_yaw_rate=0.1,  # 좌회전 편향
            )
        status = cal_model.get_calibration_status()
        # 게인 보정이 적용되었는지 확인 (좌/우 중 하나라도 변화)
        left_changed = abs(status["left_motor_gain"] - initial_left) > 1e-6
        right_changed = abs(status["right_motor_gain"] - initial_right) > 1e-6
        assert left_changed or right_changed

    def test_save_and_load_calibration(self, cal_model, tmp_path):
        cal_file = str(tmp_path / "test_cal.json")
        # 캘리브레이션 데이터 생성
        for _ in range(10):
            cal_model.update_track_width_correction(1.0, 0.85)
        assert cal_model.save_calibration(cal_file)
        assert os.path.exists(cal_file)

        # 새 모델에 로드
        new_model = CalibratedSkidSteerModel(track_width=1.4)
        assert new_model.load_calibration(cal_file)
        assert abs(new_model.effective_track_width - cal_model.effective_track_width) < 1e-4

    def test_reset_calibration(self, cal_model):
        for _ in range(10):
            cal_model.update_track_width_correction(1.0, 0.8)
        cal_model.reset_calibration()
        assert abs(cal_model.effective_track_width - 1.4) < 1e-6

    def test_get_calibration_status(self, cal_model):
        status = cal_model.get_calibration_status()
        assert "effective_track_width" in status
        assert "left_motor_gain" in status
        assert "right_motor_gain" in status
        assert "calibration_count" in status

    def test_straight_calibration(self, cal_model):
        """직진 캘리브레이션: yaw rate 샘플로 비대칭 보정."""
        samples = [0.02, 0.03, 0.01, 0.02, 0.025]
        result = cal_model.run_straight_calibration(samples, linear_speed=1.0)
        assert isinstance(result, float)

    def test_rotation_calibration(self, cal_model):
        """회전 캘리브레이션: commanded vs measured angular 비교."""
        commanded = [0.5, 1.0, 1.5, 2.0]
        measured = [0.4, 0.85, 1.25, 1.7]
        result = cal_model.run_rotation_calibration(commanded, measured)
        assert isinstance(result, float)


# ── CalibrationData ────────────────────────────────────────────

class TestCalibrationData:
    def test_default_values(self):
        data = CalibrationData()
        assert data.track_width_correction == 1.0  # 1.0 = 보정 없음 (배수)
        assert data.left_motor_gain == 1.0
        assert data.right_motor_gain == 1.0

    def test_to_dict_and_from_dict(self):
        data = CalibrationData(
            track_width_correction=0.05,
            left_motor_gain=0.95,
            right_motor_gain=1.02,
            calibration_count=50,
        )
        d = data.to_dict()
        restored = CalibrationData.from_dict(d)
        assert abs(restored.track_width_correction - 0.05) < 1e-6
        assert abs(restored.left_motor_gain - 0.95) < 1e-6
        assert restored.calibration_count == 50
