"""ICR 기반 스키드 스티어 조향 모델 모듈.

궤도차량(tracked vehicle)의 운동학적 모델을 구현한다.
좌/우 트랙 속도 차이에 의한 회전을 ICR(Instantaneous Center of Rotation)
기반으로 모델링하며, 슬립 보정을 지원한다.

Phase 2 확장:
- CalibratedSkidSteerModel: 유효 궤도 폭 온라인 보정 + 모터 비대칭 보상
"""

import json
import math
import os
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

from ad_core.datatypes import Pose2D


class SkidSteerModel:
    """ICR 기반 스키드 스티어 운동학 모델.

    좌/우 트랙 속도로부터 차량의 선속도와 각속도를 계산하고,
    ICR(순간 회전 중심)을 추정하여 다음 위치를 예측한다.

    Attributes:
        track_width: 좌/우 트랙 사이 간격 (m).
        max_speed: 트랙 최대 속도 (m/s).
        steering_efficiency: 조향 효율 계수 (0~1). 미끄러짐으로 인한 손실 반영.
        slip_ratio: 슬립 보정 팩터 (0.0 = 슬립 없음).
    """

    def __init__(
        self,
        track_width: float = 1.4,
        max_speed: float = 1.0,
        steering_efficiency: float = 0.8,
    ) -> None:
        """초기화.

        Args:
            track_width: 트랙 간격 (m). 좌/우 트랙 중심 사이 거리.
            max_speed: 각 트랙의 최대 허용 속도 (m/s).
            steering_efficiency: 조향 효율 (0~1). 이상적 조향 대비 실제 효율.
        """
        self.track_width: float = track_width
        self.max_speed: float = max_speed
        self.steering_efficiency: float = steering_efficiency
        self.slip_ratio: float = 0.0

    def twist_to_tracks(
        self, linear: float, angular: float
    ) -> Tuple[float, float]:
        """(선속도, 각속도)를 좌/우 트랙 속도로 변환한다.

        차동 구동 역기구학:
            v_left  = v - (omega * W) / (2 * eta)
            v_right = v + (omega * W) / (2 * eta)

        여기서 eta는 조향 효율이며, 효율이 낮을수록 같은 각속도를 내기 위해
        더 큰 트랙 속도 차이가 필요하다.

        Args:
            linear: 목표 선속도 (m/s).
            angular: 목표 각속도 (rad/s).

        Returns:
            (v_left, v_right) 포화 처리된 트랙 속도 (m/s).
        """
        # 조향 효율을 반영한 유효 트랙 간격
        effective_width = self.track_width / max(self.steering_efficiency, 1e-6)
        half_w = effective_width / 2.0

        v_left = linear - angular * half_w
        v_right = linear + angular * half_w

        # 포화 처리 (비율 유지)
        v_left, v_right = self._saturate(v_left, v_right)

        return v_left, v_right

    def tracks_to_twist(
        self, v_left: float, v_right: float
    ) -> Tuple[float, float]:
        """좌/우 트랙 속도를 (선속도, 각속도)로 역변환한다.

        차동 구동 순기구학:
            v     = (v_right + v_left) / 2
            omega = (v_right - v_left) * eta / W

        슬립 보정이 적용된 경우 유효 속도를 보정한다.

        Args:
            v_left: 좌측 트랙 속도 (m/s).
            v_right: 우측 트랙 속도 (m/s).

        Returns:
            (linear, angular) 선속도(m/s)와 각속도(rad/s).
        """
        # 슬립 보정 적용
        effective_left = v_left * (1.0 - self.slip_ratio)
        effective_right = v_right * (1.0 - self.slip_ratio)

        linear = (effective_right + effective_left) / 2.0
        angular = (
            (effective_right - effective_left)
            * self.steering_efficiency
            / self.track_width
        )

        return linear, angular

    def predict_pose(
        self,
        current_pose: Pose2D,
        v_left: float,
        v_right: float,
        dt: float,
    ) -> Pose2D:
        """ICR 기반으로 다음 타임스텝의 위치를 예측한다.

        직진(v_left ~= v_right) 시에는 직선 모델을 사용하고,
        회전 시에는 ICR(순간 회전 중심) 주위의 원호 모델을 사용한다.

        Args:
            current_pose: 현재 위치 및 방향.
            v_left: 좌측 트랙 속도 (m/s).
            v_right: 우측 트랙 속도 (m/s).
            dt: 시간 간격 (s).

        Returns:
            예측된 다음 위치 (Pose2D).
        """
        linear, angular = self.tracks_to_twist(v_left, v_right)

        new_pose = Pose2D(
            x=current_pose.x,
            y=current_pose.y,
            yaw=current_pose.yaw,
        )

        if abs(angular) < 1e-6:
            # 직진 모델: 곡률이 거의 0
            new_pose.x += linear * math.cos(current_pose.yaw) * dt
            new_pose.y += linear * math.sin(current_pose.yaw) * dt
        else:
            # ICR 기반 원호 모델
            radius = linear / angular  # 회전 반경 = v / omega
            icr_x = current_pose.x - radius * math.sin(current_pose.yaw)
            icr_y = current_pose.y + radius * math.cos(current_pose.yaw)

            d_theta = angular * dt
            new_pose.yaw = _normalize_angle(current_pose.yaw + d_theta)

            # ICR을 중심으로 회전
            new_pose.x = icr_x + radius * math.sin(new_pose.yaw)
            new_pose.y = icr_y - radius * math.cos(new_pose.yaw)

        return new_pose

    def estimate_icr(
        self, v_left: float, v_right: float
    ) -> float:
        """순간 회전 중심(ICR)의 y좌표를 추정한다.

        ICR은 차량 좌표계에서 좌/우 트랙 사이에 위치하며,
        그 y좌표는 회전 반경의 부호와 크기를 나타낸다.

        - v_left == v_right: ICR이 무한대 (직진)
        - v_left == -v_right: ICR이 차량 중심 (제자리 회전)
        - v_left == 0: ICR이 좌측 트랙 위 (좌측 고정 회전)

        Args:
            v_left: 좌측 트랙 속도 (m/s).
            v_right: 우측 트랙 속도 (m/s).

        Returns:
            ICR의 y좌표 (m). 차량 좌표계 기준.
            좌/우 속도가 동일하면 float('inf')를 반환.
        """
        # 슬립 보정 적용
        eff_left = v_left * (1.0 - self.slip_ratio)
        eff_right = v_right * (1.0 - self.slip_ratio)

        diff = eff_right - eff_left
        if abs(diff) < 1e-9:
            return float('inf')  # 직진 — ICR이 무한대

        half_w = self.track_width / 2.0

        # ICR y좌표: y_icr = W/2 * (v_right + v_left) / (v_right - v_left)
        # 양수 → 좌측에 ICR (우회전), 음수 → 우측에 ICR (좌회전)
        icr_y = half_w * (eff_right + eff_left) / diff

        return icr_y

    def _saturate(
        self, v_left: float, v_right: float
    ) -> Tuple[float, float]:
        """트랙 속도를 최대 속도 이내로 포화 처리한다.

        두 트랙 속도의 비율을 유지하면서 포화시킨다.

        Args:
            v_left: 좌측 트랙 속도 (m/s).
            v_right: 우측 트랙 속도 (m/s).

        Returns:
            포화 처리된 (v_left, v_right).
        """
        max_abs = max(abs(v_left), abs(v_right))
        if max_abs > self.max_speed and max_abs > 1e-9:
            scale = self.max_speed / max_abs
            v_left *= scale
            v_right *= scale
        return v_left, v_right


def _normalize_angle(angle: float) -> float:
    """각도를 [-pi, pi) 범위로 정규화한다.

    Args:
        angle: 입력 각도 (rad).

    Returns:
        정규화된 각도 (rad).
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


# ====================================================================== #
#  Phase 2: 유효 궤도 폭 보정 + 모터 비대칭 보상
# ====================================================================== #


@dataclass
class CalibrationData:
    """캘리브레이션 결과 데이터.

    파일로 저장/로드하여 재시작 시에도 보정 계수를 유지할 수 있다.

    Attributes:
        track_width_correction: 유효 궤도 폭 보정 계수.
            effective_track_width = nominal_width * correction_factor.
            1.0이면 보정 없음.
        left_motor_gain: 좌측 모터 게인 보정 계수.
            좌측 트랙 명령에 곱해지는 보정값. 1.0이면 보정 없음.
        right_motor_gain: 우측 모터 게인 보정 계수.
            우측 트랙 명령에 곱해지는 보정값. 1.0이면 보정 없음.
        calibration_count: 캘리브레이션이 수행된 총 횟수.
        track_width_samples: 궤도 폭 보정에 사용된 샘플 수.
        asymmetry_samples: 비대칭 보상에 사용된 샘플 수.
    """

    track_width_correction: float = 1.0
    left_motor_gain: float = 1.0
    right_motor_gain: float = 1.0
    calibration_count: int = 0
    track_width_samples: int = 0
    asymmetry_samples: int = 0

    def to_dict(self) -> Dict:
        """딕셔너리로 변환한다.

        Returns:
            캘리브레이션 데이터 딕셔너리.
        """
        return {
            'track_width_correction': self.track_width_correction,
            'left_motor_gain': self.left_motor_gain,
            'right_motor_gain': self.right_motor_gain,
            'calibration_count': self.calibration_count,
            'track_width_samples': self.track_width_samples,
            'asymmetry_samples': self.asymmetry_samples,
        }

    @classmethod
    def from_dict(cls, data: Dict) -> 'CalibrationData':
        """딕셔너리로부터 생성한다.

        Args:
            data: 캘리브레이션 데이터 딕셔너리.

        Returns:
            CalibrationData 인스턴스.
        """
        return cls(
            track_width_correction=data.get('track_width_correction', 1.0),
            left_motor_gain=data.get('left_motor_gain', 1.0),
            right_motor_gain=data.get('right_motor_gain', 1.0),
            calibration_count=data.get('calibration_count', 0),
            track_width_samples=data.get('track_width_samples', 0),
            asymmetry_samples=data.get('asymmetry_samples', 0),
        )


class CalibratedSkidSteerModel(SkidSteerModel):
    """보정 기능이 통합된 스키드 스티어 운동학 모델.

    기존 SkidSteerModel을 상속하여 두 가지 온라인 보정 기능을 추가한다:

    1. **유효 궤도 폭 보정 (Effective Track Width Correction)**:
       실제 궤도차량의 유효 궤도 폭은 기하학적 폭과 다르다.
       지면 조건(흙, 자갈, 아스팔트 등)에 따라 변화하며,
       회전 시 실측 IMU 각속도와 명령 각속도를 비교하여
       correction_factor를 EMA(지수이동평균)로 점진 갱신한다.

    2. **모터 비대칭 보상 (Motor Asymmetry Compensation)**:
       좌/우 트랙 모터의 응답 특성 차이를 보상한다.
       직진 명령 시 IMU yaw rate로 편향(drift)을 감지하고,
       좌/우 모터 게인을 개별 보정한다.

    기존 SkidSteerModel의 모든 인터페이스를 그대로 유지하며,
    보정이 비활성화된 경우 부모 클래스와 동일하게 동작한다.

    Attributes:
        calibration: 캘리브레이션 데이터 (보정 계수 포함).
        ema_alpha: EMA 갱신 가중치 (0 < alpha <= 1).
        calibration_enabled: 온라인 캘리브레이션 활성화 여부.
        min_angular_for_track_cal: 궤도 폭 보정을 위한 최소 각속도 임계값.
        max_drift_for_asymmetry_cal: 비대칭 보정을 위한 최대 허용 yaw rate 임계값.
        straight_speed_threshold: 직진 판정을 위한 최소 선속도 임계값.
        correction_factor_min: correction_factor 하한 (발산 방지).
        correction_factor_max: correction_factor 상한 (발산 방지).
        motor_gain_min: 모터 게인 보정 하한.
        motor_gain_max: 모터 게인 보정 상한.
    """

    def __init__(
        self,
        track_width: float = 1.4,
        max_speed: float = 1.0,
        steering_efficiency: float = 0.8,
        ema_alpha: float = 0.05,
        calibration_file: Optional[str] = None,
        calibration_enabled: bool = True,
        min_angular_for_track_cal: float = 0.1,
        max_drift_for_asymmetry_cal: float = 0.5,
        straight_speed_threshold: float = 0.1,
        correction_factor_min: float = 0.5,
        correction_factor_max: float = 2.0,
        motor_gain_min: float = 0.8,
        motor_gain_max: float = 1.2,
    ) -> None:
        """초기화.

        Args:
            track_width: 공칭(nominal) 트랙 간격 (m).
            max_speed: 각 트랙의 최대 허용 속도 (m/s).
            steering_efficiency: 조향 효율 (0~1).
            ema_alpha: EMA 갱신 가중치 (0 < alpha <= 1).
                alpha가 클수록 최근 측정에 민감하게 반응한다.
                기본값 0.05는 약 20회 측정의 이동평균에 해당.
            calibration_file: 캘리브레이션 데이터 저장/로드 파일 경로.
                None이면 파일 저장/로드를 하지 않는다.
            calibration_enabled: 온라인 캘리브레이션 활성화 여부.
                False이면 기존 보정값만 사용하고 갱신하지 않는다.
            min_angular_for_track_cal: 궤도 폭 보정을 수행할 최소 명령 각속도 (rad/s).
                이 값보다 작은 각속도에서는 측정 노이즈가 지배적이므로 보정을 건너뛴다.
            max_drift_for_asymmetry_cal: 비대칭 보정에서 유효한 최대 yaw rate (rad/s).
                이 값을 초과하는 drift는 외란으로 간주하여 무시한다.
            straight_speed_threshold: 직진 판정을 위한 최소 선속도 (m/s).
                이 속도 이상이고 명령 각속도가 거의 0일 때 직진으로 판정한다.
            correction_factor_min: track_width_correction 하한.
            correction_factor_max: track_width_correction 상한.
            motor_gain_min: 모터 게인 보정 하한.
            motor_gain_max: 모터 게인 보정 상한.
        """
        super().__init__(
            track_width=track_width,
            max_speed=max_speed,
            steering_efficiency=steering_efficiency,
        )

        # EMA 파라미터
        self.ema_alpha: float = max(0.001, min(1.0, ema_alpha))

        # 캘리브레이션 설정
        self.calibration_enabled: bool = calibration_enabled
        self._calibration_file: Optional[str] = calibration_file

        # 보정 임계값
        self.min_angular_for_track_cal: float = min_angular_for_track_cal
        self.max_drift_for_asymmetry_cal: float = max_drift_for_asymmetry_cal
        self.straight_speed_threshold: float = straight_speed_threshold

        # 보정 계수 범위 제한 (안전 클램프)
        self.correction_factor_min: float = correction_factor_min
        self.correction_factor_max: float = correction_factor_max
        self.motor_gain_min: float = motor_gain_min
        self.motor_gain_max: float = motor_gain_max

        # 캘리브레이션 데이터 초기화 또는 파일에서 로드
        self.calibration: CalibrationData = CalibrationData()
        if calibration_file:
            self._load_calibration()

        # 직진 drift 측정을 위한 누적 버퍼
        self._drift_accumulator: float = 0.0
        self._drift_sample_count: int = 0

    # ------------------------------------------------------------------ #
    #  유효 궤도 폭 (Effective Track Width)
    # ------------------------------------------------------------------ #

    @property
    def effective_track_width(self) -> float:
        """보정된 유효 궤도 폭을 반환한다.

        effective_track_width = nominal_width * correction_factor

        Returns:
            유효 궤도 폭 (m).
        """
        return self.track_width * self.calibration.track_width_correction

    # ------------------------------------------------------------------ #
    #  오버라이드: 보정된 궤도 폭과 모터 게인 적용
    # ------------------------------------------------------------------ #

    def twist_to_tracks(
        self, linear: float, angular: float
    ) -> Tuple[float, float]:
        """(선속도, 각속도)를 보정된 좌/우 트랙 속도로 변환한다.

        부모 클래스의 역기구학에 유효 궤도 폭 보정과 모터 비대칭 보상을 적용한다.

        역기구학:
            effective_W = nominal_W * correction_factor
            v_left  = (v - omega * effective_W / (2 * eta)) * left_gain
            v_right = (v + omega * effective_W / (2 * eta)) * right_gain

        Args:
            linear: 목표 선속도 (m/s).
            angular: 목표 각속도 (rad/s).

        Returns:
            (v_left, v_right) 보정 및 포화 처리된 트랙 속도 (m/s).
        """
        # 보정된 유효 폭을 사용한 역기구학
        eff_width = self.effective_track_width
        half_w = eff_width / max(self.steering_efficiency, 1e-6) / 2.0

        v_left = linear - angular * half_w
        v_right = linear + angular * half_w

        # 모터 비대칭 보상 적용
        v_left *= self.calibration.left_motor_gain
        v_right *= self.calibration.right_motor_gain

        # 포화 처리 (비율 유지)
        v_left, v_right = self._saturate(v_left, v_right)

        return v_left, v_right

    def tracks_to_twist(
        self, v_left: float, v_right: float
    ) -> Tuple[float, float]:
        """보정된 좌/우 트랙 속도를 (선속도, 각속도)로 역변환한다.

        모터 게인 보정을 역적용하고, 유효 궤도 폭을 사용하여 순기구학을 수행한다.

        Args:
            v_left: 좌측 트랙 속도 (m/s).
            v_right: 우측 트랙 속도 (m/s).

        Returns:
            (linear, angular) 선속도(m/s)와 각속도(rad/s).
        """
        # 모터 게인 보정 역적용 → 실제 트랙 유효 속도
        eff_left_gain = max(self.calibration.left_motor_gain, 1e-6)
        eff_right_gain = max(self.calibration.right_motor_gain, 1e-6)
        actual_left = v_left / eff_left_gain
        actual_right = v_right / eff_right_gain

        # 슬립 보정 적용
        effective_left = actual_left * (1.0 - self.slip_ratio)
        effective_right = actual_right * (1.0 - self.slip_ratio)

        linear = (effective_right + effective_left) / 2.0

        # 유효 궤도 폭으로 각속도 계산
        eff_width = self.effective_track_width
        angular = (
            (effective_right - effective_left)
            * self.steering_efficiency
            / max(eff_width, 1e-6)
        )

        return linear, angular

    # ------------------------------------------------------------------ #
    #  온라인 캘리브레이션 메서드
    # ------------------------------------------------------------------ #

    def update_track_width_correction(
        self,
        commanded_angular: float,
        measured_angular: float,
    ) -> None:
        """IMU 측정 각속도를 이용하여 유효 궤도 폭 보정 계수를 갱신한다.

        회전 중 명령 각속도와 실제 IMU 측정 각속도를 비교하여
        유효 궤도 폭의 보정 계수(correction_factor)를 온라인으로 추정한다.

        이론:
            명령 각속도 = (v_right - v_left) * eta / effective_W
            측정 각속도 = (v_right - v_left) * eta / actual_effective_W

            ratio = commanded / measured = actual_W / effective_W
            => actual_correction = current_correction * ratio

        EMA를 통해 점진적으로 갱신하여 노이즈에 강건하게 동작한다.

        Args:
            commanded_angular: 명령 각속도 (rad/s). twist_to_tracks에 전달한 값.
            measured_angular: IMU에서 측정된 실제 각속도 (rad/s).
        """
        if not self.calibration_enabled:
            return

        # 각속도가 너무 작으면 측정 노이즈가 지배적이므로 건너뜀
        if abs(commanded_angular) < self.min_angular_for_track_cal:
            return
        if abs(measured_angular) < 1e-6:
            return

        # 명령/측정 비율 → 현재 보정 계수의 수정이 필요한 비율
        # commanded > measured → 유효 폭이 너무 좁게 추정됨 → correction 증가
        # commanded < measured → 유효 폭이 너무 넓게 추정됨 → correction 감소
        ratio = commanded_angular / measured_angular

        # 현재 correction에 ratio를 곱한 것이 실제 correction
        new_correction = self.calibration.track_width_correction * ratio

        # 클램프하여 발산 방지
        new_correction = max(
            self.correction_factor_min,
            min(new_correction, self.correction_factor_max),
        )

        # EMA 갱신: correction = (1 - alpha) * old + alpha * new
        self.calibration.track_width_correction = (
            (1.0 - self.ema_alpha) * self.calibration.track_width_correction
            + self.ema_alpha * new_correction
        )

        self.calibration.track_width_samples += 1

    def update_motor_asymmetry(
        self,
        commanded_linear: float,
        commanded_angular: float,
        measured_yaw_rate: float,
    ) -> None:
        """직진 시 IMU yaw rate를 이용하여 모터 비대칭을 보상한다.

        직진 명령(angular ~= 0, linear > threshold)이 주어졌을 때
        실제 IMU yaw rate가 0이 아니면 좌/우 모터 응답 특성에 차이가 있는 것이다.

        측정된 drift(편향 yaw rate)를 기반으로 좌/우 모터 게인을 조정한다:
        - drift > 0 (좌회전 편향): 좌측이 느리거나 우측이 빠름
          → left_gain 증가 또는 right_gain 감소
        - drift < 0 (우회전 편향): 우측이 느리거나 좌측이 빠름
          → right_gain 증가 또는 left_gain 감소

        Args:
            commanded_linear: 명령 선속도 (m/s).
            commanded_angular: 명령 각속도 (rad/s).
            measured_yaw_rate: IMU에서 측정된 yaw rate (rad/s).
        """
        if not self.calibration_enabled:
            return

        # 직진 상태가 아니면 건너뜀
        if abs(commanded_linear) < self.straight_speed_threshold:
            return
        if abs(commanded_angular) > self.min_angular_for_track_cal:
            return

        # drift가 유효 범위를 초과하면 외란으로 간주
        drift = measured_yaw_rate
        if abs(drift) > self.max_drift_for_asymmetry_cal:
            return

        # 누적 버퍼에 추가 (노이즈 평활)
        self._drift_accumulator += drift
        self._drift_sample_count += 1

        # 충분한 샘플이 모이면 보정 적용 (최소 5개)
        if self._drift_sample_count < 5:
            return

        avg_drift = self._drift_accumulator / self._drift_sample_count

        # drift를 게인 보정으로 변환
        # 선속도에 대한 drift의 비율로 보정 크기를 결정
        # gain_delta는 작은 값이 되도록 선속도로 정규화
        gain_delta = avg_drift / max(abs(commanded_linear), 0.1)

        # 보정 적용: drift > 0이면 좌측 가속/우측 감속
        # EMA로 점진 갱신
        new_left_gain = self.calibration.left_motor_gain + gain_delta * 0.5
        new_right_gain = self.calibration.right_motor_gain - gain_delta * 0.5

        # 클램프
        new_left_gain = max(
            self.motor_gain_min, min(new_left_gain, self.motor_gain_max)
        )
        new_right_gain = max(
            self.motor_gain_min, min(new_right_gain, self.motor_gain_max)
        )

        # EMA 갱신
        self.calibration.left_motor_gain = (
            (1.0 - self.ema_alpha) * self.calibration.left_motor_gain
            + self.ema_alpha * new_left_gain
        )
        self.calibration.right_motor_gain = (
            (1.0 - self.ema_alpha) * self.calibration.right_motor_gain
            + self.ema_alpha * new_right_gain
        )

        self.calibration.asymmetry_samples += self._drift_sample_count

        # 누적 버퍼 리셋
        self._drift_accumulator = 0.0
        self._drift_sample_count = 0

    def run_straight_calibration(
        self,
        yaw_rate_samples: list,
        linear_speed: float = 0.5,
    ) -> float:
        """직진 캘리브레이션 루틴을 실행한다.

        직진 명령 시 수집된 IMU yaw rate 샘플들로부터 평균 drift를 계산하고,
        모터 비대칭 보상 계수를 일괄 갱신한다.

        이 메서드는 오프라인 캘리브레이션용으로, 차량을 직선 주행시킨 후
        수집된 데이터로 한 번에 보정 계수를 계산한다.

        Args:
            yaw_rate_samples: IMU yaw rate 측정값 리스트 (rad/s).
            linear_speed: 캘리브레이션 시 직진 속도 (m/s).

        Returns:
            측정된 평균 drift (rad/s). 양수는 좌편향, 음수는 우편향.
        """
        if not yaw_rate_samples:
            return 0.0

        avg_drift = sum(yaw_rate_samples) / len(yaw_rate_samples)

        # drift를 게인 보정으로 변환
        gain_delta = avg_drift / max(abs(linear_speed), 0.1)

        # 직접 갱신 (EMA가 아닌 일괄 보정)
        self.calibration.left_motor_gain += gain_delta * 0.5
        self.calibration.right_motor_gain -= gain_delta * 0.5

        # 클램프
        self.calibration.left_motor_gain = max(
            self.motor_gain_min,
            min(self.calibration.left_motor_gain, self.motor_gain_max),
        )
        self.calibration.right_motor_gain = max(
            self.motor_gain_min,
            min(self.calibration.right_motor_gain, self.motor_gain_max),
        )

        self.calibration.calibration_count += 1
        self.calibration.asymmetry_samples += len(yaw_rate_samples)

        return avg_drift

    def run_rotation_calibration(
        self,
        commanded_angular_samples: list,
        measured_angular_samples: list,
    ) -> float:
        """회전 캘리브레이션 루틴을 실행한다.

        회전 명령 시 명령/측정 각속도 쌍을 일괄로 분석하여
        유효 궤도 폭 보정 계수를 갱신한다.

        Args:
            commanded_angular_samples: 명령 각속도 리스트 (rad/s).
            measured_angular_samples: IMU 측정 각속도 리스트 (rad/s).

        Returns:
            계산된 보정 계수. 이전 보정 계수 대비 변화율.
        """
        if not commanded_angular_samples or not measured_angular_samples:
            return self.calibration.track_width_correction

        n = min(len(commanded_angular_samples), len(measured_angular_samples))

        # 유효한 샘플만 필터링 (최소 각속도 이상)
        ratios = []
        for i in range(n):
            cmd = commanded_angular_samples[i]
            meas = measured_angular_samples[i]
            if (abs(cmd) >= self.min_angular_for_track_cal
                    and abs(meas) > 1e-6):
                ratios.append(cmd / meas)

        if not ratios:
            return self.calibration.track_width_correction

        # 중앙값 사용 (이상치에 강건)
        ratios.sort()
        median_ratio = ratios[len(ratios) // 2]

        old_correction = self.calibration.track_width_correction
        new_correction = old_correction * median_ratio

        # 클램프
        new_correction = max(
            self.correction_factor_min,
            min(new_correction, self.correction_factor_max),
        )

        self.calibration.track_width_correction = new_correction
        self.calibration.calibration_count += 1
        self.calibration.track_width_samples += n

        return new_correction

    # ------------------------------------------------------------------ #
    #  캘리브레이션 데이터 저장/로드
    # ------------------------------------------------------------------ #

    def save_calibration(self, file_path: Optional[str] = None) -> bool:
        """캘리브레이션 데이터를 JSON 파일로 저장한다.

        Args:
            file_path: 저장 파일 경로. None이면 생성 시 지정한 경로를 사용.

        Returns:
            저장 성공 여부.
        """
        path = file_path or self._calibration_file
        if not path:
            return False

        try:
            # 디렉토리가 없으면 생성
            dir_path = os.path.dirname(path)
            if dir_path:
                os.makedirs(dir_path, exist_ok=True)

            with open(path, 'w', encoding='utf-8') as f:
                json.dump(
                    self.calibration.to_dict(),
                    f,
                    indent=2,
                    ensure_ascii=False,
                )
            return True
        except (OSError, TypeError) as e:
            # 로깅은 호출측(ROS 노드)에서 처리
            return False

    def load_calibration(self, file_path: Optional[str] = None) -> bool:
        """캘리브레이션 데이터를 JSON 파일에서 로드한다.

        Args:
            file_path: 로드 파일 경로. None이면 생성 시 지정한 경로를 사용.

        Returns:
            로드 성공 여부.
        """
        return self._load_calibration(file_path)

    def _load_calibration(self, file_path: Optional[str] = None) -> bool:
        """내부 캘리브레이션 로드 메서드.

        Args:
            file_path: 로드 파일 경로.

        Returns:
            로드 성공 여부.
        """
        path = file_path or self._calibration_file
        if not path:
            return False

        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            self.calibration = CalibrationData.from_dict(data)
            return True
        except (OSError, json.JSONDecodeError, KeyError):
            # 파일이 없거나 형식이 잘못된 경우 기본값 유지
            return False

    def reset_calibration(self) -> None:
        """모든 캘리브레이션 보정 계수를 기본값으로 초기화한다."""
        self.calibration = CalibrationData()
        self._drift_accumulator = 0.0
        self._drift_sample_count = 0

    # ------------------------------------------------------------------ #
    #  진단 정보
    # ------------------------------------------------------------------ #

    def get_calibration_status(self) -> Dict:
        """현재 캘리브레이션 상태를 반환한다.

        Returns:
            캘리브레이션 상태 딕셔너리. 디버깅 및 모니터링에 사용.
        """
        return {
            'calibration_enabled': self.calibration_enabled,
            'effective_track_width': self.effective_track_width,
            'nominal_track_width': self.track_width,
            'track_width_correction': self.calibration.track_width_correction,
            'left_motor_gain': self.calibration.left_motor_gain,
            'right_motor_gain': self.calibration.right_motor_gain,
            'motor_gain_imbalance': abs(
                self.calibration.left_motor_gain
                - self.calibration.right_motor_gain
            ),
            'calibration_count': self.calibration.calibration_count,
            'track_width_samples': self.calibration.track_width_samples,
            'asymmetry_samples': self.calibration.asymmetry_samples,
            'pending_drift_samples': self._drift_sample_count,
        }
