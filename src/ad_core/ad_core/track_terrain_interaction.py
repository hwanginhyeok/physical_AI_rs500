"""지면-궤도 상호작용 모델 모듈.

논/밭 환경 특화 지면 모델. 간소화 Bekker 모델 기반의
침하, 견인력, 선회 저항을 힘 기반으로 계산한다.
"""

import math
from dataclasses import dataclass
from enum import Enum
from typing import Dict, Tuple


class TerrainType(Enum):
    """지면-궤도 상호작용 전용 지형 유형.

    terrain_classifier.TerrainType를 확장하여
    농업용 지형(논/밭)을 세분화한다.
    """

    PAVED = "paved"
    DIRT_ROAD = "dirt_road"
    GRASS = "grass"
    GRAVEL = "gravel"
    CROP_FIELD = "crop_field"
    MUD = "mud"
    PADDY_WET = "paddy_wet"      # 젖은 논
    PADDY_DRY = "paddy_dry"      # 마른 논
    FIELD_SOFT = "field_soft"    # 부드러운 밭
    FIELD_HARD = "field_hard"    # 단단한 밭


@dataclass
class TerrainConfig:
    """지형별 토양 역학 파라미터 (간소화 Bekker 모델).

    Attributes:
        cohesion: 토양 응집력 (kPa). Mohr-Coulomb 모델의 c.
        friction_angle: 내부 마찰각 (deg). Mohr-Coulomb 모델의 phi.
        sinkage_modulus_c: 응집 침하 계수 k_c (kN/m^(n+1)).
        sinkage_modulus_phi: 마찰 침하 계수 k_phi (kN/m^(n+2)).
        sinkage_exponent: 침하 지수 n.
        slip_at_max_traction: 최대 견인력 도달 시 슬립비.
        turning_resistance_coeff: 선회 저항 계수.
        rolling_resistance_coeff: 구름 저항 계수.
    """

    cohesion: float = 10.0              # kPa
    friction_angle: float = 30.0        # deg
    sinkage_modulus_c: float = 10.0     # kN/m^(n+1)
    sinkage_modulus_phi: float = 20.0   # kN/m^(n+2)
    sinkage_exponent: float = 0.8
    slip_at_max_traction: float = 0.15
    turning_resistance_coeff: float = 0.5
    rolling_resistance_coeff: float = 0.04


# 지형별 디폴트 파라미터
TERRAIN_PARAMS: Dict[TerrainType, TerrainConfig] = {
    TerrainType.PAVED: TerrainConfig(
        cohesion=0.0,
        friction_angle=40.0,
        sinkage_modulus_c=0.0,
        sinkage_modulus_phi=1000.0,
        sinkage_exponent=1.0,
        slip_at_max_traction=0.08,
        turning_resistance_coeff=0.3,
        rolling_resistance_coeff=0.02,
    ),
    TerrainType.DIRT_ROAD: TerrainConfig(
        cohesion=5.0,
        friction_angle=28.0,
        sinkage_modulus_c=8.0,
        sinkage_modulus_phi=15.0,
        sinkage_exponent=0.9,
        slip_at_max_traction=0.12,
        turning_resistance_coeff=0.4,
        rolling_resistance_coeff=0.03,
    ),
    TerrainType.GRASS: TerrainConfig(
        cohesion=8.0,
        friction_angle=25.0,
        sinkage_modulus_c=6.0,
        sinkage_modulus_phi=12.0,
        sinkage_exponent=0.85,
        slip_at_max_traction=0.15,
        turning_resistance_coeff=0.45,
        rolling_resistance_coeff=0.04,
    ),
    TerrainType.PADDY_WET: TerrainConfig(
        cohesion=2.0,
        friction_angle=15.0,
        sinkage_modulus_c=3.0,
        sinkage_modulus_phi=5.0,
        sinkage_exponent=0.7,
        slip_at_max_traction=0.25,
        turning_resistance_coeff=0.8,
        rolling_resistance_coeff=0.08,
    ),
    TerrainType.PADDY_DRY: TerrainConfig(
        cohesion=6.0,
        friction_angle=22.0,
        sinkage_modulus_c=5.0,
        sinkage_modulus_phi=10.0,
        sinkage_exponent=0.8,
        slip_at_max_traction=0.18,
        turning_resistance_coeff=0.55,
        rolling_resistance_coeff=0.05,
    ),
    TerrainType.FIELD_SOFT: TerrainConfig(
        cohesion=4.0,
        friction_angle=20.0,
        sinkage_modulus_c=4.0,
        sinkage_modulus_phi=8.0,
        sinkage_exponent=0.75,
        slip_at_max_traction=0.20,
        turning_resistance_coeff=0.65,
        rolling_resistance_coeff=0.06,
    ),
    TerrainType.FIELD_HARD: TerrainConfig(
        cohesion=10.0,
        friction_angle=30.0,
        sinkage_modulus_c=10.0,
        sinkage_modulus_phi=20.0,
        sinkage_exponent=0.9,
        slip_at_max_traction=0.12,
        turning_resistance_coeff=0.4,
        rolling_resistance_coeff=0.03,
    ),
    TerrainType.MUD: TerrainConfig(
        cohesion=1.0,
        friction_angle=10.0,
        sinkage_modulus_c=2.0,
        sinkage_modulus_phi=3.0,
        sinkage_exponent=0.6,
        slip_at_max_traction=0.30,
        turning_resistance_coeff=1.0,
        rolling_resistance_coeff=0.10,
    ),
    TerrainType.GRAVEL: TerrainConfig(
        cohesion=0.0,
        friction_angle=35.0,
        sinkage_modulus_c=0.0,
        sinkage_modulus_phi=50.0,
        sinkage_exponent=1.0,
        slip_at_max_traction=0.10,
        turning_resistance_coeff=0.35,
        rolling_resistance_coeff=0.03,
    ),
    TerrainType.CROP_FIELD: TerrainConfig(
        cohesion=7.0,
        friction_angle=25.0,
        sinkage_modulus_c=6.0,
        sinkage_modulus_phi=12.0,
        sinkage_exponent=0.85,
        slip_at_max_traction=0.18,
        turning_resistance_coeff=0.55,
        rolling_resistance_coeff=0.05,
    ),
}


def get_terrain_config(terrain: TerrainType) -> TerrainConfig:
    """지형 유형에 대응하는 토양 파라미터를 반환한다."""
    return TERRAIN_PARAMS.get(terrain, TerrainConfig())


class TrackTerrainInteraction:
    """궤도-지면 상호작용 모델.

    간소화 Bekker 모델 기반으로 침하, 견인력, 선회 저항을 계산한다.
    """

    def __init__(
        self,
        terrain: TerrainType = TerrainType.FIELD_HARD,
        config: TerrainConfig | None = None,
        track_width: float = 0.3,
        track_length: float = 1.2,
    ) -> None:
        """초기화.

        Args:
            terrain: 지형 유형.
            config: 사용자 정의 토양 파라미터. None이면 terrain 기반 디폴트.
            track_width: 궤도 폭 (m). 단일 궤도의 접지 폭.
            track_length: 궤도 접지 길이 (m).
        """
        self.terrain = terrain
        self.config = config or get_terrain_config(terrain)
        self.track_width = track_width
        self.track_length = track_length
        self._contact_area = track_width * track_length

    def set_terrain(
        self, terrain: TerrainType, config: TerrainConfig | None = None
    ) -> None:
        """지형을 변경한다."""
        self.terrain = terrain
        self.config = config or get_terrain_config(terrain)

    @property
    def contact_area(self) -> float:
        """단일 궤도 접지 면적 (m^2)."""
        return self._contact_area

    # ------------------------------------------------------------------ #
    #  침하 모델 (간소화 Bekker)
    # ------------------------------------------------------------------ #

    def compute_sinkage(self, normal_force: float) -> float:
        """수직력에 의한 궤도 침하량을 계산한다.

        간소화 Bekker 공식:
            p = F / A  (접지압)
            z = (p / (k_c/b + k_phi))^(1/n)

        Args:
            normal_force: 단일 궤도에 가해지는 수직력 (N).

        Returns:
            침하량 (m). 0 이상.
        """
        if normal_force <= 0 or self._contact_area < 1e-6:
            return 0.0

        cfg = self.config

        # 접지압 (kPa): F(N) / A(m^2) / 1000
        pressure_kpa = normal_force / self._contact_area / 1000.0

        # Bekker 침하 계수 합산
        b = max(self.track_width, 1e-6)
        k_combined = cfg.sinkage_modulus_c / b + cfg.sinkage_modulus_phi

        if k_combined < 1e-6:
            return 0.0

        n = max(cfg.sinkage_exponent, 0.1)

        # z = (p / k)^(1/n)
        ratio = pressure_kpa / k_combined
        if ratio <= 0:
            return 0.0

        sinkage = ratio ** (1.0 / n)

        # 현실적 상한 (궤도 높이 이상 침하 방지)
        return min(sinkage, 0.5)

    # ------------------------------------------------------------------ #
    #  견인력 모델 (Mohr-Coulomb)
    # ------------------------------------------------------------------ #

    def compute_traction(
        self, normal_force: float, slip_ratio: float
    ) -> float:
        """견인력을 계산한다.

        Mohr-Coulomb 기반:
            F_max = A * (c + p * tan(phi))
            F = F_max * (1 - exp(-slip / s_opt))

        Args:
            normal_force: 수직력 (N).
            slip_ratio: 슬립비 (0~1). 0=슬립 없음, 1=완전 공전.

        Returns:
            견인력 (N). 항상 0 이상.
        """
        if normal_force <= 0 or self._contact_area < 1e-6:
            return 0.0

        cfg = self.config

        # 접지압 (kPa)
        pressure_kpa = normal_force / self._contact_area / 1000.0

        # Mohr-Coulomb 최대 전단 응력 (kPa)
        phi_rad = math.radians(cfg.friction_angle)
        max_shear_kpa = cfg.cohesion + pressure_kpa * math.tan(phi_rad)

        # 최대 견인력 (N) = tau_max(kPa) * A(m^2) * 1000
        max_traction = max_shear_kpa * self._contact_area * 1000.0

        # 슬립 의존 견인력 (지수 감쇠 모델)
        s = abs(slip_ratio)
        s_opt = max(cfg.slip_at_max_traction, 1e-6)

        if s < 1e-6:
            return 0.0

        traction_ratio = 1.0 - math.exp(-s / s_opt)

        return max_traction * traction_ratio

    # ------------------------------------------------------------------ #
    #  선회 저항
    # ------------------------------------------------------------------ #

    def compute_turning_resistance(
        self, yaw_rate: float, normal_force: float
    ) -> float:
        """선회 저항 모멘트를 계산한다.

        스키드 스티어 차량은 선회 시 궤도의 측면 미끄러짐으로 인해
        상당한 저항 토크가 발생한다.

        Args:
            yaw_rate: 요 각속도 (rad/s).
            normal_force: 총 수직력 (N).

        Returns:
            선회 저항 모멘트 (Nm). 항상 0 이상.
        """
        if abs(yaw_rate) < 1e-6 or normal_force <= 0:
            return 0.0

        cfg = self.config

        # 선회 저항 모멘트 = mu_turn * N * L/2
        # L은 궤도 접지 길이, 1/2은 평균 팔 길이
        moment = (
            cfg.turning_resistance_coeff
            * normal_force
            * self.track_length / 2.0
        )

        # 각속도에 따른 점진적 증가 (tanh로 부드러운 전환)
        speed_factor = math.tanh(abs(yaw_rate) * 5.0)

        return moment * speed_factor

    # ------------------------------------------------------------------ #
    #  구름 저항
    # ------------------------------------------------------------------ #

    def compute_rolling_resistance(self, normal_force: float) -> float:
        """구름 저항력을 계산한다.

        Args:
            normal_force: 수직력 (N).

        Returns:
            구름 저항력 (N).
        """
        if normal_force <= 0:
            return 0.0
        return self.config.rolling_resistance_coeff * normal_force

    # ------------------------------------------------------------------ #
    #  경사면 힘 분해
    # ------------------------------------------------------------------ #

    @staticmethod
    def compute_slope_forces(
        mass: float, pitch: float, roll: float
    ) -> Tuple[float, float, float]:
        """경사면에서의 중력 분해 힘을 계산한다.

        Args:
            mass: 차량 질량 (kg).
            pitch: 전후 경사각 (rad). 양수 = 오르막.
            roll: 횡경사 (rad). 양수 = 우측 높음.

        Returns:
            (along_slope, normal, lateral):
                경사방향력(N), 수직항력(N), 횡력(N).
        """
        g = 9.81
        W = mass * g
        along = W * math.sin(pitch)
        normal = W * math.cos(pitch) * math.cos(roll)
        lateral = W * math.cos(pitch) * math.sin(roll)
        return along, normal, lateral

    # ------------------------------------------------------------------ #
    #  통합 계산
    # ------------------------------------------------------------------ #

    def compute_effective_force(
        self,
        normal_force: float,
        commanded_slip: float,
        yaw_rate: float,
    ) -> Tuple[float, float, float, float]:
        """통합 힘 계산: 견인력, 구름저항, 선회저항, 침하량.

        Args:
            normal_force: 궤도당 수직력 (N).
            commanded_slip: 명령 슬립비.
            yaw_rate: 요 각속도 (rad/s).

        Returns:
            (traction, rolling_resistance, turning_moment, sinkage).
        """
        traction = self.compute_traction(normal_force, commanded_slip)
        rolling = self.compute_rolling_resistance(normal_force)
        turning = self.compute_turning_resistance(yaw_rate, normal_force * 2)
        sinkage = self.compute_sinkage(normal_force)

        return traction, rolling, turning, sinkage
