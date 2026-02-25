"""지면-궤도 상호작용 모델 모듈.

논/밭 환경 특화 지면 모델. Bekker 침하 모델 + Mohr-Coulomb 전단 +
Janosi-Hanamoto 견인력 모델 기반으로 침하, 견인력, 선회 저항을 계산한다.

레퍼런스:
    [1] Bekker, M.G. (1969). Introduction to Terrain-Vehicle Systems.
        University of Michigan Press.
    [2] Wong, J.Y. (2008). Theory of Ground Vehicles, 4th Ed. Wiley.
        Table 2.4, 2.5 — Bekker 파라미터 표준값.
    [3] Janosi, Z. & Hanamoto, B. (1961). The analytical determination of
        drawbar pull as a function of slip. Proc. 1st ISTVS Conf., Turin.
    [4] Li et al. (2021). Estimation of Soil Shear Strength Indicators
        Using Physical Properties of Paddy Soils in the Plastic State.
        Applied Sciences, 11(12), 5609.
        — 포화 논토양: c=1.89~2.35 kPa, φ=10.2~11.4°
    [5] He et al. (2020). Determining the Shear Strength and Permeability
        of Soils for Engineering of New Paddy Field Construction.
        Applied Sciences, 10(5), 1782.
        — 함수비별 c, φ 변화 곡선 (7~25% MC).
    [6] Ren et al. (2020). Quantification of the soil stiffness constants
        using physical properties of paddy soils in Yangtze Delta Plain.
        Biosystems Engineering, 200, 89-100.
        — 논토양 Bekker 침하 계수 실측 (bevameter).
    [7] Gao et al. (2022). Analysis of Effect of Grouser Height on Tractive
        Performance of Tracked Vehicle under Different Moisture Contents
        in Paddy Soil. Agriculture, 12(10), 1581.
    [8] Ding et al. (2020). Theoretical Model for Prediction of Turning
        Resistance of Tracked Vehicle on Soft Terrain. Mathematical
        Problems in Engineering, 2020, 4247904.
        — 연약 지반 선회 시 슬립비 0.3~0.6.
"""

import math
from dataclasses import dataclass
from enum import Enum
from typing import Dict, Tuple


class TerrainType(Enum):
    """지면-궤도 상호작용 전용 지형 유형."""

    PAVED = "paved"
    DIRT_ROAD = "dirt_road"
    GRASS = "grass"
    GRAVEL = "gravel"
    CROP_FIELD = "crop_field"
    MUD = "mud"
    PADDY_WET = "paddy_wet"      # 젖은 논 (포화, MC>30%)
    PADDY_DRY = "paddy_dry"      # 마른 논 (MC 15~20%)
    FIELD_SOFT = "field_soft"    # 부드러운 밭 (점토질, MC 20~25%)
    FIELD_HARD = "field_hard"    # 단단한 밭 (사질양토, MC 10~15%)


@dataclass
class TerrainConfig:
    """지형별 토양 역학 파라미터.

    Bekker 침하 모델: p = (k_c/b + k_phi) * z^n
    Mohr-Coulomb 전단: tau = c + sigma * tan(phi)
    Janosi-Hanamoto 견인: F = F_max * (1 - exp(-j/K))

    Attributes:
        cohesion: 토양 응집력 c (kPa). [2] Table 2.5, [4], [5].
        friction_angle: 내부 마찰각 phi (deg). [2] Table 2.5, [4], [5].
        sinkage_modulus_c: 응집 침하 계수 k_c (kN/m^(n+1)). [2] Table 2.4.
        sinkage_modulus_phi: 마찰 침하 계수 k_phi (kN/m^(n+2)). [2] Table 2.4.
        sinkage_exponent: 침하 지수 n. [2] Table 2.4.
        shear_deformation_modulus: Janosi 전단 변형 계수 K (m). [2] Table 2.5.
        turning_resistance_coeff: 선회 저항 계수. [8].
        rolling_resistance_coeff: 구름 저항 계수.
    """

    cohesion: float = 4.14              # kPa — [2] clayey soil
    friction_angle: float = 13.0        # deg — [2] clayey soil
    sinkage_modulus_c: float = 13.19    # kN/m^(n+1) — [2] clayey soil
    sinkage_modulus_phi: float = 692.2  # kN/m^(n+2) — [2] clayey soil
    sinkage_exponent: float = 0.5       # — [2] clayey soil
    shear_deformation_modulus: float = 0.02  # m — [2][3]
    turning_resistance_coeff: float = 0.5
    rolling_resistance_coeff: float = 0.04


# ====================================================================== #
#  지형별 디폴트 파라미터 — 레퍼런스 기반
# ====================================================================== #
#
#  Bekker 파라미터 (k_c, k_phi, n):
#    Wong [2] Table 2.4 값을 기본으로 하되, 논토양은 Ren [6]과
#    clayey soil 값에서 수분 포화 방향으로 조정.
#
#  Mohr-Coulomb (c, φ):
#    건조 토양: He [5] 실측 (함수비 10~15% → c=50~80 kPa, φ=25~30°)
#    포화 논토양: Li [4] 실측 (c=1.89~2.35 kPa, φ=10.2~11.4°)
#
#  Janosi K:
#    Wong [2] Table 2.5 — sandy loam 0.025m, clayey 0.018m.
#
#  선회 저항:
#    Ding [8] — 연약 지반에서 선회 시 슬립비 0.3~0.6.
#    계수는 추정. 실측 데이터 부족.

TERRAIN_PARAMS: Dict[TerrainType, TerrainConfig] = {
    # 포장도로: 강체 표면. Bekker 모델 비적용 (침하=0).
    TerrainType.PAVED: TerrainConfig(
        cohesion=0.0,              # 비토양 — 마찰만 존재
        friction_angle=40.0,       # 고무-아스팔트 마찰 (추정)
        sinkage_modulus_c=0.0,     # 강체 → k_combined=0 → 침하=0
        sinkage_modulus_phi=0.0,   # 강체 → k_combined=0 → 침하=0
        sinkage_exponent=1.0,
        shear_deformation_modulus=0.008,  # 빠른 그립 형성
        turning_resistance_coeff=0.3,
        rolling_resistance_coeff=0.02,
    ),
    # 비포장 도로: Wong [2] dry sand 기반
    TerrainType.DIRT_ROAD: TerrainConfig(
        cohesion=1.04,             # [2] Table 2.5 dry sand
        friction_angle=28.0,       # [2] Table 2.5 dry sand
        sinkage_modulus_c=0.99,    # [2] Table 2.4 dry sand
        sinkage_modulus_phi=1528.4,# [2] Table 2.4 dry sand
        sinkage_exponent=1.1,      # [2] Table 2.4 dry sand
        shear_deformation_modulus=0.025,  # [2] Table 2.5
        turning_resistance_coeff=0.35,
        rolling_resistance_coeff=0.025,
    ),
    # 잔디/초지: sandy loam 근사
    TerrainType.GRASS: TerrainConfig(
        cohesion=1.72,             # [2] Table 2.5 sandy loam
        friction_angle=29.0,       # [2] Table 2.5 sandy loam
        sinkage_modulus_c=5.27,    # [2] Table 2.4 sandy loam
        sinkage_modulus_phi=1515.0,# [2] Table 2.4 sandy loam
        sinkage_exponent=0.7,      # [2] Table 2.4 sandy loam
        shear_deformation_modulus=0.025,  # [2] Table 2.5
        turning_resistance_coeff=0.4,
        rolling_resistance_coeff=0.035,
    ),
    # 젖은 논 (포화, MC>30%): Li [4] + Wong [2] clayey soil 기반
    # Li [4]: c=1.89~2.35 kPa, φ=10.2~11.4° (실측)
    # Bekker: clayey soil 값에서 k_phi를 수분 포화로 1/3 감소 (추정)
    TerrainType.PADDY_WET: TerrainConfig(
        cohesion=2.1,              # [4] Li et al. 실측 평균
        friction_angle=10.8,       # [4] Li et al. 실측 평균
        sinkage_modulus_c=13.19,   # [2] clayey soil (논토양과 유사)
        sinkage_modulus_phi=230.0, # [2] clayey 692 → 포화 시 ~1/3 (추정, [6] 참조)
        sinkage_exponent=0.4,      # [2] clayey 0.5에서 포화 감소 (추정)
        shear_deformation_modulus=0.03,  # 느린 그립 형성 (추정)
        turning_resistance_coeff=0.8,    # [8] 연약지반 높은 선회 저항
        rolling_resistance_coeff=0.08,
    ),
    # 마른 논 (MC 15~20%): [5] He et al. + [2] sandy loam 기반
    # He [5]: MC 15~20% → c≈30~50 kPa, φ≈20~25°
    TerrainType.PADDY_DRY: TerrainConfig(
        cohesion=6.0,              # [5] 범위 하한 근처 (논토양은 밭보다 낮음)
        friction_angle=22.0,       # [5] MC 20% 부근
        sinkage_modulus_c=10.0,    # [2] clayey~sandy loam 중간 (추정)
        sinkage_modulus_phi=500.0, # [2] clayey 기반, 건조화로 상승 (추정)
        sinkage_exponent=0.6,      # [2] clayey~sandy loam 중간
        shear_deformation_modulus=0.022,
        turning_resistance_coeff=0.5,
        rolling_resistance_coeff=0.05,
    ),
    # 부드러운 밭 (점토질, MC 20~25%): [2] clayey soil 근사
    TerrainType.FIELD_SOFT: TerrainConfig(
        cohesion=4.14,             # [2] Table 2.5 clayey soil
        friction_angle=13.0,       # [2] Table 2.5 clayey soil
        sinkage_modulus_c=13.19,   # [2] Table 2.4 clayey soil
        sinkage_modulus_phi=692.2, # [2] Table 2.4 clayey soil
        sinkage_exponent=0.5,      # [2] Table 2.4 clayey soil
        shear_deformation_modulus=0.018,  # [2] Table 2.5 clayey
        turning_resistance_coeff=0.6,
        rolling_resistance_coeff=0.06,
    ),
    # 단단한 밭 (사질양토, MC 10~15%): [2] sandy loam 근사
    # He [5]: MC 10~15% → c≈50~80 kPa, φ≈25~30°
    TerrainType.FIELD_HARD: TerrainConfig(
        cohesion=10.0,             # [5] 사질양토 MC 10~15% 범위
        friction_angle=29.0,       # [2][5] sandy loam
        sinkage_modulus_c=5.27,    # [2] Table 2.4 sandy loam
        sinkage_modulus_phi=1515.0,# [2] Table 2.4 sandy loam
        sinkage_exponent=0.7,      # [2] Table 2.4 sandy loam
        shear_deformation_modulus=0.025,  # [2] Table 2.5 sandy loam
        turning_resistance_coeff=0.35,
        rolling_resistance_coeff=0.03,
    ),
    # 진흙 (MUD): clayey soil에서 더 연약화
    TerrainType.MUD: TerrainConfig(
        cohesion=1.5,              # [4] 포화 점토 수준
        friction_angle=8.0,        # [4]보다 더 낮은 극한 상태 (추정)
        sinkage_modulus_c=13.19,   # [2] clayey soil
        sinkage_modulus_phi=150.0, # clayey 692 → 진흙 ~1/4.5 (추정)
        sinkage_exponent=0.3,      # 극저 (추정)
        shear_deformation_modulus=0.035,
        turning_resistance_coeff=1.0,    # [8]
        rolling_resistance_coeff=0.10,
    ),
    # 자갈: Wong [2] dry sand 변형 (무응집, 높은 마찰)
    TerrainType.GRAVEL: TerrainConfig(
        cohesion=0.0,
        friction_angle=35.0,       # 자갈 마찰각 33~40° (지반공학 표준)
        sinkage_modulus_c=0.0,
        sinkage_modulus_phi=5000.0,# 높은 지지력
        sinkage_exponent=1.0,
        shear_deformation_modulus=0.01,
        turning_resistance_coeff=0.3,
        rolling_resistance_coeff=0.025,
    ),
    # 작물 밭: sandy loam ~ clayey 중간
    TerrainType.CROP_FIELD: TerrainConfig(
        cohesion=7.0,              # [5] 밭토양 중간값
        friction_angle=25.0,       # [5] MC 15~20%
        sinkage_modulus_c=8.0,     # sandy loam~clayey 중간 (추정)
        sinkage_modulus_phi=1000.0,# sandy loam~clayey 중간 (추정)
        sinkage_exponent=0.6,
        shear_deformation_modulus=0.022,
        turning_resistance_coeff=0.45,
        rolling_resistance_coeff=0.04,
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
        """견인력을 계산한다 (Janosi-Hanamoto 모델 [3]).

        Mohr-Coulomb 최대 전단 + Janosi 슬립-견인 곡선:
            F_max = A * (c + p * tan(phi))
            j = slip_ratio * L  (전단 변위, L=접지 길이)
            F = F_max * (1 - exp(-j / K))  (K: 전단 변형 계수)

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

        # Janosi-Hanamoto: 전단 변위 j = slip * L (접지 길이)
        s = abs(slip_ratio)
        if s < 1e-6:
            return 0.0

        j = s * self.track_length  # 전단 변위 (m)
        K = max(cfg.shear_deformation_modulus, 1e-6)  # 전단 변형 계수 (m)

        traction_ratio = 1.0 - math.exp(-j / K)

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
