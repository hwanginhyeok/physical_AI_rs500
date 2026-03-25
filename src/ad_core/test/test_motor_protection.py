"""MotorProtection 단위 테스트."""

import pytest

from ad_core.motor_protection import (
    MotorAlarm,
    MotorProtection,
    ProtectionConfig,
)


class TestMotorAlarm:
    def test_no_alarm(self):
        assert MotorAlarm.NONE.value == 0

    def test_alarm_combination(self):
        combined = MotorAlarm.STALL | MotorAlarm.OVER_TEMP
        assert MotorAlarm.STALL in combined
        assert MotorAlarm.OVER_TEMP in combined
        assert MotorAlarm.OVER_CURRENT not in combined


class TestMotorProtection:
    @pytest.fixture
    def prot(self):
        return MotorProtection(ProtectionConfig(
            rated_current=78.0,
            max_current=100.0,
            soft_fuse_time=2.0,
            over_temp_threshold=65.0,
            ambient_temp=25.0,
            stall_threshold=0.15,
        ))

    def test_normal_operation(self, prot):
        """정상 동작 시 알람 없음."""
        alarm = prot.update(
            command_speed=1.0, actual_speed=0.95,
            current=50.0, voltage=48.0, dt=0.01,
        )
        assert alarm == MotorAlarm.NONE
        assert not prot.is_fault

    def test_stall_alarm(self, prot):
        """STALL: 속도 오차 > 15%."""
        alarm = prot.update(
            command_speed=1.0, actual_speed=0.5,  # 50% 오차
            current=50.0, voltage=48.0, dt=0.01,
        )
        assert MotorAlarm.STALL in alarm

    def test_stall_no_alarm_within_threshold(self, prot):
        """STALL: 오차 <= 15%이면 알람 없음."""
        alarm = prot.update(
            command_speed=1.0, actual_speed=0.9,  # 10% 오차
            current=50.0, voltage=48.0, dt=0.01,
        )
        assert MotorAlarm.STALL not in alarm

    def test_over_current_soft_fuse(self, prot):
        """과전류: 정격 초과 2초 이상 → 폴트."""
        # 정격 초과 상태로 2초 이상 유지
        for _ in range(250):  # 250 * 0.01 = 2.5초
            prot.update(
                command_speed=1.0, actual_speed=1.0,
                current=90.0, voltage=48.0, dt=0.01,
            )
        assert prot.is_fault
        assert MotorAlarm.OVER_CURRENT in prot.alarm

    def test_over_current_brief_ok(self, prot):
        """과전류: 짧은 과도 전류는 폴트 안 됨."""
        for _ in range(50):  # 50 * 0.01 = 0.5초 < 2초
            prot.update(
                command_speed=1.0, actual_speed=1.0,
                current=90.0, voltage=48.0, dt=0.01,
            )
        assert not prot.is_fault

    def test_over_temp_fault(self, prot):
        """과온도: 65°C 도달 시 폴트."""
        # 고전류로 온도 상승 시뮬레이션
        for _ in range(50000):
            prot.update(
                command_speed=1.0, actual_speed=1.0,
                current=95.0, voltage=48.0, dt=0.01,
            )
            if prot.is_fault and MotorAlarm.OVER_TEMP in prot.alarm:
                break
        assert MotorAlarm.OVER_TEMP in prot.alarm

    def test_over_voltage(self, prot):
        """과전압: 정격의 110% 초과."""
        alarm = prot.update(
            command_speed=1.0, actual_speed=1.0,
            current=50.0, voltage=60.0, dt=0.01,  # 48 * 1.1 = 52.8 초과
        )
        assert MotorAlarm.OVER_VOLTAGE in alarm

    def test_under_voltage(self, prot):
        """저전압: 정격의 90% 미만."""
        alarm = prot.update(
            command_speed=1.0, actual_speed=1.0,
            current=50.0, voltage=40.0, dt=0.01,  # 48 * 0.9 = 43.2 미만
        )
        assert MotorAlarm.UNDER_VOLTAGE in alarm

    def test_inv_vel(self, prot):
        """방향 불일치: 명령 양, 실제 음."""
        alarm = prot.update(
            command_speed=1.0, actual_speed=-0.5,
            current=50.0, voltage=48.0, dt=0.01,
        )
        assert MotorAlarm.INV_VEL in alarm

    def test_reset(self, prot):
        """리셋 후 정상 상태."""
        # 먼저 폴트 상태로
        for _ in range(300):
            prot.update(
                command_speed=1.0, actual_speed=1.0,
                current=90.0, voltage=48.0, dt=0.01,
            )
        prot.reset()
        assert not prot.is_fault
        assert prot.alarm == MotorAlarm.NONE
        assert prot.temperature == prot.config.ambient_temp

    def test_clear_fault_when_cooled(self, prot):
        """온도 정상 + 전류 정상이면 폴트 해제 가능."""
        # 폴트 유발 (과전류)
        for _ in range(300):
            prot.update(
                command_speed=1.0, actual_speed=1.0,
                current=90.0, voltage=48.0, dt=0.01,
            )
        assert prot.is_fault

        # 냉각 + 전류 정상화
        prot.reset()
        prot._is_fault = True  # 폴트만 유지, 온도 리셋됨
        prot._over_current_timer = 0.0
        prot.clear_fault()
        assert not prot.is_fault

    def test_cooling_reduces_temperature(self, prot):
        """전류 0이면 온도가 주변 온도로 냉각."""
        prot._temperature = 50.0
        for _ in range(30000):
            prot.update(
                command_speed=0.0, actual_speed=0.0,
                current=0.0, voltage=48.0, dt=0.01,
            )
        # 300초(5분) 냉각 → 주변 온도에 가까워야 함 (tau=60s, 5tau=300s)
        assert prot.temperature < 27.0

    def test_zero_command_no_stall(self, prot):
        """명령 0이면 STALL 검사 안 함."""
        alarm = prot.update(
            command_speed=0.0, actual_speed=0.0,
            current=50.0, voltage=48.0, dt=0.01,
        )
        assert MotorAlarm.STALL not in alarm

    def test_normal_voltage_no_alarm(self, prot):
        """정상 전압 범위 내에서 알람 없음."""
        alarm = prot.update(
            command_speed=1.0, actual_speed=1.0,
            current=50.0, voltage=48.0, dt=0.01,
        )
        assert MotorAlarm.OVER_VOLTAGE not in alarm
        assert MotorAlarm.UNDER_VOLTAGE not in alarm
