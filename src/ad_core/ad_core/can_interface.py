"""차량 통신 인터페이스 추상 클래스.

CAN 통신을 추상화하여 다양한 백엔드(실차 CAN, ROS2 토픽, 시뮬레이션)를
동일한 인터페이스로 사용할 수 있게 한다.

구현체:
    - adtpc: CanVehicleInterface (python-can으로 실제 CAN 통신)
    - ROS2: RosVehicleInterface (ROS2 토픽 변환)
"""

from abc import ABC, abstractmethod
from typing import Optional

from ad_core.datatypes import VehicleCommand, VehicleFeedback


class VehicleInterface(ABC):
    """차량 통신 인터페이스 ABC.

    send_command()로 제어 명령을 전송하고,
    get_feedback()로 차량 상태를 수신한다.
    """

    @abstractmethod
    def send_command(self, cmd: VehicleCommand) -> bool:
        """차량에 제어 명령을 전송한다.

        Args:
            cmd: 전송할 제어 명령.

        Returns:
            전송 성공 여부.
        """
        ...

    @abstractmethod
    def get_feedback(self) -> Optional[VehicleFeedback]:
        """차량으로부터 상태 피드백을 수신한다.

        Returns:
            수신된 피드백. 수신 실패 시 None.
        """
        ...

    @abstractmethod
    def connect(self) -> bool:
        """차량과의 연결을 시작한다.

        Returns:
            연결 성공 여부.
        """
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """차량과의 연결을 종료한다."""
        ...

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """현재 연결 상태를 반환한다."""
        ...
