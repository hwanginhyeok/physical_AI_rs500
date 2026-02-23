"""CAN ↔ ROS2 브릿지 노드.

SS500 CAN 버스 메시지를 ROS2 토픽으로 변환하고,
ROS2 제어 명령을 CAN 메시지로 변환하여 전송한다.

이 노드는 VehicleInterface ABC를 ROS2 토픽 기반으로 구현한다.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32

from ad_core.datatypes import VehicleCommand, VehicleFeedback
from ad_core.can_interface import VehicleInterface

from ad_can_bridge.ss500_codec import (
    CAN_ID_ADT2VCU1,
    CAN_ID_VCU2ADT1,
    CAN_ID_VCU2ADT2,
    ADT2VCU1,
    VCU2ADT1,
    VCU2ADT2,
    SS500Encoder,
    SS500Decoder,
)


class RosVehicleInterface(VehicleInterface):
    """ROS2 토픽 기반 VehicleInterface 구현체.

    CAN 통신 대신 ROS2 토픽을 사용하여 VehicleInterface를 구현.
    실차에서는 CanVehicleInterface(python-can)로 교체된다.
    """

    def __init__(self, node: Node):
        self._node = node
        self._connected = False
        self._last_feedback = None

        # 퍼블리셔: 차량 명령 (Twist로 발행)
        self._cmd_pub = node.create_publisher(
            Twist, '/vehicle/cmd_vel', 10)

        # 퍼블리셔: CAN 상태 정보
        self._status_pub = node.create_publisher(
            String, '/vehicle/status', 10)

    def send_command(self, cmd: VehicleCommand) -> bool:
        if not self._connected:
            return False

        twist = Twist()
        twist.linear.x = cmd.linear_velocity
        twist.angular.z = cmd.angular_velocity
        self._cmd_pub.publish(twist)
        return True

    def get_feedback(self):
        return self._last_feedback

    def connect(self) -> bool:
        self._connected = True
        self._node.get_logger().info('[CAN 브릿지] ROS 인터페이스 연결')
        return True

    def disconnect(self) -> None:
        self._connected = False
        self._node.get_logger().info('[CAN 브릿지] ROS 인터페이스 연결 해제')

    @property
    def is_connected(self) -> bool:
        return self._connected


class CANBridgeNode(Node):
    """CAN ↔ ROS2 브릿지 노드.

    SS500 CAN 프로토콜을 ROS2 토픽으로 변환한다.
    실차에서는 python-can을 통해 실제 CAN 버스에 연결되고,
    개발 환경에서는 ROS2 토픽으로 시뮬레이션된다.
    """

    def __init__(self):
        super().__init__('can_bridge_node')

        # CAN 코덱
        self._encoder = SS500Encoder()
        self._decoder = SS500Decoder()

        # Alive Counter (0~15 순환)
        self._alive_counter = 0

        # ROS2 인터페이스
        self._vehicle_interface = RosVehicleInterface(self)
        self._vehicle_interface.connect()

        # cmd_vel 구독 -> CAN 명령으로 변환
        self._cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10)

        # CAN 상태 퍼블리셔
        self._feedback_pub = self.create_publisher(
            String, '/can_bridge/feedback', 10)

        # 50ms 주기 타이머 (CAN 전송 주기)
        self._timer = self.create_timer(0.05, self._periodic_send)

        # 마지막 수신 명령
        self._last_cmd = Twist()

        self.get_logger().info('[CAN 브릿지] 노드 시작')

    def _cmd_vel_callback(self, msg: Twist) -> None:
        """cmd_vel 토픽 수신 콜백."""
        self._last_cmd = msg

    def _periodic_send(self) -> None:
        """주기적 CAN 메시지 전송 (50ms)."""
        # cmd_vel -> ADT2VCU1 인코딩
        adt_msg = ADT2VCU1(
            adt_valid=1,
            auto_ctrl_enable=1,
            left_speed_cmd_kmh=self._last_cmd.linear.x * 3.6,  # m/s -> km/h
            right_speed_cmd_kmh=self._last_cmd.linear.x * 3.6,
            alive_counter=self._alive_counter,
        )

        # Alive Counter 순환
        self._alive_counter = (self._alive_counter + 1) & 0x0F

        # 인코딩 (실차에서는 python-can으로 전송)
        can_data = self._encoder.encode_adt2vcu1(adt_msg)

        self.get_logger().debug(
            f'[CAN 브릿지] ADT2VCU1 인코딩: '
            f'L={adt_msg.left_speed_cmd_kmh:.2f} km/h, '
            f'R={adt_msg.right_speed_cmd_kmh:.2f} km/h, '
            f'alive={adt_msg.alive_counter}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = CANBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[CAN 브릿지] 노드 종료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
