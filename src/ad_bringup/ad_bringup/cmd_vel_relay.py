#!/usr/bin/env python3
"""cmd_vel Relay Node (C61-fix).

velocity_smoother의 cmd_vel_smoothed를 구독하여 cmd_vel로 publish.
collision_monitor 우회용.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        
        # Subscriber: velocity_smoother output
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel_smoothed',
            self.listener_callback,
            10
        )
        
        # Publisher: to Gazebo (bypass collision_monitor)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.get_logger().info('cmd_vel_relay started: cmd_vel_smoothed -> cmd_vel')

    def listener_callback(self, msg):
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
