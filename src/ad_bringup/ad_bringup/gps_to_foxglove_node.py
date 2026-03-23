"""Convert sensor_msgs/NavSatFix to foxglove_msgs/LocationFix for Foxglove Map panel."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

try:
    from foxglove_msgs.msg import LocationFix
    FOXGLOVE_AVAILABLE = True
except ImportError:
    FOXGLOVE_AVAILABLE = False


class GpsToFoxglove(Node):

    def __init__(self):
        super().__init__('gps_to_foxglove')

        if not FOXGLOVE_AVAILABLE:
            self.get_logger().warn(
                'foxglove_msgs not installed. '
                'Install with: sudo apt install ros-${ROS_DISTRO}-foxglove-msgs'
            )
            return

        self.pub_ = self.create_publisher(LocationFix, '/foxglove/location', 10)
        self.sub_ = self.create_subscription(
            NavSatFix, '/sensor/gps', self.gps_callback, 10
        )
        self.get_logger().info('GPS to Foxglove converter started')

    def gps_callback(self, msg: NavSatFix):
        out = LocationFix()
        out.latitude = msg.latitude
        out.longitude = msg.longitude
        out.altitude = msg.altitude
        self.pub_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = GpsToFoxglove()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
