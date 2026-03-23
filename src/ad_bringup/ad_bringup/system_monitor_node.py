"""Monitor system health and publish diagnostics at 1 Hz."""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class SystemMonitor(Node):

    CAMERA_TOPICS = [
        '/sensor/camera/front/image',
        '/sensor/camera/left/image',
        '/sensor/camera/right/image',
    ]

    GPS_STATUS_NAMES = {
        -1: 'NO_FIX',
        0: 'FIX',
        1: 'SBAS_FIX',
        2: 'GBAS_FIX',
    }

    EKF_COVARIANCE_WARN_THRESHOLD = 10.0

    def __init__(self):
        super().__init__('system_monitor')

        # Camera FPS tracking
        self._cam_counts: dict[str, int] = {}
        self._cam_fps: dict[str, float] = {}
        for topic in self.CAMERA_TOPICS:
            self._cam_counts[topic] = 0
            self._cam_fps[topic] = 0.0
            self.create_subscription(Image, topic, self._make_cam_cb(topic), 10)

        # GPS state
        self._gps_status: int = -1
        self.create_subscription(NavSatFix, '/sensor/gps', self._gps_cb, 10)

        # EKF state
        self._ekf_cov_diag: list[float] = []
        self.create_subscription(Odometry, '/odometry/local', self._ekf_cb, 10)

        # Diagnostics publisher
        self._diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # 1 Hz timer for diagnostics + FPS window reset
        self._last_reset = time.monotonic()
        self.create_timer(1.0, self._timer_cb)

        self.get_logger().info('System monitor started')

    # -- Subscriber callbacks ------------------------------------------------

    def _make_cam_cb(self, topic: str):
        def cb(_msg: Image):
            self._cam_counts[topic] += 1
        return cb

    def _gps_cb(self, msg: NavSatFix):
        self._gps_status = msg.status.status

    def _ekf_cb(self, msg: Odometry):
        cov = msg.pose.covariance
        # Diagonal indices for 6x6 row-major: 0, 7, 14, 21, 28, 35
        self._ekf_cov_diag = [cov[i] for i in (0, 7, 14, 21, 28, 35)]

    # -- Timer callback ------------------------------------------------------

    def _timer_cb(self):
        now = time.monotonic()
        dt = now - self._last_reset
        if dt <= 0.0:
            dt = 1.0

        # Compute FPS and reset counters
        for topic in self.CAMERA_TOPICS:
            self._cam_fps[topic] = self._cam_counts[topic] / dt
            self._cam_counts[topic] = 0
        self._last_reset = now

        # Build diagnostics
        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()

        # Camera statuses
        for topic in self.CAMERA_TOPICS:
            fps = self._cam_fps[topic]
            name = topic.split('/')[-2]  # e.g. "front", "left", "right"
            level = DiagnosticStatus.OK if fps > 0.0 else DiagnosticStatus.WARN
            status = DiagnosticStatus()
            status.level = level
            status.name = f'Camera/{name}'
            status.message = f'{fps:.1f} Hz'
            status.values = [KeyValue(key='fps', value=f'{fps:.1f}')]
            diag.status.append(status)

        # GPS status
        gps_status = DiagnosticStatus()
        gps_name = self.GPS_STATUS_NAMES.get(self._gps_status, f'UNKNOWN({self._gps_status})')
        gps_status.level = (
            DiagnosticStatus.OK if self._gps_status >= 0 else DiagnosticStatus.WARN
        )
        gps_status.name = 'GPS'
        gps_status.message = gps_name
        gps_status.values = [
            KeyValue(key='status', value=str(self._gps_status)),
            KeyValue(key='status_name', value=gps_name),
        ]
        diag.status.append(gps_status)

        # EKF status
        ekf_status = DiagnosticStatus()
        ekf_status.name = 'EKF'
        if self._ekf_cov_diag:
            labels = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
            max_cov = max(self._ekf_cov_diag)
            ekf_status.level = (
                DiagnosticStatus.WARN
                if max_cov > self.EKF_COVARIANCE_WARN_THRESHOLD
                else DiagnosticStatus.OK
            )
            ekf_status.message = f'max_cov={max_cov:.4f}'
            for label, val in zip(labels, self._ekf_cov_diag):
                ekf_status.values.append(
                    KeyValue(key=f'cov_{label}', value=f'{val:.6f}')
                )
        else:
            ekf_status.level = DiagnosticStatus.STALE
            ekf_status.message = 'No odometry received'
        diag.status.append(ekf_status)

        self._diag_pub.publish(diag)


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
