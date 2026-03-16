"""Mono Depth ROS2 Node (C66).

실물 카메라 RGB → Depth 추정 → PointCloud2 변환.
시뮬레이션에서는 Gazebo rgbd_camera가 직접 PointCloud2를 발행하므로 불필요.
실차 배포 시에만 활성화.

지원 모델:
- MiDaS v3.1 (Small/DPT-Large)
- Depth Anything v2 (Small/Base)

토픽 구조:
  /sensor/camera/{name}/image (입력) → /sensor/camera/{name}/points (출력)
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from std_msgs.msg import Header

from typing import Optional
import struct


class MonoDepthNode(Node):
    """단안 카메라 RGB → Depth → PointCloud2 변환 노드.

    실차 전용. 시뮬레이션에서는 Gazebo rgbd_camera가 대체.
    """

    CAMERA_NAMES = ['front', 'left', 'right']

    def __init__(self):
        super().__init__('mono_depth_node')

        # 파라미터
        self.declare_parameter('model_type', 'midas_small')  # midas_small, midas_large, depth_anything_small, depth_anything_base
        self.declare_parameter('device', '')  # auto
        self.declare_parameter('max_depth', 10.0)  # meters
        self.declare_parameter('min_depth', 0.3)
        self.declare_parameter('inference_rate', 10.0)  # Hz
        self.declare_parameter('camera_topic_prefix', '/sensor/camera')
        self.declare_parameter('publish_depth_image', True)

        model_type = self.get_parameter('model_type').value
        self.max_depth = self.get_parameter('max_depth').value
        self.min_depth = self.get_parameter('min_depth').value
        self.inference_rate = self.get_parameter('inference_rate').value
        camera_prefix = self.get_parameter('camera_topic_prefix').value
        self.publish_depth_image = self.get_parameter('publish_depth_image').value

        self.get_logger().info(f'MonoDepth initializing: model={model_type}')

        # Depth 모델 로드
        self._model = None
        self._transform = None
        self._load_model(model_type)

        # cv_bridge
        from cv_bridge import CvBridge
        self._bridge = CvBridge()

        # QoS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 카메라별 구독/발행
        self._camera_info = {}  # name → CameraInfo
        self._last_inference = {}
        self._processing = {}

        for cam in self.CAMERA_NAMES:
            # 이미지 구독
            self.create_subscription(
                Image,
                f'{camera_prefix}/{cam}/image',
                lambda msg, name=cam: self._image_callback(msg, name),
                qos,
            )
            # CameraInfo 구독 (한 번만)
            self.create_subscription(
                CameraInfo,
                f'{camera_prefix}/{cam}/camera_info',
                lambda msg, name=cam: self._camera_info_callback(msg, name),
                qos,
            )
            # PointCloud2 발행
            setattr(self, f'_pc_pub_{cam}', self.create_publisher(
                PointCloud2,
                f'{camera_prefix}/{cam}/points',
                10,
            ))
            # Depth 이미지 발행 (디버그)
            if self.publish_depth_image:
                setattr(self, f'_depth_pub_{cam}', self.create_publisher(
                    Image,
                    f'{camera_prefix}/{cam}/depth',
                    10,
                ))

            self._last_inference[cam] = self.get_clock().now()
            self._processing[cam] = False
            self.get_logger().info(f'  Camera: {cam}')

        self.get_logger().info('MonoDepth initialized')

    def _load_model(self, model_type: str):
        """Depth 추정 모델 로드."""
        device_param = self.get_parameter('device').value

        try:
            import torch
            device = device_param if device_param else ('cuda' if torch.cuda.is_available() else 'cpu')

            if model_type.startswith('midas'):
                self._load_midas(model_type, device)
            elif model_type.startswith('depth_anything'):
                self._load_depth_anything(model_type, device)
            else:
                self.get_logger().error(f'Unknown model: {model_type}')
                return

            self._device = device
            self.get_logger().info(f'  Model loaded on {device}')

        except ImportError as e:
            self.get_logger().warn(
                f'Depth model unavailable ({e}). '
                'Install: pip install timm torch torchvision'
            )

    def _load_midas(self, model_type: str, device: str):
        """MiDaS 모델 로드."""
        import torch

        variant = 'MiDaS_small' if 'small' in model_type else 'DPT_Large'
        self._model = torch.hub.load('intel-isl/MiDaS', variant, trust_repo=True)
        self._model.to(device).eval()

        transforms = torch.hub.load('intel-isl/MiDaS', 'transforms', trust_repo=True)
        self._transform = transforms.small_transform if 'small' in model_type else transforms.dpt_transform

    def _load_depth_anything(self, model_type: str, device: str):
        """Depth Anything v2 모델 로드."""
        import torch

        variant = 'small' if 'small' in model_type else 'base'
        # Depth Anything v2는 torch.hub으로 로드
        self._model = torch.hub.load(
            'LiheYoung/Depth-Anything',
            f'DepthAnything_{variant}',
            trust_repo=True,
        )
        self._model.to(device).eval()
        self._transform = None  # Depth Anything은 내부 전처리

    def _camera_info_callback(self, msg: CameraInfo, camera_name: str):
        """CameraInfo 수신 (한 번만)."""
        if camera_name not in self._camera_info:
            self._camera_info[camera_name] = msg
            self.get_logger().info(
                f'  CameraInfo received: {camera_name} '
                f'({msg.width}x{msg.height})'
            )

    def _image_callback(self, msg: Image, camera_name: str):
        """이미지 수신 → Depth 추정 → PointCloud2 발행."""
        if self._model is None:
            return

        # 속도 제한
        now = self.get_clock().now()
        elapsed = (now - self._last_inference[camera_name]).nanoseconds / 1e9
        if elapsed < (1.0 / self.inference_rate):
            return

        if self._processing[camera_name]:
            return

        self._processing[camera_name] = True
        self._last_inference[camera_name] = now

        try:
            # 이미지 변환
            cv_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

            # Depth 추정
            depth_map = self._estimate_depth(cv_image)

            # PointCloud2 생성 + 발행
            cam_info = self._camera_info.get(camera_name)
            if cam_info:
                pc_msg = self._depth_to_pointcloud(depth_map, cam_info, msg.header)
                pub = getattr(self, f'_pc_pub_{camera_name}')
                pub.publish(pc_msg)

            # Depth 이미지 발행 (디버그)
            if self.publish_depth_image:
                depth_vis = (depth_map / self.max_depth * 255).clip(0, 255).astype(np.uint8)
                depth_msg = self._bridge.cv2_to_imgmsg(depth_vis, encoding='mono8')
                depth_msg.header = msg.header
                depth_pub = getattr(self, f'_depth_pub_{camera_name}', None)
                if depth_pub:
                    depth_pub.publish(depth_msg)

        except Exception as e:
            self.get_logger().error(f'{camera_name} depth error: {e}')

        finally:
            self._processing[camera_name] = False

    def _estimate_depth(self, rgb_image: np.ndarray) -> np.ndarray:
        """RGB 이미지 → depth map (meters)."""
        import torch

        if self._transform:
            # MiDaS
            input_batch = self._transform(rgb_image).to(self._device)
        else:
            # Depth Anything — 내부 전처리
            input_batch = torch.from_numpy(rgb_image).permute(2, 0, 1).unsqueeze(0)
            input_batch = input_batch.float().to(self._device) / 255.0

        with torch.no_grad():
            prediction = self._model(input_batch)

        # 리사이즈 + 정규화
        prediction = torch.nn.functional.interpolate(
            prediction.unsqueeze(1),
            size=rgb_image.shape[:2],
            mode='bilinear',
            align_corners=False,
        ).squeeze().cpu().numpy()

        # inverse depth → metric depth (근사)
        # MiDaS는 inverse depth를 출력하므로 변환 필요
        depth = prediction
        if depth.max() > 0:
            depth = depth / depth.max() * self.max_depth

        depth = np.clip(depth, self.min_depth, self.max_depth)
        return depth

    def _depth_to_pointcloud(
        self,
        depth_map: np.ndarray,
        camera_info: CameraInfo,
        header: Header,
    ) -> PointCloud2:
        """Depth map + CameraInfo → PointCloud2."""
        h, w = depth_map.shape
        fx = camera_info.k[0]
        fy = camera_info.k[4]
        cx = camera_info.k[2]
        cy = camera_info.k[5]

        # 다운샘플 (성능)
        step = 4
        points = []

        for v in range(0, h, step):
            for u in range(0, w, step):
                z = depth_map[v, u]
                if z < self.min_depth or z > self.max_depth:
                    continue
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy
                points.append(struct.pack('fff', x, y, z))

        # PointCloud2 메시지 구성
        pc_msg = PointCloud2()
        pc_msg.header = header
        pc_msg.height = 1
        pc_msg.width = len(points)
        pc_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        pc_msg.is_bigendian = False
        pc_msg.point_step = 12
        pc_msg.row_step = 12 * len(points)
        pc_msg.data = b''.join(points)
        pc_msg.is_dense = True

        return pc_msg


def main(args=None):
    rclpy.init(args=args)
    node = MonoDepthNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
