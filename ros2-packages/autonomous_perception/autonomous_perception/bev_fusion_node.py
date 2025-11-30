import json
from pathlib import Path
from typing import Dict, List

import numpy as np
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import Detection3DArray

from .engines import build_engine
from .profiling import ResourceMonitor
from .utils import (image_msg_to_numpy, pointcloud2_to_numpy,
                    make_detection_array_msg, make_occupancy_grid)


class DebugRecorder:
    """Utility that conditionally saves intermediate tensors and metadata to disk."""

    def __init__(self, enabled: bool, output_dir: str, flush_every: int = 20):
        self._enabled = enabled
        self._output_dir = Path(output_dir)
        self._flush_every = max(1, flush_every)
        self._buffer: List[Dict] = []
        if self._enabled:
            self._output_dir.mkdir(parents=True, exist_ok=True)

    def record(self, payload: Dict):
        if not self._enabled:
            return
        self._buffer.append(payload)
        if len(self._buffer) >= self._flush_every:
            self.flush()

    def flush(self):
        if not self._enabled or not self._buffer:
            return
        target = self._output_dir / f"dump_{rclpy.clock.Clock().now().nanoseconds}.json"
        with target.open('w', encoding='utf-8') as fp:
            json.dump(self._buffer, fp, indent=2)
        self._buffer.clear()


class BEVFusionNode(Node):
    def __init__(self):
        super().__init__('bev_fusion_node')
        self.bridge = CvBridge()
        self._declare_parameters()
        self._load_calibration()

        self.engine = build_engine(
            engine_type=self.get_parameter('engine_type').get_parameter_value().string_value,
            model_path=self.get_parameter('model_path').get_parameter_value().string_value,
            device=self.get_parameter('device').get_parameter_value().string_value,
            logger=self.get_logger(),
        )

        self.monitor = ResourceMonitor(
            enable_gpu=self.get_parameter('monitor.enable_gpu').get_parameter_value().bool_value,
            gpu_id=self.get_parameter('monitor.gpu_id').get_parameter_value().integer_value,
        )

        self.debug_recorder = DebugRecorder(
            enabled=self.get_parameter('debug_dump.enable').get_parameter_value().bool_value,
            output_dir=self.get_parameter('debug_dump.output_dir').get_parameter_value().string_value,
            flush_every=self.get_parameter('debug_dump.flush_every').get_parameter_value().integer_value,
        )

        self.camera_topics = self.get_parameter('camera_topics').get_parameter_value().string_array_value
        self.lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        self.sync_slop = self.get_parameter('sync_slop').get_parameter_value().double_value

        self._init_publishers()
        self._init_subscribers()
        self.get_logger().info('BEV Fusion node initialized.')

    # ----------------------- Parameter helpers ---------------------------------
    def _declare_parameters(self):
        self.declare_parameter('camera_topics', ['/vehicle/camera/front/image_color'])
        self.declare_parameter('lidar_topic', '/vehicle/lidar_points')
        self.declare_parameter('engine_type', 'mock')
        self.declare_parameter('model_path', '')
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('calibration_path', '')
        self.declare_parameter('sync_slop', 0.05)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('occupancy_resolution', 0.5)
        self.declare_parameter('occupancy_size', [120, 120])
        self.declare_parameter('occupancy_frame', 'map')
        self.declare_parameter('diagnostics_period', 1.0)
        self.declare_parameter('monitor.enable_gpu', True)
        self.declare_parameter('monitor.gpu_id', 0)
        self.declare_parameter('debug_dump.enable', False)
        self.declare_parameter('debug_dump.output_dir', str(Path.home() / '.ros' / 'autonomous_perception' / 'debug'))
        self.declare_parameter('debug_dump.flush_every', 4)

    def _load_calibration(self):
        calib_path = self.get_parameter('calibration_path').get_parameter_value().string_value
        if calib_path and Path(calib_path).exists():
            with open(calib_path, 'r', encoding='utf-8') as fp:
                import yaml
                self.calibration = yaml.safe_load(fp)
            self.get_logger().info(f'Loaded calibration from {calib_path}')
        else:
            self.calibration = {}
            if calib_path:
                self.get_logger().warn(f'Calibration file {calib_path} not found. Proceeding with defaults.')

    # ------------------------- ROS setup ---------------------------------------
    def _init_publishers(self):
        self.detections_pub = self.create_publisher(Detection3DArray, '/perception/detections_3d', 10)
        self.occupancy_pub = self.create_publisher(OccupancyGrid, '/perception/occupancy', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/perception/diagnostics', 10)

    def _init_subscribers(self):
        qos = QoSProfile(depth=5)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        self.image_subs: List[Subscriber] = []
        for topic in self.camera_topics:
            self.get_logger().info(f'Subscribing to camera topic {topic}')
            self.image_subs.append(Subscriber(self, Image, topic, qos_profile=qos))

        lidar_qos = QoSProfile(depth=10)
        lidar_qos.reliability = ReliabilityPolicy.RELIABLE
        lidar_qos.history = HistoryPolicy.KEEP_LAST
        self.lidar_sub = Subscriber(self, PointCloud2, self.lidar_topic, qos_profile=lidar_qos)

        self.sync = ApproximateTimeSynchronizer(
            self.image_subs + [self.lidar_sub],
            queue_size=10,
            slop=self.sync_slop,
            allow_headerless=False
        )
        self.sync.registerCallback(self._synced_callback)

        diag_period = self.get_parameter('diagnostics_period').get_parameter_value().double_value
        self.create_timer(diag_period, self._publish_diagnostics)

    # ------------------------ Callback pipeline --------------------------------
    def _synced_callback(self, *msgs):
        *image_msgs, lidar_msg = msgs
        images = [image_msg_to_numpy(msg, self.bridge) for msg in image_msgs]
        lidar = pointcloud2_to_numpy(lidar_msg)

        inference_inputs = {
            'images': images,
            'lidar': lidar,
            'intrinsics': self.calibration.get('intrinsics', {}),
            'extrinsics': self.calibration.get('extrinsics', {}),
        }

        engine_outputs = self.engine.infer(inference_inputs)
        detection_msg = make_detection_array_msg(engine_outputs['detections'], frame_id=self.lidar_topic)
        occupancy_msg = make_occupancy_grid(
            engine_outputs['occupancy'],
            resolution=self.get_parameter('occupancy_resolution').get_parameter_value().double_value,
            frame_id=self.get_parameter('occupancy_frame').get_parameter_value().string_value,
        )

        stamp = self.get_clock().now().to_msg()
        detection_msg.header.stamp = stamp
        occupancy_msg.header.stamp = stamp

        self.detections_pub.publish(detection_msg)
        self.occupancy_pub.publish(occupancy_msg)

        self.debug_recorder.record({
            'header': {'sec': stamp.sec, 'nanosec': stamp.nanosec},
            'detections': engine_outputs['detections'],
            'occupancy_shape': np.shape(engine_outputs['occupancy']).tolist() if isinstance(engine_outputs['occupancy'], np.ndarray) else None,
        })

    def _publish_diagnostics(self):
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 'autonomous_perception/bev_fusion'
        status.level = DiagnosticStatus.OK
        status.message = 'OK'

        cpu, ram = self.monitor.sample_cpu_ram()
        status.values.append(KeyValue(key='cpu_percent', value=f'{cpu:.2f}'))
        status.values.append(KeyValue(key='ram_mb', value=f'{ram:.2f}'))
        if self.monitor.enable_gpu:
            gpu_stats = self.monitor.sample_gpu()
            for gpu_stat in gpu_stats:
                status.values.append(KeyValue(key=gpu_stat['key'], value=gpu_stat['value']))

        diagnostics.status.append(status)
        self.diag_pub.publish(diagnostics)
        self.debug_recorder.flush()


def main(args=None):
    rclpy.init(args=args)
    node = BEVFusionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
