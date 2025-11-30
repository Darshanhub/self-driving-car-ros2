import time
from typing import Any, Dict

import numpy as np

from .base import Engine


class MockEngine(Engine):
    """Fallback engine used before real BEV/TensorRT weights are wired in."""

    def _load_model(self):
        self.logger.info('MockEngine initialized; no weights loaded.')

    def infer(self, inputs: Dict[str, Any]) -> Dict[str, Any]:
        start = time.time()
        images = inputs.get('images', [])
        lidar = inputs.get('lidar')
        detections = []
        if lidar is not None and len(lidar) > 0:
            center = np.mean(lidar[:, :2], axis=0)
            detections.append({
                'center': [float(center[0]), float(center[1]), 0.0],
                'size': [2.0, 4.0, 1.5],
                'yaw': 0.0,
                'class_id': 1,
                'score': 0.1,
            })
        occupancy = np.zeros((120, 120), dtype=np.float32)
        latency = (time.time() - start) * 1000.0
        self.logger.debug(f'Mock inference latency: {latency:.2f} ms for {len(images)} images')
        return {
            'detections': detections,
            'occupancy': occupancy,
            'latency_ms': latency,
        }
