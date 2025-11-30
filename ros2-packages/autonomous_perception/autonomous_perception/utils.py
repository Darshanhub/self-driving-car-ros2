from typing import Any, Dict, Iterable, List

import numpy as np
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose, BoundingBox3D


def image_msg_to_numpy(msg: Image, bridge) -> np.ndarray:
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
    return cv_image


def pointcloud2_to_numpy(msg: PointCloud2) -> np.ndarray:
    points = []
    for p in point_cloud2.read_points(msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True):
        points.append(p)
    if not points:
        return np.zeros((0, 4), dtype=np.float32)
    return np.asarray(points, dtype=np.float32)


def make_detection_array_msg(detections: List[Dict[str, Any]], frame_id: str) -> Detection3DArray:
    array = Detection3DArray()
    array.header.frame_id = frame_id
    for det in detections:
        detection = Detection3D()
        detection.header.frame_id = frame_id
        box = BoundingBox3D()
        box.center.position.x = det['center'][0]
        box.center.position.y = det['center'][1]
        box.center.position.z = det['center'][2]
        box.size.x, box.size.y, box.size.z = det['size']
        box.center.orientation.w = 1.0
        detection.bbox = box
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.id = int(det.get('class_id', 0))
        hypothesis.score = float(det.get('score', 0.0))
        hypothesis.pose.pose = Pose()
        hypothesis.pose.pose.position.x = det['center'][0]
        hypothesis.pose.pose.position.y = det['center'][1]
        hypothesis.pose.pose.position.z = det['center'][2]
        detection.results.append(hypothesis)
        array.detections.append(detection)
    return array


def make_occupancy_grid(occupancy: np.ndarray, resolution: float, frame_id: str) -> OccupancyGrid:
    grid = OccupancyGrid()
    grid.header.frame_id = frame_id
    height, width = occupancy.shape[:2]
    grid.info.resolution = resolution
    grid.info.width = width
    grid.info.height = height
    grid.info.origin.position.x = - (width * resolution) / 2.0
    grid.info.origin.position.y = - (height * resolution) / 2.0
    grid.data = (occupancy.flatten() * 100).astype(np.int8).tolist()
    return grid
