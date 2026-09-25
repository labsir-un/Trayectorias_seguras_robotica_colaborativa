#!/usr/bin/env python3

"""
PointCloud2 3D extraction utilities.

An image-space bounding box is converted to a 3D position by sampling
a small neighborhood around its center in an organized PointCloud2.

The RGB image and point cloud must be spatially aligned.
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


@dataclass
class PointCloud3DResult:
    """3D position obtained from a PointCloud2 region."""

    position: Optional[np.ndarray]
    valid_points: int
    confidence: float


class PointCloudObjectDetector:
    """Estimate an object's 3D position from its RGB bounding box."""

    def __init__(
        self,
        sample_half_size: int = 8,
        minimum_valid_points: int = 10,
        maximum_range: float = 5.0,
    ) -> None:

        self._sample_half_size = int(sample_half_size)
        self._minimum_valid_points = int(minimum_valid_points)
        self._maximum_range = float(maximum_range)

    def bbox_to_3d(
        self,
        cloud: PointCloud2,
        bbox: tuple[float, float, float, float],
    ) -> PointCloud3DResult:
        """Estimate XYZ from a bounding-box center neighborhood."""

        if cloud.width <= 0 or cloud.height <= 0:
            return PointCloud3DResult(None, 0, 0.0)

        x1, y1, x2, y2 = bbox

        center_x = int(round((x1 + x2) / 2.0))
        center_y = int(round((y1 + y2) / 2.0))

        center_x = int(np.clip(center_x, 0, cloud.width - 1))
        center_y = int(np.clip(center_y, 0, cloud.height - 1))

        half = self._sample_half_size

        u1 = max(0, center_x - half)
        u2 = min(cloud.width - 1, center_x + half)
        v1 = max(0, center_y - half)
        v2 = min(cloud.height - 1, center_y + half)

        try:
            points = point_cloud2.read_points_numpy(
                cloud,
                field_names=("x", "y", "z"),
                skip_nans=False,
            )
        except Exception:
            points = np.asarray(
                list(
                    point_cloud2.read_points(
                        cloud,
                        field_names=("x", "y", "z"),
                        skip_nans=False,
                    )
                ),
                dtype=np.float64,
            )

        points = np.asarray(points)

        if points.ndim == 3:
            region = points[v1:v2 + 1, u1:u2 + 1, :3]

        elif points.ndim == 2:
            if points.shape[0] != cloud.width * cloud.height:
                return PointCloud3DResult(None, 0, 0.0)

            organized = points.reshape(
                cloud.height,
                cloud.width,
                3,
            )
            region = organized[v1:v2 + 1, u1:u2 + 1, :3]

        else:
            return PointCloud3DResult(None, 0, 0.0)

        points_xyz = region.reshape(-1, 3).astype(np.float64)

        valid = np.isfinite(points_xyz).all(axis=1)
        points_xyz = points_xyz[valid]

        if points_xyz.size == 0:
            return PointCloud3DResult(None, 0, 0.0)

        distances = np.linalg.norm(points_xyz, axis=1)

        valid_distance = np.isfinite(distances)
        valid_distance &= distances > 0.05
        valid_distance &= distances <= self._maximum_range

        points_xyz = points_xyz[valid_distance]

        valid_count = int(points_xyz.shape[0])

        if valid_count < self._minimum_valid_points:
            return PointCloud3DResult(None, valid_count, 0.0)

        position = np.median(points_xyz, axis=0)

        expected_points = float((2 * half + 1) ** 2)
        confidence = float(
            np.clip(valid_count / expected_points, 0.0, 1.0)
        )

        return PointCloud3DResult(
            position=position,
            valid_points=valid_count,
            confidence=confidence,
        )
