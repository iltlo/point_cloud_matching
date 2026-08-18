"""Point cloud preprocessing utilities."""

import numpy as np
import open3d as o3d
from dataclasses import dataclass
from typing import Optional


@dataclass
class PreprocessParams:
    voxel_resolution: float = 0.2
    max_range: float = 80.0
    min_range: float = 0.5


class Preprocessing:
    def __init__(self, params: PreprocessParams):
        self.params = params

    def load_cloud(self, path: str) -> Optional[o3d.geometry.PointCloud]:
        """Load a PCD/PLY file. Returns None on failure."""
        try:
            cloud = o3d.io.read_point_cloud(path)
            if cloud.is_empty():
                return None
            return cloud
        except Exception:
            return None

    def range_filter(self, cloud: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """Remove points too close or too far from centroid."""
        points = np.asarray(cloud.points)
        centroid = points.mean(axis=0)
        distances = np.linalg.norm(points - centroid, axis=1)
        mask = (distances >= self.params.min_range) & (distances <= self.params.max_range)
        return cloud.select_by_index(np.where(mask)[0])

    def voxel_downsample(self, cloud: o3d.geometry.PointCloud,
                         resolution: Optional[float] = None) -> o3d.geometry.PointCloud:
        """Voxel grid downsample."""
        res = resolution if resolution else self.params.voxel_resolution
        return cloud.voxel_down_sample(voxel_size=res)

    def process(self, cloud: o3d.geometry.PointCloud,
                voxel_resolution: Optional[float] = None) -> o3d.geometry.PointCloud:
        """Full pipeline: range filter + voxel downsample."""
        cloud = self.range_filter(cloud)
        cloud = self.voxel_downsample(cloud, voxel_resolution)
        return cloud