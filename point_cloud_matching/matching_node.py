#!/usr/bin/env python3
"""ROS2 Service Node for Point Cloud Matching."""

import rclpy
from rclpy.node import Node
import numpy as np
import open3d as o3d

from geometry_msgs.msg import TransformStamped, Transform
from point_cloud_matching.srv import MatchClouds

from point_cloud_matching.preprocessing import Preprocessing, PreprocessParams
from point_cloud_matching.matching_engine import (
    MatchingEngine, CoarseParams, FineParams, QualityParams
)


class MatchingNode(Node):
    def __init__(self):
        super().__init__('matching_node')

        # Declare parameters
        self.declare_parameter('default_voxel_resolution', 0.05)
        self.declare_parameter('max_range', 80.0)
        self.declare_parameter('min_range', 0.5)

        self.declare_parameter('kiss_matcher.voxel_size', 0.3)

        self.declare_parameter('fine_matching.method', 'VGICP')
        self.declare_parameter('fine_matching.max_iterations', 100)
        self.declare_parameter('fine_matching.max_correspondence_distance', 1.0)
        self.declare_parameter('fine_matching.voxel_resolution', 1.0)
        self.declare_parameter('fine_matching.num_threads', 4)
        self.declare_parameter('fine_matching.num_neighbors', 20)

        self.declare_parameter('quality.min_fitness_score', 0.3)
        self.declare_parameter('quality.max_rmse', 0.5)

        # Read parameters
        preprocess_params = PreprocessParams(
            voxel_resolution=self.get_parameter('default_voxel_resolution').value,
            max_range=self.get_parameter('max_range').value,
            min_range=self.get_parameter('min_range').value,
        )

        coarse_params = CoarseParams(
            voxel_size=self.get_parameter('kiss_matcher.voxel_size').value,
        )

        fine_params = FineParams(
            method=self.get_parameter('fine_matching.method').value,
            max_iterations=self.get_parameter('fine_matching.max_iterations').value,
            max_correspondence_distance=self.get_parameter(
                'fine_matching.max_correspondence_distance').value,
            voxel_resolution=self.get_parameter('fine_matching.voxel_resolution').value,
            num_threads=self.get_parameter('fine_matching.num_threads').value,
            num_neighbors=self.get_parameter('fine_matching.num_neighbors').value,
        )

        quality_params = QualityParams(
            min_fitness_score=self.get_parameter('quality.min_fitness_score').value,
            max_rmse=self.get_parameter('quality.max_rmse').value,
        )

        # Initialize components
        self.preprocessing = Preprocessing(preprocess_params)
        self.engine = MatchingEngine(coarse_params, fine_params, quality_params, logger=self.get_logger())
        self.default_voxel_resolution = preprocess_params.voxel_resolution

        # Create service
        self.srv = self.create_service(
            MatchClouds,
            'match_clouds',
            self.handle_request
        )

        self.get_logger().info('Point Cloud Matching service ready.')

    def handle_request(self, request, response):
        """Handle incoming matching requests."""
        self.get_logger().info(
            f'Received request: ref={request.reference_path}, '
            f'src={request.target_path}'
        )

        # 1. Load point clouds
        ref_cloud = self.preprocessing.load_cloud(request.reference_path)
        src_cloud = self.preprocessing.load_cloud(request.target_path)

        if ref_cloud is None or src_cloud is None:
            response.success = False
            response.message = 'Failed to load one or both PCD files.'
            return response

        # 2. Preprocess
        voxel_res = (request.voxel_resolution
                     if request.voxel_resolution > 0.0
                     else self.default_voxel_resolution)

        ref_processed = self.preprocessing.process(ref_cloud, voxel_res)
        src_processed = self.preprocessing.process(src_cloud, voxel_res)

        ref_points = np.asarray(ref_processed.points)
        src_points = np.asarray(src_processed.points)

        self.get_logger().info(
            f'Preprocessed: ref={len(ref_points)} pts, src={len(src_points)} pts'
        )

        # Validate minimum point count
        if len(ref_points) < 100 or len(src_points) < 100:
            response.success = False
            response.message = (
                f'Insufficient points after preprocessing: '
                f'ref={len(ref_points)}, src={len(src_points)}'
            )
            return response

        # 3. Match
        if request.skip_coarse:
            initial_guess = self._transform_msg_to_matrix(request.initial_guess)
            result = self.engine.match_with_initial_guess(
                ref_points, src_points, initial_guess
            )
        else:
            result = self.engine.match(ref_points, src_points)

        # 4. Populate response
        response.success = result.success
        response.message = result.message
        response.fitness_score = result.fitness_score
        response.rmse = result.rmse
        response.elapsed_ms = result.elapsed_ms
        response.result_transform = self._matrix_to_transform_stamped(
            result.transformation
        )

        self.get_logger().info(
            f'Matching {"succeeded" if result.success else "FAILED"} '
            f'in {result.elapsed_ms:.1f} ms '
            f'(fitness={result.fitness_score:.3f}, rmse={result.rmse:.4f})'
        )

        return response

    def _transform_msg_to_matrix(self, transform: Transform) -> np.ndarray:
        """Convert geometry_msgs/Transform to 4x4 numpy matrix."""
        from scipy.spatial.transform import Rotation

        q = transform.rotation
        t = transform.translation

        rot = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        T = np.eye(4)
        T[:3, :3] = rot
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    def _matrix_to_transform_stamped(self, matrix: np.ndarray) -> TransformStamped:
        """Convert 4x4 numpy matrix to geometry_msgs/TransformStamped."""
        from scipy.spatial.transform import Rotation

        ts = TransformStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = 'reference'
        ts.child_frame_id = 'source'

        ts.transform.translation.x = float(matrix[0, 3])
        ts.transform.translation.y = float(matrix[1, 3])
        ts.transform.translation.z = float(matrix[2, 3])

        rot = Rotation.from_matrix(matrix[:3, :3]).as_quat()  # [x, y, z, w]
        ts.transform.rotation.x = float(rot[0])
        ts.transform.rotation.y = float(rot[1])
        ts.transform.rotation.z = float(rot[2])
        ts.transform.rotation.w = float(rot[3])

        return ts


def main(args=None):
    rclpy.init(args=args)
    node = MatchingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()