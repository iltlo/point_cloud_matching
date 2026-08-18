#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from point_cloud_matching.srv import MatchClouds
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


class MatchVisualizerNode(Node):
    def __init__(self):
        super().__init__('match_visualizer_node')

        self._client_cb_group = MutuallyExclusiveCallbackGroup()

        # Use a reentrant callback group so nested service calls don't deadlock
        self.cb_group = ReentrantCallbackGroup()

        # Service client to matching_node
        self.match_client = self.create_client(
            MatchClouds, '/match_clouds', 
            # callback_group=self.cb_group
            callback_group=self._client_cb_group
        )

        # Service server for visualization requests
        self.viz_service = self.create_service(
            MatchClouds, '/visualize_match', self.visualize_callback,
            callback_group=self.cb_group)

        # Publishers (transient local so RViz can pick them up late)
        qos = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_ref = self.create_publisher(PointCloud2, '/viz/reference_cloud', qos)
        self.pub_src = self.create_publisher(PointCloud2, '/viz/source_cloud', qos)
        self.pub_aligned = self.create_publisher(PointCloud2, '/viz/aligned_cloud', qos)

        self.get_logger().info('Match Visualizer ready. Call /visualize_match to match and visualize.')

    def visualize_callback(self, request, response):
        self.get_logger().info('Visualize request received, calling /match_clouds...')

        # Wait for matching service
        if not self.match_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/match_clouds service not available')
            response.success = False
            return response

        # # Forward the request to matching_node
        # future = self.match_client.call_async(request)
        # rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        # if future.result() is None:
        #     self.get_logger().error('Match service call failed')
        #     response.success = False
        #     return response

        # match_response = future.result()
# Forward the request to matching_node
        future = self.match_client.call_async(request)

        # DON'T use spin_until_future_complete — it conflicts with the executor
        import time
        timeout = 30.0
        start = time.time()
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
            if time.time() - start > timeout:
                self.get_logger().error('Match service call timed out')
                response.success = False
                return response

        if future.result() is None:
            self.get_logger().error('Match service call failed')
            response.success = False
            return response

        match_response = future.result()
        self.get_logger().info(
            # f'Match result: fitness={match_response.fitness:.3f}, rmse={match_response.rmse:.4f}')
            f'Match result: fitness={match_response.fitness_score:.3f}, rmse={match_response.rmse:.4f}')

        # Load clouds and publish
        # ref_pcd = o3d.io.read_point_cloud(request.reference_path)
        # src_pcd = o3d.io.read_point_cloud(request.target_path)
        ref_pcd = o3d.io.read_point_cloud(request.reference_path).voxel_down_sample(0.05)
        src_pcd = o3d.io.read_point_cloud(request.target_path).voxel_down_sample(0.05)


        # Apply the returned transform to source
        transform_matrix = self.transform_msg_to_matrix(match_response.result_transform.transform)

        import copy

        src_aligned = copy.deepcopy(src_pcd)
        src_aligned.transform(transform_matrix)

        # Publish colored clouds
        stamp = self.get_clock().now().to_msg()
        self.pub_ref.publish(self.pcd_to_ros2(ref_pcd, stamp, [1.0, 0.0, 0.0]))      # Red
        self.pub_src.publish(self.pcd_to_ros2(src_pcd, stamp, [0.0, 0.0, 1.0]))       # Blue
        self.pub_aligned.publish(self.pcd_to_ros2(src_aligned, stamp, [0.0, 1.0, 0.0]))  # Green

        self.get_logger().info('Published visualization clouds.')

        # response.success = match_response.success
        # response.fitness = match_response.fitness
        # response.rmse = match_response.rmse
        # response.transform = match_response.transform
        # return response
        response.success = match_response.success
        response.fitness_score = match_response.fitness_score
        response.rmse = match_response.rmse
        response.result_transform = match_response.result_transform
        return response

    def transform_msg_to_matrix(self, transform_msg):
        from scipy.spatial.transform import Rotation
        t = transform_msg.translation
        r = transform_msg.rotation
        rot = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        matrix = np.eye(4)
        matrix[:3, :3] = rot
        matrix[:3, 3] = [t.x, t.y, t.z]
        return matrix

    def pcd_to_ros2(self, pcd, stamp, color_rgb):
        points = np.asarray(pcd.points).astype(np.float32)
        n = len(points)

        # Pack RGB into a float32
        r, g, b = int(color_rgb[0]*255), int(color_rgb[1]*255), int(color_rgb[2]*255)
        rgb_int = (r << 16) | (g << 8) | b
        rgb_packed = np.full((n, 1), rgb_int, dtype=np.uint32).view(np.float32)

        # XYZRGB data
        data = np.hstack([points, rgb_packed]).astype(np.float32)

        msg = PointCloud2()
        msg.header = Header(frame_id='map', stamp=stamp)
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n
        msg.data = data.tobytes()
        msg.is_dense = True
        return msg


def main():
    rclpy.init()
    node = MatchVisualizerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()