import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('point_cloud_matching'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='point_cloud_matching',
            executable='matching_node',
            name='matching_node',
            parameters=[config],
            output='screen',
        )
    ])