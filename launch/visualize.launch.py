import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('point_cloud_matching')
    rviz_config = os.path.join(pkg_dir, 'config', 'matching_viz.rviz')

    return LaunchDescription([
        # Matching engine node
        Node(
            package='point_cloud_matching',
            executable='matching_node',
            name='matching_node',
            output='screen',
        ),
        # Visualization node — run via python3 explicitly to avoid shebang issue
        Node(
            package='point_cloud_matching',
            executable='visualizer_node',
            name='match_visualizer_node',
            output='screen',
            prefix='python3',
        ),
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])