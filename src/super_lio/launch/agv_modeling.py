"""Super-LIO as the odometry / motion-compensation front end for AGV target modeling.

Publishes the undistorted scan on /lio/cloud_body (LiDAR frame, scan-end stamp)
together with /lio/odom_body (world <- lidar, identical stamp). Pair it with
agv_target_modeling_lite running `odom_source: lio`.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('super_lio'), 'config', 'agv_modeling.yaml'
    )

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Super-LIO parameter file.',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Set true when replaying a bag with --clock.',
    )

    super_lio_node = Node(
        package='super_lio',
        executable='super_lio_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return LaunchDescription([config_arg, use_sim_time_arg, super_lio_node])
