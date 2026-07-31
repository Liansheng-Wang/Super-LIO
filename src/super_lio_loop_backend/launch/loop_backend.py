import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("super_lio_loop_backend")
    config = os.path.join(package_share, "config", "loop_backend.yaml")
    return LaunchDescription(
        [
            Node(
                package="super_lio_loop_backend",
                executable="loop_backend_node",
                name="super_lio_loop_backend",
                output="screen",
                parameters=[config],
            )
        ]
    )
