# This file is licensed under MIT-0 (https://github.com/aws/mit-0)
# which can be found in the 'LICENSE' file in this repository.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('remote_operator')
    rviz_config_path = os.path.join(package_share, 'config', 'remote_operator.rviz')

    return LaunchDescription([
        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen',
        ),
    ])
