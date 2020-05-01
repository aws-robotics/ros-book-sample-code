# This file is licensed under MIT-0 (https://github.com/aws/mit-0)
# which can be found in the 'LICENSE' file in this repository.

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        ComposableNodeContainer(
            name='robot_runtime',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='fake_robot',
                    plugin='fake_robot::FakeLaserNode',
                    name='fake_laser',
                ),
                ComposableNode(
                    package='fake_robot',
                    plugin='fake_robot::LaserListenerNode',
                    name='laser_listener',
                ),
            ],
        ),
    ])
