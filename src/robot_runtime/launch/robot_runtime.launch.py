# This file is licensed under MIT0 (https://github.com/aws/mit0)
# which can be found in the 'LICENSE' file in this repository.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LifecycleNode
from launch_ros.descriptions import ComposableNode


MEASUREMENT_PERIOD = 'measurement_period'
PUBLISH_PERIOD = 'publish_period'
DEFAULT_MEASUREMENT_PERIOD_IN_MS = '1000'
DEFAULT_PUBLISH_PERIOD_IN_MS = '10000'


def generate_launch_description():
    """Launch example instrumented nodes."""
    launch_description = LaunchDescription()

    launch_description.add_action(DeclareLaunchArgument(
        MEASUREMENT_PERIOD,
        default_value=DEFAULT_MEASUREMENT_PERIOD_IN_MS,
        description='The period (in ms) between each subsequent metrics measurement made'
                    ' by the collector nodes'))
    launch_description.add_action(DeclareLaunchArgument(
        PUBLISH_PERIOD,
        default_value=DEFAULT_PUBLISH_PERIOD_IN_MS,
        description='The period (in ms) between each subsequent metrics message published'
                    ' by the collector nodes'))
    node_parameters = [
        {MEASUREMENT_PERIOD: LaunchConfiguration(MEASUREMENT_PERIOD)},
        {PUBLISH_PERIOD: LaunchConfiguration(PUBLISH_PERIOD)}]

    # Collect, aggregate, and measure system CPU % used
    system_cpu_node = LifecycleNode(
        package='system_metrics_collector',
        name='linux_system_cpu_collector',
        executable='linux_cpu_collector',
        output='screen',
        parameters=node_parameters,
    )

    # Collect, aggregate, and measure system memory % used
    system_memory_node = LifecycleNode(
        package='system_metrics_collector',
        name='linux_system_memory_collector',
        executable='linux_memory_collector',
        output='screen',
        parameters=node_parameters,
    )

    # Instrument the fake_laser_node demo to collect, aggregate, and publish it's CPU % + memory % used
    fake_laser_container = ComposableNodeContainer(
        name='fake_laser_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='fake_robot',
                plugin='fake_robot::FakeLaserNode',
                name='fake_laser_node'),
            ComposableNode(
                package='system_metrics_collector',
                plugin='system_metrics_collector::LinuxProcessCpuMeasurementNode',
                name='fake_laser_process_cpu_node',
                parameters=node_parameters,
            ),
            ComposableNode(
                package='system_metrics_collector',
                plugin='system_metrics_collector::LinuxProcessMemoryMeasurementNode',
                name='fake_laser_process_memory_node',
                parameters=node_parameters,
            )
        ],
        output='screen',
    )

    # Instrument the fake_laser_node demo to collect, aggregate, and publish it's CPU % + memory % used
    fake_laser_listener_container = ComposableNodeContainer(
        name='laser_listener_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='fake_robot',
                plugin='fake_robot::LaserListenerNode',
                name='laser_listener'),
            ComposableNode(
                package='system_metrics_collector',
                plugin='system_metrics_collector::LinuxProcessCpuMeasurementNode',
                name='fake_laser_listener_process_cpu_node',
                parameters=node_parameters,
            ),
            ComposableNode(
                package='system_metrics_collector',
                plugin='system_metrics_collector::LinuxProcessMemoryMeasurementNode',
                name='fake_laser_listener_process_memory_node',
                parameters=node_parameters,
            )
        ],
        output='screen',
    )

    launch_description.add_action(system_memory_node)
    launch_description.add_action(system_cpu_node)
    launch_description.add_action(fake_laser_container)
    launch_description.add_action(fake_laser_listener_container)

    return launch_description