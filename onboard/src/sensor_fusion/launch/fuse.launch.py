import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    """
    Generate the launch description for the sensor fusion nodes.

    Returns:
        LaunchDescription: The launch description object containing the nodes to be launched.
    """
    ld = LaunchDescription()

    robot_name = os.getenv('ROBOT_NAME')

    pkg_sensor_fusion = Path(get_package_share_directory('sensor_fusion'))

    # Start robot localization using an Extended Kalman filter
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization_node',
        output='screen',
        parameters=[str(Path(get_package_share_directory('sensor_fusion')) / 'config' / f'{robot_name}.yaml')],
        remappings=[
            ('odometry/filtered', 'state'),
        ],
        # robot_localization probes for the next unused sensor index (e.g. imu1) by declaring it and
        # then failing to read it, which makes rclcpp log a spurious "parameter is not initialized"
        # warning. Raising the rclcpp logger to ERROR hides that noise without silencing the node.
        arguments=['--ros-args', '--log-level', 'rclcpp:=ERROR'])

    ld.add_action(robot_localization)

    # Start DVL to odometry publisher
    ld.add_action(IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(pkg_sensor_fusion / 'launch' / 'dvl_to_odom.xml')),
    ))

    return ld
