import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Generate the launch description for the sensor fusion nodes.

    Returns:
        LaunchDescription: The launch description object containing the nodes to be launched.
    """
    ld = LaunchDescription()

    robot_name = os.getenv('ROBOT_NAME')

    # Start robot localization using an Extended Kalman filter
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization_node',
        output='screen',
        parameters=[str(Path(get_package_share_directory('sensor_fusion')) / 'config' / f'{robot_name}.yaml')],
        remappings=[
            ('odometry/filtered', 'state'),
        ])

    ld.add_action(robot_localization)
    return ld
