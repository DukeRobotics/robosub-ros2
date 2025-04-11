import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    """
    Generate a ROS 2 launch description for offboard comms.

    Returns:
        LaunchDescription: The launch description containing the included launch files.
    """
    robot_name = os.getenv('ROBOT_NAME')

    pkg_offboard_comms = Path(get_package_share_directory('offboard_comms'))

    ld = LaunchDescription()

    if robot_name in ['oogway', 'oogway_shell']:
        ld.add_action(IncludeLaunchDescription(
            XMLLaunchDescriptionSource(str(pkg_offboard_comms / 'launch' / 'dvl_pathfinder.xml')),
        ))
    elif robot_name == 'crush':
        ld.add_action(IncludeLaunchDescription(
            XMLLaunchDescriptionSource(str(pkg_offboard_comms / 'launch' / 'dvl_wayfinder.xml')),
        ))

    ld.add_action(IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(pkg_offboard_comms / 'launch' / 'gyro.xml')),
    ))

    ld.add_action(IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(pkg_offboard_comms / 'launch' / 'peripheral.xml')),
    ))

    ld.add_action(IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(pkg_offboard_comms / 'launch' / 'thrusters.xml')),
    ))

    return ld
