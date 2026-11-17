import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
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
    ld.add_action(DeclareLaunchArgument(
        'pathfinder_format', default_value='pd6', choices=['pd6', 'pd13', 'pd0'],
        description='Reader for the Pathfinder device output; does not reconfigure the device.',
    ))

    if robot_name in ['oogway', 'oogway_shell']:
        use_pd0 = PythonExpression(["'", LaunchConfiguration('pathfinder_format'), "' == 'pd0'"])
        ld.add_action(IncludeLaunchDescription(
            XMLLaunchDescriptionSource(str(pkg_offboard_comms / 'launch' / 'dvl_pathfinder_raw.xml')),
            condition=UnlessCondition(use_pd0),
        ))
        ld.add_action(IncludeLaunchDescription(
            XMLLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('dvl_pathfinder'), 'launch', 'dvl_pathfinder.xml',
            ])),
            condition=IfCondition(use_pd0),
        ))

    ld.add_action(IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(pkg_offboard_comms / 'launch' / 'gyro.xml')),
    ))

    ld.add_action(IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(pkg_offboard_comms / 'launch' / 'modem.xml')),
    ))

    ld.add_action(IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(pkg_offboard_comms / 'launch' / 'peripheral.xml')),
    ))

    ld.add_action(IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(pkg_offboard_comms / 'launch' / 'thrusters.xml')),
    ))

    return ld
