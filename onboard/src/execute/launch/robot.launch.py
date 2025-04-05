import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    """
    Generate a ROS 2 launch description that launches all nodes needed for the robot to function, except task planning.

    Returns:
        LaunchDescription: The launch description containing the included launch files.
    """
    robot_name = os.getenv('ROBOT_NAME')

    # Get package share directories
    pkg_controls = Path(get_package_share_directory('controls'))
    pkg_cv = Path(get_package_share_directory('cv'))
    pkg_offboard_comms = Path(get_package_share_directory('offboard_comms'))
    pkg_sensor_fusion = Path(get_package_share_directory('sensor_fusion'))
    pkg_sonar = Path(get_package_share_directory('sonar'))
    pkg_static_transforms = Path(get_package_share_directory('static_transforms'))
    pkg_system_utils = Path(get_package_share_directory('system_utils'))
    pkg_vectornav = Path(get_package_share_directory('vectornav'))

    ld = LaunchDescription()

    # Add included launch files
    ld.add_action(IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(pkg_controls / 'launch' / 'controls.xml')),
    ))
    ld.add_action(IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(pkg_cv / 'launch' / 'usb_camera_detectors.xml')),
    ))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg_offboard_comms / 'launch' / 'offboard_comms.launch.py')),
    ))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg_sensor_fusion / 'launch' / 'fuse.launch.py')),
    ))

    if robot_name in ['oogway', 'oogway_shell']:
        ld.add_action(IncludeLaunchDescription(
            XMLLaunchDescriptionSource(str(pkg_sonar / 'launch' / 'sonar.xml')),
        ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg_static_transforms / 'launch' / 'static_transforms.launch.py')),
    ))
    ld.add_action(IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(pkg_system_utils / 'launch' / 'system_utils.xml')),
    ))
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg_vectornav / 'launch' / 'vectornav.launch.py')),
    ))

    return ld
