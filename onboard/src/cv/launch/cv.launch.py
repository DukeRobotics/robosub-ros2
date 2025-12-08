import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    """
    Generate a ROS 2 launch description based on the given XML file.

    Returns:
        LaunchDescription: The launch description containing the included launch files.
    """
    robot_name = os.getenv('ROBOT_NAME')

    pkg_cv = Path(get_package_share_directory('cv'))

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'prequal',
        default_value='false',
        description='Whether this is being launched for a prequal run',
    ))

    if robot_name in ['oogway', 'oogway_shell']:
        ld.add_action(IncludeLaunchDescription(
            XMLLaunchDescriptionSource(str(pkg_cv / 'launch' / 'depthai_spatial_detection.xml')),
            launch_arguments={'camera': 'front'}.items(),
        ))

    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(pkg_cv / 'launch' / 'usb_camera_detectors.launch.py')),
        launch_arguments=[
            ('enable_lane_marker', LaunchConfiguration('prequal')),
        ],
    ))

    return ld
