from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Generate a ROS 2 launch description for the vectornav package.

    This function sets up and returns a LaunchDescription object that includes
    the configuration for launching the vectornav and vn_sensor_msgs nodes. Both
    nodes use the configuration file located at 'config/vectornav.yaml' within
    the vectornav package directory.

    Returns:
        LaunchDescription: A LaunchDescription object containing the launch configuration.
    """
    this_dir = Path(get_package_share_directory('vectornav'))
    path_to_config_file = this_dir / 'config' / 'vectornav.yaml'

    # Vectornav
    start_vectornav_cmd = Node(
        package='vectornav',
        executable='vectornav',
        output='screen',
        parameters=[path_to_config_file])

    start_vectornav_sensor_msgs_cmd = Node(
        package='vectornav',
        executable='vn_sensor_msgs',
        output='screen',
        parameters=[path_to_config_file])

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(start_vectornav_cmd)
    ld.add_action(start_vectornav_sensor_msgs_cmd)

    return ld
