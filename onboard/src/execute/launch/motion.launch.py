from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    """
    Generate launch description for the state nodes.

    Returns:
        LaunchDescription: The launch description object containing the nodes to be launched.
    """
    ld = LaunchDescription()

    controls = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(Path(get_package_share_directory('controls')) / 'launch' /
                                       'controls.xml')),
    )

    offboard_comms = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(Path(get_package_share_directory('offboard_comms')) / 'launch' /
                                       'offboard_comms.xml')),
    )

    sensor_fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('sensor_fusion')) / 'launch' /
                                          'fuse.launch.py')),
    )

    static_transforms = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('static_transforms')) / 'launch' /
                                          'static_transforms.launch.py')),
    )

    system_utils = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(Path(get_package_share_directory('system_utils')) / 'launch' /
                                       'system_utils.xml')),
    )

    usb_camera = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(str(Path(get_package_share_directory('cv')) / 'launch' /
                                       'usb_camera_connect.xml')),
    )

    vectornav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('vectornav')) / 'launch' /
                                          'vectornav.launch.py')),
    )

    ld.add_action(controls)
    ld.add_action(offboard_comms)
    ld.add_action(sensor_fusion)
    ld.add_action(static_transforms)
    ld.add_action(system_utils)
    ld.add_action(usb_camera)
    ld.add_action(vectornav)

    return ld
