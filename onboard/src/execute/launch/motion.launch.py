import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

    offboard_comms = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(get_package_share_directory('offboard_comms'), 'launch', 'offboard_comms.xml')),
    )

    vectornav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('vectornav'), 'launch', 'vectornav.launch.py')),
    )

    static_transforms = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('static_transforms'), 'launch', 'static_transforms.launch.py')),
    )

    sensor_fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('sensor_fusion'), 'launch', 'fuse.launch.py')),
    )

    controls = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(get_package_share_directory('controls'), 'launch', 'controls.xml')),
    )

    usb_camera = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(get_package_share_directory('cv'), 'launch', 'usb_camera_connect.xml')),
    )

    system_utils = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(get_package_share_directory('system_utils'), 'launch', 'system_utils.xml')),
    )

    ld.add_action(offboard_comms)
    ld.add_action(vectornav)
    ld.add_action(static_transforms)
    ld.add_action(sensor_fusion)
    ld.add_action(controls)
    ld.add_action(usb_camera)
    ld.add_action(system_utils)

    return ld
