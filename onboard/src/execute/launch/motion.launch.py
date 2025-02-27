import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import XMLLaunchDescriptionSource, PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    sensors = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(os.path.join(get_package_share_directory('data_pub'), 'launch', 'pub_all_launch.xml'))
    )

    vectornav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('vectornav'), 'launch', 'vectornav.launch.py'))
    )

    static_transforms = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('static_transforms'), 'launch', 'static_transforms.launch.py'))
    )

    sensor_fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('sensor_fusion'), 'launch', 'fuse.launch.py'))
    )

    ld.add_action(sensors)
    ld.add_action(vectornav)
    ld.add_action(static_transforms)
    ld.add_action(sensor_fusion)

    return ld
