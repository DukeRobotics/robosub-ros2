from pathlib import Path

import rclpy
import yaml
from launch import LaunchDescription, LaunchService
from launch.actions import LogInfo
from launch_ros.actions import Node as LaunchNode
from rclpy.node import Node


def connect_all(node: any) -> None:  # noqa: ARG001
    """Get all cameras."""
    # Get camera specs. In ROS2, we need to load the YAML file directly.
    camera_config_path = './onboard/src/cv/config/usb_cameras.yaml'
    with Path.open(camera_config_path) as f:
        cameras = yaml.safe_load(f)

    # List to hold launch nodes
    launch_nodes = []

    for camera_name, camera in cameras.items():
        device_path = camera['device_path']
        topic = camera['topic']
        # Declare node and arguments for each camera
        launch_nodes.append(
            LaunchNode(
                package='cv',
                executable='usb_camera',
                name=camera_name,
                output='screen',
                parameters=[{'topic': topic, 'device_path': device_path, 'framerate': -1}],
            ),
        )

    return launch_nodes

def generate_launch_description() -> LaunchDescription:
    """Generate launch description of launching USB cameras."""
    # Launch description setup
    return LaunchDescription(
        [
            LogInfo(msg='Launching USB Camera nodes...'),
            *connect_all(Node('usb_camera_connect_all')),
        ],
    )

def main() -> None:
    """Start rclpy."""
    rclpy.init()

    # Use a launch file in ROS2
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())

    launch_service.run()

    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
