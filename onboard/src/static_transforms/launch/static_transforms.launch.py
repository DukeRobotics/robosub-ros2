import os
from pathlib import Path

import resource_retriever as rr
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node


def make_transform_publisher(node_name: str, transform: dict) -> Node:
    """
    Create a tf2 static transform publisher node for a given transform.

    Args:
        node_name (str): The name of the transform
        transform (dict): A dict containing transform values and frame names with labels 'x', 'y', 'z', 'roll', 'pitch'
                          'yaw', 'frame_id', and 'child_frame'.
    """
    return Node(name=node_name,
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['--x', str(transform['x']),
                           '--y', str(transform['y']),
                           '--z', str(transform['z']),
                           '--roll', str(transform['roll']),
                           '--pitch', str(transform['pitch']),
                           '--yaw', str(transform['yaw']),
                           '--frame-id', transform['frame_id'],
                           '--child-frame-id', transform['child_frame']])

def generate_launch_description() -> LaunchDescription:
    """
    Generate a ROS 2 launch description for static transforms.

    This function sets up and returns a LaunchDescription object that includes
    the configuration for launching multiple static transform publishers. Each
    transform publisher is created using the `make_transform_publisher` function
    with predefined transform values.

    Returns:
        LaunchDescription: A LaunchDescription object containing the launch configuration.
    """
    ld = LaunchDescription()

    robot_name = os.getenv('ROBOT_NAME')
    config_yaml_path = f'package://static_transforms/config/{robot_name}.yaml'
    config_file_resolved_path = rr.get_filename(config_yaml_path, use_protocol=False)

    with Path(config_file_resolved_path).open() as f:
        transforms = yaml.safe_load(f)['transforms']

        for transform_name, transform_info in transforms.items():
            ld.add_action(make_transform_publisher(transform_name + '_static_tranform', transform_info))

    return ld
