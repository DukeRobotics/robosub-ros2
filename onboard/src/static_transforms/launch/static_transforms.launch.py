import os
from pathlib import Path

import resource_retriever as rr
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node

TRANFORM_YAML_LABELS = ('x', 'y', 'z', 'roll', 'pitch', 'yaw', 'frame_id', 'child_frame')

def make_transform_publisher(node_name: str, transform: list) -> Node:
    """
    Create a tf2 static transform publisher node for a given transform.

    Args:
        node_name (str): The name of the transform
        transform (list): Transform values and frame names, in the form
            [x, y, z, roll, pitch, yaw, frame_id, child_frame_id].
    """
    return Node(name=node_name,
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['--x', str(transform[0]),
                           '--y', str(transform[1]),
                           '--z', str(transform[2]),
                           '--roll', str(transform[3]),
                           '--pitch', str(transform[4]),
                           '--yaw', str(transform[5]),
                           '--frame-id', transform[6],
                           '--child-frame-id', transform[7]])

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

    robot_name = os.getenv('ROBOT_NAME', 'oogway')
    config_yaml_path = f'package://static_transforms/config/{robot_name}.yaml'
    config_file_resolved_path = rr.get_filename(config_yaml_path, use_protocol=False)

    with Path(config_file_resolved_path).open() as f:
        transforms = yaml.safe_load(f)['transforms']

        for transform_name, transform_info in transforms.items():
            transform_list = [transform_info[label] for label in TRANFORM_YAML_LABELS]
            ld.add_action(make_transform_publisher(transform_name + '_static_tranform', transform_list))

    return ld
