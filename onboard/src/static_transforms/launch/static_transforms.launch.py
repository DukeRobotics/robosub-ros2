import os
from pathlib import Path

import resource_retriever as rr
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node

ROBOT_NAME = os.getenv('ROBOT_NAME', 'oogway')
STATIC_TRANSFORMS_PATH_TEMPLATE = 'package://offboard_comms/{subpath}' # Probably wrong, it's copy pasted
CONFIG_YAML_PATH = STATIC_TRANSFORMS_PATH_TEMPLATE.format(subpath=f'config/{ROBOT_NAME}.yaml')

OUTPUT_PREFIX = Path(__file__).name.capitalize()

error_when_loading_yaml = True
try:
    config_file_resolved_path = rr.get_filename(CONFIG_YAML_PATH, use_protocol=False)
    with Path(config_file_resolved_path).open() as f:
        config_data = yaml.safe_load(f)
        TRANSFORMS_DATA = config_data['transforms']

    error_when_loading_yaml = False

except FileNotFoundError:
    print(
        f'{OUTPUT_PREFIX}: FATAL ERROR: Could not find config YAML file at "{config_file_resolved_path}". '
        'Please make sure the file exists.',
    )

except yaml.YAMLError as e:
    if hasattr(e, 'problem_mark'):
        mark = e.problem_mark
        print(
            f'{OUTPUT_PREFIX}: FATAL ERROR: Config YAML file is not in valid YAML format at line {mark.line + 1} and '
            f'column {mark.column + 1}.',
        )

    print(
        f'{OUTPUT_PREFIX}: FATAL ERROR: Could not parse config YAML file at "{config_file_resolved_path}". '
        f'Please make sure the file is in valid YAML format.',
    )

except KeyError:
    print(
        f'{OUTPUT_PREFIX}: FATAL ERROR: Config YAML file at "{config_file_resolved_path}" does not contain required '
        'top-level key "transforms".',
    )

except Exception as e:  # noqa: BLE001
    print(
        f'{OUTPUT_PREFIX}: FATAL ERROR: An unexpected error occurred when loading the config YAML file at '
        f'"{config_file_resolved_path}": {e}',
    )

def make_transform_publisher(transform: list) -> Node:
    """
    Create a tf2 static transform publisher node for a given transform.

    Args:
        transform (list): Transform values and frame names, in the form
            [x, y, z, roll, pitch, yaw, frame_id, child_frame_id].
    """
    return Node(package='tf2_ros',
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

    for transform in TRANSFORMS_DATA:
        transform_list = [transform['x'], transform['y'], transform['z'], transform['roll'], transform['pitch'],
                            transform['yaw'], transform['frame_id'], transform['child_frame']]
        ld.add_action(make_transform_publisher(transform_list))

    return ld
