#!/usr/bin/env python3

import contextlib
import os
from pathlib import Path

import numpy as np
import pandas as pd
import rclpy
import resource_retriever as rr
import tf2_geometry_msgs
import tf2_ros
import yaml
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sympy import Matrix, rad
from sympy.matrices import rot_ccw_axis1, rot_ccw_axis2, rot_ccw_axis3
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from transforms3d.euler import euler2quat, quat2euler

CONTROLS_PACKAGE_PATH = 'package://controls/'
CONFIG_FILE_PATH_TEMPLATE = CONTROLS_PACKAGE_PATH + 'config/%s.yaml'

def get_robot_name() -> str:
    """
    Get the robot name from the user.

    Returns:
        str: The robot name.
    """
    # Get the default value from the ROBOT_NAME environment variable
    default_robot_name = os.getenv('ROBOT_NAME')

    # Ask the user for the robot name
    user_robot_name = input(f"Enter the robot name (press enter for default '{default_robot_name}'): ")

    # Use the default value if the user input is empty
    return user_robot_name.strip() if user_robot_name.strip() else default_robot_name

def get_transform(node: Node, tf_buffer: Buffer) -> TransformStamped:
    """
    Spin the ROS2 node and wait to receive a static transform from `base_link` to `corner_link`.

    Args:
        node (Node): The ROS2 node.
        tf_buffer (Buffer): The TF2 buffer for transform lookup.

    Returns:
        TransformStamped: The transform from `base_link` to `corner_link`.
    """
    starting_time = Clock().now()
    last_message_time = Clock().now()

    # Loops for 5 seconds to check for transformation
    while rclpy.ok() and Clock().now() - starting_time < Duration(seconds=5):
        # Log message every second
        if Clock().now() - last_message_time >= Duration(seconds=1):
            print('Waiting for transform from base_link to corner_link...')
            last_message_time = Clock().now()

        # Attempt to receive transformation
        with contextlib.suppress(tf2_ros.LookupException, tf2_ros.ConnectivityException):
            return tf_buffer.lookup_transform('base_link', 'corner_link', Time())

        rclpy.spin_once(node, timeout_sec=0.1)

    # Raise error if transformation not found
    error_msg = (
        'Could not get base_link to corner_link transform. Launch the "corner_link_static_tranform" node and ensure '
        'that the transform is being published.'
    )
    raise RuntimeError(error_msg)

def rotation_matrix(roll: float, pitch: float, yaw: float) -> Matrix:
    """
    Get the rotation matrix for given roll, pitch, and yaw Euler angles.

    Order of rotations: roll -> pitch -> yaw. See https://en.wikipedia.org/wiki/Rotation_matrix#In_three_dimensions.

    Args:
        roll (float): The roll angle in radians.
        pitch (float): The pitch angle in radians.
        yaw (float): The yaw angle in radians.
    """
    roll_matrix = rot_ccw_axis1(roll)
    pitch_matrix = rot_ccw_axis2(pitch)
    yaw_matrix = rot_ccw_axis3(yaw)
    return yaw_matrix * pitch_matrix * roll_matrix

def compute_force_torque(thruster: dict, corner_to_base_link_transform: TransformStamped) -> tuple[Matrix, Matrix]:
    """
    Compute the force and torque vectors for a given thruster.

    Args:
        thruster (dict): The thruster data.
        corner_to_base_link_transform (Pose): The transform needed to find base link position of thrusters.

    Returns:
        Tuple[Matrix, Matrix]: The force and torque vectors.
    """
    # Create pose message for thruster's position
    pose = Pose()
    pose.position = Point(x=thruster['pos'][0], y=thruster['pos'][1], z=thruster['pos'][2])
    quat = euler2quat(*map(rad, thruster['rpy']))
    pose.orientation = Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0])

    # Transform thruster position from corner_link to base_link
    pose = tf2_geometry_msgs.do_transform_pose(pose, corner_to_base_link_transform)

    # Convert transformed position to Matrix
    pos = Matrix([pose.position.x, pose.position.y, pose.position.z])

    # Convert transformed orientation to Euler angles
    rpy_radians = quat2euler([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])

    # Get flipped value of thrusters
    flipped = -1 if thruster['flipped'] else 1

    # Compute force vector
    r_matrix = rotation_matrix(*rpy_radians)
    force = r_matrix * Matrix([1, 0, 0]) * flipped

    # Compute torque vector
    torque = pos.cross(force)

    return force, torque

def to_csv(data: pd.DataFrame, file_path: Path) -> None:
    """
    Convert a pandas dataframe to a CSV file and remove the last newline character.

    Args:
        data (pd.DataFrame): The data to convert to a CSV file.
        file_path (Path): The path to save the CSV file to.
    """
    data.to_csv(file_path, index=False, header=False)
    with file_path.open('rb') as file:
        file_data = file.read()
    with file_path.open('wb') as file:
        file.write(file_data[:-1])

def main() -> None:
    """Compute the wrench matrix and its pseudoinverse for a robot using symbolic math. Save the matrices as CSVs."""
    rclpy.init()
    node = Node('wrench_matrix_computation')

    # Check if static_transforms node is running
    if 'corner_link_static_tranform' not in node.get_node_names():
        error_msg = (
            'The "corner_link_static_tranform" node is not running. Static transforms must be published before running '
            'this script.'
        )
        raise RuntimeError(error_msg)

    # Spin the node and wait to recieve a static transform
    tf_buffer = Buffer()
    _ = TransformListener(tf_buffer, node)
    transformation = get_transform(node, tf_buffer)

    # Get the path to the config file for the robot the user would like to compute the wrench matrix for
    robot_name = get_robot_name()
    config_file_path = Path(rr.get_filename(CONFIG_FILE_PATH_TEMPLATE % robot_name, use_protocol=False))

    # Read YAML data
    with config_file_path.open() as file:
        vehicle = yaml.safe_load(file)

    # Initialize wrench matrix
    wrench_matrix = Matrix.zeros(6, len(vehicle['thrusters']))

    # Compute force and torque vectors and add them to the wrench matrix
    for idx, thruster in enumerate(vehicle['thrusters']):
        force, torque = compute_force_torque(thruster, transformation)

        # Add force and torque to the wrench matrix
        wrench_matrix[0:3, idx] = force
        wrench_matrix[3:6, idx] = torque

    # Compute pseudoinverse of wrench matrix
    wrench_matrix_pinv = wrench_matrix.pinv()

    # Convert sympy matrices to numpy matrices
    wrench_matrix_array = np.array(wrench_matrix).astype(np.float64)
    wrench_matrix_pinv_array = np.array(wrench_matrix_pinv).astype(np.float64)

    # Convert numpy matrices to pandas dataframes
    wrench_matrix_df = pd.DataFrame(wrench_matrix_array)
    wrench_matrix_pinv_df = pd.DataFrame(wrench_matrix_pinv_array)

    # Set all values less than e-10 to 0
    wrench_matrix_df = wrench_matrix_df.round(10)
    wrench_matrix_pinv_df = wrench_matrix_pinv_df.round(10)

    # Convert negative zeroes to positive zeroes
    wrench_matrix_df = wrench_matrix_df.replace(-0.0, 0.0)
    wrench_matrix_pinv_df = wrench_matrix_pinv_df.replace(-0.0, 0.0)

    # Get full paths to CSV files
    wrench_matrix_file_path = Path(rr.get_filename(CONTROLS_PACKAGE_PATH + vehicle['wrench_matrix_file_path'],
                                                   use_protocol=False))
    wrench_matrix_pinv_file_path = Path(rr.get_filename(CONTROLS_PACKAGE_PATH + vehicle['wrench_matrix_pinv_file_path'],
                                                        use_protocol=False))

    # Export data to CSV files
    to_csv(wrench_matrix_df, wrench_matrix_file_path)
    to_csv(wrench_matrix_pinv_df, wrench_matrix_pinv_file_path)

    # Print data to console
    print('Wrench matrix:')
    print(wrench_matrix_df)
    print()
    print('Wrench matrix pseudoinverse:')
    print(wrench_matrix_pinv_df)
    print()
    print(f'Saved wrench matrix to {wrench_matrix_file_path}')
    print(f'Saved wrench matrix pseudoinverse to {wrench_matrix_pinv_file_path}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
