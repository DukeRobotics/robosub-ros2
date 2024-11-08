import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    # Start robot localization using an Extended Kalman filter
    robot_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory('sensor_fusion'), 'params', 'main.yaml')],
        remappings=[
            ("odometry/filtered", "state")
        ])

    #TODO also add a node to publish robot URDF (which we have??)

    ld.add_action(robot_localization)
    return ld