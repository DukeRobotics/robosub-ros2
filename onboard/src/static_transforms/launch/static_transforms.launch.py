from launch import LaunchDescription
from launch_ros.actions import Node

# Static transforms
CORNER_LINK_TRANSFORM = [0.219075, -0.3095625, 0.19764375, 0, 0, 0, 'base_link', 'corner_link']
DVL_LINK_TRANSFORM = [-0.20955, 0.0809625, -0.276225, 0, 0, 0, 'corner_link', 'dvl_link']
IMU_LINK_TRANSFORM = [-0.219075, 0.3095625, -0.11241875, 0, 0, 0, 'corner_link', 'imu_link']
SONAR_LINK_TRANSFORM = [0.067, 0.31, 0.05, 0, 0, 0, 'corner_link', 'sonar_link']
CAMERAS_LINK_TRANSFORM = [0, 0.0362, 0, 0, 0, 0, 'corner_link', 'cameras_link']

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

    ld.add_action(make_transform_publisher(CORNER_LINK_TRANSFORM))
    ld.add_action(make_transform_publisher(DVL_LINK_TRANSFORM))
    ld.add_action(make_transform_publisher(IMU_LINK_TRANSFORM))
    ld.add_action(make_transform_publisher(SONAR_LINK_TRANSFORM))
    ld.add_action(make_transform_publisher(SONAR_LINK_TRANSFORM))
    ld.add_action(make_transform_publisher(CAMERAS_LINK_TRANSFORM))
    return ld
