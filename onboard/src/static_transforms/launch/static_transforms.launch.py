from launch import LaunchDescription
from launch_ros.actions import Node

# DEFINE TRANSFORMS HERE
CORNER_LINK_TRANSFORM = [0.219075, -0.3095625, 0.19764375, 0, 0, 0, 'base_link', 'corner_link']
DVL_LINK_TRANSFORM = [-0.20955, 0.0809625, -0.276225, 0, 0, 0, 'corner_link', 'dvl_link']
IMU_LINK_TRANSFORM = [-0.219075, 0.3095625, -0.11241875, 0, 0, 0, 'corner_link', 'imu_link']
SONAR_LINK_TRANSFORM = [0.067, 0.31, 0.05, 0, 0, 0, 'corner_link', 'sonar_link']
CAMERAS_LINK_TRANSFORM = [0, 0.0362, 0, 0, 0, 0, 'corner_link', 'cameras_link']

# Create the tf2 node for each transform
def make_transform_publisher(transform):
    return Node(package='tf2_ros',
                executable='static_transform_publisher',
                output='screen',
                arguments=['--x', str(transform[0]),
                           '--y', str(transform[1]),
                           '--z', str(transform[2]),
                           '--roll', str(transform[3]),
                           '--pitch', str(transform[4]),
                           '--yaw', str(transform[5]),
                           '--frame-id', transform[6],
                           '--child-frame-id', transform[7]])

# Create the launch description and populate
def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(make_transform_publisher(CORNER_LINK_TRANSFORM))
    ld.add_action(make_transform_publisher(DVL_LINK_TRANSFORM))
    ld.add_action(make_transform_publisher(IMU_LINK_TRANSFORM))
    ld.add_action(make_transform_publisher(SONAR_LINK_TRANSFORM))
    ld.add_action(make_transform_publisher(SONAR_LINK_TRANSFORM))
    ld.add_action(make_transform_publisher(CAMERAS_LINK_TRANSFORM))
    return ld
