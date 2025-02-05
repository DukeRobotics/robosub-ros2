
#
# launch file for composable node container
#
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """
    Generate a ROS 2 launch description for the vectornav package using composable nodes.

    This function sets up and returns a LaunchDescription object that includes
    the configuration for launching the vectornav and vn_sensor_msgs composable nodes
    within a ComposableNodeContainer. The nodes use the configuration files located at
    'config/vectornav_composable.yaml' and 'config/vn_sensor_msgs_composable.yaml' within
    the vectornav package directory.

    Returns:
        LaunchDescription: A LaunchDescription object containing the launch configuration.
    """
    con = ComposableNodeContainer(
        name='vectornav_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='vectornav',
                plugin='vectornav::Vectornav',
                name='vectornav',
                parameters=[PathJoinSubstitution(
                    [FindPackageShare('vectornav'),
                     'config', 'vectornav_composable.yaml'])],
                remappings=[],
                extra_arguments=[{'use_intra_process_comms': True}]),
            ComposableNode(
                package='vectornav',
                plugin='vectornav::VnSensorMsgs',
                name='vn_sensor_msgs',
                parameters=[PathJoinSubstitution(
                    [FindPackageShare('vectornav'),
                     'config', 'vn_sensor_msgs_composable.yaml'])],
                remappings=[],
                extra_arguments=[{'use_intra_process_comms': True}]),
            ],
        output='screen')

    return LaunchDescription([con])

