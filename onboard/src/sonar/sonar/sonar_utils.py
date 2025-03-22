import numpy as np
import rclpy
import tf2_geometry_msgs
import tf2_ros

SONAR_CENTER_GRADIANS = 200
RADIANS_PER_GRADIAN = np.pi / 200
GRADIANS_PER_DEGREE = 400 / 360

def transform_pose(buffer: tf2_ros.Buffer, pose: tf2_geometry_msgs.PoseStamped,
                   source_frame: str, target_frame: str) -> tf2_geometry_msgs.PoseStamped:
    """
    Transform pose from source reference frame to target reference frame.

    Args:
        buffer (tf2_ros.Buffer): The tf2 buffer containing transform data.
        pose (Pose): Pose in the source reference frame.
        source_frame (str): Name of the source frame (e.g., 'sonar_ping_360').
        target_frame (str): Name of the target frame (e.g., 'camera_depthai_front').

    Returns:
        Pose: Pose in the target reference frame.
    """
    try:
        # Wait for transform to be available
        transform = buffer.lookup_transform(
            target_frame, source_frame, rclpy.time.Time(),
        )

        # Transform the pose
        return tf2_geometry_msgs.do_transform_pose(pose, transform)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        error_message = f'Failed to transform pose: {e}'
        raise RuntimeError(error_message) from e


def centered_gradians_to_radians(angle_gradians: float) -> float:
    """
    Convert gradians centered at 200 to radians centered at 0.

    Args:
        angle_gradians (float): Angle in gradians where 200
            (Sonar.SONAR_CENTER_GRADIANS) is forward.

    Returns:
        float: Angle in radians.
    """
    return (angle_gradians - SONAR_CENTER_GRADIANS) * RADIANS_PER_GRADIAN


def degrees_to_centered_gradians(angle_degrees: float) -> int:
    """
    Convert degrees centered at 0 to gradians centered at 200.

    Args:
        angle_degrees (float): Angle in degrees where 0 is forward.

    Returns:
        int: Angle in gradians where 200 (Sonar.SONAR_CENTER_GRADIANS) is forward.
    """
    angle_gradians = angle_degrees * GRADIANS_PER_DEGREE
    angle_gradians_centered = angle_gradians + SONAR_CENTER_GRADIANS
    return int(angle_gradians_centered)
