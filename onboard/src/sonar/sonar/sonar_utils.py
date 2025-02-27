from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
import tf2_geometry_msgs

SONAR_CENTER_GRADIANS = 200
RADIANS_PER_GRADIAN = np.pi / 200
GRADIANS_PER_DEGREE = 400 / 360


def transform_pose(buffer, pose, source_frame, target_frame):
    """ Transforms pose from base reference frame to target reference frame.

    Args:
        buffer (tf2_ros.Buffer): The tf2 buffer containing transform data.
        pose (Pose): Pose in the source reference frame.
        source_frame (str): Name of the source frame (e.g., 'sonar_link').
        target_frame (str): Name of the target frame (e.g., 'cameras_link').

    Returns:
        Pose: Pose in the target reference frame.
    """
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = source_frame
    pose_stamped.header.stamp = rclpy.time.Time().to_msg()  # Correct ROS2 timestamping

    try:
        # Wait for transform (Use async future in ROS2)
        transform = buffer.lookup_transform(
            target_frame, source_frame, rclpy.time.Time()
        )

        # Transform the pose
        transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
        return transformed_pose.pose

    except Exception as e:
        raise RuntimeError(f"Transform failed: {e}")


def centered_gradians_to_radians(angle_gradians):
    """ Converts gradians centered at 200 to radians centered at 0

    Args:
        angle_gradians (float): Angle in gradians where 200
                (Sonar.SONAR_CENTER_GRADIANS) is forward

    Returns:
        float: Angle in radians
    """
    angle_radians = (angle_gradians -
                     SONAR_CENTER_GRADIANS) * RADIANS_PER_GRADIAN
    return angle_radians


def degrees_to_centered_gradians(angle_degrees):
    """ Converts degrees centered at 0 to gradians centered at 200

    Args:
        angle_degrees (float): Angle in degrees where 0 is forward

    Returns:
        int: Angle in gradians where 200 (Sonar.SONAR_CENTER_GRADIANS)
             is forward
    """

    angle_gradians = angle_degrees * GRADIANS_PER_DEGREE
    angle_gradians_centered = angle_gradians + SONAR_CENTER_GRADIANS
    return int(angle_gradians_centered)
