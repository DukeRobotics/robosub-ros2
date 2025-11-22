import numpy as np
import rclpy
import tf2_geometry_msgs
import tf2_ros

RADIANS_PER_GRADIAN = np.pi / 200
GRADIANS_PER_DEGREE = 400 / 360

def transform_pose(buffer: tf2_ros.Buffer, pose: tf2_geometry_msgs.PoseStamped,
                   source_frame_id: str, target_frame_id: str) -> tf2_geometry_msgs.PoseStamped:
    """
    Transform pose from source reference frame to target reference frame.

    Args:
        buffer (tf2_ros.Buffer): The tf2 buffer containing transform data.
        pose (Pose): Pose in the source reference frame.
        source_frame_id (str): Name of the source frame (e.g., 'sonar_ping_360').
        target_frame_id (str): Name of the target frame (e.g., 'camera_depthai_front').

    Returns:
        Pose: Pose in the target reference frame.
    """
    try:
        # Wait for transform to be available
        transform = buffer.lookup_transform(
            target_frame_id, source_frame_id, rclpy.time.Time(),
        )

        # Transform the pose
        return tf2_geometry_msgs.do_transform_pose(pose, transform)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException,
            tf2_ros.InvalidArgumentException) as e:
        error_message = f'Failed to transform pose: {e}'
        raise RuntimeError(error_message) from e


def centered_gradians_to_radians(angle_gradians: float, center_gradians: float, increase_ccw: bool) -> float:
    """
    Convert gradians centered at center_gradians to radians centered at 0.

    Args:
        angle_gradians (float): Angle in gradians where center_gradians is forward.
        center_gradians (float): Center gradians to use for conversion.
        increase_ccw (bool): If True, sonar angles increase counter-clockwise when viewed from above the robot.

    Returns:
        float: Angle in radians where 0 is forward.
    """
    return (angle_gradians - center_gradians) * RADIANS_PER_GRADIAN * (1 if increase_ccw else -1)


def degrees_to_centered_gradians(angle_degrees: float, center_gradians: float, increase_ccw: bool) -> int:
    """
    Convert degrees centered at 0 to gradians centered at center_gradians.

    Args:
        angle_degrees (float): Angle in degrees where 0 is forward.
        center_gradians (float): Center gradians to use for conversion.
        increase_ccw (bool): If True, sonar angles increase counter-clockwise when viewed from above the robot.

    Returns:
        int: Angle in gradians where center_gradians is forward.
    """
    angle_gradians = angle_degrees * GRADIANS_PER_DEGREE * (1 if increase_ccw else -1)
    angle_gradians_centered = angle_gradians + center_gradians
    return int(angle_gradians_centered)

def polar2cart(sonar_sweep: np.ndarray, deg_low: int, deg_high: int) -> np.ndarray:
    """
    Convert a polar sonar sweep to a Cartesian grid.

    Args:
        sonar_sweep (np.ndarray): Polar sonar sweep data.
        deg_low (int): Lower bound of the angle in degrees.
        deg_high (int): Upper bound of the angle in degrees.

    Returns:
        np.ndarray: Cartesian grid representation of the sonar sweep.
    """
    num_angles, num_samples = sonar_sweep.shape
    print(num_angles, num_samples)
    grid_size = 2 * (num_samples - 1) + 1
    center = num_samples
    cartesian_grid = np.zeros((grid_size, grid_size), dtype=float)

    angles = np.linspace(deg_low, deg_high, num_angles, endpoint=True)

    for i, angle_deg in enumerate(angles):
        angle_rad = angle_deg * np.pi / 200.0
        for j in range(num_samples):
            distance = j
            x = round(center + distance * np.cos(angle_rad)) # negative bcs idk why lol
            y = round(center + distance * np.sin(angle_rad))
            val = sonar_sweep[i, j]
            intensity = val
            if 0 <= x < grid_size and 0 <= y < grid_size:
                cartesian_grid[y, x] = max(cartesian_grid[y, x], intensity)

    return cartesian_grid
