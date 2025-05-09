import numpy as np
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from rclpy.time import Time
from transforms3d.euler import euler2quat, quat2euler
from transforms3d.quaternions import qmult


def vector3_to_numpy(vector: Vector3) -> np.ndarray:
    """
    Convert a Vector3 to a numpy array.

    Args:
        vector: The Vector3 to convert.

    Returns:
        The numpy array. The order of the elements is [x, y, z].
    """
    return np.array([vector.x, vector.y, vector.z])


def point_to_numpy(point: Point) -> np.ndarray:
    """
    Convert a Point to a numpy array.

    Args:
        point: The Point to convert.

    Returns:
        The numpy array. The order of the elements is [x, y, z].
    """
    return np.array([point.x, point.y, point.z])


def transforms3d_quat_to_geometry_quat(quat: np.ndarray) -> Quaternion:
    """
    Convert a quaternion from the transforms3d library to a geometry_msgs/Quaternion.

    Args:
        quat: The transforms3d quaternion to convert.

    Returns:
        The converted geometry_msgs/Quaternion.
    """
    return Quaternion(x=quat[1], y=quat[2], z=quat[3], w=quat[0])


def geometry_quat_to_transforms3d_quat(quat: Quaternion) -> np.ndarray:
    """
    Convert a quaternion from a geometry_msgs/Quaternion to the format used by the transforms3d library.

    Args:
        quat: The geometry_msgs/Quaternion to convert.

    Returns:
        The converted quaternion in the format used by the transforms3d library.
    """
    return np.array([quat.w, quat.x, quat.y, quat.z])


def point_linear_distance(point1: Point, point2: Point) -> float:
    """
    Find the linear distance between two points in 3D space.

    Args:
        point1: The first point.
        point2: The second point.

    Returns:
        The linear (Euclidean) distance between point1 and point2.
    """
    return np.linalg.norm(point_to_numpy(point1) - point_to_numpy(point2))


def geometry_quat_to_euler_angles(quat: Quaternion) -> Vector3:
    """
    Convert a geometry_msgs/Quaternion to Euler angles (roll, pitch, yaw).

    Args:
        quat: The Quaternion to convert.

    Returns:
        A Vector3 containing the Euler angles in radians, in the order of (roll, pitch, yaw).
    """
    transforms3d_quat = geometry_quat_to_transforms3d_quat(quat)
    euler_angles = quat2euler(transforms3d_quat)
    return Vector3(x=euler_angles[0], y=euler_angles[1], z=euler_angles[2])

def vector3_linear_distance(vector1: Vector3, vector2: Vector3) -> float:
    """
    Find the linear distance between two vectors in 3D space.

    Args:
        vector1: The first vector.
        vector2: The second vector.

    Returns:
        The linear (Euclidean) distance between vector1 and vector2.
    """
    return np.linalg.norm(vector3_to_numpy(vector1) - vector3_to_numpy(vector2))

def vector_between_points(point1: Point, point2: Point) -> Vector3:
    """
    Obtain a vector that points from point2 to point1.

    Args:
        point1: The first point.
        point2: The second point.

    Returns:
        Vector3: The vector pointing from point2 to point1. Equivalent to point1 - point2.
    """
    return Vector3(x=point1.x - point2.x, y=point1.y - point2.y, z=point1.z - point2.z)

def angular_distance_quat(quat1: Quaternion, quat2: Quaternion) -> Vector3:
    """
    Find the difference between two orientations (quaternions).

    Args:
        quat1: The first orientation.
        quat2: The second orientation.

    Returns:
        The magnitude of the two orientations' differences in each axis.
    """
    rpy1 = quat2euler(geometry_quat_to_transforms3d_quat(quat1))
    rpy2 = quat2euler(geometry_quat_to_transforms3d_quat(quat2))
    return angular_distance_rpy(rpy1, rpy2)


def angular_distance_rpy(rpy1: tuple[float, float, float], rpy2: tuple[float, float, float]) -> Vector3:
    """
    Find the difference between two orientations, provided as (roll, pitch, yaw).

    All arguments must be in radians, or they all must be in degrees.
    The return value will be in the same units as the input.

    Args:
        rpy1: The first orientation with three values, in order: roll, pitch, yaw.
        rpy2: The second orientation with three values, in order: roll, pitch, yaw.

    Returns:
        The magnitude of the differences between the two orientations in each axis, in the same units as the input.
    """
    roll = np.fabs(rpy1[0] - rpy2[0])
    pitch = np.fabs(rpy1[1] - rpy2[1])
    yaw = np.fabs(rpy1[2] - rpy2[2])
    return Vector3(x=roll, y=pitch, z=yaw)


def at_pose(current_pose: Pose, desired_pose: Pose, x_tol: float = 0.05, y_tol: float = 0.05, z_tol: float = 0.05,
        roll_tol: float = 0.2, pitch_tol: float = 0.3, yaw_tol: float = 0.10) -> bool:
    """
    Check if current pose is within tolerance of a desired pose (position and orientation).

    Args:
        current_pose: The current pose.
        desired_pose: The desired pose.
        x_tol: The allowable linear distance in meters in the X axis for the robot to be considered at the desired pose.
        y_tol: The allowable linear distance in meters in the Y axis for the robot to be considered at the desired pose.
        z_tol: The allowable linear distance in meters in the Z axis for the robot to be considered at the desired pose.
        roll_tol: The allowable angular distance in radians in the roll axis for the robot to be considered at the
        desired pose.
        pitch_tol: The allowable angular distance in radians in the pitch axis for the robot to be considered at the
        desired pose.
        yaw_tol: The allowable angular distance in radians in the yaw axis for the robot to be considered at the
        desired pose.

    Returns:
        True if the current pose is within the tolerances of the desired pose.
    """
    linear_dist = vector_between_points(current_pose.position, desired_pose.position)
    linear = all((abs(linear_dist.x) < x_tol, abs(linear_dist.y) < y_tol, abs(linear_dist.z) < z_tol))

    angular_dist = angular_distance_quat(current_pose.orientation, desired_pose.orientation)
    angular = all((abs(angular_dist.x) < roll_tol, abs(angular_dist.y) < pitch_tol, abs(angular_dist.z) < yaw_tol))

    return linear and angular


def at_vel(current_twist: Twist, desired_twist: Twist, linear_tol: float = 0.02, angular_tol: float = 0.02) -> bool:
    """
    Check if current twist is within tolerance of a desired twist (linear and angular velocities).

    Args:
        current_twist: The current twist.
        desired_twist: The desired twist.
        linear_tol: The allowable distance between the current and desired linear velocity vectors in m/s for the robot
        to be considered at the desired velocity.
        angular_tol: The allowable difference between the current and desired angular velocity vectors in rad/s for the
        robot to be considered at the desired velocity.

    Returns:
        True if the current twist is within the tolerances of the desired twist.
    """
    linear = vector3_linear_distance(current_twist.linear, desired_twist.linear) < linear_tol
    angular = vector3_linear_distance(current_twist.angular, desired_twist.angular) < angular_tol
    return linear and angular


def stopped_at_pose(current_pose: Pose, desired_pose: Pose, current_twist: Twist,
                    pose_tolerances: Twist | None = None) -> bool:
    """
    Check if the robot is at the desired pose and has stopped moving.

    Args:
        current_pose: The current pose.
        desired_pose: The desired pose.
        current_twist: The current twist.
        pose_tolerances: Tolerances to use if current_pose is at desired_pose. If None, default values will be used.

    Returns:
        True if the robot is at the desired pose and has stopped moving, within the default tolerances.
    """
    if pose_tolerances:
        at_desired_pose = at_pose(current_pose, desired_pose, x_tol=pose_tolerances.linear.x,
                                  y_tol=pose_tolerances.linear.y, z_tol=pose_tolerances.linear.z,
                                  roll_tol=pose_tolerances.angular.x, pitch_tol=pose_tolerances.angular.y,
                                  yaw_tol=pose_tolerances.angular.z)
    else:
        at_desired_pose = at_pose(current_pose, desired_pose)
    at_desired_vel = at_vel(current_twist, Twist())

    return at_desired_pose and at_desired_vel


def transform_pose(tf_buffer: tf2_ros.Buffer, base_frame: str, target_frame: str, pose: Pose) -> Pose:
    """
    Transform a pose from one frame to another. Uses the latest transform available at the time of the call.

    Args:
        tf_buffer: The transform buffer.
        base_frame: The frame of the input pose.
        target_frame: The frame to transform the pose to.
        pose: The pose to transform.

    Returns:
        The transformed pose.
    """
    trans = tf_buffer.lookup_transform(target_frame, base_frame, Time())
    return tf2_geometry_msgs.do_transform_pose(pose, trans)


def add_poses(pose_list: list[Pose]) -> Pose:
    """
    Add a list of poses together. Sums the positions and multiplies the orientations.

    Args:
        pose_list: A list of poses to add together.

    Returns:
        The sum of the poses.
    """
    p_sum = Point(0, 0, 0)
    q_sum = np.array([1, 0, 0, 0])

    for pose in pose_list:
        p_sum.x += pose.position.x
        p_sum.y += pose.position.y
        p_sum.z += pose.position.z
        q_sum = qmult(geometry_quat_to_transforms3d_quat(pose.orientation), q_sum)

    pose_sum = Pose()
    pose_sum.position = p_sum
    pose_sum.orientation = transforms3d_quat_to_geometry_quat(q_sum)
    return pose_sum


def parse_pose(pose: Pose) -> dict:
    """
    Convert a Pose message to a dictionary.

    Args:
        pose: The Pose message to convert.

    Returns:
        A dictionary with the position and orientation of the pose. It has the keys 'x', 'y', 'z', 'roll', 'pitch', and
        'yaw'. Roll, pitch, and yaw are in radians.
    """
    pose_dict = {'x': pose.position.x, 'y': pose.position.y, 'z': pose.position.z}
    pose_dict['roll'], pose_dict['pitch'], pose_dict['yaw'] = \
        quat2euler(geometry_quat_to_transforms3d_quat(pose.orientation))
    return pose_dict


def create_pose(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> Pose:
    """
    Create a Pose message from position and orientation values.

    Args:
        x: The x position.
        y: The y position.
        z: The z position.
        roll: The roll orientation in radians.
        pitch: The pitch orientation in radians.
        yaw: The yaw orientation in radians.

    Returns:
        The Pose message.
    """
    pose = Pose()
    pose.position = Point(x=x, y=y, z=z)
    pose.orientation = transforms3d_quat_to_geometry_quat(euler2quat(roll, pitch, yaw))
    return pose


def local_pose_to_global(tf_buffer: tf2_ros.Buffer, pose: Pose) -> Pose:
    """
    Convert a pose from local coordinates to global coordinates.

    Args:
        tf_buffer: The transform buffer.
        pose: The pose to convert, in "base_link" frame.

    Returns:
        The global pose in "odom" frame.
    """
    return transform_pose(tf_buffer, 'base_link', 'odom', pose)


def global_pose_to_local(tf_buffer: tf2_ros.Buffer, pose: Pose) -> Pose:
    """
    Convert a pose from global coordinates to local coordinates.

    Args:
        tf_buffer: The transform buffer.
        pose: The pose to convert, in "odom" frame.

    Returns:
        The local pose in "base_link" frame.
    """
    return transform_pose(tf_buffer, 'odom', 'base_link', pose)
