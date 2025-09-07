import copy

from geometry_msgs.msg import Pose, Twist
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.logging import get_logger
from task_planning.interface.controls import Controls
from task_planning.interface.state import State
from task_planning.task import Task, Yield, task
from task_planning.utils import coroutine_utils, geometry_utils
from transforms3d.euler import euler2quat, quat2euler

logger = get_logger('move_tasks')

@task
async def move_to_pose_global(_self: Task, pose: Pose, pose_tolerances: Twist | None = None, timeout: int = 20) -> \
        Task[None, Pose | None, None]:
    """
    Move to a global pose in the "odom" frame.

    This asynchronous task moves the robot to the specified global pose within the "odom" frame.
    The function returns either when the robot reaches the given pose with zero velocity
    (within tolerance) or when the timeout is reached.

    Args:
        self (Task): The task instance managing the movement operation.
        pose (Pose): The target global pose to move to.
        pose_tolerances (Twist, optional): If this is not None, this task will end when the robot's pose has reached the
            desired pose within these tolerances.
        timeout (int, optional): The maximum number of seconds to attempt reaching the pose
                                 before timing out. Defaults to 20.

    Returns:
        Task[None, Pose | None, None]: Returns a task that sends the new global pose and
                                       returns the reached pose if successful, or None on timeout.

    Send:
        Pose: A new global pose to move to during the operation.
    """
    Controls().start_new_move()
    Controls().publish_desired_position(pose)
    start_time = Clock().now()
    while not geometry_utils.stopped_at_pose(State().state.pose.pose, pose, State().state.twist.twist,
                                             pose_tolerances=pose_tolerances):
        # Allow users of this task to update the pose
        new_pose = await Yield()
        if new_pose is not None:
            pose = new_pose

        Controls().publish_desired_position(pose)

        # Check if the timeout has been reached
        if (Clock().now() - start_time) > Duration(seconds=timeout):
            logger.warning('Move to pose timed out')
            return None


@task
async def move_to_pose_local(self: Task, pose: Pose, keep_orientation: bool = False, depth_level: float | None = None,
                             pose_tolerances: Twist | None = None, timeout: int = 30) -> \
                                Task[None, Pose | None, None]:
    """
    Move to a local pose in the "base_link" frame.

    The method moves the robot to the specified local pose and ensures it reaches the target
    with zero velocity within a small tolerance. The pose is transformed into the global
    frame for execution. Optionally, the robot can keep its level orientation during the movement.

    Args:
        self (Task): The task instance on which the method is called.
        pose (Pose): The local pose to move to, specified in the "base_link" frame.
        keep_orientation (bool, optional): If True, maintains the robot's orientation during movement. Defaults to
            False.
        depth_level (float, optional): The depth, as provided by the pressure sensor, the robot should move to. If this
            is not None, the Z value of the provided pose will be overridden. Defaults to None.
        pose_tolerances (Twist, optional): If this is not None, this task will end when the robot's pose has reached the
            desired pose within these tolerances.
        timeout (int, optional): The maximum number of seconds to attempt reaching the pose
            before timing out. Defaults to 30.

    Returns:
        Task[None, Pose | None, None]: A coroutine that completes when the robot reaches the target pose or the timeout
            expires.

    Send:
        Pose: A new local pose to move to.
    """
    def send_transformer(local_pose: Pose | None) -> Pose | None:
        if local_pose is None:
            return None

        if depth_level is not None:
            if local_pose.position.z != 0:
                logger.warning(f'Depth level of {depth_level} provided but Z value of pose is not zero: '
                               f'{local_pose.position.z}')
            depth_delta = depth_level - State().depth
            local_pose.position.z = depth_delta
        global_pose = geometry_utils.local_pose_to_global(State().tf_buffer, pose)

        if keep_orientation:
            orig_euler_angles = quat2euler(geometry_utils.geometry_quat_to_transforms3d_quat(
                State().orig_state.pose.pose.orientation))
            euler_angles = quat2euler(geometry_utils.geometry_quat_to_transforms3d_quat(global_pose.orientation))
            global_pose.orientation = geometry_utils.transforms3d_quat_to_geometry_quat(
                euler2quat(orig_euler_angles[0], orig_euler_angles[1], euler_angles[2]))

        return global_pose

    global_pose = send_transformer(pose)

    return await coroutine_utils.transform(
        move_to_pose_global(global_pose, pose_tolerances=pose_tolerances, timeout=timeout, parent=self),
            send_transformer=send_transformer)

# TODO: deprecate this, use correct_yaw from base_comp_task instead
@task
async def yaw_from_local_pose(self: Task, yaw: int, timeout: int = 30) -> None:
    """
    Yaw from current local position to some offset yaw

    Args:
        self (Task): The task instance on which the method is called.
        yaw (int): The amount to yaw, in radian.
        time_limit (int, optional): The time limit (in seconds) for reaching the pose. Defaults to 30.

    Returns:
        None
    """

    logger.info(f'Yawing {yaw} from current position')
    await move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, yaw),
            keep_orientation=True,
            time_limit = timeout,
            parent=self,
        )
    logger.info(f'Finished yaw')


@task
async def move_with_velocity(_self: Task, twist: Twist) -> Task[None, Twist | None, None]:
    """
    Move with a given velocity. Returns when the robot is moving with the given velocity.

    Args:
        twist: Desired velocity

    Send:
        New desired velocity to move with
    """
    Controls().start_new_move()
    Controls().publish_desired_velocity(twist)
    while not geometry_utils.at_vel(State().state.twist.twist, twist):
        new_twist = await Yield()
        if new_twist is not None:
            twist = new_twist

        Controls().publish_desired_velocity(twist)


@task
async def move_with_power_for_seconds(_self: Task, power: Twist, seconds: float) -> Task[None, Twist | None, None]:
    """
    Move with a given power for a given number of seconds. Returns when the time has elapsed.

    Args:
        power: Desired power
        seconds: Number of seconds to move with the given power

    Send:
        New desired power to move with
    """
    Controls().publish_desired_power(power)
    endtime = Clock().now() + seconds
    while (Clock().now() < endtime):
        new_power = await Yield()
        if new_power is not None:
            power = new_power

        Controls().publish_desired_power(power)


@task
async def hold_position(_self: Task) -> Task[bool, None, None]:
    """
    Hold the position and orientation the robot is at when this task is first run. Does not return.

    Yields:
        If the robot is at the pose it should be holding with zero velocity, within a small tolerance
    """
    pose_to_hold = copy.deepcopy(State().state.pose.pose)
    while True:
        await Yield(geometry_utils.stopped_at_pose(State().state.pose.pose, pose_to_hold, State().state.twist.twist))
        Controls().publish_desired_position(pose_to_hold)


@task
async def depth_correction(self: Task, desired_depth: float) -> Task[None, None, None]:
    """
    Perform depth correction to achieve the desired depth.

    This asynchronous task calculates the difference between the current depth and
    the desired depth, then moves the system to the target depth using a local pose adjustment.

    Args:
        self: Task instance.
        desired_depth (float): The target depth to which the system should move.

    Returns:
        Task[None, None, None]: An asynchronous task indicating the depth correction process.
    """
    logger.info(f'State().depth: {State().depth}')
    depth_delta = desired_depth - State().depth
    logger.info(f'depth_delta: {depth_delta}')

    logger.info(f'Started depth correction {depth_delta}')
    await move_to_pose_local(
        geometry_utils.create_pose(0, 0, depth_delta, 0, 0, 0),
        pose_tolerances=create_twist_tolerance(linear_z=0.17),
        timeout=15,
        parent=self)
    logger.info(f'Finished depth correction {depth_delta}')

@task
async def move_x(self: Task, step: float = 1.0) -> None:
    """
    Move the system along the X-axis.

    This asynchronous task moves the system by a specified step along the X-axis.

    Args:
        self: Task instance.
        step (float, optional): The distance to move along the X-axis. Defaults to 1.0.

    Returns:
        None
    """
    await move_to_pose_local(geometry_utils.create_pose(step, 0, 0, 0, 0, 0),
        keep_orientation=True,
        timeout=10,
        pose_tolerances=create_twist_tolerance(linear_x=0.15),
        parent=self)
    logger.info(f'Moved x {step}')

@task
async def move_y(self: Task, step: float = 1.0) -> None:
    """
    Move the system along the Y-axis.

    This asynchronous task moves the system by a specified step along the Y-axis.

    Args:
        self: Task instance.
        step (float, optional): The distance to move along the Y-axis. Defaults to 1.0.

    Returns:
        None
    """
    await move_to_pose_local(geometry_utils.create_pose(0, step, 0, 0, 0, 0), parent=self)
    logger.info(f'Moved y {step}')


Direction = tuple[float, float, float] | tuple[float, float, float, float, float, float]
Directions = list[Direction]


@task
async def move_with_directions(self: Task,
                               directions: Directions,
                               depth_level: float | None = None,
                               yaw_heading: float | None = None,
                               correct_yaw: bool = False,
                               correct_depth: bool = False,
                               keep_orientation: bool = False,
                               timeout: int = 30,
                               ) -> None:
    """
    Move the robot to multiple poses defined by the provided directions.

    This method iterates over a list of directions, moving the robot to each specified pose in local coordinates.
    Each direction must be a tuple of length 3 or 6. Optionally, it can correct the yaw and/or depth after each
    movement.

    Args:
        self: Task instance.
        directions (Directions): A list of tuples, where each tuple specifies the target pose.
            - Tuples of length 3 represent (x, y, z).
            - Tuples of length 6 represent (x, y, z, roll, pitch, yaw).
        correct_yaw (bool, optional): If True, corrects the yaw after moving to a pose. Defaults to False.
        correct_depth (bool, optional): If True, corrects the depth after moving to a pose. Defaults to False.
        keep_orientation (bool, optional): If True, corrects orientation after moving to a pose. Defaults to False.
        timeout (int, optional): The maximum number of seconds to attempt reaching the pose
            before timing out. Defaults to 30.

    Raises:
        ValueError: If a direction tuple in the list is not of length 3 or 6.

    Returns:
        None
    """
    for direction in directions:
        assert len(direction) in [3, 6], 'Each tuple in the directions list must be of length 3 or 6. Tuple '
        f'{direction} has length {len(direction)}.'
        logger.info(f'Starting move to {direction}')
        orig_gyro = State().gyro_euler_angles.z
        await move_to_pose_local(
            geometry_utils.create_pose(direction[0], direction[1], direction[2], 0, 0, 0),
            keep_orientation=keep_orientation,
            depth_level=depth_level,
            pose_tolerances=create_twist_tolerance(linear_x=0.1, linear_y=0.07, linear_z=0.07),
            timeout=timeout,
            parent=self)
        logger.info(f'Moved to {direction}')

        if correct_yaw:
            logger.info(f'Correcting yaw {orig_gyro - State().gyro_euler_angles.z}')
            await move_to_pose_local(geometry_utils.create_pose(0,0,0,0,0,orig_gyro - State().gyro_euler_angles.z),
                                     timeout=timeout,
                                     parent=self)
        if correct_depth:
            await depth_correction(depth_level, parent=self)


def create_twist_tolerance(
    linear_x: float = 0.05,
    linear_y: float = 0.05,
    linear_z: float = 0.05,
    angular_roll: float = 0.2,
    angular_pitch: float = 0.3,
    angular_yaw: float = 0.1,
) -> Twist:
    """
    Create a Twist message to represent pose or velocity tolerances.

    Args:
        linear_x (float): Tolerance in X (forward/backward)
        linear_y (float): Tolerance in Y (left/right)
        linear_z (float): Tolerance in Z (up/down)
        angular_roll (float): Tolerance in roll (rotation around X)
        angular_pitch (float): Tolerance in pitch (rotation around Y)
        angular_yaw (float): Tolerance in yaw (rotation around Z)

    Returns:
        Twist: A Twist message with specified tolerances.
    """
    return geometry_utils.create_twist(x=linear_x, y=linear_y, z=linear_z,
                                       roll=angular_roll, pitch=angular_pitch, yaw=angular_yaw)

