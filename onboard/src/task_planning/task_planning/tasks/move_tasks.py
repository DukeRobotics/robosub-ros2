import copy
import math
from collections.abc import Callable
from typing import cast

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


# EXPERIMENTAL: keep_depth
def _local_move_to_global_pose(local_pose: Pose, keep_orientation: bool = False,
                               depth_level: float | None = None) -> Pose:
    """
    Convert a local ("base_link") pose to a global ("odom") pose, applying depth and orientation overrides.

    This mirrors the transform previously inlined in move_to_pose_local's send_transformer. It is a module-level
    function so it can be reused as a per-tick recompute callback for the keep_depth feature.

    Args:
        local_pose (Pose): The local pose to convert, in "base_link" frame.
        keep_orientation (bool, optional): If True, override the orientation to keep the robot's original roll and pitch
            (level) while preserving the target yaw. Defaults to False.
        depth_level (float, optional): The depth, as provided by the pressure sensor, the robot should move to. If this
            is not None, the Z value of the resulting global pose is adjusted so the robot reaches this depth. Defaults
            to None.

    Returns:
        Pose: The converted global pose in "odom" frame.
    """
    global_pose = geometry_utils.local_pose_to_global(State().tf_buffer, local_pose)

    if depth_level is not None:
        if local_pose.position.z != 0:
            logger.warning(f'Depth level of {depth_level} provided but Z value of pose is not zero: '
                           f'{local_pose.position.z}')
        depth_delta = depth_level - State().depth
        global_pose.position.z += depth_delta

    if keep_orientation:
        orig_euler_angles = quat2euler(geometry_utils.geometry_quat_to_transforms3d_quat(
            State().orig_state.pose.pose.orientation))
        euler_angles = quat2euler(geometry_utils.geometry_quat_to_transforms3d_quat(global_pose.orientation))
        global_pose.orientation = geometry_utils.transforms3d_quat_to_geometry_quat(
            euler2quat(orig_euler_angles[0], orig_euler_angles[1], euler_angles[2]))

    return global_pose


@task
async def move_to_pose_global(_self: Task, pose: Pose, pose_tolerances: Twist | None = None, timeout: int = 20,
                              recompute_pose: Callable[[], Pose] | None = None) -> \
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
        recompute_pose (Callable[[], Pose], optional): EXPERIMENTAL (keep_depth). If provided, this callback is invoked
            each loop iteration (when no new pose is sent) to recompute the desired global pose. Used to continuously
            refresh the depth setpoint from live pressure readings. Defaults to None.

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
        # EXPERIMENTAL: keep_depth - refresh the desired pose (e.g. depth setpoint) from live state each tick
        elif recompute_pose is not None:
            pose = recompute_pose()

        Controls().publish_desired_position(pose)

        # Check if the timeout has been reached
        if (Clock().now() - start_time) > Duration(seconds=timeout):
            logger.warning('Move to pose timed out')
            return None


@task
async def move_to_pose_local(self: Task, pose: Pose, keep_orientation: bool = False, depth_level: float | None = None,
                             pose_tolerances: Twist | None = None, timeout: int = 15,
                             keep_depth: bool = False) -> \
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
        keep_depth (bool, optional): EXPERIMENTAL. If True (and depth_level is set), the desired global Z is recomputed
            from the live pressure reading each control tick, so the robot holds depth_level throughout the move rather
            than only compensating at the start. Requires depth_level; falls back to static behavior with a warning if
            depth_level is None. Defaults to False.

    Returns:
        Task[None, Pose | None, None]: A coroutine that completes when the robot reaches the target pose or the timeout
            expires.

    Send:
        Pose: A new local pose to move to.
    """
    euler = geometry_utils.geometry_quat_to_euler_angles(pose.orientation)
    logger.info(
        f'Moving to pose local: pos=({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f}), '
        f'rpy=({euler.x:.2f}, {euler.y:.2f}, {euler.z:.2f})',
    )
    def send_transformer(local_pose: Pose | None) -> Pose | None:
        if local_pose is None:
            return None

        return _local_move_to_global_pose(local_pose, keep_orientation=keep_orientation, depth_level=depth_level)

    # EXPERIMENTAL: keep_depth - hold the horizontal target fixed while continuously refreshing the depth setpoint
    if keep_depth:
        if depth_level is None:
            logger.warning('keep_depth is True but depth_level is None; falling back to static depth behavior')
        else:
            # Compute the horizontal/orientation target once, fixed in odom (same as a normal move). Only the depth
            # (Z) is refreshed each tick so the robot converges on depth_level without the x/y target sliding forward.
            base_global_pose = send_transformer(pose)

            def recompute_pose() -> Pose:
                # Command the robot's current odom Z plus the live pressure error, so the Z setpoint tracks depth_level
                # while x/y and orientation stay pinned to base_global_pose.
                updated_pose = copy.deepcopy(base_global_pose)
                updated_pose.position.z = State().state.pose.pose.position.z + (depth_level - State().depth)
                return updated_pose

            return await move_to_pose_global(base_global_pose, pose_tolerances=pose_tolerances, timeout=timeout,
                                             recompute_pose=recompute_pose, parent=self)
    global_pose = send_transformer(pose)

    return await coroutine_utils.transform(
        move_to_pose_global(global_pose, pose_tolerances=pose_tolerances, timeout=timeout, parent=self),
            send_transformer=send_transformer)


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
        geometry_utils.create_pose(0, 0, 0, 0, 0, 0),
        depth_level=desired_depth,
        pose_tolerances=create_twist_tolerance(linear_z=0.1),
        timeout=15,
        parent=self)
    logger.info(f'Finished depth correction {depth_delta}')


@task
async def move_x(self: Task, step: float = 1.0, depth_level: float | None = None, keep_depth: bool = False) -> None:
    """
    Move the system along the X-axis.

    This asynchronous task moves the system by a specified step along the X-axis.

    Args:
        self: Task instance.
        step (float, optional): The distance to move along the X-axis. Defaults to 1.0.
        depth_level (float, optional): The depth, as provided by the pressure sensor, the robot should hold during the
            move. If this is not None, the Z value of the pose is overridden. Defaults to None.
        keep_depth (bool, optional): EXPERIMENTAL. If True (and depth_level is set), hold depth_level continuously
            throughout the move. Defaults to False.

    Returns:
        None
    """
    await move_to_pose_local(geometry_utils.create_pose(step, 0, 0, 0, 0, 0),
        keep_orientation=True,
        depth_level=depth_level,
        keep_depth=keep_depth,
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
# A direction entry may optionally be paired with a per-leg timeout override, e.g. ((1, 0, 0), 5).
# This is unambiguous because a bare Direction is always a flat tuple of floats, never a tuple of tuples.
DirectionSpec = Direction | tuple[Direction, int]
Directions = list[DirectionSpec]

# Maps a maximum leg distance (in meters) to the timeout (in seconds) that should be used for legs up to that
# distance. Keys need not be sorted when the dict is created; lookup handles that. Example:
#   {1: 8, 3: 15, 6: 25}
# means: legs of distance <=1m get 8s, <=3m (but >1m) get 15s, <=6m (but >3m) get 25s, and anything longer than
# 6m also gets 25s (the timeout for the largest threshold).
DistanceTimeouts = dict[float, int]


def _leg_distance(direction: Direction) -> float:
    """Compute the straight-line distance (ignoring orientation) that a leg's direction tuple travels."""
    return math.sqrt(direction[0] ** 2 + direction[1] ** 2 + direction[2] ** 2)


def _timeout_for_distance(distance: float, distance_timeouts: DistanceTimeouts | None, default_timeout: int) -> int:
    """Look up the timeout for a leg of the given distance in distance_timeouts, falling back to default_timeout."""
    if not distance_timeouts:
        return default_timeout
    for max_distance in sorted(distance_timeouts):
        if distance <= max_distance:
            return distance_timeouts[max_distance]
    # Distance exceeds every threshold; use the timeout for the largest one.
    return distance_timeouts[max(distance_timeouts)]


def _resolve_leg(direction_spec: DirectionSpec, default_timeout: int,
                 distance_timeouts: DistanceTimeouts | None) -> tuple[Direction, int]:
    """
    Resolve a directions-list entry into its (direction, timeout) parts.

    Precedence, highest to lowest:
        1. An explicit per-leg timeout override (e.g. ((1, 0, 0), 5)).
        2. A lookup in distance_timeouts based on the leg's travel distance.
        3. default_timeout.
    """
    if len(direction_spec) == 2 and isinstance(direction_spec[0], tuple):
        direction, leg_timeout = direction_spec
        return direction, leg_timeout

    direction = cast('Direction', direction_spec)
    return direction, _timeout_for_distance(_leg_distance(direction), distance_timeouts, default_timeout)


@task
async def move_with_directions(self: Task,
                               directions: Directions,
                               depth_level: float | None = None,
                               correct_yaw: bool = False,
                               correct_depth: bool = False,
                               keep_orientation: bool = False,
                               timeout: int = 30,
                               distance_timeouts: DistanceTimeouts | None = None,
                               keep_depth: bool = False,
                               pose_tolerances: Twist | None = None,
                               ) -> None:
    """
    Move the robot to multiple poses defined by the provided directions.

    This method iterates over a list of directions, moving the robot to each specified pose in local coordinates.
    Each direction must be a tuple of length 3 or 6. Optionally, it can correct the yaw and/or depth after each
    movement.

    Args:
        self: Task instance.
        directions (Directions): A list of entries, where each entry specifies the target pose for a leg of the
            move, and optionally a per-leg timeout override.
            - Tuples of length 3 represent (x, y, z).
            - Tuples of length 6 represent (x, y, z, roll, pitch, yaw).
            - Either of the above may instead be wrapped as (direction, leg_timeout) to force an exact timeout for
                just that leg, taking priority over distance_timeouts and timeout. For example:
                [(1, 0, 0), ((5, 0, 0), 25), (0, 1, 0)]
                moves 1m using distance_timeouts/timeout as usual, then 5m with an exact 25s timeout, then 1m
                using distance_timeouts/timeout again.
        depth_level (float, optional): The depth, as provided by the pressure sensor, the robot should move to. If this
            is not None, the Z value of the provided pose will be overridden. Defaults to None.
        correct_yaw (bool, optional): If True, corrects the yaw after moving to a pose. Defaults to False.
        correct_depth (bool, optional): If True, corrects the depth after moving to a pose. Defaults to False.
        keep_orientation (bool, optional): If True, corrects orientation after moving to a pose. Defaults to False.
        timeout (int, optional): The default maximum number of seconds to attempt reaching each leg's pose before
            timing out, used when a leg's timeout isn't otherwise determined by distance_timeouts or a per-leg
            override. Defaults to 30.
        distance_timeouts (DistanceTimeouts, optional): A dict mapping a maximum leg distance (in meters) to the
            timeout (in seconds) to use for legs up to that distance, letting you pass plain distance tuples and
            have the appropriate timeout picked automatically based on how far each leg travels. For example,
            {1: 8, 3: 15, 6: 25} gives legs of distance <=1m a timeout of 8s, legs >1m and <=3m a timeout of 15s,
            legs >3m and <=6m a timeout of 25s, and any leg longer than 6m also gets 25s (the largest threshold's
            timeout). Ignored for legs with an explicit per-leg "timeout" override. Defaults to None (use
            `timeout` for every leg).
        keep_depth (bool, optional): EXPERIMENTAL. If True (and depth_level is set), hold depth_level continuously
            throughout each leg of the move. Defaults to False.
        pose_tolerances (Twist, optional): The pose tolerances used to determine when each leg has arrived. If None, a
            default tolerance of create_twist_tolerance(linear_x=0.1, linear_y=0.07, linear_z=0.07, angular_yaw=0.05)
            is used. Defaults to None.

    Raises:
        ValueError: If a direction tuple in the list is not of length 3 or 6.

    Returns:
        None.
    """
    leg_pose_tolerances = pose_tolerances if pose_tolerances is not None else \
        create_twist_tolerance(linear_x=0.1, linear_y=0.07, linear_z=0.07, angular_yaw=0.05)

    for direction_spec in directions:
        direction, leg_timeout = _resolve_leg(direction_spec, timeout, distance_timeouts)
        assert len(direction) in [3, 6], 'Each tuple in the directions list must be of length 3 or 6. Tuple '
        f'{direction} has length {len(direction)}.'
        logger.info(f'Starting move to {direction} (timeout={leg_timeout})')
        orig_gyro = State().gyro_euler_angles.z
        await move_to_pose_local(
            geometry_utils.create_pose(direction[0], direction[1], direction[2], 0, 0, 0),
            keep_orientation=keep_orientation,
            depth_level=depth_level,
            keep_depth=keep_depth,
            pose_tolerances=leg_pose_tolerances,
            timeout=leg_timeout,
            parent=self)
        logger.info(f'Moved to {direction}')

        if correct_yaw:
            logger.info(f'Correcting yaw {orig_gyro - State().gyro_euler_angles.z}')
            await move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, orig_gyro - State().gyro_euler_angles.z),
                                     timeout=leg_timeout,
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

