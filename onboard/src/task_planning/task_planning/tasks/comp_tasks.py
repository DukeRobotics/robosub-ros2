# ruff: noqa

import copy
import math
from collections.abc import Coroutine
from enum import Enum

import numpy as np
from custom_msgs.msg import ControlTypes
from geometry_msgs.msg import Twist
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.logging import get_logger
from transforms3d.euler import quat2euler

from task_planning.interface.controls import Controls
from task_planning.interface.cv import CV, CVObjectType
from task_planning.interface.marker_dropper import MarkerDropper, MarkerDropperStates
from task_planning.interface.state import State
from task_planning.task import Task, Yield, task
from task_planning.tasks import cv_tasks, move_tasks
from task_planning.utils import geometry_utils
from task_planning.utils.coroutine_utils import sleep

# TODO: move stablize() to move_tasks.py
#
# TODO: see if we can remove sleep() since we already have sleep() in coroutine_utils.py
#
# TODO: create a common skeleton @task class/interface with all the common functions to remove redundancy:
# - move_x
# - move_y
# - move_z
# - correct_x
# - correct_y
# - correct_z
# - correct_yaw
# - correct_roll_and_pitch
# - get_yaw_correction
# - ...
# These implementations can be overridden by the tasks that uses that interface.
#
# TODO: look into creating common higher level routines:
# - yaw_to_cv_object
# - spiral search (e.g. spiral_bin_search)
# - logarithmic search (stretch)
# - track and align with object center for bottom camera (e.g. search_for_bins & center_path_marker)
# - track and move toward CV object (e.g. move_to_pink_bins & move_to_buoy)
#     - takes in the termination condition function as a parameter
#     - can improve on cv_tasks.move_to_cv_obj implementation (or replace it completely)

logger = get_logger('comp_tasks')

RECT_HEIGHT_METERS = 0.3048


@task
async def gate_style_task(self: Task, depth_level=0.9) -> Task[None, None, None]:
    """
    Complete two full barrel rolls.
    """
    logger.info('Started gate style task')

    DEPTH_LEVEL = State().orig_depth - depth_level

    async def sleep(secs):
        duration = Duration(seconds=secs)
        start_time = Clock().now()
        while start_time + duration > Clock().now():
            await Yield()

    async def roll():
        power = Twist()
        power.angular.x = 1
        Controls().publish_desired_power(power)
        logger.info('Published roll power')

        await sleep(2.25)

        logger.info('Completed roll')

        Controls().publish_desired_power(Twist())
        logger.info('Published zero power')

        await sleep(2)

        logger.info('Completed zero')

    await move_tasks.depth_correction(DEPTH_LEVEL, parent=self)
    await roll()
    State().reset_pose()
    await roll()
    State().reset_pose()
    await sleep(3)
    await move_tasks.depth_correction(DEPTH_LEVEL, parent=self)
    await sleep(3)

    imu_orientation = State().imu.orientation
    euler_angles = quat2euler([imu_orientation.w, imu_orientation.x, imu_orientation.y, imu_orientation.z])
    roll_correction = -euler_angles[0]
    pitch_correction = -euler_angles[1]

    logger.info(f'Roll, pitch correction: {roll_correction, pitch_correction}')
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, roll_correction, pitch_correction, 0),
                                        parent=self)
    State().reset_pose()
    logger.info('Reset orientation')


@task
async def buoy_task(self: Task, turn_to_face_buoy: bool = False, depth: float = 0.7) -> Task[None, None, None]:
    """Circumnavigate the buoy. Requires robot to have submerged 0.5 meters."""
    logger.info('Starting buoy task')

    DEPTH_LEVEL = State().orig_depth - depth

    async def correct_y() -> Coroutine[None, None, None]:
        await cv_tasks.correct_y('buoy', parent=self)

    async def correct_z() -> Coroutine[None, None, None]:
        await cv_tasks.correct_z(prop='buoy', parent=self)

    async def correct_depth() -> Coroutine[None, None, None]:
        await move_tasks.correct_depth(desired_depth=DEPTH_LEVEL, parent=self)
    self.correct_depth = correct_depth

    async def move_x(step:float = 1) -> Coroutine[None, None, None]:
        await move_tasks.move_x(step=step, parent=self)

    def get_step_size(dist:float, dist_threshold:float) -> float:
        if dist > 3:
            return 2
        if dist > 2:
            return 1
        if dist > 1.5:
            return 0.5
        return min(dist - dist_threshold + 0.1, 0.25)

    async def move_to_buoy(buoy_dist_threshold=1):
        buoy_dist = CV().bounding_boxes[CVObjectType.BUOY].coords.x
        await correct_y()
        await correct_depth()

        while buoy_dist > buoy_dist_threshold:
            await move_x(step=get_step_size(buoy_dist, buoy_dist_threshold))
            logger.info(f"Buoy dist: {CV().bounding_boxes[CVObjectType.BUOY].coords.x}")
            await correct_y()
            if buoy_dist < 3:
                await correct_z()
            else:
                await correct_depth()

            await Yield()
            buoy_dist = CV().bounding_boxes[CVObjectType.BUOY]
            logger.info(f"Buoy dist: {CV().bounding_boxes[CVObjectType.BUOY].coords.x}")

        await correct_z()

    await move_to_buoy()

    start_imu_orientation = copy.deepcopy(State().imu.orientation)
    start_imu_euler_angles = quat2euler(geometry_utils.geometry_quat_to_transforms3d_quat(start_imu_orientation))

    def get_yaw_correction():
        cur_imu_orientation = copy.deepcopy(State().imu.orientation)
        cur_imu_euler_angles = quat2euler(geometry_utils.geometry_quat_to_transforms3d_quat(cur_imu_orientation))

        return start_imu_euler_angles[2] - cur_imu_euler_angles[2]

    async def correct_yaw():
        yaw_correction = get_yaw_correction()
        logger.info(f'Yaw correction: {yaw_correction}')
        sign = 1 if yaw_correction > 0.1 else (-1 if yaw_correction < -0.1 else 0)
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, yaw_correction + (sign * 0.1)),
            keep_level=True,
            parent=self,
        )
        logger.info('Corrected yaw')
    self.correct_yaw = correct_yaw

    async def move_with_directions(directions, correct_yaw=True):
        await move_tasks.move_with_directions(directions, correct_yaw=correct_yaw, correct_depth=True, parent=self)

    if turn_to_face_buoy:
        def get_step_size_move_away(dist, dist_threshold):
            if dist < 0.75:
                return -0.5
            return max(dist - dist_threshold - 0.1, -0.25)

        async def move_away_from_buoy(buoy_dist_threshold=1.0):
            logger.info('Moving away from buoy')
            buoy_dist = CV().bounding_boxes[CVObjectType.BUOY].coords.x
            await correct_y()
            await correct_z()
            while buoy_dist < buoy_dist_threshold:
                await move_x(step=get_step_size_move_away(buoy_dist, buoy_dist_threshold))

                logger.info(f"Buoy dist: {CV().bounding_boxes[CVObjectType.BUOY].coords.x}")
                await correct_y()
                await correct_z()
                await Yield()
                buoy_dist = CV().bounding_boxes[CVObjectType.BUOY].coords.x
                logger.info(f"Buoy dist: {CV().bounding_boxes[CVObjectType.BUOY].coords.x}")

            logger.info('Moved away from buoy')

        # Circumnavigate buoy
        for _ in range(4):
            DEPTH_LEVEL = State().depth
            directions = [
                (0, 1.5, 0),
                (1, 0, 0),
            ]
            await move_with_directions(directions, correct_yaw=False)
            await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, -math.radians(90)),
                                                parent=self)
            logger.info('Yaw 90 deg')
            await move_away_from_buoy()

    else:
        directions = [
            (0, 1.25, 0),
            (2.25, 0, 0),
            (0, -2.5, 0),
            (-2.5, 0, 0),
            (0, 1.25, 0),
        ]
        await move_with_directions(directions, correct_yaw=False)

        await move_to_buoy()


@task
async def after_buoy_task(self: Task):

    DEPTH_LEVEL = State().depth
    latency_threshold = 3

    async def correct_depth():
        await move_tasks.correct_depth(desired_depth=DEPTH_LEVEL, parent=self)
    self.correct_depth = correct_depth

    def is_receiving_cv_data():
        return Clock().now().seconds_nanoseconds()[0] - CV().bounding_boxes[CVObjectType.PATH_MARKER].header.stamp.secs < latency_threshold

    def stabilize():
        pose_to_hold = copy.deepcopy(State().state.pose.pose)
        Controls().publish_desired_position(pose_to_hold)

    directions = [
        (0, -1.25, 0),
        (2.25, 0, 0),
        (0, 2.5, 0),
        (-2.5, 0, 0),
        (0, -1.25, 0),
    ]
    circumnavigate_task = move_tasks.move_with_directions(
        directions, correct_yaw=False, correct_depth=True, parent=self)

    while not circumnavigate_task.done:
        circumnavigate_task.step()
        if is_receiving_cv_data():
            stabilize()
            await sleep(5)
            break

        await Yield()

    await move_tasks.move_with_directions([(0, 0, 0, 0, 0, -math.radians(90))], parent=self)

    await align_path_marker(direction=-1, parent=self)

    DEPTH_LEVEL = State().orig_depth - 0.7

    directions = [
        (2, 0, 0),
        (2, 0, 0),
        (2, 0, 0),
        (1, 0, 0),
    ]

    await move_tasks.move_with_directions(directions, correct_yaw=False, correct_depth=True, parent=self)

    found_bins = await spiral_bin_search(parent=self)

    if found_bins:
        await bin_task(parent=self)

    await yaw_to_cv_object('bin_pink_front', direction=1, yaw_threshold=math.radians(15),
                           depth_level=1.0, parent=Task.MAIN_ID)

    await octagon_task(direction=1, parent=self)


@task
async def buoy_to_octagon(self: Task, direction: int = 1, move_forward: int = 0):
    DEPTH_LEVEL = State().orig_depth - 0.7

    logger.info('Started buoy to octagon')

    async def move_with_directions(directions):
        await move_tasks.move_with_directions(directions, correct_yaw=False, correct_depth=True, parent=self)

    async def correct_depth() -> Coroutine[None, None, None]:
        await move_tasks.correct_depth(desired_depth=DEPTH_LEVEL, parent=self)
    self.correct_depth = correct_depth

    # Move towards octagon
    directions = [
        (0, 2 * direction, 0),
        (0, 2 * direction, 0),
        (0, 2 * direction, 0),
        (0, 1 * direction, 0),
        (move_forward, 0, 0),
    ]
    await move_with_directions(directions)


@task
async def buoy_circumnavigation_power(self: Task, depth: float = 0.7) -> Task[None, None, None]:
    """
    Perform a buoy circumnavigation task with a specified depth adjustment.

    Args:
        self (Task): The task instance.
        depth (float): The depth offset to adjust the circumnavigation. Default is 0.7.

    Returns:
        Task[None, None, None]: The result of the circumnavigation task.
    """
    DEPTH_LEVEL = State().orig_depth - depth

    def publish_power() -> None:
        power = Twist()
        power.linear.y = 0.9
        power.angular.z = -0.1
        Controls().set_axis_control_type(x=ControlTypes.DESIRED_POWER, y=ControlTypes.DESIRED_POWER,
                                         yaw=ControlTypes.DESIRED_POWER)
        Controls().publish_desired_power(power, set_control_types=False)

    def stabilize() -> None:
        pose_to_hold = copy.deepcopy(State().state.pose.pose)
        Controls().publish_desired_position(pose_to_hold)

    async def correct_depth() -> Coroutine[None, None, None]:
        await move_tasks.correct_depth(desired_depth=DEPTH_LEVEL, parent=self)

    for _ in range(4):
        publish_power()
        logger.info('Publish power')
        await sleep(6)
        logger.info('Sleep 5 (1)')
        stabilize()
        logger.info('Stabilized')
        await sleep(5)
        logger.info('Sleep 5 (2)')
        await correct_depth()
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 1, 0, 0, 0, 0), parent=self)


@task
async def initial_submerge(self: Task, submerge_dist: float) -> Task[None, None, None]:
    """
    Submerge the robot a given amount.

    Args:
        submerge_dist: The distance to submerge the robot in meters.
    """
    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, submerge_dist, 0, 0, 0),
        keep_level=True,
        parent=self,
    )
    logger.info(f'Submerged {submerge_dist} meters')

    async def correct_roll_and_pitch():
        imu_orientation = State().imu.orientation
        euler_angles = quat2euler([imu_orientation.w, imu_orientation.x, imu_orientation.y, imu_orientation.z])
        roll_correction = -euler_angles[0] * 1.2
        pitch_correction = -euler_angles[1] * 1.2

        logger.info(f'Roll, pitch correction: {roll_correction, pitch_correction}')
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, roll_correction, pitch_correction, 0),
                                            parent=self)

    await correct_roll_and_pitch()


@task
async def coin_flip(self: Task, depth_level=0.7) -> Task[None, None, None]:
    """
    Perform the coin flip task, adjusting the robot's yaw and depth.

    The coin flip task involves correcting the robot's yaw to return it to its original orientation and then adjusting its depth. The task continuously calculates yaw corrections based on the difference between the current and original orientations, making incremental adjustments until the yaw is within a specified threshold. After correcting yaw, the robot adjusts its depth to reach the desired level.

    Args:
        self (Task): The task instance managing the execution of the coin flip task.
        depth_level (float): The depth adjustment level relative to the robot's original depth. Default is 0.7.

    Returns:
        Task[None, None, None]: The result of the task execution.

    Detailed Process:
        1. Calculate the desired yaw correction using the difference between the original and current IMU orientations.
        2. Gradually adjust yaw in steps, ensuring the correction does not exceed the maximum allowed yaw change.
        3. Once the yaw is corrected to within 5 degrees, adjust the robot's depth to the specified level.
        4. Log each step of the process for debugging and traceability.

    Logging:
        - Logs the initial start of the coin flip task.
        - Logs intermediate yaw corrections and desired yaw adjustments.
        - Logs depth corrections and the final completion of the task.

    Example:
        >>> await coin_flip(task_instance, depth_level=0.5)

    Notes:
        - Uses `State` to access robot's current and original states, including depth and IMU orientation.
        - Uses `geometry_utils` to create poses for yaw and depth corrections.
        - The task continuously loops until the yaw correction is within the specified threshold (±5 degrees).
    """
    logger.info('Started coin flip')
    DEPTH_LEVEL = State().orig_depth - depth_level
    MAXIMUM_YAW = math.radians(30)

    def get_step_size(desired_yaw):
        return min(abs(desired_yaw), MAXIMUM_YAW)

    def get_yaw_correction():
        orig_imu_orientation = copy.deepcopy(State().orig_imu.orientation)
        orig_imu_euler_angles = quat2euler(geometry_utils.geometry_quat_to_transforms3d_quat(orig_imu_orientation))

        cur_imu_orientation = copy.deepcopy(State().imu.orientation)
        cur_imu_euler_angles = quat2euler(geometry_utils.geometry_quat_to_transforms3d_quat(cur_imu_orientation))

        correction = orig_imu_euler_angles[2] - cur_imu_euler_angles[2]

        sign_correction = np.sign(correction)
        desired_yaw = sign_correction * get_step_size(correction)
        logger.info(f'Coinflip: desired_yaw = {desired_yaw}')

        return correction
        # return desired_yaw

    while abs(yaw_correction := get_yaw_correction()) > math.radians(5):
        logger.info(f'Yaw correction: {yaw_correction}')
        sign = 1 if yaw_correction > 0.1 else (-1 if yaw_correction < -0.1 else 0)
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, yaw_correction + (sign * 0.1)),
            keep_level=True,
            parent=self,
        )
        logger.info('Back to original orientation')

    logger.info(f'Final yaw correction: {get_yaw_correction()}')

    depth_delta = DEPTH_LEVEL - State().depth
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, depth_delta, 0, 0, 0), parent=self)
    logger.info(f'Corrected depth {depth_delta}')

    logger.info('Completed coin flip')


@task
async def gate_task_dead_reckoning(self: Task) -> Task[None, None, None]:
    logger.info('Started gate task')
    DEPTH_LEVEL = State().orig_depth - 0.6
    STEPS = 5

    for _ in range(STEPS):
        await move_tasks.move_x(step=1, parent=self)
        await move_tasks.correct_depth(desired_depth=DEPTH_LEVEL, parent=self)

    logger.info('Moved through gate')


@task
async def gate_task(self: Task, offset: int = 0, direction: int = 1) -> Task[None, None, None]:
    """
    Asynchronous task to perform gate-related operations.
    """
    logger.info('Started gate task')
    depth_level = State().orig_depth - 0.7

    async def correct_y(factor: int=1) -> None:
        await cv_tasks.correct_y(prop='gate_red_cw', add_factor=0.2 + offset, mult_factor=factor, parent=self)

    async def correct_z() -> None:
        await cv_tasks.correct_z(prop='gate_red_cw', parent=self)

    async def correct_depth() -> None:
        await move_tasks.correct_depth(desired_depth=depth_level, parent=self)
    self.correct_depth = correct_depth

    async def move_x(step=1) -> None:
        await move_tasks.move_x(step=step, parent=self)

    def get_step_size(dist: float) -> float:
        dist_threshold = 4
        if dist > dist_threshold:
            return 1
        return max(dist-3 + 0.25, 0.25)


    async def sleep(secs: float) -> None:
        duration = Duration(seconds=secs)
        start_time = Clock().now()
        while start_time + duration > Clock().now():
            await Yield()

    logger.info('Begin sleep')
    await sleep(2)
    logger.info('End sleep')

    gate_dist = CV().bounding_boxes[CVObjectType.GATE_SAWFISH].coords.x
    # await correct_y(factor=0.5)
    await correct_depth()
    num_corrections = 0
    while gate_dist > 3:
        await move_x(step=get_step_size(gate_dist))
        await yaw_to_cv_object('gate_red_cw', direction=-1, yaw_threshold=math.radians(10),
                               latency_threshold=2, depth_level=0.6, parent=self),
        # await correct_y(factor=(0.5 if num_corrections < 0 else 1))
        await correct_depth()
        await Yield()
        gate_dist = CV().bounding_boxes[CVObjectType.GATE_SAWFISH].coords.x
        logger.info(f'Gate dist: {gate_dist}')
        num_corrections += 1

    directions = [
        (2, 0, 0),
        (0, 0.2 * direction, 0),
        (2, 0, 0),
        (1, 0, 0),
    ]

    await move_tasks.move_with_directions(directions, correct_yaw=False, correct_depth=True, parent=self)

    logger.info('Moved through gate')


@task
async def yaw_to_cv_object(self: Task, cv_object: CVObjectType, direction=1,
                           yaw_threshold=math.radians(30), latency_threshold=10,
                           depth_level=0.5) -> Task[None, None, None]:
    """
    Corrects the yaw relative to the CV object
    """
    DEPTH_LEVEL = State().orig_depth - depth_level
    MAXIMUM_YAW = math.radians(20)

    logger.info('Starting yaw_to_cv_object')

    async def sleep(secs):
        duration = Duration(seconds=secs)
        start_time = Clock().now()
        while start_time + duration > Clock().now():
            await Yield()

    async def correct_depth():
        # await move_tasks.depth_correction(DEPTH_LEVEL, parent=self)
        await move_tasks.depth_correction(desired_depth=DEPTH_LEVEL, parent=self)

    def is_receiving_cv_data():
        return cv_object in CV().bounding_boxes and \
                Clock().now().seconds_nanoseconds()[0] - CV().bounding_boxes[cv_object].header.stamp.secs < latency_threshold

    def get_step_size(desired_yaw):
        # desired yaw in radians
        return min(abs(desired_yaw), MAXIMUM_YAW)

    async def yaw_until_object_detection():
        while not is_receiving_cv_data():
            logger.info(f'No {cv_object} detection, setting yaw setpoint {MAXIMUM_YAW}')
            await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, MAXIMUM_YAW * direction),
                                                parent=self)
            await correct_depth()
            await Yield()

    # Yaw until object detection
    await correct_depth()
    await yaw_until_object_detection()

    logger.info(f'{cv_object} detected. Now centering {cv_object} in frame...')

    # Center detected object in camera frame
    cv_object_yaw = CV().bounding_boxes[cv_object].yaw
    await correct_depth()
    logger.info(f'abs(cv_object_yaw): {abs(cv_object_yaw)}')
    logger.info(f'yaw_threshold: {yaw_threshold}')
    while abs(cv_object_yaw) > yaw_threshold:
        sign_cv_object_yaw = np.sign(cv_object_yaw)
        correction = get_step_size(cv_object_yaw)
        desired_yaw = sign_cv_object_yaw * correction

        logger.info(f'Detected yaw {cv_object_yaw} is greater than threshold {yaw_threshold}. Yawing: {desired_yaw}')
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, desired_yaw),
                                            parent=self)
        await correct_depth()
        await Yield()

        if (not is_receiving_cv_data()):
            logger.info(f'{cv_object} detection lost, running yaw_until_object_detection()')
            await yaw_until_object_detection()

        cv_object_yaw = CV().bounding_boxes[cv_object].yaw

    logger.info(f'{cv_object} centered.')

    await correct_depth()


@task
async def align_path_marker(self: Task, direction=1) -> Task[None, None, None]:
    """
    Corrects the yaw relative to the CV object
    """
    DEPTH_LEVEL = State().orig_depth - 0.5
    MAXIMUM_YAW = math.radians(30)
    YAW_THRESHOLD = math.radians(5)
    PIXEL_THRESHOLD = 70

    logger.info('Starting align path marker')

    async def move_x(step=1):
        await move_tasks.move_x(step=step, parent=self)

    async def move_y(step=1):
        await move_tasks.move_y(step=step, parent=self)

    async def correct_depth():
        await move_tasks.depth_correction(desired_depth=DEPTH_LEVEL, parent=self)

    async def sleep(secs):
        duration = Duration(seconds=secs)
        start_time = Clock().now()
        while start_time + duration > Clock().now():
            await Yield()

    def get_step_size(desired_yaw):
        # desired yaw in rads
        return min(abs(desired_yaw), MAXIMUM_YAW)

    def get_step_mult_factor(dist, threshold):
        if abs(dist) < threshold:
            return 0
        if dist > threshold:
            return 1
        return -1

    async def center_path_marker(pixel_threshold: float, step_size=0.20, x_offset=0, y_offset=0) -> None:
        logger.info(CV().distances[CVObjectType.PATH_MARKER])
        pixel_x = CV().distances[CVObjectType.PATH_MARKER].x + x_offset
        pixel_y = CV().distances[CVObjectType.PATH_MARKER].y + y_offset

        count = 1
        while (max(pixel_x, pixel_y) > pixel_threshold or min(pixel_x, pixel_y) < -pixel_threshold):
            logger.info(CV().distances[CVObjectType.PATH_MARKER])

            await move_x(step=step_size * get_step_mult_factor(pixel_x, pixel_threshold))
            await move_y(step=step_size * get_step_mult_factor(pixel_y, pixel_threshold))

            pixel_x = CV().distances[CVObjectType.PATH_MARKER].x + x_offset
            pixel_y = CV().distances[CVObjectType.PATH_MARKER].y + y_offset

            if count % 3 == 0:
                logger.info('Correcting depth')
                await correct_depth()

            await Yield()

            logger.info(f'x: {pixel_x}, y: {pixel_y}')

            count += 1

        logger.info('Finished centering path marker')
        logger.info(f'x: {pixel_x}, y: {pixel_y}')

    logger.info('Now aligning path marker in frame...')

    # Center detected object in camera frame
    path_marker_yaw = CV().bounding_boxes[CVObjectType.PATH_MARKER].yaw
    await correct_depth()
    logger.info(f"abs(path_marker_yaw) = '{abs(path_marker_yaw)}")
    logger.info(f'yaw_threshold = {YAW_THRESHOLD!s}')

    while abs(path_marker_yaw) > YAW_THRESHOLD:
        sign_path_marker_yaw = np.sign(path_marker_yaw)
        correction = get_step_size(path_marker_yaw)
        desired_yaw = sign_path_marker_yaw * correction

        logger.info(f'Detected yaw {path_marker_yaw} is greater than threshold {YAW_THRESHOLD}. Yawing: {desired_yaw}',
                      )
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, desired_yaw),
                                            parent=self)
        await correct_depth()
        await center_path_marker(pixel_threshold=PIXEL_THRESHOLD)
        await correct_depth()

        await Yield()

        path_marker_yaw = CV().bounding_boxes[CVObjectType.PATH_MARKER].yaw

    logger.info('Path marker centered.')

    await correct_depth()


@task
async def center_path_marker(self: Task):
    DEPTH_LEVEL = State().orig_depth - 0.5
    PIXEL_THRESHOLD = 70

    async def correct_depth(desired_depth=DEPTH_LEVEL):
        await move_tasks.correct_depth(desired_depth=desired_depth, parent=self)
    self.correct_depth = correct_depth

    async def move_x(step=1):
        await move_tasks.move_x(step=step, parent=self)

    async def move_y(step=1):
        await move_tasks.move_y(step=step, parent=self)

    def get_step_mult_factor(dist: float, threshold: float) -> int:
        if abs(dist) < threshold:
            return 0
        if dist > threshold:
            return 1
        return -1

    async def center_path_marker(pixel_threshold: float, step_size=0.20, x_offset=0, y_offset=0) -> None:
        logger.info(CV().distances[CVObjectType.PATH_MARKER])
        pixel_x = CV().distances[CVObjectType.PATH_MARKER].x + x_offset
        pixel_y = CV().distances[CVObjectType.PATH_MARKER].y + y_offset

        count = 1
        while (max(pixel_x, pixel_y) > pixel_threshold or min(pixel_x, pixel_y) < -pixel_threshold):
            logger.info(CV().distances[CVObjectType.PATH_MARKER])

            await move_x(step=step_size * get_step_mult_factor(pixel_x, pixel_threshold))
            await move_y(step=step_size * get_step_mult_factor(pixel_y, pixel_threshold))

            pixel_x = CV().distances[CVObjectType.PATH_MARKER].x + x_offset
            pixel_y = CV().distances[CVObjectType.PATH_MARKER].y + y_offset

            if count % 3 == 0:
                logger.info('Correcting depth')
                await correct_depth()

            await Yield()

            logger.info(f'x: {pixel_x}, y: {pixel_y}')

            count += 1

        logger.info('Finished centering path marker')
        logger.info(f'x: {pixel_x}, y: {pixel_y}')

    await correct_depth()
    await center_path_marker(pixel_threshold=PIXEL_THRESHOLD, x_offset=-120, y_offset=-120)


@task
async def path_marker_to_pink_bin(self: Task, maximum_distance: int = 6):
    DEPTH_LEVEL = State().orig_depth - 0.5
    AREA_THRESHOLD = 1000
    LATENCY_THRESHOLD = 1

    logger.info('Starting path marker to bins')

    async def correct_depth(desired_depth: float) -> None:
        await move_tasks.correct_depth(desired_depth=desired_depth, parent=self)

    def is_receiving_bin_data(bin_object: CVObjectType, latest_detection_time) -> bool:
        if not latest_detection_time or bin_object not in CV().bounding_boxes:
            return False

        width = CV().bounding_boxes[bin_object].width
        height = CV().bounding_boxes[bin_object].height

        return width * height >= AREA_THRESHOLD and \
            Clock().now().seconds_nanoseconds()[0] - CV().bounding_boxes[bin_object].header.stamp.secs < LATENCY_THRESHOLD and \
            abs(CV().bounding_boxes[bin_object].header.stamp.secs - latest_detection_time) < LATENCY_THRESHOLD

    async def move_x(step : float =1 ) -> None:
        await move_tasks.move_x(step=step, parent=self)

    def stabilize() -> None:
        pose_to_hold = copy.deepcopy(State().state.pose.pose)
        Controls().publish_desired_position(pose_to_hold)

    async def sleep(secs: float) -> None:
        duration = Duration(seconds=secs)
        start_time = Clock().now()
        while start_time + duration > Clock().now():
            await Yield()

    async def move_to_bins() -> None:
        count = 1
        bin_red_time = None
        bin_blue_time = None

        await move_x(step=1)

        while not is_receiving_bin_data(CVObjectType.BIN_RED, bin_red_time) \
                or not is_receiving_bin_data(CVObjectType.BIN_BLUE, bin_blue_time):
            bin_red_time = CV().bounding_boxes[CVObjectType.BIN_RED].header.stamp.secs
            bin_blue_time = CV().bounding_boxes[CVObjectType.BIN_BLUE].header.stamp.secs

            await correct_depth(DEPTH_LEVEL)

            is_receiving_red_bin_data = is_receiving_bin_data(CVObjectType.BIN_RED, bin_red_time)
            is_receiving_blue_bin_data = is_receiving_bin_data('bin_blue', bin_blue_time)

            logger.info(f'Receiving red bin data: {is_receiving_red_bin_data}')
            logger.info(f'Receiving blue bin data: {is_receiving_blue_bin_data}')

            step = 0.5 if (is_receiving_red_bin_data or is_receiving_blue_bin_data) else 1
            await move_x(step=step)

            await Yield()

            if count >= maximum_distance:
                logger.info('Bin not spotted, exiting the loop...')
                break

            count += 1

        logger.info('Reached pink bins, stabilizing...')
        stabilize()
        await sleep(5)

    await move_to_bins()


@task
async def spiral_bin_search(self: Task) -> Task[None, None, None]:
    DEPTH_LEVEL = State().orig_depth - 0.5
    AREA_THRESHOLD = 1000
    LATENCY_THRESHOLD = 1

    class Direction(Enum):
        FORWARD = 1,
        BACK = 2,
        LEFT = 3,
        RIGHT = 4

    def publish_power(direction: Direction):
        power = Twist()

        if direction == Direction.FORWARD:
            power.linear.x = 0.7
        elif direction == Direction.BACK:
            power.linear.x = -0.7
        elif direction == Direction.LEFT:
            power.linear.y = 1.0
        elif direction == Direction.RIGHT:
            power.linear.y = -1.0

        if direction in [Direction.FORWARD, Direction.BACK]:
            Controls().set_axis_control_type(x=ControlTypes.DESIRED_POWER, y=ControlTypes.DESIRED_POSITION)
        elif direction in [Direction.LEFT, Direction.RIGHT]:
            Controls().set_axis_control_type(x=ControlTypes.DESIRED_POSITION, y=ControlTypes.DESIRED_POWER)

        Controls().publish_desired_power(power, set_control_types=False)

    async def move_step(direction: Direction, steps: float):
        pose = geometry_utils.create_pose(0, 0, 0, 0, 0, 0)

        if direction == Direction.FORWARD:
            pose.position.x = steps
        elif direction == Direction.BACK:
            pose.position.x = -steps
        elif direction == Direction.LEFT:
            pose.position.y = steps
        elif direction == Direction.RIGHT:
            pose.position.y = -steps

        await move_tasks.move_to_pose_local(pose, parent=self)

    DIRECTIONS = [
        (Direction.FORWARD, 1),
        (Direction.LEFT, 1),
        (Direction.BACK, 2),
        (Direction.RIGHT, 2),
        (Direction.FORWARD, 3),
        (Direction.LEFT, 3),
        (Direction.BACK, 4),
        (Direction.RIGHT, 4),
        (Direction.FORWARD, 5),
        (Direction.LEFT, 5),
        (Direction.BACK, 6),
        (Direction.RIGHT, 6),
        (Direction.FORWARD, 7),
        (Direction.LEFT, 7),
        (Direction.BACK, 8),
        (Direction.RIGHT, 8),
    ]

    async def correct_depth(desired_depth: float) -> None:
        await move_tasks.correct_depth(desired_depth=desired_depth, parent=self)

    def is_receiving_bin_data(bin_object: CVObjectType, latest_detection_time: float) -> bool:
        if not latest_detection_time or bin_object not in CV().bounding_boxes:
            return False

        width = CV().bounding_boxes[bin_object].width
        height = CV().bounding_boxes[bin_object].height

        return width * height >= AREA_THRESHOLD and \
            Clock().now().seconds_nanoseconds()[0] - CV().bounding_boxes[bin_object].header.stamp.secs < LATENCY_THRESHOLD and \
            abs(CV().bounding_boxes[bin_object].header.stamp.secs - latest_detection_time) < LATENCY_THRESHOLD

    def stabilize() -> None:
        pose_to_hold = copy.deepcopy(State().state.pose.pose)
        Controls().publish_desired_position(pose_to_hold)

    async def sleep(secs: float) -> None:
        duration = Duration(seconds=secs)
        start_time = Clock().now()
        while start_time + duration > Clock().now():
            await Yield()

    async def search_for_bins() -> bool:
        logger.info('Searching for red/blue bins...')
        bin_red_time = None
        bin_blue_time = None

        for direction, secs in DIRECTIONS:
            bin_found = False
            secs *= 0.5
            logger.info(f'Publishing power: {direction, secs}')

            await move_step(direction, secs)

            iterations = secs / 0.1
            for _ in range(int(iterations)):
                bin_red_time = CV().bounding_boxes[CVObjectType.BIN_RED].header.stamp.secs
                bin_blue_time = CV().bounding_boxes[CVObjectType.BIN_BLUE].header.stamp.secs

                is_receiving_red_bin_data = is_receiving_bin_data(CVObjectType.BIN_RED, bin_red_time)
                is_receiving_blue_bin_data = is_receiving_bin_data(CVObjectType.BIN_BLUE, bin_blue_time)

                bin_found = is_receiving_red_bin_data and is_receiving_blue_bin_data

                if bin_found:
                    break

                await sleep(0.1)
                await Yield()

            await correct_depth(DEPTH_LEVEL)

            if bin_found:
                logger.info('Found bin, terminating...')
                break

        logger.info(f'Received red bin data: {is_receiving_red_bin_data}')
        logger.info(f'Received blue bin data: {is_receiving_blue_bin_data}')

        return bin_found

    return (await search_for_bins())


@task
async def bin_task(self: Task) -> Task[None, None, None]:
    """
    Detects and drops markers into the red bin. Requires robot to have submerged 0.7 meters.
    """
    logger.info('Started bin task')
    START_DEPTH_LEVEL = State().orig_depth - 0.6
    START_PIXEL_THRESHOLD = 70
    MID_DEPTH_LEVEL = State().orig_depth - 1.0
    MID_PIXEL_THRESHOLD = 30

    FRAME_AREA = 480 * 600

    TIMEOUT = Duration(seconds=240)

    start_time = Clock().now()

    drop_marker = MarkerDropper().drop_marker

    async def correct_x(target: float) -> None:
        await cv_tasks.correct_x(prop=target, parent=self)

    async def correct_y(target: float) -> None:
        await cv_tasks.correct_y(prop=target, parent=self)

    async def correct_z():
        pass

    async def correct_depth(desired_depth: float) -> None:
        await move_tasks.correct_depth(desired_depth=desired_depth, parent=self)
    self.correct_depth = correct_depth

    async def correct_yaw() -> None:
        yaw_correction = CV().angles[CVObjectType.BIN_WHOLE]
        logger.info(f'Yaw correction: {yaw_correction}')
        sign = 1 if yaw_correction > 0.1 else (-1 if yaw_correction < -0.1 else 0)
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, yaw_correction + (sign * 0.1)),
            keep_level=True,
            parent=self,
        )
        logger.info('Corrected yaw')
    self.correct_yaw = correct_yaw

    async def correct_roll_and_pitch() -> None:
        imu_orientation = State().imu.orientation
        euler_angles = quat2euler([imu_orientation.w, imu_orientation.x, imu_orientation.y, imu_orientation.z])
        roll_correction = -euler_angles[0] * 1.2
        pitch_correction = -euler_angles[1] * 1.2

        logger.info(f'Roll, pitch correction: {roll_correction, pitch_correction}')
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, roll_correction, pitch_correction, 0),
                                            parent=self)

    async def move_x(step=1):
        await move_tasks.move_x(step=step, parent=self)

    async def move_y(step=1):
        await move_tasks.move_y(step=step, parent=self)

    def get_step_mult_factor(dist, threshold):
        if abs(dist) < threshold:
            return 0
        if dist > threshold:
            return 1
        return -1

    async def sleep(secs):
        duration = Duration(seconds=secs)
        start_time = Clock().now()
        while start_time + duration > Clock().now():
            await Yield()

    async def track_bin(target: CVObjectType, desired_depth, pixel_threshold, step_size=0.20, x_offset=0, y_offset=0):
        logger.info(CV().distances[target])
        pixel_x = CV().distances[target].x + x_offset
        pixel_y = CV().distances[target].y + y_offset

        width = CV().bounding_boxes[CVObjectType.BIN_RED].width
        height = CV().bounding_boxes[CVObjectType.BIN_RED].height

        count = 1
        while (max(pixel_x, pixel_y) > pixel_threshold or min(pixel_x, pixel_y) < -pixel_threshold) \
                and width * height <= 1/3 * FRAME_AREA:
            logger.info(CV().distances[target])

            await move_x(step=step_size * get_step_mult_factor(pixel_x, pixel_threshold))
            await move_y(step=step_size * get_step_mult_factor(pixel_y, pixel_threshold))

            width = CV().bounding_boxes[CVObjectType.BIN_RED].width
            height = CV().bounding_boxes[CVObjectType.BIN_RED].height

            pixel_x = CV().distances[target].x + x_offset
            pixel_y = CV().distances[target].y + y_offset

            if count % 3 == 0:
                logger.info('correcting depth')
                await correct_depth(desired_depth=desired_depth)
                logger.info('correcting roll and pitch')
                await correct_roll_and_pitch()

            if width * height >= 1/6 * FRAME_AREA and \
                    abs(pixel_x) < pixel_threshold * 1.75 and abs(pixel_y) < pixel_threshold * 1.75:
                logger.info(f'Reached area threshold: area = {width * height}')
                break

            if Clock().now() - start_time > TIMEOUT:
                logger.warning('Track bin timed out')
                break

            await Yield()

            logger.info(f'x: {pixel_x}, y: {pixel_y}, area: {width * height}')

            count += 1

        logger.info('Finished tracking bin')
        logger.info(f'x: {pixel_x}, y: {pixel_y}, area: {width * height}')

    await correct_depth(desired_depth=START_DEPTH_LEVEL)
    await track_bin(target='bin_red', desired_depth=START_DEPTH_LEVEL, pixel_threshold=START_PIXEL_THRESHOLD)

    await correct_yaw()

    await correct_depth(desired_depth=MID_DEPTH_LEVEL)
    await track_bin(target='bin_red', desired_depth=MID_DEPTH_LEVEL, pixel_threshold=MID_PIXEL_THRESHOLD,
                    step_size=0.18, y_offset=30, x_offset=25)

    drop_marker(MarkerDropperStates.LEFT)
    logger.info('Dropped left marker')
    await sleep(3)

    drop_marker(MarkerDropperStates.RIGHT)
    logger.info('Dropped right marker')
    await sleep(2)

    await correct_depth(desired_depth=START_DEPTH_LEVEL)
    logger.info(f'Corrected depth to {START_DEPTH_LEVEL}')

    logger.info('Completed bin task')


@task
async def octagon_task(self: Task, direction: int = 1) -> Task[None, None, None]:
    """
    Detects, move towards the pink bins, then surfaces inside the octagon. Requires robot to have submerged 0.7 meters.
    """
    # NOTE: due to CV interface refactoring, pink bins are no longer object types available through the CV interface.
    logger.info('Starting octagon task')

    DEPTH_LEVEL_AT_BINS = State().orig_depth - 1.0
    DEPTH_LEVEL_ABOVE_BINS = State().orig_depth - 0.6
    LATENCY_THRESHOLD = 2
    CONTOUR_SCORE_THRESHOLD = 1000

    async def correct_depth(desired_depth):
        await move_tasks.correct_depth(desired_depth=desired_depth, parent=self)

    async def correct_yaw():
        yaw_correction = CV().cv_data['bin_pink_front'].yaw
        logger.info(f'Yaw correction: {yaw_correction}')
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, yaw_correction * 0.7),
            keep_level=True,
            parent=self,
        )
        logger.info('Corrected yaw')
    self.correct_yaw = correct_yaw

    def is_receiving_pink_bin_data(latest_detection_time):
        return latest_detection_time and 'bin_pink_bottom' in CV().cv_data and \
            CV().cv_data['bin_pink_bottom'].score >= CONTOUR_SCORE_THRESHOLD and \
            Clock().now().seconds_nanoseconds()[0] - CV().cv_data['bin_pink_bottom'].header.stamp.secs < LATENCY_THRESHOLD and \
            abs(CV().cv_data['bin_pink_bottom'].header.stamp.secs - latest_detection_time) < LATENCY_THRESHOLD

    def publish_power() -> None:
        power = Twist()
        power.linear.x = 0.3
        Controls().set_axis_control_type(x=ControlTypes.DESIRED_POWER)
        Controls().publish_desired_power(power, set_control_types=False)

    async def move_x(step=1) -> None:
        await move_tasks.move_x(step=step, parent=self)

    def stabilize() -> None:
        pose_to_hold = copy.deepcopy(State().state.pose.pose)
        Controls().publish_desired_position(pose_to_hold)

    async def sleep(secs) -> None:
        duration = Duration(seconds=secs)
        start_time = Clock().now()
        while start_time + duration > Clock().now():
            await Yield()

    def get_step_size(last_step_size):
        bin_pink_score = CV().cv_data['bin_pink_front'].score
        step = 0

        low_score = 200
        med_score = 1000
        high_score = 3000

        if bin_pink_score < low_score:
            step = 3
        elif bin_pink_score < med_score:
            step = 2
        elif bin_pink_score < high_score:
            step = 1
        else:
            step = 0.75

        return min(step, last_step_size)

    async def move_to_pink_bins() -> None:
        count = 1
        latest_detection_time = None
        moved_above = False

        last_step_size = float('inf')
        await move_x(step=1)
        while not is_receiving_pink_bin_data(latest_detection_time) and not moved_above:
            if 'bin_pink_bottom' in CV().cv_data:
                latest_detection_time = CV().cv_data['bin_pink_bottom'].header.stamp.secs

            await correct_depth(DEPTH_LEVEL_AT_BINS if not moved_above else DEPTH_LEVEL_ABOVE_BINS)
            if not moved_above:
                await yaw_to_cv_object('bin_pink_front', direction=direction, yaw_threshold=math.radians(15),
                                       depth_level=0.9, parent=Task.MAIN_ID)

            step = get_step_size(last_step_size)
            await move_x(step=step)
            last_step_size = step

            logger.info(f'Bin pink front score: {CV().cv_data['bin_pink_front'].score}')

            score_threshold = 4000
            if CV().cv_data['bin_pink_front'].score > score_threshold and not moved_above:
                await correct_depth(DEPTH_LEVEL_ABOVE_BINS + 0.1)
                moved_above = True

                logger.info('Moved above pink bins')

            await Yield()

            count += 1

            logger.info(f'Receiving pink bin data: {is_receiving_pink_bin_data(latest_detection_time)}')

        if moved_above:
            await move_tasks.move_with_directions([(1.5, 0, 0)], parent=self)
        else:
            logger.info('Detected bin_pink_bottom')

        logger.info('Reached pink bins, stabilizing...')
        stabilize()
        await sleep(5)

    await move_to_pink_bins()

    logger.info('Surfacing...')
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, State().orig_depth - State().depth, 0, 0, 0),
                                        timeout=10, parent=self)
    logger.info('Finished surfacing')
