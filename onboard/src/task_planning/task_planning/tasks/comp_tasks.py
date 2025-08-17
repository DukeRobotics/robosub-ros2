# ruff: noqa

import copy
import math
from collections.abc import Coroutine
from enum import Enum
from typing import Literal

import numpy as np
from custom_msgs.msg import ControlTypes
from geometry_msgs.msg import Twist, Vector3
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.logging import get_logger
from transforms3d.euler import quat2euler
from typing import TYPE_CHECKING, cast

from task_planning.interface.controls import Controls
from task_planning.interface.cv import CV, CVObjectType
from task_planning.interface.servos import Servos, MarkerDropperStates, TorpedoStates
from task_planning.interface.state import State
from task_planning.interface.ivc import IVC, IVCMessageType
from task_planning.task import Task, Yield, task
from task_planning.tasks import cv_tasks, move_tasks, util_tasks, ivc_tasks
from task_planning.utils import geometry_utils

from rclpy.clock import Clock

if TYPE_CHECKING:
    from custom_msgs.srv import SendModemMessage

from task_planning.utils.other_utils import get_robot_name, RobotName
# from sonar.sonar.sonar import Sonar

# TODO: move stablize() to move_tasks.py
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

    async def roll():
        power = Twist()
        power.angular.x = 1.0
        Controls().publish_desired_power(power)
        logger.info('Published roll power')

        if get_robot_name() == RobotName.OOGWAY:
            await util_tasks.sleep(2.25, parent=self)
        else:
            await util_tasks.sleep(1.40, parent=self)

        logger.info('Completed roll')

        Controls().publish_desired_power(Twist())
        logger.info('Published zero power')

        await util_tasks.sleep(2, parent=self)

        logger.info('Completed zero')

    await move_tasks.depth_correction(DEPTH_LEVEL, parent=self)
    await roll()
    State().reset_pose()
    await util_tasks.sleep(3, parent=self)

    await move_tasks.depth_correction(DEPTH_LEVEL, parent=self)
    await roll()
    State().reset_pose()
    await util_tasks.sleep(3, parent=self)

    await move_tasks.depth_correction(DEPTH_LEVEL, parent=self)
    await util_tasks.sleep(3, parent=self)

    imu_orientation = State().imu.orientation
    euler_angles = quat2euler([imu_orientation.w, imu_orientation.x, imu_orientation.y, imu_orientation.z])
    roll_correction = -euler_angles[0]
    pitch_correction = -euler_angles[1]

    logger.info(f'Roll, pitch, yaw correction: {roll_correction, pitch_correction}')
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
        await cv_tasks.correct_y(prop=CVObjectType.BUOY, mult_factor=0.4, parent=self)

    async def correct_z() -> Coroutine[None, None, None]:
        await cv_tasks.correct_z(prop=CVObjectType.BUOY, parent=self)

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
            keep_orientation=True,
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
        if CV().is_receiving_recent_cv_data(CVObjectType.PATH_MARKER, latency_threshold):
            stabilize()
            await util_tasks.sleep(5, parent=self)
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
async def gate_to_octagon(self: Task, direction: int = 1, move_forward: int = 0):
    DEPTH_LEVEL = State().orig_depth - 0.7

    logger.info('Started gate to octagon')

    async def move_with_directions(directions, depth_level=-0.7):
        await move_tasks.move_with_directions(directions, depth_level, correct_yaw=False, correct_depth=True, parent=self)

    directions = [
        (4, 0, 0),
        (4, 0, 0),
        (4, 0, 0),
        (4, 0, 0),
    ]
    await move_with_directions(directions, depth_level=DEPTH_LEVEL)


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
        await util_tasks.sleep(6, parent=self)
        logger.info('Sleep 5 (1)')
        stabilize()
        logger.info('Stabilized')
        await util_tasks.sleep(5, parent=self)
        logger.info('Sleep 5 (2)')
        await correct_depth()
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 1, 0, 0, 0, 0), parent=self)


@task
async def initial_submerge(self: Task, submerge_dist: float, z_tolerance: float = 0.1, enable_controls_flag: bool = False) -> Task[None, None, None]:
    """
    Submerge the robot a given amount.

    Args:
        submerge_dist: The distance to submerge the robot in meters.
        enable_controls_flag: Flag to wait for ENABLE_CONTROLS status when true.
    """
    while enable_controls_flag and not Controls().enable_controls_status.data:
        await Yield()

    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, submerge_dist, 0, 0, 0),
        keep_orientation=False,
        pose_tolerances = move_tasks.create_twist_tolerance(linear_z = z_tolerance),
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
async def coin_flip(self: Task, depth_level=0.7, enable_same_direction=True) -> Task[None, None, None]:
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
        logger.info(f'Coinflip: imu_yaw_correction = {desired_yaw}')

        return desired_yaw

    def get_gyro_yaw_correction(return_raw=True):
        orig_gyro_orientation = copy.deepcopy(State().orig_gyro.pose.pose.orientation)
        orig_gyro_euler_angles = quat2euler(geometry_utils.geometry_quat_to_transforms3d_quat(orig_gyro_orientation))

        cur_gyro_orientation = copy.deepcopy(State().gyro.pose.pose.orientation)
        cur_gyro_euler_angles = quat2euler(geometry_utils.geometry_quat_to_transforms3d_quat(cur_gyro_orientation))

        raw_correction = cur_gyro_euler_angles[2] - orig_gyro_euler_angles[2]
        correction = (-raw_correction) % (2 * np.pi)

        logger.info(f'Coinflip: raw_gyro_yaw_correction = {raw_correction}')
        logger.info(f'Coinflip: processed_gyro_yaw_correction = {correction}')

        if return_raw:
            return raw_correction
        else:
            return correction

    if (enable_same_direction):
        while abs(get_gyro_yaw_correction(return_raw=True)) > math.radians(5):
            yaw_correction = get_gyro_yaw_correction(return_raw=False)
            logger.info(f'Yaw correction: {yaw_correction}')
            if (yaw_correction > np.pi):
                yaw_correction -= np.pi
                await move_tasks.move_to_pose_local(
                    geometry_utils.create_pose(0, 0, 0, 0, 0, np.pi),
                    pose_tolerances = move_tasks.create_twist_tolerance(angular_yaw = 0.05),
                    parent=self,
                    # TODO: maybe set yaw tolerance?
                )

                logger.info('Yaw correct 180')

            logger.info(f'Yaw correct remainder: {yaw_correction}')
            await move_tasks.move_to_pose_local(
                geometry_utils.create_pose(0, 0, 0, 0, 0, yaw_correction),
                parent=self,
                # TODO: maybe set yaw tolerance?
            )
    else:
        while abs(get_gyro_yaw_correction(return_raw=True)) > math.radians(5):
            yaw_correction = get_gyro_yaw_correction(return_raw=False)
            logger.info(f'Yaw correction: {yaw_correction}')

            await move_tasks.move_to_pose_local(
                geometry_utils.create_pose(0, 0, 0, 0, 0, yaw_correction),
                parent=self,
                # TODO: maybe set yaw tolerance?
            )

    logger.info('Step Completed')
    logger.info(f'Final yaw offset: {get_gyro_yaw_correction(return_raw=True)}')

    depth_delta = DEPTH_LEVEL - State().depth
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, depth_delta, 0, 0, 0), parent=self)
    logger.info(f'Corrected depth {depth_delta}')

    logger.info('Completed coin flip')


@task
async def gate_task_dead_reckoning(self: Task, depth_level=-0.7) -> Task[None, None, None]:
    logger.info('Started gate task')
    if get_robot_name() == RobotName.OOGWAY:
        await move_tasks.move_with_directions([(3, 0, 0)], depth_level=depth_level, correct_depth=True, correct_yaw=True, parent=self)
        await move_tasks.move_with_directions([(2, 0, 0)], depth_level=depth_level, correct_depth=True, correct_yaw=True, parent=self)
    elif get_robot_name() == RobotName.CRUSH:
        await move_tasks.move_with_directions([(3, 0, 0)], depth_level=depth_level, correct_depth=True, correct_yaw=True, parent=self)
        await move_tasks.move_with_directions([(2, 0, 0)], depth_level=depth_level, correct_depth=True, correct_yaw=True, parent=self)
    logger.info('Moved through gate')


@task
async def gate_task(self: Task, offset: int = 0, direction: int = 1) -> Task[None, None, None]:
    """
    Asynchronous task to perform gate-related operations.
    NOTE: This code is assuming we choose the sawfish side of the gate.
    """
    logger.info('Started gate task')
    depth_level = State().orig_depth - 0.7

    async def correct_y(factor: int=1) -> None:
        await cv_tasks.correct_y(prop=CVObjectType.GATE_SAWFISH, add_factor=0.2 + offset, mult_factor=factor, parent=self)

    async def correct_z() -> None:
        await cv_tasks.correct_z(prop=CVObjectType.GATE_SAWFISH, parent=self)

    async def correct_depth() -> None:
        await move_tasks.correct_depth(desired_depth=depth_level, parent=self)

    async def move_x(step=1) -> None:
        await move_tasks.move_x(step=step, parent=self)

    def get_step_size(dist: float) -> float:
        dist_threshold = 4
        if dist > dist_threshold:
            return 1
        return max(dist-3 + 0.25, 0.25)

    logger.info('Begin sleep')
    await util_tasks.sleep(2, parent=self)
    logger.info('End sleep')

    gate_dist = CV().bounding_boxes[CVObjectType.GATE_SAWFISH].coords.x
    # await correct_y(factor=0.5)
    await correct_depth()
    num_corrections = 0
    while gate_dist > 3:
        await move_x(step=get_step_size(gate_dist))
        await yaw_to_cv_object(CVObjectType.GATE_SAWFISH, direction=-1, yaw_threshold=math.radians(10),
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
async def yaw_to_cv_object_vel(self: Task, cv_object: CVObjectType, direction=1,
                                yaw_threshold=math.radians(40), latency_threshold=10,
                                depth_level=0.5) -> Task[None, None, None]:
    """
    Corrects the yaw relative to the CV object via simple linear velocity control.
    """
    DEPTH_LEVEL = State().orig_depth - depth_level
    MAXIMUM_YAW = math.radians(20)
    SCALE_FACTOR = 0.1

    logger.info('Starting yaw_to_cv_object')

    async def correct_depth():
        # await move_tasks.depth_correction(DEPTH_LEVEL, parent=self)
        await move_tasks.depth_correction(desired_depth=DEPTH_LEVEL, parent=self)

    async def yaw_until_object_detection():
        while not CV().is_receiving_recent_cv_data(cv_object, latency_threshold):
            logger.info(f'No {cv_object} detection, setting yaw setpoint {MAXIMUM_YAW}')
            await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, MAXIMUM_YAW * direction),
                                                pose_tolerances=Twist(linear=Vector3(x=0.05, y=0.05, z=0.05), angular=Vector3(x=0.2, y=0.3, z=0.2)),
                                                parent=self)
            await correct_depth()
            await Yield()

    # Yaw until object detection
    await correct_depth()
    await yaw_until_object_detection()

    logger.info(f'{cv_object} detected. Now centering {cv_object} in frame...')

    # Center detected object in camera frame
    cv_object_yaw = CV().bounding_boxes[cv_object].yaw # degrees
    await correct_depth()
    logger.info(f'abs(cv_object_yaw): {abs(cv_object_yaw)}')
    logger.info(f'yaw_threshold: {yaw_threshold}')
    while abs(cv_object_yaw) > yaw_threshold:
        logger.info(f'Detected yaw {cv_object_yaw} is greater than threshold {yaw_threshold}. Setting yaw power to: {cv_object_yaw * SCALE_FACTOR}')
        Controls().publish_desired_power(Twist(angular=Vector3(z=cv_object_yaw * SCALE_FACTOR)))
        # await correct_depth()
        # await Yield()

        if (not CV().is_receiving_recent_cv_data(cv_object, latency_threshold)):
            logger.info(f'{cv_object} detection lost, running yaw_until_object_detection()')
            await yaw_until_object_detection()

        cv_object_yaw = CV().bounding_boxes[cv_object].yaw

    logger.info(f'{cv_object} centered.')

    await correct_depth()

@task
async def yaw_to_cv_object(self: Task, cv_object: CVObjectType, direction=1,
                           yaw_threshold=math.radians(40), latency_threshold=10,
                           depth_level=0.5) -> Task[None, None, None]:
    """
    Corrects the yaw relative to the CV object
    """
    DEPTH_LEVEL = State().orig_depth - depth_level
    MAXIMUM_YAW = math.radians(20)
    SCALE_FACTOR = 0.5 # How much the correction should be scaled down from yaw calculation

    logger.info('Starting yaw_to_cv_object')

    async def correct_depth():
        # await move_tasks.depth_correction(DEPTH_LEVEL, parent=self)
        await move_tasks.depth_correction(desired_depth=DEPTH_LEVEL, parent=self)

    def get_step_size(desired_yaw):
        # desired yaw in radians
        return min(abs(desired_yaw), MAXIMUM_YAW)

    def get_yaw_threshold(desired_yaw, cv_x):
        if cv_x > 10:
            yaw_threshold = desired_yaw * 2
        if cv_x > 6:
            yaw_threshold = desired_yaw * 1.5
        else:
            yaw_threshold = desired_yaw * 1

        return yaw_threshold

    async def yaw_until_object_detection():
        while not CV().is_receiving_recent_cv_data(cv_object, latency_threshold):
            logger.info(f'No {cv_object} detection, setting yaw setpoint {MAXIMUM_YAW}')
            await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, MAXIMUM_YAW * direction),
                                                pose_tolerances=Twist(linear=Vector3(x=0.05, y=0.05, z=0.05), angular=Vector3(x=0.2, y=0.3, z=0.2)),
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

    while abs(cv_object_yaw) > get_yaw_threshold(yaw_threshold, CV().bounding_boxes[cv_object].coords.x):
        sign_cv_object_yaw = np.sign(cv_object_yaw)
        correction = get_step_size(SCALE_FACTOR*cv_object_yaw) # Scale down CV yaw value

        if get_robot_name() == RobotName.OOGWAY:
            desired_yaw = sign_cv_object_yaw * correction
        else:
            desired_yaw = -1 * sign_cv_object_yaw * correction

        logger.info(f'Detected yaw {cv_object_yaw} is greater than threshold {yaw_threshold}. Actually yawing: {desired_yaw}')
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, desired_yaw),
                                            pose_tolerances = move_tasks.create_twist_tolerance(angular_yaw = 0.15),
                                            parent=self)
        await correct_depth()
        await Yield()

        if (not CV().is_receiving_recent_cv_data(cv_object, latency_threshold)):
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
    logger.info(f'yaw_threshold = {YAW_THRESHOLD}')

    while abs(path_marker_yaw) > YAW_THRESHOLD:
        sign_path_marker_yaw = np.sign(path_marker_yaw)
        correction = get_step_size(path_marker_yaw)
        desired_yaw = sign_path_marker_yaw * correction

        logger.info(f'Detected yaw {path_marker_yaw} is greater than threshold {YAW_THRESHOLD}. Yawing: {desired_yaw}')
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, desired_yaw), parent=self)
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

    def is_receiving_bin_data(bin_object: CVObjectType, last_detection_time) -> bool:
        width = CV().bounding_boxes[bin_object].width
        height = CV().bounding_boxes[bin_object].height

        return width * height >= AREA_THRESHOLD \
            and CV().is_receiving_recent_cv_data(bin_object, LATENCY_THRESHOLD, last_detection_time)

    async def move_x(step: float = 1) -> None:
        await move_tasks.move_x(step=step, parent=self)

    def stabilize() -> None:
        pose_to_hold = copy.deepcopy(State().state.pose.pose)
        Controls().publish_desired_position(pose_to_hold)

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
            is_receiving_blue_bin_data = is_receiving_bin_data(CVObjectType.BIN_BLUE, bin_blue_time)

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
        await util_tasks.sleep(5, parent=self)

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

    def is_receiving_bin_data(bin_object: CVObjectType, last_detection_time) -> bool:
        width = CV().bounding_boxes[bin_object].width
        height = CV().bounding_boxes[bin_object].height

        return width * height >= AREA_THRESHOLD \
            and CV().is_receiving_recent_cv_data(bin_object, LATENCY_THRESHOLD, last_detection_time)

    def stabilize() -> None:
        pose_to_hold = copy.deepcopy(State().state.pose.pose)
        Controls().publish_desired_position(pose_to_hold)

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

                await util_tasks.sleep(0.1, parent=self)
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

    drop_marker = Servos().drop_marker

    async def correct_x(target: CVObjectType) -> None:
        await cv_tasks.correct_x(prop=target, parent=self)

    async def correct_y(target: CVObjectType) -> None:
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
            keep_orientation=True,
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
    await track_bin(target=CVObjectType.BIN_RED, desired_depth=START_DEPTH_LEVEL, pixel_threshold=START_PIXEL_THRESHOLD)

    await correct_yaw()

    await correct_depth(desired_depth=MID_DEPTH_LEVEL)
    await track_bin(target=CVObjectType.BIN_RED, desired_depth=MID_DEPTH_LEVEL, pixel_threshold=MID_PIXEL_THRESHOLD,
                    step_size=0.18, y_offset=30, x_offset=25)

    drop_marker(MarkerDropperStates.LEFT)
    logger.info('Dropped left marker')
    await util_tasks.sleep(3, parent=self)

    drop_marker(MarkerDropperStates.RIGHT)
    logger.info('Dropped right marker')
    await util_tasks.sleep(2, parent=self)

    await correct_depth(desired_depth=START_DEPTH_LEVEL)
    logger.info(f'Corrected depth to {START_DEPTH_LEVEL}')

    logger.info('Completed bin task')


@task
async def octagon_task(self: Task, direction: int = 1) -> Task[None, None, None]:
    """
    Detects, move towards the yellow bins, then surfaces inside the octagon. Requires robot to have submerged 0.7 meters.
    """
    logger.info('Starting octagon task')

    DEPTH_LEVEL_AT_BINS = State().orig_depth - 1.2 # Depth for beginning of task and corrections during forward movement
    DEPTH_LEVEL_ABOVE_BINS = State().orig_depth - 0.9 # Depth for going above bin before forward move
    LATENCY_THRESHOLD = 2 # Latency for seeing the bottom bin
    CONTOUR_SCORE_THRESHOLD = 2000 # Required bottom bin area for valid detection

    SCORE_THRESHOLD = 7500 # Area of bin before beginning surface logic for front camera
    POST_FRONT_THRESHOLD_FORWARD_DISTANCE = 0.4 # in meters

    # Forward navigation case constants
    LOW_SCORE = 1500
    LOW_STEP_SIZE = 0.75
    MED_SCORE = 3500
    MED_STEP_SIZE = 0.5
    HIGH_SCORE = 5000
    HIGH_STEP_SIZE = 0.35
    VERY_HIGH_STEP_SIZE = 0.2

    # LOW_SCORE = 1000
    # LOW_STEP_SIZE = 1.75
    # MED_SCORE = 2000
    # MED_STEP_SIZE = 1.25
    # HIGH_SCORE = 4000
    # HIGH_STEP_SIZE = 0.75
    # VERY_HIGH_STEP_SIZE = 0.25

    async def correct_depth(desired_depth):
        if abs(State().depth - desired_depth) < 0.15:
            logger.info(f'Depth delta is below threshold, skipping.')
            return
        await move_tasks.correct_depth(desired_depth=desired_depth, parent=self)

    async def correct_yaw():
        yaw_correction = CV().bounding_boxes[CVObjectType.BIN_PINK_FRONT].yaw
        logger.info(f'Yaw correction: {yaw_correction}')
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, yaw_correction * 0.7),
            keep_orientation=True,
            parent=self,
        )
        logger.info('Corrected yaw')

    self.correct_yaw = correct_yaw

    def is_receiving_pink_bin_data(latest_detection_time):
        return latest_detection_time and CVObjectType.BIN_PINK_BOTTOM in CV().bounding_boxes and \
            CV().bounding_boxes[CVObjectType.BIN_PINK_BOTTOM].score >= CONTOUR_SCORE_THRESHOLD and \
            Clock().now().seconds_nanoseconds()[0] - CV().bounding_boxes[CVObjectType.BIN_PINK_BOTTOM].header.stamp.sec < LATENCY_THRESHOLD and \
            abs(CV().bounding_boxes[CVObjectType.BIN_PINK_BOTTOM].header.stamp.secs - latest_detection_time) < LATENCY_THRESHOLD

    async def move_x(step=1) -> None:
        await move_tasks.move_x(step=step, parent=self)

    def stabilize() -> None:
        pose_to_hold = copy.deepcopy(State().state.pose.pose)
        Controls().publish_desired_position(pose_to_hold)

    def get_step_size(last_step_size):
        bin_pink_score = CV().bounding_boxes[CVObjectType.BIN_PINK_FRONT].score
        step = 0

        if bin_pink_score < LOW_SCORE:
            step = LOW_STEP_SIZE
        elif bin_pink_score < MED_SCORE:
            step = MED_STEP_SIZE
        elif bin_pink_score < HIGH_SCORE:
            step = HIGH_STEP_SIZE
        else:
            step = VERY_HIGH_STEP_SIZE

        return min(step, last_step_size)

    async def move_to_pink_bins() -> None:
        logger.info("Beginning move to bins")

        count = 1
        latest_detection_time = None
        moved_above = False

        if not is_receiving_pink_bin_data(latest_detection_time):
            latency_threshold = 10
            MAXIMUM_YAW = math.radians(30)
            step = 1
            while not CV().is_receiving_recent_cv_data(CVObjectType.BIN_PINK_FRONT, latency_threshold):
                if step <= 3:
                    angle = MAXIMUM_YAW
                elif step == 4:
                    angle = -2 * MAXIMUM_YAW
                elif step <= 8:
                    angle = -1 * MAXIMUM_YAW
                else:
                    angle = 3 * MAXIMUM_YAW
                    logger.info(f'Yawed to find object more than  times, breaking loop.')

                logger.info(f'No {CVObjectType.BIN_PINK_FRONT} detection, setting yaw setpoint {angle}')
                await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, angle * direction),
                                                    depth_level=DEPTH_LEVEL_AT_BINS,
                                                    pose_tolerances=Twist(linear=Vector3(x=0.05, y=0.05, z=0.05), angular=Vector3(x=0.2, y=0.3, z=0.3)),
                                                    time_limit=10,
                                                    parent=self)

                if step > 8:
                    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0.5, 0, 0, 0, 0, 0),
                                                    depth_level=DEPTH_LEVEL_AT_BINS,
                                                    pose_tolerances=Twist(linear=Vector3(x=0.05, y=0.05, z=0.05), angular=Vector3(x=0.2, y=0.3, z=0.3)),
                                                    time_limit=10,
                                                    parent=self)
                    break

                # await correct_depth()
                step += 1
                await Yield()

        last_step_size = float('inf')
        while not is_receiving_pink_bin_data(latest_detection_time) and not moved_above:
            if CVObjectType.BIN_PINK_BOTTOM in CV().bounding_boxes:
                latest_detection_time = CV().bounding_boxes[CVObjectType.BIN_PINK_BOTTOM].header.stamp.sec
                check = is_receiving_pink_bin_data(latest_detection_time)
                if check:
                    logger.info(f'Bottom camera detects bin, ending loop.')
                    break

            logger.info("Have not moved above, depth correcting and beginning sequence.")
            await correct_depth(DEPTH_LEVEL_AT_BINS if not moved_above else DEPTH_LEVEL_ABOVE_BINS)
            if not moved_above:
                logger.info("Yawing to pink bin front")
                status = await yaw_to_cv_object(CVObjectType.BIN_PINK_FRONT, direction=direction, yaw_threshold=math.radians(15),
                                       depth_level=1.2, parent=Task.MAIN_ID)
                # At this point, we cannot find the object anymore. we hope to surface in the octagon atp
                if not status:
                    logger.info("Yaw failed, ending.")
                    break

            logger.info(f'Bin pink front score: {CV().bounding_boxes[CVObjectType.BIN_PINK_FRONT].score}')

            if CV().bounding_boxes[CVObjectType.BIN_PINK_FRONT].score > SCORE_THRESHOLD and not moved_above:
                logger.info(f'Beginning to move above bin')
                await correct_depth(DEPTH_LEVEL_ABOVE_BINS)
                moved_above = True
                logger.info('Moved above pink bins, based on front camera.')

            if not moved_above:
                step = get_step_size(last_step_size)
                logger.info(f'Moving step size {step}')
                await move_x(step=step)
                logger.info(f'Finished forwards move')
                last_step_size = step

            await Yield()

            count += 1

            logger.info(f'Receiving bottom bin data: {is_receiving_pink_bin_data(latest_detection_time)}')

        if moved_above:
            await move_tasks.move_with_directions([(POST_FRONT_THRESHOLD_FORWARD_DISTANCE, 0, 0)], parent=self)
        else:
            logger.info('Detected bin_pink_bottom, or yaw has failed (and copium ensues)')

        logger.info('Reached pink bins, stabilizing...')
        stabilize()
        await util_tasks.sleep(5, parent=self)

    async def face_fish(yaw_left: bool = True, closer_banner: bool = True):
        direction = 1 if yaw_left else -1
        yaw_distance = np.pi/4 if closer_banner else 3*np.pi/4
        await orient_to_wall(parent=self)
        await move_tasks.yaw_from_local_pose(direction*yaw_distance, parent=self)

    await move_to_pink_bins()
    await face_fish(yaw_left=True, closer_banner=True)

    logger.info('Surfacing...')
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, State().orig_depth - State().depth, 0, 0, 0),
                                        time_limit=10, parent=self)
    logger.info('Finished surfacing')

@task
async def torpedo_task(self: Task, first_target: CVObjectType, depth_level=0.5, direction=1) -> Task[None, None, None]:
    # await scan_for_torpedo()
    logger.info('Starting torpedo task')
    DEPTH_LEVEL = State().orig_depth - depth_level

    async def correct_y() -> Coroutine[None, None, None]:
        await cv_tasks.correct_y(prop=CVObjectType.TORPEDO_BANNER, parent=self)

    async def correct_z() -> Coroutine[None, None, None]:
        await cv_tasks.correct_z(prop=CVObjectType.TORPEDO_BANNER, parent=self)

    async def correct_depth() -> Coroutine[None, None, None]:
        await move_tasks.correct_depth(desired_depth=DEPTH_LEVEL, parent=self)

    async def move_x(step: float = 1) -> Coroutine[None, None, None]:
        logger.info(f"Moving forward {step}")
        await move_tasks.move_x(step=step, parent=self)

    def get_step_size(dist:float, dist_threshold:float) -> float:
        goal_step = 0.25
        if dist > 10:
            goal_step = 4
        elif dist > 6:
            goal_step = 3
        elif dist > 4:
            goal_step = 1
        elif dist > 1.5:
            goal_step = 0.5
        return min(dist - dist_threshold + 0.1, goal_step)

    async def move_to_torpedo(torpedo_dist_threshold=2):
        await yaw_to_cv_object(CVObjectType.TORPEDO_BANNER, direction=direction, yaw_threshold=math.radians(15),
        depth_level=depth_level, parent=self)
        torpedo_dist = CV().bounding_boxes[CVObjectType.TORPEDO_BANNER].coords.x
        await correct_y()
        await correct_depth()

        while torpedo_dist > torpedo_dist_threshold:
            logger.info(f"Torpedo dist x: {CV().bounding_boxes[CVObjectType.TORPEDO_BANNER].coords.x}")
            logger.info(f"Torpedo dist y: {CV().bounding_boxes[CVObjectType.TORPEDO_BANNER].coords.y}")
            await move_x(step=get_step_size(torpedo_dist, torpedo_dist_threshold))

            await yaw_to_cv_object(CVObjectType.TORPEDO_BANNER, direction=-1, yaw_threshold=math.radians(15),
            depth_level=depth_level, parent=self)
            logger.info(f"Yaw corrected")
            await correct_y()

            if torpedo_dist < 3:
                await correct_z() # CV-based z-axis correction
            else:
                await correct_depth()

            await Yield()
            torpedo_dist = CV().bounding_boxes[CVObjectType.TORPEDO_BANNER].coords.x

        logger.info(f"Finished moving forwards to torpedo: {torpedo_dist}m away")

        await correct_z()


    await move_to_torpedo()
    logger.info(f"Finished moving forwards to torpedo")

    # Small offset to counteract camera positioning
    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, -0.5, 0.2, 0, 0, 0),
        parent=self,
    )

    # Determine which animal to target first and which to target second
    if first_target == CVObjectType.TORPEDO_REEF_SHARK_TARGET:
        second_target = CVObjectType.TORPEDO_SAWFISH_TARGET
    elif first_target == CVObjectType.TORPEDO_SAWFISH_TARGET:
        second_target = CVObjectType.TORPEDO_REEF_SHARK_TARGET
    else:
        raise ValueError(f"Invalid first_animal: {first_target}. Must be CVObjectType.TORPEDO_REEF_SHARK_TARGET or CVObjectType.TORPEDO_SAWFISH_TARGET")

    # Center to first animal
    animal = first_target
    target_y = CV().bounding_boxes[animal].coords.y
    target_z = CV().bounding_boxes[animal].coords.z+0.1
    logger.info(f"Aligning to first target {animal} at y={target_y} and z={target_z}")
    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, target_y, target_z, 0, 0, 0),
        parent=self,
    )

    # Fire first torpedo
    logger.info(f"Firing torpedo RIGHT")
    await Servos().fire_torpedo(TorpedoStates.RIGHT)

    # Wait for torpedo servo is available
    await util_tasks.sleep(Duration(seconds=3), parent=self)

    # Move back to see full banner
    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, -target_y, -target_z, 0, 0, 0),
        parent=self,
    )

    # Center to second animal
    animal = second_target
    target_y = CV().bounding_boxes[animal].coords.y - 0.1
    target_z = CV().bounding_boxes[animal].coords.z + 0.1
    logger.info(f"Aligning to second target {animal} at y={target_y} and z={target_z}")
    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, target_y, target_z, 0, 0, 0),
        parent=self,
    )

    # Fire second torpedo
    logger.info(f"Firing torpedo LEFT")
    await Servos().fire_torpedo(TorpedoStates.LEFT)

    logger.info(f"Torpedo task completed")

@task
async def torpedo_task_old(self: Task,
                       target_animal: Literal[CVObjectType.TORPEDO_SAWFISH,
                                              CVObjectType.TORPEDO_REEF_SHARK] = CVObjectType.TORPEDO_REEF_SHARK,
                       depth_level: float = 0.5) -> Task[None, None, None]:
    """
    TODO: Test this task
    """
    step_size = 1
    shooting_distance = 1

    async def correct_depth() -> None:
        await move_tasks.correct_depth(desired_depth=depth_level, parent=self)

    async def yaw_to_torpedo_banner() -> None:
        await yaw_to_cv_object(CVObjectType.TORPEDO_BANNER, direction=1, yaw_threshold=math.radians(15),
                               depth_level=depth_level, parent=Task.MAIN_ID)

    def get_step_size(dist: float) -> float:
        return min(dist - step_size, shooting_distance)

    async def correct_y() -> Coroutine[None, None, None]:
        await cv_tasks.correct_y(prop=CVObjectType.TORPEDO_BANNER, parent=self)

    async def correct_z() -> Coroutine[None, None, None]:
        await cv_tasks.correct_z(prop=CVObjectType.TORPEDO_BANNER, parent=self)

    async def move_x(step:float = 1) -> Coroutine[None, None, None]:
        await move_tasks.move_x(step=step, parent=self)

    # async def correct_yaw_with_depthai() -> None:
    #     yaw_correction = CV().bounding_boxes[CVObjectType.TORPEDO_BANNER].yaw * 0.3
    #     logger.info(f'Yaw correction: {yaw_correction}')
    #     sign = 1 if yaw_correction > 0.1 else (-1 if yaw_correction < -0.1 else 0)
    #     await move_tasks.move_to_pose_local(
    #         geometry_utils.create_pose(0, 0, 0, 0, 0, yaw_correction + (sign * 0.1)),
    #         keep_orientation=True,
    #         parent=self,
    #     )
    #     logger.info('Corrected yaw')

    async def move_to_torpedo_banner():
        await yaw_to_torpedo_banner()
        # await correct_depth()

        banner_dist = CV().bounding_boxes[CVObjectType.TORPEDO_BANNER].coords.x
        logger.info(f'banner x dist: {banner_dist}')
        while banner_dist > shooting_distance:
            logger.info(f'step_size: {get_step_size(banner_dist)}')
            await move_x(step=get_step_size(banner_dist))
            await yaw_to_torpedo_banner()
            logger.info(f"Torpedo banner dist: {CV().bounding_boxes[CVObjectType.TORPEDO_BANNER].coords.x}")
            await correct_y()
            if banner_dist < 3:
                await correct_z()
            # else:
            #     await correct_depth()

            await Yield()
            banner_dist = CV().bounding_boxes[CVObjectType.TORPEDO_BANNER].coords.x
            logger.info(f"Torpedo banner dist: {CV().bounding_boxes[CVObjectType.TORPEDO_BANNER].coords.x}")

    await move_to_torpedo_banner()

    async def center_with_torpedo_target():
        # await correct_yaw_with_depthai() # Hopefully we know we are normal to the torpedo banner now and also centered with the banner.
        # await correct_depth()
        target_dist_y = CV().bounding_boxes[target_animal].coords.y
        target_dist_z = CV().bounding_boxes[target_animal].coords.z
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, target_dist_y, target_dist_z, 0, 0, 0),
            keep_orientation=True,
            parent=self,
        )
        logger.info(f'Centered on torpedo target, y: {CV().bounding_boxes[CVObjectType.TORPEDO_BANNER].coords.y}, \
                                                  z: {CV().bounding_boxes[CVObjectType.TORPEDO_BANNER].coords.z}')

        # Move the Z to account for distance between camera and torpedo launcher
        offset = -0.05
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, offset, 0, 0, 0),
            keep_orientation=True,
            parent=self,
        )

        logger.info('FIRING TORPEDO')
        # await Servos().fire_torpedo(TorpedoStates.RIGHT)

    await center_with_torpedo_target()

@task
async def ivc_send_then_receive(self: Task[None, None, None], msg_to_send: IVCMessageType, msg_to_receive: IVCMessageType, timeout: float = 60) -> Task[None, None, None]:
    await ivc_tasks.ivc_send(msg_to_send, parent = self) # Send crush is done with gate

    count = 2
    # Wait for Oogway to say starting/acknowledge command
    while count != 0 and await ivc_tasks.ivc_receive(timeout = timeout, parent = self) != msg_to_receive:
        logger.info(f'Unexpected message received. Remaining attempts: {count}')
        count -= 1

@task
async def ivc_receive_then_send(self: Task[None, None, None], msg: IVCMessageType, timeout: float = 60) -> Task[None, None, None]:
    if timeout <= 0:
        return

    # Wait until Crush is done with gate
    while await ivc_tasks.ivc_receive(timeout = timeout, parent = self) != IVCMessageType.CRUSH_GATE:
        logger.info(f'Unexpected message received.')

    await ivc_tasks.ivc_send(msg, parent = self) # Oogway says ok and starting

@task
async def delineate_ivc_log(self: Task[None, None, None]) -> Task[None, None, None]:
    """Append a header to the IVC log file."""
    seconds, nanoseconds = Clock().now().seconds_nanoseconds()
    timestamp = ivc_tasks.ros_timestamp_to_pacific_time(seconds, nanoseconds)

    with open("ivc_log.txt", "a") as f:
        f.write(f"----- NEW RUN STARTED AT {timestamp} -----\n")

@task
async def add_to_ivc_log(self: Task[None, None, None], message: str) -> Task[None, None, None]:
    """Add a message to the IVC log file."""
    with open("ivc_log.txt", "a") as f:
        f.write(f'{message}\n')

@task
async def send_torpedo_ivc(self: Task[None, None, None]) -> Task[None, None, None]:
    """Fu"""
    await ivc_tasks.ivc_send(IVCMessageType.OOGWAY_TORPEDOES, parent = self)
    await util_tasks.sleep(5, parent=self)
    await ivc_tasks.ivc_send(IVCMessageType.OOGWAY_TORPEDOES, parent = self)
    await util_tasks.sleep(5, parent=self)
    await ivc_tasks.ivc_send(IVCMessageType.OOGWAY_TORPEDOES, parent = self)

@task
async def orient_to_wall(self: Task[None, None, None],
                         start_angle: float = -15.0,
                         end_angle: float = 15.0,
                         distance: float = 20.0) -> Task[None, None, None]:
    """
    Orient the robot to a wall using sonar sweep.
    """
    async def get_sonar_normal_angle() -> float:
        """
        Returns the yaw angle of the wall in degrees.
        """
        # Call sonar sweep request
        sonar_future = Sonar().sweep(start_angle, end_angle, distance)
        if sonar_future is not None:
            logger.info("Sonar sweep request sent, waiting for response...")
            # Wait for the sonar sweep to complete
            sonar_response = await sonar_future
            logger.info(f"Sonar sweep response received: {sonar_response}")
            return sonar_response.normal_angle
        else:
            logger.warning("Sonar sweep request failed - bypass mode or service unavailable")
            return math.nan

    def convert_sonar_output_to_yaw(sonar_normal: float) -> float:
        """
        Returns how much robot needs to yaw to be normal to surface it scans. Input degrees, output radians.
        """
        yaw_in_degrees = 180 - sonar_normal
        return yaw_in_degrees * np.pi / 180

    sonar_output = await get_sonar_normal_angle()

    if not math.isnan(sonar_output):
        yaw_delta = convert_sonar_output_to_yaw(sonar_output)
        logger.info(f"Yaw delta from sonar sweep: {yaw_delta} radians")
        # Move to the desired yaw angle
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, yaw_delta),
            time_limit=10,
            parent=self,
        )
    else:
        logger.warning("No yaw delta received from sonar sweep, skipping orientation.")
