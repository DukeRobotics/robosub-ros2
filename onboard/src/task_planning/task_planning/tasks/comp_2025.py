# ruff: noqa

"""Entirety of competition tasks used for the RoboSub 2025 competition, archived for reference."""

import copy
import math
from collections.abc import Coroutine

import numpy as np
from geometry_msgs.msg import Twist, Vector3
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.logging import get_logger
from transforms3d.euler import quat2euler

from task_planning.interface.controls import Controls
from task_planning.interface.cv import CV, CVObjectType
from task_planning.interface.servos import Servos, TorpedoStates
from task_planning.interface.state import State
from task_planning.interface.ivc import IVCMessageType
from task_planning.task import Task, Yield, task
from task_planning.tasks import cv_tasks, move_tasks, util_tasks, ivc_tasks
from task_planning.utils import geometry_utils
from task_planning.utils.other_utils import get_robot_name, RobotName

from rclpy.clock import Clock

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
    await util_tasks.sleep(2.5, parent=self)

    await move_tasks.depth_correction(DEPTH_LEVEL, parent=self)
    await roll()
    State().reset_pose()
    await util_tasks.sleep(2.5, parent=self)

    await move_tasks.depth_correction(DEPTH_LEVEL, parent=self)
    await util_tasks.sleep(2.5, parent=self)

    imu_orientation = State().imu.orientation
    euler_angles = quat2euler([imu_orientation.w, imu_orientation.x, imu_orientation.y, imu_orientation.z])
    roll_correction = -euler_angles[0]
    pitch_correction = -euler_angles[1]

    logger.info(f'Roll, pitch, yaw correction: {roll_correction, pitch_correction}')
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, roll_correction, pitch_correction, 0),
                                        parent=self)
    logger.info('Reset orientation')


@task
async def initial_submerge(self: Task, submerge_dist: float, z_tolerance: float = 0.1, enable_controls_flag: bool = False, time_limit: int = 30) -> Task[None, None, None]:
    """
    Submerge the robot a given amount.

    Args:
        submerge_dist: The distance to submerge the robot in meters.
        enable_controls_flag: Flag to wait for ENABLE_CONTROLS status when true.
    """
    logger.info("Starting initial submerge")

    while enable_controls_flag and not Controls().enable_controls_status.data:
        await Yield()

    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, submerge_dist, 0, 0, 0),
        keep_orientation=True,
        pose_tolerances=move_tasks.create_twist_tolerance(linear_z=z_tolerance),
        time_limit=time_limit,
        parent=self,
    )
    logger.info(f'Submerged {submerge_dist} meters')


@task
async def coin_flip(self: Task, depth_level=0.7, enable_same_direction=True, time_limit: int=15) -> Task[None, None, None]:
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
                    pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
                    time_limit=time_limit,
                    parent=self,
                )

                logger.info('Yaw correct 180')

            logger.info(f'Yaw correct remainder: {yaw_correction}')
            await move_tasks.move_to_pose_local(
                geometry_utils.create_pose(0, 0, 0, 0, 0, yaw_correction),
                parent=self,
            )
    else:
        while abs(get_gyro_yaw_correction(return_raw=True)) > math.radians(5):
            yaw_correction = get_gyro_yaw_correction(return_raw=False)
            logger.info(f'Yaw correction: {yaw_correction}')

            await move_tasks.move_to_pose_local(
                geometry_utils.create_pose(0, 0, 0, 0, 0, yaw_correction),
                time_limit=time_limit,
                parent=self,
            )

    logger.info('Step Completed')
    logger.info(f'Final yaw offset: {get_gyro_yaw_correction(return_raw=True)}')

    depth_delta = DEPTH_LEVEL - State().depth
    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, depth_delta, 0, 0, 0),
        keep_orientation=True,
        parent=self)
    logger.info(f'Corrected depth {depth_delta}')

    logger.info('Completed coin flip')


# NOTE: This task was not used in the 2025 competition flow, but it is kept for reference
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
        return max(dist - 2.75, 0.25)

    logger.info('Begin sleep')
    await util_tasks.sleep(2, parent=self)
    logger.info('End sleep')

    gate_dist = CV().bounding_boxes[CVObjectType.GATE_SAWFISH].coords.x
    await correct_depth()
    num_corrections = 0
    while gate_dist > 3:
        await move_x(step=get_step_size(gate_dist))
        await yaw_to_cv_object(CVObjectType.GATE_SAWFISH, direction=-1, yaw_threshold=math.radians(10),
                               latency_threshold=2, depth_level=0.6, parent=self),
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
async def gate_task_dead_reckoning(self: Task, depth_level=0.7) -> Task[None, None, None]:
    DEPTH_LEVEL = State().orig_depth - depth_level
    logger.info('Started gate task')
    if get_robot_name() == RobotName.OOGWAY:
        # Go through gate
        await move_tasks.move_with_directions([(3, 0, 0)], depth_level=depth_level, correct_depth=True, correct_yaw=True, parent=self)
        await move_tasks.move_with_directions([(3, 0, 0)], depth_level=depth_level, correct_depth=True, correct_yaw=True, parent=self)

        # Dead reckon a bit to torpedo
        await move_tasks.move_with_directions([(0, 3, 0)], depth_level=depth_level, correct_depth=True, correct_yaw=True, parent=self)
        await move_tasks.move_with_directions([(0, 3, 0)], depth_level=depth_level, correct_depth=True, correct_yaw=True, parent=self)
    elif get_robot_name() == RobotName.CRUSH:
        directions = [
            (2, 0, 0),
            (3, 0, 0),
        ]
        await move_tasks.move_with_directions(directions, depth_level=DEPTH_LEVEL, correct_depth=True, correct_yaw=True, keep_orientation=True, time_limit=15, parent=self)
    logger.info('Moved through gate, and strafed.')


@task
async def slalom_task_dead_reckoning(self: Task, depth_level=1.1) -> Task[None, None, None]:
    DEPTH_LEVEL = State().orig_depth - depth_level
    logger.info('Started slalom task')
    if get_robot_name() == RobotName.OOGWAY:
        pass
    elif get_robot_name() == RobotName.CRUSH:
        directions = [
            (2, 0, 0),
            (2, 0, 0),
            (2, 0, 0),
        ]
        await move_tasks.move_with_directions(directions, depth_level=DEPTH_LEVEL, correct_depth=True, correct_yaw=True, keep_orientation=True, time_limit=20, parent=self)
    logger.info('Moved through slalom')


@task
async def slalom_to_octagon_dead_reckoning(self: Task, depth_level=1.1) -> Task[None, None, None]:
    DEPTH_LEVEL = State().orig_depth - depth_level
    latency_threshold = 10

    async def face_fish(yaw_left: bool = True, closer_banner: bool = True):
        direction = 1 if yaw_left else -1
        yaw_distance = np.pi / 4 if closer_banner else 3*np.pi/4
        await orient_to_wall(parent=self)
        await orient_to_wall(parent=self)
        await move_tasks.yaw_from_local_pose(direction*yaw_distance, parent=self)

    logger.info('Started slalom task')
    if get_robot_name() == RobotName.OOGWAY:
        pass
    elif get_robot_name() == RobotName.CRUSH:
        before_cv_directions = [
            (2, 0, 0),
            (2, 0, 0),
        ]
        await move_tasks.move_with_directions(before_cv_directions, depth_level=DEPTH_LEVEL, correct_depth=True, correct_yaw=True, keep_orientation=True, time_limit=15, parent=self)

        logger.info("Checking yellow bin detection")
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
                logger.info('Yawed to find object more than 9 times, breaking loop.')

            # TODO: some bugs here
            logger.info(f'No {cv_object} detection, setting yaw setpoint {angle}')
            await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, angle * direction),
                                                depth_level=depth_level,
                                                pose_tolerances=Twist(linear=Vector3(x=0.05, y=0.05, z=0.05), angular=Vector3(x=0.2, y=0.3, z=0.3)),
                                                time_limit=10,
                                                parent=self)

            if step > 8:
                break

            step += 1
            await Yield()

        after_cv_directions = [(2, 0, 0)]
        await move_tasks.move_with_directions(after_cv_directions, depth_level=DEPTH_LEVEL, correct_depth=True, correct_yaw=True, keep_orientation=True, time_limit=15, parent=self)

        await face_fish(yaw_left=False, closer_banner=False)

        logger.info('Surfacing...')
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, State().orig_depth - State().depth, 0, 0, 0),
                                            time_limit=10, parent=self)
        logger.info('Finished surfacing')


# NOTE: This task was not used in the 2025 competition flow, but it is kept for reference
@task
async def return_task_dead_reckoning(self: Task, depth_level=0.7) -> Task[None, None, None]:
    logger.info('Started gate return task')
    DEPTH_LEVEL = State().orig_depth - depth_level
    if get_robot_name() == RobotName.OOGWAY:
        pass
    elif get_robot_name() == RobotName.CRUSH:
        directions = [
            (-3, 0, 0),
            (-3, 0, 0),
            (-3, 0, 0),
            (-3, 0, 0),
        ]
        await move_tasks.move_with_directions(directions, depth_level=DEPTH_LEVEL, correct_depth=True, correct_yaw=True, keep_orientation=True, time_limit=15, parent=self)
        logger.info('Moved through gate return')


# NOTE: This task was not used in the 2025 competition flow, but it is kept for reference
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

        if (not CV().is_receiving_recent_cv_data(cv_object, latency_threshold)):
            logger.info(f'{cv_object} detection lost, running yaw_until_object_detection()')
            await yaw_until_object_detection()

        cv_object_yaw = CV().bounding_boxes[cv_object].yaw

    logger.info(f'{cv_object} centered.')

    await correct_depth()


@task
async def yaw_to_cv_object(self: Task, cv_object: CVObjectType, direction=1,
                           yaw_threshold=math.radians(40), latency_threshold=10,
                           depth_level=0.5) -> Task[None, None, None] | bool:
    """
    Corrects the yaw relative to the CV object.
    """
    DEPTH_LEVEL = State().orig_depth - depth_level
    MAXIMUM_YAW = math.radians(35)
    SCALE_FACTOR = 0.4 # How much the correction should be scaled down from yaw calculation

    logger.info('Starting yaw_to_cv_object')

    async def correct_depth():
        await move_tasks.depth_correction(desired_depth=DEPTH_LEVEL, parent=self)

    def get_step_size(desired_yaw):
        # Desired yaw in radians
        return min(abs(desired_yaw), MAXIMUM_YAW)

    def get_yaw_threshold(desired_yaw, cv_x):
        if cv_x < 128 or cv_x > 512:
            yaw_threshold = desired_yaw * 2
        if cv_x < 256 or cv_x > 384:
            yaw_threshold = desired_yaw * 1.75
        else:
            yaw_threshold = desired_yaw * 1.35

        return yaw_threshold

    async def yaw_until_object_detection():
        logger.info("Beginning yaw_util_object_detection task")
        MAXIMUM_YAW = math.radians(30)
        step = 1
        while not CV().is_receiving_recent_cv_data(cv_object, latency_threshold):
            if step <= 3:
                angle = MAXIMUM_YAW
            elif step == 4:
                angle = -2 * MAXIMUM_YAW
            elif step <= 8:
                angle = -1 * MAXIMUM_YAW
            else:
                angle = 3 * MAXIMUM_YAW
                logger.info(f'Yawed to find object more than 9 times, breaking loop.')

            logger.info(f'No {cv_object} detection, setting yaw setpoint {angle}')
            await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, angle * direction),
                                                depth_level=depth_level,
                                                pose_tolerances=Twist(linear=Vector3(x=0.05, y=0.05, z=0.05), angular=Vector3(x=0.2, y=0.3, z=0.3)),
                                                time_limit=10,
                                                parent=self)

            if step > 8:
                return False

            # await correct_depth()
            step += 1
            await Yield()
        return True

    def robust_yaw_average(yaws):
        yaws_sorted = sorted(yaws)
        trimmed = yaws_sorted[1:-1]  # remove min and max
        return sum(trimmed) / len(trimmed)

    # Yaw until object detection
    found = await yaw_until_object_detection()

    # Could not find, so just surface and pray...
    if not found:
        return False

    logger.info(f'{cv_object} detected. Now centering {cv_object} in frame...')

    # Center detected object in camera frame

    # Take average of 5 numbers and use that for yaw to try and offset outliers
    logger.info('Taking in 5 frames to calculate yaw offset')
    cv_yaws = []
    for _ in range(0,5):
        cv_yaws.append(CV().bounding_boxes[cv_object].yaw)
        await util_tasks.sleep(0.25, parent=self)

    cv_object_yaw = robust_yaw_average(cv_yaws)

    await correct_depth()
    logger.info(f'abs(cv_object_yaw): {abs(cv_object_yaw)}')
    logger.info(f'yaw_threshold: {yaw_threshold}')

    step = 1
    while abs(cv_object_yaw) > get_yaw_threshold(yaw_threshold, CV().bounding_boxes[cv_object].coords.x):
        # If we have made 3 corrections already, trust that yaw is reasonable and continue forward
        if step > 3:
            logger.info(f'Yaw has been correcting more than 3 times, breaking loop.')
            break

        sign_cv_object_yaw = np.sign(cv_object_yaw)
        correction = get_step_size(SCALE_FACTOR * cv_object_yaw) # Scale down CV yaw value

        # Robot agnostic code base fails once again
        if get_robot_name() == RobotName.OOGWAY:
            desired_yaw = sign_cv_object_yaw * correction
        else:
            desired_yaw = -1 * sign_cv_object_yaw * correction

        # Actually do the yaw itself, and then correct depth
        logger.info(f'Detected yaw {cv_object_yaw} is greater than threshold {yaw_threshold}. Actually yawing: {desired_yaw}')
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, desired_yaw),
                                            pose_tolerances = move_tasks.create_twist_tolerance(angular_yaw = 0.15),
                                            parent=self)
        await correct_depth()
        await Yield()

        # Over correction
        if (not CV().is_receiving_recent_cv_data(cv_object, latency_threshold)):
            logger.info(f'{cv_object} detection lost, running yaw_until_object_detection()')
            await yaw_until_object_detection()

        # Recalculate the yaw
        logger.info('Taking in 5 frames to calculate yaw offset')
        cv_yaws = []
        for i in range(0,5):
            cv_yaws.append(CV().bounding_boxes[cv_object].yaw)
            await util_tasks.sleep(0.25, parent=self)

        cv_object_yaw = robust_yaw_average(cv_yaws)
        step += 1

    logger.info(f'{cv_object} centered, or limit has been reached.')

    await correct_depth()
    return True


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
    logger.info("Firing torpedo LEFT")
    await Servos().fire_torpedo(TorpedoStates.LEFT)

    logger.info("Torpedo task completed")


@task
async def orient_to_wall(self: Task[None, None, None],
                         end_angle: float = 15.0,
                         start_angle: float = -15.0,
                         distance: float = 10.0) -> Task[None, None, None]:
    """
    Orient the robot to a wall using sonar sweep.
    """
    def convert_sonar_output_to_yaw(sonar_normal: float) -> float:
        """
        Returns how much robot needs to yaw to be normal to surface it scans. Input degrees, output radians.
        """
        yaw_in_degrees = 180 - sonar_normal
        return yaw_in_degrees * np.pi / 180

    count = 2
    got_valid_sweep = False

    while not got_valid_sweep and count > 0:
        count -= 1

        # TODO: sonar_output not defined
        if sonar_output is None:
            logger.info(f"Did not get valid sweep object, trying again {count} times")
            continue

        got_valid_sweep = True

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


@task
async def crush_ivc_spam(self: Task[None, None, None], msg_to_send: IVCMessageType, timeout: float = 60) -> Task[None, None, None]:
    while True:
        await ivc_tasks.ivc_send(msg_to_send, parent = self) # Send crush is done with gate


@task
async def delineate_ivc_log(self: Task[None, None, None]) -> Task[None, None, None]:
    """Append a header to the IVC log file."""
    with open("ivc_log.txt", "a") as f:
        f.write("----- NEW RUN STARTED -----\n")

