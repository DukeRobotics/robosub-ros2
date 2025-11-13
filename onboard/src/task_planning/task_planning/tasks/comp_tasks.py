# ruff: noqa: ARG001, D103, D417, ERA001, N806, PLR2004, PLR0915

import math
from enum import Enum

import numpy as np
from custom_msgs.msg import ControlTypes
from geometry_msgs.msg import Twist, Vector3
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.logging import get_logger
from task_planning.interface.controls import Controls
from task_planning.interface.cv import CV, CVObjectType
from task_planning.interface.servos import MarkerDropperStates, TorpedoStates
from task_planning.interface.sonar import Sonar
from task_planning.interface.state import State
from task_planning.task import Task, Yield
from task_planning.tasks import move_tasks, servos_tasks, util_tasks
from task_planning.tasks.base_comp_task import CompTask, comp_task
from task_planning.utils import geometry_utils
from task_planning.utils.other_utils import RobotName, get_robot_name

# TODO: look into creating common higher level routines:
# - move_until_object_detection (e.g. path_marker_to_marker_dropper_bins)
# - spiral search (e.g. spiral_bin_search)
# - logarithmic search (stretch)
# - track and align with object center for bottom camera (e.g. search_for_bins & center_path_marker & track_bin)
# - track and move toward CV object (e.g. move_to_pink_bins & move_to_buoy & move_to_torpedo)
#     - takes in the termination condition function as a parameter
#     - can improve on cv_tasks.move_to_cv_obj implementation (or replace it completely)

# TODO: implement higher-level routines
# TODO: refactor sonar_tasks
# TODO: refactor ivc_tasks

logger = get_logger('comp_tasks')


@comp_task
async def initial_submerge(self: CompTask, depth_level: float, z_tolerance: float = 0.1,
                           enable_controls_flag: bool = False, timeout: int = 30) -> Task[None, None, None]:
    """
    Submerge the robot a given amount.

    Args:
        depth_level: The distance to submerge the robot in meters.
        enable_controls_flag: Flag to wait for ENABLE_CONTROLS status when true.
    """
    logger.info('[initial_submerge] Starting initial submerge')

    while enable_controls_flag and not Controls().enable_controls_status.data:
        await Yield()

    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, -depth_level, 0, 0, 0),
        keep_orientation=True,
        pose_tolerances=move_tasks.create_twist_tolerance(linear_z=z_tolerance),
        timeout=timeout,
        parent=self,
    )
    logger.info(f'[initial_submerge] Submerged {depth_level} meters')


@comp_task
async def coin_flip(self: CompTask, depth_level: float = 0.7,
                    enable_same_direction: bool = True, timeout: int = 15) -> Task[None, None, None]:
    """
    Perform the coin flip task, adjusting the robot's yaw and depth.

    The coin flip task involves correcting the robot's yaw to return it to its original orientation
    and then adjusting its depth. The task continuously calculates yaw corrections based on the difference
    between the current and original orientations, making incremental adjustments until the yaw is within
    a specified threshold. After correcting yaw, the robot adjusts its depth to reach the desired level.

    Args:
        self (CompTask): The task instance managing the execution of the coin flip task.
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

    if enable_same_direction:
        while abs(State().get_gyro_yaw_correction(return_raw=True)) > math.radians(5):
            yaw_correction = State().get_gyro_yaw_correction(return_raw=False, maximum_yaw=2*np.pi)
            logger.info(f'[coin_flip] Yaw correction: {yaw_correction}')

            if yaw_correction > np.pi:
                yaw_correction -= np.pi
                await self.correct_yaw(np.pi, timeout=timeout)
                logger.info('[coin_flip] Yaw correct 180')

            logger.info(f'[coin_flip] Yaw correct remainder: {yaw_correction}')
            await self.correct_yaw(yaw_correction, timeout=timeout)

    else:
        while abs(State().get_gyro_yaw_correction(return_raw=True)) > math.radians(5):
            yaw_correction = State().get_gyro_yaw_correction(return_raw=False, maximum_yaw=2*np.pi)
            logger.info(f'[coin_flip] Yaw correction: {yaw_correction}')

            await self.correct_yaw(yaw_correction, timeout=timeout)

    logger.info(f'[coin_flip] Final yaw offset: {State().get_gyro_yaw_correction(return_raw=True)}')

    await self.correct_depth(DEPTH_LEVEL)
    logger.info('[coin_flip] Completed coin flip')


@comp_task
async def gate_task(self: CompTask, offset: int = 0, direction: int = 1) -> Task[None, None, None]:
    """NOTE: This code is assuming we choose the sawfish side of the gate."""
    logger.info('[gate_task] Started gate task')

    DEPTH_LEVEL = State().orig_depth - 0.7

    def get_step_size(dist: float) -> float:
        dist_threshold = 4
        if dist > dist_threshold:
            return 1
        return max(dist - 2.75, 0.25)

    await util_tasks.sleep(2, parent=self)

    gate_dist = CV().bounding_boxes[CVObjectType.GATE_SAWFISH].coords.x
    # await self.correct_y_to_cv_obj(CVObjectType.GATE_SAWFISH, add_factor=0.2 + offset, mult_factor=0.5)
    await self.correct_depth(DEPTH_LEVEL)

    num_corrections = 0
    while gate_dist > 3:
        await self.move_x(step=get_step_size(gate_dist))

        await yaw_to_cv_object(CVObjectType.GATE_SAWFISH, direction=-1, yaw_threshold=math.radians(10),
                               latency_threshold=2, depth_level=0.6, parent=self)
        # await self.correct_y_to_cv_obj(CVObjectType.GATE_SAWFISH, add_factor=0.2 + offset,
        #                                mult_factor=(0.5 if num_corrections < 0 else 1))
        await self.correct_depth(DEPTH_LEVEL)

        await Yield()
        gate_dist = CV().bounding_boxes[CVObjectType.GATE_SAWFISH].coords.x
        logger.info(f'[gate_task] Gate dist: {gate_dist}')

        num_corrections += 1

    directions = [
        (2, 0, 0),
        (0, 0.2 * direction, 0),
        (2, 0, 0),
        (1, 0, 0),
    ]

    await self.move_with_directions(directions, correct_yaw=False)
    logger.info('[gate_task] Moved through gate')


@comp_task
async def gate_style_task(self: CompTask, depth_level: float = 0.9) -> Task[None, None, None]:
    """Complete two full barrel rolls."""
    logger.info('[gate_style_task] Started gate style task')

    DEPTH_LEVEL = State().orig_depth - depth_level

    async def roll() -> None:
        power = Twist()
        power.angular.x = 1.0
        Controls().publish_desired_power(power)
        logger.info('[gate_style_task] Published roll power')

        if get_robot_name() == RobotName.OOGWAY:
            await util_tasks.sleep(2.25, parent=self)
        else:
            await util_tasks.sleep(1.40, parent=self)

        logger.info('[gate_style_task] Completed roll')

        Controls().publish_desired_power(Twist())
        logger.info('[gate_style_task] Published zero power')

        await util_tasks.sleep(2, parent=self)
        logger.info('[gate_style_task] Completed zero')

    await self.correct_depth(DEPTH_LEVEL)
    await roll()
    State().reset_pose()
    await util_tasks.sleep(2.5, parent=self)

    await self.correct_depth(DEPTH_LEVEL)
    await roll()
    State().reset_pose()
    await util_tasks.sleep(2.5, parent=self)

    await self.correct_depth(DEPTH_LEVEL)
    await util_tasks.sleep(2.5, parent=self)

    await self.correct_roll_and_pitch()
    logger.info('[gate_style_task] Reset orientation')


@comp_task
async def gate_task_dead_reckoning(self: CompTask, depth_level: float = 0.7) -> Task[None, None, None]:
    logger.info('[gate_task_dead_reckoning] Started gate task')

    DEPTH_LEVEL = State().orig_depth - depth_level

    directions = []
    if get_robot_name() == RobotName.OOGWAY:
        directions = [
            # Go through gate
            (3, 0, 0),
            (3, 0, 0),
            # Dead reckon to torpedo
            (0, 3, 0),
            (0, 3, 0),
        ]

    elif get_robot_name() == RobotName.CRUSH:
        directions = [
            (2, 0, 0),
            (3, 0, 0),
        ]

    await self.move_with_directions(directions, depth_level=DEPTH_LEVEL, timeout=15)
    logger.info('[gate_task_dead_reckoning] Moved through gate, and strafed.')


@comp_task
async def yaw_until_object_detection(self: CompTask, cv_object: CVObjectType, depth_level: float = 0.7,
                                     latency_threshold: int = 10, direction: int = 1) -> Task[None, None, None] | bool:
    logger.info('[yaw_until_object_detection] Beginning yaw_util_object_detection task')

    MAXIMUM_YAW = math.radians(30)

    iteration = 1
    while not CV().is_receiving_recent_cv_data(cv_object, latency_threshold):
        if iteration <= 3:
            angle = MAXIMUM_YAW
        elif iteration == 4:
            angle = -2 * MAXIMUM_YAW
        elif iteration <= 8:
            angle = -1 * MAXIMUM_YAW
        else:
            angle = 3 * MAXIMUM_YAW
            logger.info(f'[yaw_until_object_detection] Yawed to find {cv_object} more than 9 times, breaking loop.')

        logger.info(f'[yaw_until_object_detection] No {cv_object} detection, setting yaw setpoint {angle}')
        await self.correct_yaw(angle * direction, yaw_tolerance=0.3, depth_level=depth_level, timeout=10)

        if iteration > 8:
            return False

        # await self.correct_depth(depth_level)
        iteration += 1
        await Yield()

    return True


@comp_task
async def yaw_to_cv_object(self: CompTask, cv_object: CVObjectType, direction: int = 1,
                           yaw_threshold: float = math.radians(40), latency_threshold: int = 10,
                           depth_level: float = 0.5,
                           use_position_control: bool = True) -> Task[None, None, None] | bool:
    """Corrects the yaw relative to the CV object."""
    DEPTH_LEVEL = State().orig_depth - depth_level
    MAXIMUM_YAW = math.radians(30)
    POSITION_SCALE_FACTOR = 0.4  # How much the correction should be scaled down from yaw calculation
    VELOCITY_SCALE_FACTOR = 0.1

    logger.info('[yaw_to_cv_object] Starting yaw_to_cv_object')

    def get_step_size(desired_yaw: float) -> float:
        # desired yaw in radians
        return min(abs(desired_yaw), MAXIMUM_YAW)

    def get_yaw_threshold(desired_yaw: float, cv_x: float) -> float:
        return desired_yaw * 1.75 if cv_x < 256 or cv_x > 384 else desired_yaw * 1.35

    async def get_robust_cv_object_yaw() -> float:
        # Take average of 5 numbers and use that for yaw to try and offset outliers
        logger.info('[yaw_to_cv_object] Taking in 5 frames to calculate yaw offset')

        cv_yaws = []
        for _ in range(5):
            cv_yaws.append(CV().bounding_boxes[cv_object].yaw)
            await util_tasks.sleep(0.25, parent=self)

        yaws_sorted = sorted(cv_yaws)
        trimmed = yaws_sorted[1:-1]  # remove min and max
        return sum(trimmed) / len(trimmed)

    # Yaw until object detection
    # await self.correct_depth(DEPTH_LEVEL)
    found = await yaw_until_object_detection(cv_object, depth_level=DEPTH_LEVEL,
                                             latency_threshold=latency_threshold, direction=direction, parent=self)

    # Could not find, so just surface and pray...
    if not found:
        return False

    logger.info(f'[yaw_to_cv_object] {cv_object} detected. Now centering {cv_object} in frame...')

    # Center detected object in camera frame
    cv_object_yaw = await get_robust_cv_object_yaw()

    await self.correct_depth(DEPTH_LEVEL)
    logger.info(f'[yaw_to_cv_object] abs(cv_object_yaw): {abs(cv_object_yaw)}')
    logger.info(f'[yaw_to_cv_object] yaw_threshold: {yaw_threshold}')

    step = 1
    while abs(cv_object_yaw) > get_yaw_threshold(yaw_threshold, CV().bounding_boxes[cv_object].coords.x):
        # If we have made 3 corrections already, trust that yaw is reasonable and continue forward
        if step > 3:
            logger.info('[yaw_to_cv_object] Yaw has been corrected more than 3 times, breaking loop.')
            break

        # Actually do the yaw itself, and then correct depth
        if use_position_control:
            sign_cv_object_yaw = np.sign(cv_object_yaw)
            correction = get_step_size(POSITION_SCALE_FACTOR * cv_object_yaw)  # Scale down CV yaw value

            # Robot agnostic code base fails once again
            if get_robot_name() == RobotName.OOGWAY:
                desired_yaw = sign_cv_object_yaw * correction
            else:
                desired_yaw = -1 * sign_cv_object_yaw * correction

            logger.info(f'[yaw_to_cv_object] Detected yaw {cv_object_yaw} is greater than threshold {yaw_threshold}. '
                        f'Actually yawing: {desired_yaw}')
            await self.correct_yaw(desired_yaw, yaw_tolerance=0.15)

        else:
            logger.info(f'[yaw_to_cv_object] Detected yaw {cv_object_yaw} is greater than threshold {yaw_threshold}. '
                        f'Setting yaw power to: {cv_object_yaw * VELOCITY_SCALE_FACTOR}')
            Controls().publish_desired_power(Twist(angular=Vector3(z=cv_object_yaw * VELOCITY_SCALE_FACTOR)))

        await self.correct_depth(DEPTH_LEVEL)
        await Yield()

        # Over correction
        if (not CV().is_receiving_recent_cv_data(cv_object, latency_threshold)):
            logger.info(f'[yaw_to_cv_object] {cv_object} detection lost, running yaw_until_object_detection()')
            await yaw_until_object_detection(cv_object, depth_level=DEPTH_LEVEL,
                                             latency_threshold=latency_threshold, direction=direction, parent=self)

        # Recalculate the yaw
        cv_object_yaw = await get_robust_cv_object_yaw()

        step += 1

    logger.info(f'[yaw_to_cv_object] {cv_object} centered, or limit has been reached.')

    await self.correct_depth(DEPTH_LEVEL)

    return True


@comp_task
async def buoy_task(self: CompTask, turn_to_face_buoy: bool = False,
                    depth_level: float = 0.7) -> Task[None, None, None]:
    """Circumnavigate the buoy. Requires robot to have submerged 0.5 meters."""
    logger.info('[buoy_task] Starting buoy task')

    DEPTH_LEVEL = State().orig_depth - depth_level

    def get_step_size(dist: float, dist_threshold: float) -> float:
        if dist > 3:
            return 2
        if dist > 2:
            return 1
        if dist > 1.5:
            return 0.5
        return min(dist - dist_threshold + 0.1, 0.25)

    async def move_to_buoy(buoy_dist_threshold: float = 1) -> None:
        buoy_dist = CV().bounding_boxes[CVObjectType.BUOY].coords.x
        await self.correct_y_to_cv_obj(CVObjectType.BUOY, mult_factor=0.4)
        await self.correct_depth(DEPTH_LEVEL)

        while buoy_dist > buoy_dist_threshold:
            await self.move_x(step=get_step_size(buoy_dist, buoy_dist_threshold))
            logger.info(f'[buoy_task] Buoy dist: {CV().bounding_boxes[CVObjectType.BUOY].coords.x}')

            await self.correct_y_to_cv_obj(CVObjectType.BUOY, mult_factor=0.4)
            if buoy_dist < 3:
                await self.correct_z_to_cv_obj(CVObjectType.BUOY)
            else:
                await self.correct_depth(DEPTH_LEVEL)

            await Yield()
            buoy_dist = CV().bounding_boxes[CVObjectType.BUOY].coords.x
            logger.info(f'[buoy_task] Buoy dist: {CV().bounding_boxes[CVObjectType.BUOY].coords.x}')

            await self.correct_z_to_cv_obj(CVObjectType.BUOY)

    await move_to_buoy()

    if turn_to_face_buoy:
        def get_step_size_move_away(dist: float, dist_threshold: float) -> float:
            if dist < 0.75:
                return -0.5
            return max(dist - dist_threshold - 0.1, -0.25)

        async def move_away_from_buoy(buoy_dist_threshold: float = 1.0) -> None:
            logger.info('[buoy_task] Moving away from buoy')
            buoy_dist = CV().bounding_boxes[CVObjectType.BUOY].coords.x
            await self.correct_y_to_cv_obj(CVObjectType.BUOY, mult_factor=0.4)
            await self.correct_z_to_cv_obj(CVObjectType.BUOY)

            while buoy_dist < buoy_dist_threshold:
                await self.move_x(step=get_step_size_move_away(buoy_dist, buoy_dist_threshold))

                logger.info(f'[buoy_task] Buoy dist: {CV().bounding_boxes[CVObjectType.BUOY].coords.x}')
                await self.correct_y_to_cv_obj(CVObjectType.BUOY, mult_factor=0.4)
                await self.correct_z_to_cv_obj(CVObjectType.BUOY)

                await Yield()
                buoy_dist = CV().bounding_boxes[CVObjectType.BUOY].coords.x
                logger.info(f'[buoy_task] Buoy dist: {CV().bounding_boxes[CVObjectType.BUOY].coords.x}')

            logger.info('[buoy_task] Moved away from buoy')

        # Circumnavigate buoy
        for _ in range(4):
            directions = [
                (0, 1.5, 0),
                (1, 0, 0),
            ]
            await self.move_with_directions(directions, correct_yaw=False, correct_depth=True)
            await self.correct_yaw(-np.pi / 2)
            logger.info('[buoy_task] Yaw 90 degrees')
            await move_away_from_buoy()

    else:
        directions = [
            (0, 1.25, 0),
            (2.25, 0, 0),
            (0, -2.5, 0),
            (-2.5, 0, 0),
            (0, 1.25, 0),
        ]
        await self.move_with_directions(directions, correct_yaw=False, correct_depth=True)
        await move_to_buoy()


@comp_task
async def buoy_circumnavigation_power(self: CompTask, depth_level: float = 0.7) -> Task[None, None, None]:
    """
    Perform a buoy circumnavigation task with a specified depth adjustment.

    Args:
        self (Task): The task instance.
        depth (float): The depth offset to adjust the circumnavigation. Default is 0.7.

    Returns:
        Task[None, None, None]: The result of the circumnavigation task.
    """
    DEPTH_LEVEL = State().orig_depth - depth_level

    def publish_power() -> None:
        power = Twist()
        power.linear.y = 0.9
        power.angular.z = -0.1

        Controls().set_axis_control_type(x=ControlTypes.DESIRED_POWER, y=ControlTypes.DESIRED_POWER,
                                         yaw=ControlTypes.DESIRED_POWER)
        Controls().publish_desired_power(power, set_control_types=False)

    for _ in range(4):
        publish_power()
        logger.info('[buoy_circumnavigation_power] Publish power')

        await util_tasks.sleep(5, parent=self)
        logger.info('[buoy_circumnavigation_power] Sleep 5 (1)')

        await self.stabilize()
        logger.info('[buoy_circumnavigation_power] Stabilized')
        await util_tasks.sleep(5, parent=self)
        logger.info('[buoy_circumnavigation_power] Sleep 5 (2)')

        await self.correct_depth(DEPTH_LEVEL)
        await self.move_y(step=1)


@comp_task
async def after_buoy_task(self: CompTask) -> Task[None, None, None]:
    LATENCY_THRESHOLD = 3

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
        if CV().is_receiving_recent_cv_data(CVObjectType.PATH_MARKER, latency=LATENCY_THRESHOLD):
            await self.stabilize()
            await util_tasks.sleep(5, parent=self)
            break

        await Yield()

    await self.correct_yaw(-np.pi / 2)

    await align_path_marker(direction=-1, parent=self)

    directions = [
        (2, 0, 0),
        (2, 0, 0),
        (2, 0, 0),
        (1, 0, 0),
    ]

    await self.move_with_directions(directions, correct_yaw=False, correct_depth=True)

    found_bins = await spiral_bins_search(parent=self)

    if found_bins:
        await marker_dropper_task(parent=self)

    await yaw_to_cv_object(CVObjectType.BIN_PINK_FRONT, direction=1, yaw_threshold=math.radians(15),
                           depth_level=1.0, parent=Task.MAIN_ID)

    await octagon_task(direction=1, parent=self)


@comp_task
async def buoy_to_octagon(self: CompTask, direction: int = 1, move_forward: int = 0) -> Task[None, None, None]:
    logger.info('[buoy_to_octagon] Started buoy to octagon')

    # Move towards octagon
    directions = [
        (0, 2 * direction, 0),
        (0, 2 * direction, 0),
        (0, 2 * direction, 0),
        (0, 1 * direction, 0),
        (move_forward, 0, 0),
    ]
    await self.move_with_directions(directions, correct_yaw=False)


@comp_task
async def gate_to_octagon(self: CompTask, depth_level: float = 1, timeout: int = 30) -> Task[None, None, None]:
    DEPTH_LEVEL = State().orig_depth - depth_level

    logger.info('[gate_to_octagon] Started gate to octagon')

    directions = [
        (3, 0, 0),
        (3, 0, 0),
        (3, 0, 0),
    ]
    await self.move_with_directions(directions, depth_level=DEPTH_LEVEL, timeout=timeout)


@comp_task
async def slalom_task_dead_reckoning(self: CompTask, depth_level: float = 1.1) -> Task[None, None, None]:
    DEPTH_LEVEL = State().orig_depth - depth_level

    logger.info('[slalom_task_dead_reckoning] Started slalom task')

    if get_robot_name() == RobotName.OOGWAY:
        pass
    elif get_robot_name() == RobotName.CRUSH:
        directions = [
            (2, 0, 0),
            (2, 0, 0),
            (2, 0, 0),
        ]
        await self.move_with_directions(directions, depth_level=DEPTH_LEVEL, timeout=20)

    logger.info('[slalom_task_dead_reckoning] Finished slalom task')


@comp_task
async def slalom_to_octagon_dead_reckoning(self: CompTask, depth_level: float = 1.1) -> Task[None, None, None]:
    DEPTH_LEVEL = State().orig_depth - depth_level
    LATENCY_THRESHOLD = 10

    async def face_fish(yaw_left: bool = True, closer_banner: bool = True) -> None:
        direction = 1 if yaw_left else -1
        yaw_distance = np.pi / 4 if closer_banner else 3 * np.pi / 4
        await orient_to_wall(parent=self)
        await orient_to_wall(parent=self)
        await self.correct_yaw(direction * yaw_distance)

    logger.info('[slalom_to_octagon_dead_reckoning] Started slalom task')
    if get_robot_name() == RobotName.OOGWAY:
        pass
    elif get_robot_name() == RobotName.CRUSH:
        before_cv_directions = [
            (2, 0, 0),
            (2, 0, 0),
        ]
        await self.move_with_directions(before_cv_directions, depth_level=DEPTH_LEVEL, timeout=15)

        logger.info('[slalom_to_octagon_dead_reckoning] Checking pink bin detection')
        await yaw_until_object_detection(CVObjectType.BIN_PINK_FRONT,
                                         depth_level=DEPTH_LEVEL, latency=LATENCY_THRESHOLD, parent=self)

        after_cv_directions = [
            (2, 0, 0),
        ]
        await self.move_with_directions(after_cv_directions, depth_level=DEPTH_LEVEL, timeout=15)

        await face_fish(yaw_left=False, closer_banner=False)

        logger.info('[slalom_to_octagon_dead_reckoning] Surfacing...')

        await self.correct_depth(State().orig_depth)
        logger.info('[slalom_to_octagon_dead_reckoning] Finished surfacing')


@comp_task
async def center_path_marker(self: CompTask, depth_level: float = 0.5) -> Task[None, None, None]:
    DEPTH_LEVEL = State().orig_depth - depth_level
    PIXEL_THRESHOLD = 70
    STEP_SIZE = 0.2

    def get_step_mult_factor(dist: float, threshold: float) -> int:
        if abs(dist) < threshold:
            return 0
        if dist > threshold:
            return 1
        return -1

    await self.correct_depth(DEPTH_LEVEL)

    logger.info(f'[center_path_marker] Path marker pixel distances: {CV().distances[CVObjectType.PATH_MARKER]}')
    pixel_x = CV().distances[CVObjectType.PATH_MARKER].x
    pixel_y = CV().distances[CVObjectType.PATH_MARKER].y

    count = 1
    while (max(pixel_x, pixel_y) > PIXEL_THRESHOLD or min(pixel_x, pixel_y) < -PIXEL_THRESHOLD):
        await self.move_x(step=STEP_SIZE * get_step_mult_factor(pixel_x, PIXEL_THRESHOLD))
        await self.move_y(step=STEP_SIZE * get_step_mult_factor(pixel_y, PIXEL_THRESHOLD))

        logger.info(f'[center_path_marker] Path marker pixel distances: {CV().distances[CVObjectType.PATH_MARKER]}')
        pixel_x = CV().distances[CVObjectType.PATH_MARKER].x
        pixel_y = CV().distances[CVObjectType.PATH_MARKER].y

        if count % 2 == 0:
            logger.info('[center_path_marker] Correcting depth')
            await self.correct_depth(DEPTH_LEVEL)

        await Yield()

        count += 1

    logger.info('[center_path_marker] Finished centering path marker')


@comp_task
async def align_path_marker(self: CompTask, depth_level: float = 0.5) -> Task[None, None, None]:
    """Corrects the yaw relative to the CV object. Follows the yaw and center loop."""
    DEPTH_LEVEL = State().orig_depth - depth_level
    MAXIMUM_YAW = math.radians(30)
    YAW_THRESHOLD = math.radians(5)
    PIXEL_THRESHOLD = 70

    logger.info('[align_path_marker] Starting align path marker')

    def get_step_size(desired_yaw: float) -> float:
        # desired yaw in radians
        return min(abs(desired_yaw), MAXIMUM_YAW)

    await self.correct_depth(DEPTH_LEVEL)

    logger.info('[align_path_marker] Now aligning path marker in frame...')
    # Center detected path marker in camera frame
    path_marker_yaw = CV().bounding_boxes[CVObjectType.PATH_MARKER].yaw
    logger.info(f'[align_path_marker] abs(path_marker_yaw) = {abs(path_marker_yaw)}')
    logger.info(f'[align_path_marker] yaw_threshold = {YAW_THRESHOLD}')

    while abs(path_marker_yaw) > YAW_THRESHOLD:
        sign_path_marker_yaw = np.sign(path_marker_yaw)
        correction = get_step_size(path_marker_yaw)
        desired_yaw = sign_path_marker_yaw * correction

        # Yaw to align with path marker
        logger.info(f'[align_path_marker] Detected yaw {path_marker_yaw} is greater than threshold {YAW_THRESHOLD}. '
                    f'Yawing: {desired_yaw}')
        await self.correct_yaw(desired_yaw, yaw_tolerance=YAW_THRESHOLD)
        await self.correct_depth(DEPTH_LEVEL)

        # Recenter path marker in camera frame
        await center_path_marker(pixel_threshold=PIXEL_THRESHOLD)
        await self.correct_depth(DEPTH_LEVEL)

        await Yield()

        path_marker_yaw = CV().bounding_boxes[CVObjectType.PATH_MARKER].yaw

    logger.info('[align_path_marker] Path marker centered and aligned.')

    await self.correct_depth(DEPTH_LEVEL)


@comp_task
async def path_marker_to_marker_dropper_bins(self: CompTask, maximum_distance: float = 6) -> Task[None, None, None]:
    DEPTH_LEVEL = State().orig_depth - 0.5
    AREA_THRESHOLD = 1000
    LATENCY_THRESHOLD = 1

    logger.info('[path_marker_to_marker_dropper_bins] Starting path marker to marker dropper bins')

    def is_receiving_bin_data(bin_object: CVObjectType, last_detection_time: int | None) -> bool:
        width = CV().bounding_boxes[bin_object].width
        height = CV().bounding_boxes[bin_object].height

        return width * height >= AREA_THRESHOLD \
            and CV().is_receiving_recent_cv_data(bin_object, LATENCY_THRESHOLD, last_detection_time)

    await self.move_x(step=1)

    total_distance = 0
    bin_red_time = None
    bin_blue_time = None

    while not is_receiving_bin_data(CVObjectType.BIN_RED, bin_red_time) \
            or not is_receiving_bin_data(CVObjectType.BIN_BLUE, bin_blue_time):
        bin_red_time = CV().bounding_boxes[CVObjectType.BIN_RED].header.stamp.secs
        bin_blue_time = CV().bounding_boxes[CVObjectType.BIN_BLUE].header.stamp.secs

        await self.correct_depth(DEPTH_LEVEL)

        is_receiving_red_bin_data = is_receiving_bin_data(CVObjectType.BIN_RED, bin_red_time)
        is_receiving_blue_bin_data = is_receiving_bin_data(CVObjectType.BIN_BLUE, bin_blue_time)

        logger.info(f'[path_marker_to_marker_dropper_bins] Receiving red bin data: {is_receiving_red_bin_data}')
        logger.info(f'[path_marker_to_marker_dropper_bins] Receiving blue bin data: {is_receiving_blue_bin_data}')

        step = 0.5 if (is_receiving_red_bin_data or is_receiving_blue_bin_data) else 1
        await self.move_x(step=step)

        await Yield()

        total_distance += step
        if total_distance >= maximum_distance:
            logger.info('[path_marker_to_marker_dropper_bins] Marker dropper bins not spotted, exiting the loop...')
            break

    logger.info('[path_marker_to_marker_dropper_bins] Reached marker dropper bins, stabilizing...')
    await self.stabilize()

    await util_tasks.sleep(5, parent=self)


@comp_task
async def spiral_bins_search(self: CompTask, depth_level: float = 0.5,
                             spiral_step_size: float = 0.5) -> Task[None, None, None] | bool:
    DEPTH_LEVEL = State().orig_depth - depth_level
    AREA_THRESHOLD = 1000
    LATENCY_THRESHOLD = 1

    class Direction(Enum):
        FORWARD = 'forward'
        BACK = 'backward'
        LEFT = 'left'
        RIGHT = 'right'

    MOVE_FUNC_MAPPING = {
        Direction.FORWARD: lambda s: move_tasks.move_x(step=s, parent=self),
        Direction.BACK:    lambda s: move_tasks.move_x(step=-s, parent=self),
        Direction.LEFT:    lambda s: move_tasks.move_y(step=s, parent=self),
        Direction.RIGHT:   lambda s: move_tasks.move_y(step=-s, parent=self),
    }

    SPIRAL_PATTERN: list[tuple[Direction, int]] = [
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

    def is_receiving_bin_data(bin_object: CVObjectType, last_detection_time: int | None) -> bool:
        width = CV().bounding_boxes[bin_object].width
        height = CV().bounding_boxes[bin_object].height

        return width * height >= AREA_THRESHOLD \
            and CV().is_receiving_recent_cv_data(bin_object, LATENCY_THRESHOLD, last_detection_time)

    logger.info('[spiral_bins_search] Searching for marker dropper bins...')
    bin_red_time = None
    bin_blue_time = None

    for direction, raw_distance in SPIRAL_PATTERN:
        distance = raw_distance * spiral_step_size
        logger.info(f'[spiral_bins_search] Moving {direction.value} by {distance} units')

        move_task = MOVE_FUNC_MAPPING[direction](distance)

        while not move_task.done:
            move_task.step()

            # TODO: for a generic spiral search task, abstract the search termination logic into a boolean function
            bin_red_time = CV().bounding_boxes[CVObjectType.BIN_RED].header.stamp.secs
            bin_blue_time = CV().bounding_boxes[CVObjectType.BIN_BLUE].header.stamp.secs

            is_receiving_red_bin_data = is_receiving_bin_data(CVObjectType.BIN_RED, bin_red_time)
            is_receiving_blue_bin_data = is_receiving_bin_data(CVObjectType.BIN_BLUE, bin_blue_time)

            if is_receiving_red_bin_data and is_receiving_blue_bin_data:
                logger.info('[spiral_bins_search] Found marker dropper bins, stabilize and terminating...')
                await self.stabilize()
                await util_tasks.sleep(5, parent=self)
                return True

            await util_tasks.sleep(0.1, parent=self)
            await Yield()

        await self.correct_depth(DEPTH_LEVEL)

    logger.info(f'[spiral_bins_search] Received red: {is_receiving_bin_data(CVObjectType.BIN_RED, bin_red_time)}')
    logger.info(f'[spiral_bins_search] Received blue: {is_receiving_bin_data(CVObjectType.BIN_BLUE, bin_blue_time)}')

    logger.info('[spiral_bins_search] Spiral search completed without finding marker dropper bins, stabilizing...')
    await self.stabilize()
    await util_tasks.sleep(5, parent=self)

    return False


@comp_task
async def marker_dropper_task(self: CompTask) -> Task[None, None, None]:
    """Detect and drop markers into the red bin. Requires robot to have submerged 0.7 meters."""
    START_DEPTH_LEVEL = State().orig_depth - 0.6
    START_PIXEL_THRESHOLD = 70
    MID_DEPTH_LEVEL = State().orig_depth - 1.0
    MID_PIXEL_THRESHOLD = 30
    YAW_THRESHOLD = math.radians(5)
    FRAME_AREA = 480 * 600
    TIMEOUT = Duration(seconds=240)

    logger.info('[marker_dropper_task] Started marker dropper task')

    start_time = Clock().now()

    def get_step_mult_factor(dist: float, threshold: float) -> int:
        if abs(dist) < threshold:
            return 0
        if dist > threshold:
            return 1
        return -1

    async def track_bin(target: CVObjectType, desired_depth: float, pixel_threshold: float,
                        step_size: float = 0.20, x_offset: float = 0, y_offset: float = 0) -> None:
        logger.info(f'[marker_dropper_task] Target pixel distances: {CV().distances[target]}')
        pixel_x = CV().distances[target].x + x_offset
        pixel_y = CV().distances[target].y + y_offset

        width = CV().bounding_boxes[target].width
        height = CV().bounding_boxes[target].height

        count = 1
        while (max(pixel_x, pixel_y) > pixel_threshold or min(pixel_x, pixel_y) < -pixel_threshold) \
                and width * height <= 1/3 * FRAME_AREA:
            await self.move_x(step=step_size * get_step_mult_factor(pixel_x, pixel_threshold))
            await self.move_y(step=step_size * get_step_mult_factor(pixel_y, pixel_threshold))

            logger.info(f'[marker_dropper_task] Target pixel distances: {CV().distances[target]}')
            width = CV().bounding_boxes[target].width
            height = CV().bounding_boxes[target].height

            pixel_x = CV().distances[target].x + x_offset
            pixel_y = CV().distances[target].y + y_offset

            if count % 2 == 0:
                logger.info('[marker_dropper_task] Correcting depth')
                await self.correct_depth(desired_depth=desired_depth)

                logger.info('[marker_dropper_task] Correcting roll and pitch')
                await self.correct_roll_and_pitch(mult_factor=1.2)

            if width * height >= 1/6 * FRAME_AREA and \
                    abs(pixel_x) < pixel_threshold * 1.75 and abs(pixel_y) < pixel_threshold * 1.75:
                logger.info(f'[marker_dropper_task] Reached area threshold: area = {width * height}')
                break

            if Clock().now() - start_time > TIMEOUT:
                logger.warning('[marker_dropper_task] Track bin timed out')
                break

            await Yield()

            logger.info(f'[marker_dropper_task] Target x: {pixel_x}, y: {pixel_y}, area: {width * height}')

            count += 1

        logger.info('[marker_dropper_task] Finished tracking bin')
        logger.info(f'[marker_dropper_task] Target x: {pixel_x}, y: {pixel_y}, area: {width * height}')

    await self.correct_depth(desired_depth=START_DEPTH_LEVEL)
    await track_bin(target=CVObjectType.BIN_RED, desired_depth=START_DEPTH_LEVEL, pixel_threshold=START_PIXEL_THRESHOLD)

    await self.correct_yaw(CV().angles[CVObjectType.BIN_WHOLE], yaw_tolerance=YAW_THRESHOLD)

    await self.correct_depth(desired_depth=MID_DEPTH_LEVEL)
    await track_bin(target=CVObjectType.BIN_RED, desired_depth=MID_DEPTH_LEVEL, pixel_threshold=MID_PIXEL_THRESHOLD,
                    step_size=0.18, y_offset=30, x_offset=25)

    await servos_tasks.drop_marker(MarkerDropperStates.LEFT, parent=self)
    await servos_tasks.drop_marker(MarkerDropperStates.RIGHT, parent=self)

    await self.correct_depth(desired_depth=START_DEPTH_LEVEL)
    logger.info(f'[marker_dropper_task] Corrected depth to {START_DEPTH_LEVEL}')

    logger.info('[marker_dropper_task] Completed marker dropper task')


@comp_task
async def torpedo_task(self: CompTask, first_target: CVObjectType,
                       depth_level: float = 0.5, direction: int = 1) -> Task[None, None, None]:
    assert first_target in [CVObjectType.TORPEDO_REEF_SHARK_TARGET, CVObjectType.TORPEDO_SAWFISH_TARGET], \
        f'Invalid first_animal: {first_target}. Must be \
            CVObjectType.TORPEDO_REEF_SHARK_TARGET or CVObjectType.TORPEDO_SAWFISH_TARGET'

    logger.info('[torpedo_task] Starting torpedo task')

    DEPTH_LEVEL = State().orig_depth - depth_level

    def get_step_size(dist: float, dist_threshold: float) -> float:
        if dist > 10:
            goal_step = 4
        elif dist > 6:
            goal_step = 3
        elif dist > 4:
            goal_step = 1
        elif dist > 1.5:
            goal_step = 0.5
        else:
            goal_step = 0.25
        return min(dist - dist_threshold + 0.1, goal_step)

    async def move_to_torpedo(torpedo_dist_threshold: float = 2) -> None:
        await yaw_to_cv_object(CVObjectType.TORPEDO_BANNER, direction=direction, yaw_threshold=math.radians(15),
                               depth_level=depth_level, parent=self)

        torpedo_dist = CV().bounding_boxes[CVObjectType.TORPEDO_BANNER].coords.x

        await self.correct_y_to_cv_obj(CVObjectType.TORPEDO_BANNER)
        await self.correct_depth(DEPTH_LEVEL)

        while torpedo_dist > torpedo_dist_threshold:
            logger.info(f'[torpedo_task] Torpedo dist x: {CV().bounding_boxes[CVObjectType.TORPEDO_BANNER].coords.x}')
            logger.info(f'[torpedo_task] Torpedo dist y: {CV().bounding_boxes[CVObjectType.TORPEDO_BANNER].coords.y}')
            await self.move_x(step=get_step_size(torpedo_dist, torpedo_dist_threshold))

            await yaw_to_cv_object(CVObjectType.TORPEDO_BANNER, direction=-1, yaw_threshold=math.radians(15),
                                   depth_level=depth_level, parent=self)
            logger.info('[torpedo_task] Yaw corrected')

            await self.correct_y_to_cv_obj(CVObjectType.TORPEDO_BANNER)

            if torpedo_dist < 3:
                await self.correct_z_to_cv_obj(CVObjectType.TORPEDO_BANNER)  # CV-based z-axis correction
            else:
                await self.correct_depth(DEPTH_LEVEL)

            await Yield()
            torpedo_dist = CV().bounding_boxes[CVObjectType.TORPEDO_BANNER].coords.x

        logger.info(f'[torpedo_task] Finished moving forwards to torpedo: {torpedo_dist}m away')

        await self.correct_z_to_cv_obj(CVObjectType.TORPEDO_BANNER)  # CV-based z-axis correction

    await move_to_torpedo()
    logger.info('[torpedo_task] Finished moving forwards to torpedo')

    # Small offset to counteract camera positioning
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, -0.5, 0.2, 0, 0, 0), parent=self)

    # Determine which animal to target first and which to target second
    if first_target == CVObjectType.TORPEDO_REEF_SHARK_TARGET:
        second_target = CVObjectType.TORPEDO_SAWFISH_TARGET
    elif first_target == CVObjectType.TORPEDO_SAWFISH_TARGET:
        second_target = CVObjectType.TORPEDO_REEF_SHARK_TARGET

    # Center to first animal
    animal = first_target
    target_y = CV().bounding_boxes[animal].coords.y
    target_z = CV().bounding_boxes[animal].coords.z + 0.1

    logger.info(f'[torpedo_task] Aligning to first target {animal} at y={target_y} and z={target_z}')
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, target_y, target_z, 0, 0, 0), parent=self)

    # Fire first torpedo
    await servos_tasks.fire_torpedo(TorpedoStates.RIGHT, parent=self)

    # Move back to see full banner
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, -target_y, -target_z, 0, 0, 0), parent=self)

    # Center to second animal
    animal = second_target
    target_y = CV().bounding_boxes[animal].coords.y - 0.1
    target_z = CV().bounding_boxes[animal].coords.z + 0.1
    logger.info(f'[torpedo_task] Aligning to second target {animal} at y={target_y} and z={target_z}')
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, target_y, target_z, 0, 0, 0), parent=self)

    # Fire second torpedo
    await servos_tasks.fire_torpedo(TorpedoStates.LEFT, parent=self)

    logger.info('[torpedo_task] Torpedo task completed')


@comp_task
async def octagon_task(self: CompTask, direction: int = 1) -> Task[None, None, None]:
    """
    Detect, move towards the yellow bins, then surface inside the octagon.

    Requires robot to have submerged 0.7 meters.
    """
    logger.info('[octagon_task] Starting octagon task')

    DEPTH_LEVEL_AT_BINS = State().orig_depth - 1.2  # Depth for task beginning and corrections during forward movement
    DEPTH_LEVEL_ABOVE_BINS = State().orig_depth - 0.9  # Depth for going above bin before forward move
    LATENCY_THRESHOLD = 2  # Latency for seeing the bottom bin
    CONTOUR_SCORE_THRESHOLD = 2000  # Required bottom bin area for valid detection
    SCORE_THRESHOLD = 7500  # Area of bin before beginning surface logic for front camera
    POST_FRONT_THRESHOLD_FORWARD_DISTANCE = 0.4  # in meters

    # Forward navigation case constants
    LOW_SCORE = 1500
    LOW_STEP_SIZE = 0.75
    MED_SCORE = 3500
    MED_STEP_SIZE = 0.5
    HIGH_SCORE = 5000
    HIGH_STEP_SIZE = 0.35
    VERY_HIGH_STEP_SIZE = 0.2

    def is_receiving_pink_bin_data(latest_detection_time: int | None) -> bool:
        return latest_detection_time and \
            CV().bounding_boxes[CVObjectType.BIN_PINK_BOTTOM].score >= CONTOUR_SCORE_THRESHOLD and \
            CV().is_receiving_recent_cv_data(CVObjectType.BIN_PINK_BOTTOM, LATENCY_THRESHOLD, latest_detection_time)

    def get_step_size(last_step_size: float) -> float:
        bin_pink_score = CV().bounding_boxes[CVObjectType.BIN_PINK_FRONT].score

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
        logger.info('[octagon_task] Beginning move to pink bins')

        pink_bin_found = await yaw_until_object_detection(CVObjectType.BIN_PINK_FRONT, depth_level=DEPTH_LEVEL_AT_BINS,
                                                          direction=direction, parent=self)

        if not pink_bin_found:
            await move_tasks.move_to_pose_local(
                geometry_utils.create_pose(0.5, 0, 0, 0, 0, 0),
                depth_level=DEPTH_LEVEL_AT_BINS,
                pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.3),
                timeout=10,
                parent=self,
            )

        latest_detection_time = None
        moved_above = False
        last_step_size = float('inf')

        while not is_receiving_pink_bin_data(latest_detection_time) and not moved_above:
            latest_detection_time = CV().bounding_boxes[CVObjectType.BIN_PINK_BOTTOM].header.stamp.secs
            if is_receiving_pink_bin_data(latest_detection_time):
                logger.info('[octagon_task] Bottom camera detects pink bin, ending loop.')
                break

            logger.info('[octagon_task] Have not moved above, depth correcting and beginning sequence.')
            await self.correct_depth(DEPTH_LEVEL_AT_BINS if not moved_above else DEPTH_LEVEL_ABOVE_BINS,
                                     skip_threshold=0.15)

            if not moved_above:
                logger.info('[octagon_task] Yawing to pink bin front')
                status = await yaw_to_cv_object(CVObjectType.BIN_PINK_FRONT, direction=direction,
                                                yaw_threshold=math.radians(15), depth_level=1.2, parent=Task.MAIN_ID)

                # At this point, we cannot find the object anymore. We hope to surface in the octagon at this point
                if not status:
                    logger.info('[octagon_task] Yaw failed, ending.')
                    break

            logger.info(f'[octagon_task] Bin front score: {CV().bounding_boxes[CVObjectType.BIN_PINK_FRONT].score}')

            if CV().bounding_boxes[CVObjectType.BIN_PINK_FRONT].score > SCORE_THRESHOLD and not moved_above:
                logger.info('[octagon_task] Beginning to move above bin')
                await self.correct_depth(DEPTH_LEVEL_ABOVE_BINS, skip_threshold=0.15)

                logger.info('[octagon_task] Moved above pink bins, based on front camera.')
                moved_above = True

            if not moved_above:
                step = get_step_size(last_step_size)

                logger.info(f'[octagon_task] Moving step size {step}')
                await self.move_x(step=step)

                last_step_size = step

            await Yield()

            logger.info(f'[octagon_task] Receiving bottom bin: {is_receiving_pink_bin_data(latest_detection_time)}')

        if moved_above:
            await self.move_x(step=POST_FRONT_THRESHOLD_FORWARD_DISTANCE)
        else:
            logger.info('[octagon_task] Detected bin_pink_bottom, or yaw has failed.')

        logger.info('[octagon_task] Reached pink bins, stabilizing...')
        await self.stabilize()
        await util_tasks.sleep(5, parent=self)

    async def face_fish(yaw_left: bool = True, closer_banner: bool = True) -> None:
        direction = 1 if yaw_left else -1
        yaw_distance = np.pi / 4 if closer_banner else 3 * np.pi / 4
        await orient_to_wall(parent=self)
        await self.correct_yaw(direction * yaw_distance)

    await move_to_pink_bins()
    await face_fish(yaw_left=True, closer_banner=True)

    logger.info('[octagon_task] Surfacing...')
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, State().orig_depth - State().depth, 0, 0, 0),
                                        timeout=10, parent=self)
    logger.info('[octagon_task] Finished surfacing')


@comp_task
async def orient_to_wall(self: CompTask[None, None, None],
                         end_angle: float = 15.0,
                         start_angle: float = -15.0,
                         distance: float = 10.0) -> Task[None, None, None]:
    """Orient the robot to a wall using sonar sweep."""
    async def get_sonar_normal_angle() -> float | None:
        """Return the yaw angle of the wall in degrees."""
        # Call sonar sweep request
        sonar_future = Sonar().sweep(start_angle, end_angle, distance)
        if sonar_future is not None:
            logger.info('[orient_to_wall] Sonar sweep request sent, waiting for response...')

            # Wait for the sonar sweep to complete
            sonar_response = await sonar_future
            logger.info(f'[orient_to_wall] Sonar sweep response received: {sonar_response}')

            if not sonar_response.is_object:
                logger.info('[orient_to_wall] Did not get valid object from scan')
                return None

            return sonar_response.normal_angle

        logger.warning('[orient_to_wall] Sonar sweep request failed - bypass mode or service unavailable')
        return math.nan

    def convert_sonar_output_to_yaw(sonar_normal: float) -> float:
        """Return how much robot needs to yaw to be normal to surface it scans. Input degrees, output radians."""
        yaw_in_degrees = 180 - sonar_normal
        return yaw_in_degrees * np.pi / 180

    count = 2
    got_valid_sweep = False

    while not got_valid_sweep and count > 0:
        count -= 1

        sonar_output = await get_sonar_normal_angle()

        if sonar_output is None:
            logger.info(f'[orient_to_wall] Did not get valid sweep object, trying again {count} times')
            continue

        got_valid_sweep = True

        if not math.isnan(sonar_output):
            yaw_delta = convert_sonar_output_to_yaw(sonar_output)
            logger.info(f'[orient_to_wall] Yaw delta from sonar sweep: {yaw_delta} radians')

            # Move to the desired yaw angle
            await self.correct_yaw(yaw_delta, timeout=10)
        else:
            logger.warning('[orient_to_wall] No yaw delta received from sonar sweep, skipping orientation.')


@comp_task
async def return_task_dead_reckoning(self: CompTask, depth_level: float = 0.7) -> Task[None, None, None]:
    DEPTH_LEVEL = State().orig_depth - depth_level

    logger.info('[return_task_dead_reckoning] Started gate return task')

    if get_robot_name() == RobotName.OOGWAY:
        pass
    elif get_robot_name() == RobotName.CRUSH:
        directions = [
            (-3, 0, 0),
            (-3, 0, 0),
            (-3, 0, 0),
            (-3, 0, 0),
        ]
        await self.move_with_directions(directions, depth_level=DEPTH_LEVEL, timeout=15)

    logger.info('[return_task_dead_reckoning] Moved through gate return')
