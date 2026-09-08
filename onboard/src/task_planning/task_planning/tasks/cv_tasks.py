import copy
import math

from geometry_msgs.msg import Twist, Vector3
from rclpy.clock import Clock
from rclpy.logging import get_logger
from task_planning.interface.controls import Controls
from task_planning.interface.cv import CV, CVObjectType
from task_planning.interface.state import State
from task_planning.task import Task, Yield, task
from task_planning.tasks import move_tasks, util_tasks
from task_planning.utils import geometry_utils

logger = get_logger('cv_tasks')

@task
async def yaw_until_object_detection(self: Task, cv_object: CVObjectType, search_direction: int = 1,
                                     depth_threshold: float = 0.2, depth_level: float = 0.5,
                                    ) -> Task[None, str | None, None] | bool:
    """
    Yaws until an object is detected by CV.

    Returns when the robot looking at the CV object, within a small tolerance.

    Args:
        self: Task instance.
        cv_object: CV class name of the object to yaw to
        search_direction: If no CV object in view, which direction should it search in.
                            1 for positive yaw, -1 for negative yaw.
        depth_threshold: The error in depth that will cause a depth-correct call
        depth_level: Desire depth level to hold throughout the task. Must be positive.

    Send:
        CV class name of new object to yaw to
    """
    logger.info('[cv_tasks.yaw_until_object_detection] Beginning yaw_util_object_detection task')

    yaw_pid_step_size = math.radians(90) # How big a single step is
    num_steps = 3 # How many steps are taken in each direction
    depth_level = State().orig_depth - depth_level

    @task
    async def correct_depth(self: Task) -> None:
        await move_tasks.depth_correction(desired_depth=depth_level, parent=self)

    @task
    async def object_search_pattern(self: Task) -> bool:
        step = 0
        logger.info(f'[cv_tasks.yaw_until_object_detection] Depth level for search: {depth_level}')
        move_task = move_tasks.move_to_pose_local(
                        geometry_utils.create_pose(0, 0, 0, 0, 0, yaw_pid_step_size * search_direction),
                        depth_level=depth_level,
                        pose_tolerances=Twist(linear=Vector3(x=0.05, y=0.05, z=0.05),
                                              angular=Vector3(x=0.2, y=0.3, z=0.3)),
                        timeout=10,
                        parent=self)
        move_task.step()

        while True:
            if move_task.done:
                step += 1

                if step <= num_steps - 1:
                    angle = yaw_pid_step_size
                elif step == num_steps:
                    angle = -num_steps * yaw_pid_step_size
                elif step <= num_steps * 2:
                    angle = -1 * yaw_pid_step_size
                elif step == num_steps * 2 + 1:
                    angle = num_steps * yaw_pid_step_size
                else:
                    break

                logger.info(f'[cv_tasks.yaw_until_object_detection] On step {step}, desired angle is {angle}')
                move_task = move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0,
                                                                                     angle * search_direction),
                                                          depth_level=depth_level,
                                                          pose_tolerances=Twist(linear=Vector3(x=0.05, y=0.05, z=0.05),
                                                                                angular=Vector3(x=0.2, y=0.3, z=0.3)),
                                                          timeout=10,
                                                          parent=self)

            move_task.step()
            await Yield()

        logger.info('[cv_tasks.yaw_until_object_detection] Could not find object, breaking loop')
        return False

    object_search_task = None

    while not CV().is_receiving_recent_cv_data(cv_object, 10):
        if object_search_task is None:
            object_search_task = object_search_pattern(parent=self)

        object_search_task.step()

        await util_tasks.sleep(0.1, parent=self)

        if object_search_task.done:
            logger.info('[cv_tasks.yaw_until_object_detection] Failed to find object. Exiting search pattern')
            return False

        if (abs(State().depth - depth_level) > depth_threshold):
            logger.info(f'[cv_tasks.yaw_until_object_detection] State: {State().depth}, level: {depth_level}')
            logger.info('[cv_tasks.yaw_until_object_detection] Correcting depth')
            await correct_depth(parent=self)

    logger.info('[cv_tasks.yaw_until_object_detection] Found object. Exiting search pattern')
    return True

@task
async def yaw_to_cv_obj(self: Task, cv_object: CVObjectType, search_direction: int = 1,
                        yaw_threshold: float = math.radians(10), depth_threshold: float = 0.2,
                        depth_level: float = 0.5, pid_timeout: float = 20) -> Task[None, str | None, None] | bool:
    """
    Yaw to an object detected by CV.

    Returns when the robot looking at the CV object, within a small tolerance.

    Args:
        self: Task instance.
        cv_object: CV class name of the object to yaw to
        search_direction: If no CV object in view, which direction should it search in.
                                1 for positive yaw, -1 for negative yaw.
        yaw_threshold: Tolerance for completing the task
        depth_threshold: The error in depth that will cause a depth-correct call
        depth_level: Desire depth level to hold throughout the task. Must be positive.
        pid_timeout: Time within PID loop before automatically breaking out

    Send:
        CV class name of new object to yaw to
    """
    logger.info('[cv_tasks.yaw_to_cv_obj] Starting yaw_to_cv_object. Yawing until object detection...')

    # depth_level from callers is positive meters submerged.
    # yaw_until_object_detection expects that positive form and converts internally.
    # Keep a positive copy so re-search never double-converts.
    positive_depth_level = depth_level
    absolute_depth_level = State().orig_depth - depth_level

    if not CV().is_receiving_recent_cv_data(cv_object, 10):
        found_object = await yaw_until_object_detection(
            cv_object, search_direction, depth_threshold, positive_depth_level, parent=self,
        )
        if not found_object:
            logger.info('[cv_tasks.yaw_to_cv_obj] Never found CV Object. Ending yaw_to_cv_object.')
            return False

    @task
    async def correct_depth(self: Task) -> None:
        await move_tasks.depth_correction(desired_depth=absolute_depth_level, parent=self)

    cv_object_yaw = CV().angles[cv_object]
    move_to_pose_task = move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, 0, 0, 0, cv_object_yaw),
        depth_level=absolute_depth_level,
        pose_tolerances=Twist(
            linear=Vector3(x=0.05, y=0.05, z=0.05),
            angular=Vector3(x=0.2, y=0.3, z=yaw_threshold),
        ),
        timeout=30,
        parent=self,
    )

    move_to_pose_task.step()

    clock = Clock()
    starting_time = clock.now()

    while not move_to_pose_task.done:
        if abs(State().depth - absolute_depth_level) > depth_threshold:
            await correct_depth(parent=self)

        if not CV().is_receiving_recent_cv_data(cv_object, 10):
            logger.info('[cv_tasks.yaw_to_cv_obj] Lost sight of cv object. Yawing until object detection...')
            found_object = await yaw_until_object_detection(
                cv_object, search_direction, depth_threshold, positive_depth_level, parent=self,
            )
            if not found_object:
                logger.info('[cv_tasks.yaw_to_cv_obj] Could not regain sight of cv object. Ending yaw_to_cv_object.')
                return False

        cv_object_yaw = CV().angles[cv_object]
        new_pose = geometry_utils.create_pose(0, 0, 0, 0, 0, cv_object_yaw)
        move_to_pose_task.send(new_pose)

        await util_tasks.sleep(0.1, parent=self)

        if (clock.now() - starting_time).nanoseconds * 1e-9 > pid_timeout:
            logger.info('[cv_tasks.yaw_to_cv_obj] Timeout elapsed, finishing yaw_to_cv_obj')
            return True

    logger.info('[cv_tasks.yaw_to_cv_obj] PID loop complete, finishing yaw_to_cv_obj')
    return True


@task
async def move_to_cv_obj(self: Task, cv_object: CVObjectType, target_distance: float = 1, search_direction: int = 1,
                         depth_threshold: float = 0.2, depth_level: float = 0.5) -> Task[None, str | None, None] | bool:
    """
    Continuously move toward a CV object in robot X/Y/Z while tracking yaw.

    Searches/yaws until the object is detected and centered first, then each control loop
    sends a local pose (forward, lateral, vertical, yaw) from DepthAI robot-frame coords.
    Far away, vertical motion holds the commanded depth_level; when close enough, vertical
    tracks coords.z. Lateral always tracks coords.y. After reaching target_distance, yaws
    once more to recenter the object in frame, then holds pose.

    Args:
        self: Task instance.
        cv_object: CV class name of the object to move to
        target_distance: The goal distance to be from the CV object when finished
        search_direction: If no CV object in view, which direction should it search in.
                                1 for positive yaw, -1 for negative yaw.
        depth_threshold: Depth error threshold used during search/yaw helpers.
        depth_level: Positive meters submerged to hold when far from the object.
    Send:
        CV class name of new object to move to
    """
    yaw_threshold = math.radians(10)
    yaw_stop_threshold = math.radians(25)
    biggest_forward_step = 0.3
    max_lateral_step = 0.5
    max_vertical_step = 0.5
    y_gain = 0.4
    close_threshold = 100
    arrival_y_tol = 0.15
    detection_latency = 10

    # depth_level from callers is positive meters submerged.
    positive_depth_level = depth_level
    absolute_depth_level = State().orig_depth - depth_level

    pose_tolerances = Twist(
        linear=Vector3(x=0.05, y=0.05, z=0.05),
        angular=Vector3(x=0.2, y=0.3, z=yaw_threshold),
    )

    def clip(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def compute_tracking_steps() -> tuple[float, float, float, float]:
        coords = CV().bounding_boxes[cv_object].coords
        logger.info(f'[cv_tasks.move_to_cv_obj] Coords: {coords}')
        yaw = CV().angles[cv_object]
        forward_step = min(biggest_forward_step, max(0.0, math.exp(-4 + 0.5*coords.x) + 0.25))
        y_step = clip(coords.y * y_gain, max_lateral_step)
        if coords.x > close_threshold:
            z_step = clip(absolute_depth_level - State().depth, max_vertical_step)
        else:
            z_step = clip(coords.z, max_vertical_step)
        return forward_step, y_step, z_step, yaw

    def has_valid_detection() -> bool:
        if not CV().is_receiving_recent_cv_data(cv_object, detection_latency):
            return False
        # Placeholder / empty detections have coords.x == 0; require a real forward range.
        return CV().bounding_boxes[cv_object].coords.x > 0.0

    async def search_for_object() -> bool:
        logger.info('[cv_tasks.move_to_cv_obj] Searching for object before approach...')
        found = await yaw_to_cv_obj(
            cv_object, search_direction, yaw_threshold, depth_threshold,
            positive_depth_level, parent=self,
        )
        if not found or not has_valid_detection():
            logger.info('[cv_tasks.move_to_cv_obj] Search failed or detection invalid.')
            return False
        return True

    if not await search_for_object():
        logger.info('[cv_tasks.move_to_cv_obj] Never found CV object. Ending move_to_cv_obj.')
        return False

    logger.info('[cv_tasks.move_to_cv_obj] Beginning continuous move to cv object.')

    # touching_x_boundary = [0, 0, 0, 0, 0]
    # touching_y_boundary = [0, 0, 0, 0, 0]
    move_task = None

    def stop_tracking_move() -> None:
        """Close any in-progress move and hold the current pose (avoids coast/drift)."""
        nonlocal move_task
        if move_task is not None and not move_task.done:
            move_task.close()
        move_task = None
        Controls().publish_desired_position(copy.deepcopy(State().state.pose.pose))

    while True:
        if not has_valid_detection() or abs(CV().angles[cv_object]) > yaw_stop_threshold:
            logger.info('[cv_tasks.move_to_cv_obj] Lost sight or yawed too far. Re-searching...')
            if move_task is not None and not move_task.done:
                move_task.close()
            move_task = None
            if not await search_for_object():
                logger.info('[cv_tasks.move_to_cv_obj] Could not regain sight. Ending move_to_cv_obj.')
                stop_tracking_move()
                return False
            continue

        forward_step, y_step, z_step, cv_object_yaw = compute_tracking_steps()
        bbox = CV().bounding_boxes[cv_object]
        coords = bbox.coords

        # Arrival: within target distance and roughly centered laterally.
        if coords.x <= target_distance and abs(coords.y) <= arrival_y_tol:
            logger.info(
                f'[cv_tasks.move_to_cv_obj] Reached target '
                f'(x={coords.x:.2f}m, y={coords.y:.2f}m). Finishing.',
            )
            break

        # touching_x_boundary.append(1 if (bbox.xmin < 0.01 or bbox.xmax > 0.99) else 0)
        # touching_y_boundary.append(1 if (bbox.ymin < 0.01 or bbox.ymax > 0.99) else 0)
        # touching_x_boundary.pop(0)
        # touching_y_boundary.pop(0)

        # if sum(touching_x_boundary) >= 4:
        #     touching_x_boundary = [0, 0, 0, 0, 0]
        #     logger.info('[cv_tasks.move_to_cv_obj] Object approached x bounds. Calling yaw to cv object.')
        #     if not await search_for_object():
        #         logger.info('[cv_tasks.move_to_cv_obj] Failure. Object never found, finishing move_to_cv_obj')
        #         return False
        #     move_task = None
        #     if CV().bounding_boxes[cv_object].xmin < 0.01 or CV().bounding_boxes[cv_object].xmax > 0.99:
        #         logger.info('[cv_tasks.move_to_cv_obj] Object still in x bounds, backing up.')
        #         await move_tasks.move_to_pose_local(
        #             geometry_utils.create_pose(-0.5, 0, 0, 0, 0, CV().angles[cv_object]),
        #             depth_level=absolute_depth_level,
        #             pose_tolerances=pose_tolerances,
        #             timeout=30,
        #             parent=self,
        #         )
        #     continue

        # if sum(touching_y_boundary) >= 4:
        #     touching_y_boundary = [0, 0, 0, 0, 0]
        #     logger.info('[cv_tasks.move_to_cv_obj] Object approached y bounds. Correcting yaw.')
        #     if not await search_for_object():
        #         logger.info('[cv_tasks.move_to_cv_obj] Failure. Object never found, finishing move_to_cv_obj')
        #         return False
        #     move_task = None
        #     if CV().bounding_boxes[cv_object].ymin < 0.01 or CV().bounding_boxes[cv_object].ymax > 0.99:
        #         logger.info('[cv_tasks.move_to_cv_obj] Object still in y bounds, backing up.')
        #         await move_tasks.move_to_pose_local(
        #             geometry_utils.create_pose(-0.5, 0, 0, 0, 0, CV().angles[cv_object]),
        #             depth_level=absolute_depth_level,
        #             pose_tolerances=pose_tolerances,
        #             timeout=30,
        #             parent=self,
        #         )
        #     continue

        # Do not pass depth_level here — that would override pose Z and break continuous tracking.
        if move_task is None or move_task.done:
            # Zero local pose completes immediately; only start a move with a real command.
            if forward_step == 0.0 and abs(y_step) < 1e-3 and abs(z_step) < 1e-3:
                await util_tasks.sleep(0.05, parent=self)
                continue
            move_task = move_tasks.move_to_pose_local(
                geometry_utils.create_pose(forward_step, y_step, z_step, 0, 0, cv_object_yaw),
                pose_tolerances=pose_tolerances,
                timeout=240,
                parent=self,
            )
            move_task.step()
            await util_tasks.sleep(0.05, parent=self)
            continue

        logger.info(
            f'[cv_tasks.move_to_cv_obj] Tracking pose '
            f'x={forward_step:.2f} y={y_step:.2f} z={z_step:.2f} yaw={cv_object_yaw:.3f}',
        )
        new_pose = geometry_utils.create_pose(forward_step, y_step, z_step, 0, 0, cv_object_yaw)
        move_task.send(new_pose)
        await util_tasks.sleep(0.05, parent=self)

    logger.info('[cv_tasks.move_to_cv_obj] Continuous move complete, recentering on object...')
    if move_task is not None and not move_task.done:
        move_task.close()
    move_task = None

    # Hold depth at arrival (may differ from commanded depth_level after Z tracking).
    hold_depth = State().orig_depth - State().depth
    if not await yaw_to_cv_obj(
        cv_object, search_direction, yaw_threshold, depth_threshold,
        hold_depth, parent=self,
    ):
        logger.info('[cv_tasks.move_to_cv_obj] Final recenter failed; holding current pose.')

    stop_tracking_move()
    return True


@task
async def correct_x(self: Task, prop: CVObjectType,
                    add_factor: float = 0, mult_factor: float = 1) -> Task[None, None, None]:
    """
    Correct x-coordinate based on CV data.

    Args:
        self: Task instance.
        prop: The CV object to use for correction.
        add_factor: Additive offset. Defaults to 0.
        mult_factor: Multiplicative offset. Defaults to 1.
    """
    x = (CV().bounding_boxes[prop].coords.x + add_factor) * mult_factor
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(x, 0, 0, 0, 0, 0), parent=self)
    logger.info(f'Corrected x {x}')


@task
async def correct_y(self: Task, prop: CVObjectType,
                    add_factor: float = 0, mult_factor: float = 1) -> Task[None, None, None]:
    """
    Correct y-coordinate based on CV data.

    Args:
        self: Task instance.
        prop: The CV object to use for correction.
        add_factor: Additive offset. Defaults to 0.
        mult_factor: Multiplicative offset. Defaults to 1.
    """
    y = (CV().bounding_boxes[prop].coords.y + add_factor) * mult_factor
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, y, 0, 0, 0, 0), parent=self)
    logger.info(f'Corrected y {y}')


@task
async def correct_z(self: Task, prop: CVObjectType,
                    add_factor: float = 0, mult_factor: float = 1) -> Task[None, None, None]:
    """
    Correct z-coordinate based on CV data.

    Args:
        self: Task instance.
        prop: The CV object to use for correction.
        add_factor: Additive offset. Defaults to 0.
        mult_factor: Multiplicative offset. Defaults to 1.
    """
    z = (CV().bounding_boxes[prop].coords.z + add_factor) * mult_factor
    await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, z, 0, 0, 0), parent=self)
    logger.info(f'Corrected z {z}')
