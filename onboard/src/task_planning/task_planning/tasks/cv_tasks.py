import math

from geometry_msgs.msg import Twist, Vector3
from rclpy.clock import Clock
from rclpy.logging import get_logger
from task_planning.interface.cv import CV, CVObjectType
from task_planning.interface.state import State
from task_planning.task import Task, Yield, task
from task_planning.tasks import move_tasks, util_tasks
from task_planning.utils import geometry_utils
from task_planning.interface.controls import Controls

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
async def yaw_to_cv_obj(self: Task, cv_object: CVObjectType , search_direction: int = 1,
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

    # At this point, depth_level is positive, since yaw_until_object_detection expects positive depth_level
    if not CV().is_receiving_recent_cv_data(cv_object, 10):
        found_object = await yaw_until_object_detection(cv_object, search_direction,
                                                        depth_threshold, depth_level, parent=self)
        if not found_object:
            logger.info('[cv_tasks.yaw_to_cv_obj] Never found CV Object. Ending yaw_to_cv_object.')
            return False

    # From this point on, all remaining code expects negative depth level
    depth_level = State().orig_depth - depth_level

    @task
    async def correct_depth(self: Task) -> None:
        await move_tasks.depth_correction(desired_depth=depth_level, parent=self)


    cv_object_yaw = CV().angles[cv_object]
    move_to_pose_task = move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, cv_object_yaw),
                            depth_level=depth_level,
                            pose_tolerances=Twist(linear=Vector3(x=0.05, y=0.05, z=0.05),
                                                    angular=Vector3(x=0.2, y=0.3, z=yaw_threshold)),
                            timeout=30,
                            parent=self)

    move_to_pose_task.step()

    clock = Clock()
    starting_time = clock.now()

    while not move_to_pose_task.done:
        if abs(State().depth - depth_level) > depth_threshold:
            await correct_depth(parent=self)

        if not CV().is_receiving_recent_cv_data(cv_object, 10):
            logger.info('[cv_tasks.yaw_to_cv_obj] Lost sight of cv object. Yawing until object detection...')
            found_object = await yaw_until_object_detection(cv_object, search_direction,
                                                        depth_threshold, depth_level, parent=self)
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
                        depth_threshold: float = 0.2, depth_level: float = 0.5,) -> Task[None, str | None, None] | bool:
    """
    Move to the pose of an object detected by CV.

    Returns when the robot is at the object's pose with zero velocity, within a small tolerance.

    Args:
        self: Task instance.
        cv_object: CV class name of the object to move to
        target_distance: The goal distance to be from the CV object when finished
        search_direction: If no CV object in view, which direction should it search in.
                                1 for positive yaw, -1 for negative yaw.
        depth_threshold: The error in depth that will cause a depth-correct call
        depth_level: Desire depth level to hold throughout the task. Must be positive.
    Send:
        CV class name of new object to move to
    """
    yaw_threshold = math.radians(10)
    yaw_stop_threshold = math.radians(25)
    biggest_forward_step = 2

    depth_level_to_pass = depth_level # STUPID STUPID STUPID, PLEASE FIX DEPTH STUFF NEXT YEAR
    depth_level = State().orig_depth - depth_level

    @task
    async def correct_depth(self: Task) -> None:
        await move_tasks.depth_correction(desired_depth=depth_level, parent=self)

    cv_object_yaw = CV().angles[cv_object]
    current_dist = CV().bounding_boxes[cv_object].coords.x + CV().bounding_boxes[cv_object].coords.y
    current_goal_distance = min(biggest_forward_step, current_dist - target_distance)
    move_task = move_tasks.move_to_pose_local(
                            geometry_utils.create_pose(current_goal_distance, 0, 0, 0, 0, cv_object_yaw),
                            depth_level=depth_level,
                            pose_tolerances=Twist(linear=Vector3(x=0.05, y=0.05, z=0.05),
                                                    angular=Vector3(x=0.2, y=0.3, z=yaw_threshold)),
                            timeout=240,
                            parent=self)
    move_task.step()

    logger.info(f'[cv_tasks.move_to_cv_obj] Beginning move to cv object.')

    touching_x_boundary = [0,0,0,0,0]
    touching_y_boundary = [0,0,0,0,0]

    while not move_task.done:
        if abs(State().depth - depth_level) > depth_threshold:
            await correct_depth(parent=self)

        cv_object_yaw = CV().angles[cv_object]
        current_dist = CV().bounding_boxes[cv_object].coords.x + CV().bounding_boxes[cv_object].coords.y
        current_goal_distance = min(biggest_forward_step, current_dist - target_distance)

        touching_x_boundary.append(1 if (CV().bounding_boxes[cv_object].xmin < 0.01 or CV().bounding_boxes[cv_object].xmax > 0.99) else 0)
        touching_y_boundary.append(1 if (CV().bounding_boxes[cv_object].ymin < 0.01 or CV().bounding_boxes[cv_object].ymax > 0.99) else 0)
        touching_x_boundary.pop(0)
        touching_y_boundary.pop(0)

        if sum(touching_x_boundary) >= 4:
            touching_x_boundary = [0,0,0,0,0]
            logger.info(f'[cv_tasks.move_to_cv_obj] Object approached x bounds. Calling yaw to cv object.')
            yaw_task = await yaw_to_cv_obj(cv_object, search_direction, yaw_threshold, depth_threshold,
                                            depth_level_to_pass, parent=self)
            if not yaw_task:
                logger.info('[cv_tasks.move_to_cv_obj] Failure. Object never found, finishing move_to_cv_obj')
                return False

            if CV().bounding_boxes[cv_object].xmin < 0.01 or CV().bounding_boxes[cv_object].xmax > 0.99:
                logger.info('[cv_tasks.move_to_cv_obj] Object still in x bounds, backing up.')
                await move_tasks.move_to_pose_local(
                            geometry_utils.create_pose(-0.5, 0, 0, 0, 0, cv_object_yaw),
                            depth_level=depth_level,
                            pose_tolerances=Twist(linear=Vector3(x=0.05, y=0.05, z=0.05),
                                                    angular=Vector3(x=0.2, y=0.3, z=yaw_threshold)),
                            timeout=30,
                            parent=self)

        if  sum(touching_y_boundary) >= 4:
            touching_y_boundary = [0,0,0,0,0]
            logger.info(f'[cv_tasks.move_to_cv_obj] Object approached y bounds. Correcting Yaw.')

            yaw_task = await yaw_to_cv_obj(cv_object, search_direction, yaw_threshold, depth_threshold,
                                            depth_level_to_pass, parent=self)
            if not yaw_task:
                logger.info('[cv_tasks.move_to_cv_obj] Failure. Object never found, finishing move_to_cv_obj')
                return False

            if CV().bounding_boxes[cv_object].ymin < 0.01 or CV().bounding_boxes[cv_object].ymax > 0.99:
                logger.info('[cv_tasks.move_to_cv_obj] Object still in y bounds, backing up.')
                await move_tasks.move_to_pose_local(
                            geometry_utils.create_pose(-0.5, 0, 0, 0, 0, cv_object_yaw),
                            depth_level=depth_level,
                            pose_tolerances=Twist(linear=Vector3(x=0.05, y=0.05, z=0.05),
                                                    angular=Vector3(x=0.2, y=0.3, z=yaw_threshold)),
                            timeout=30,
                            parent=self)

        if not CV().is_receiving_recent_cv_data(cv_object, 10) or abs(cv_object_yaw) > yaw_stop_threshold:
            logger.info(f'[cv_tasks.move_to_cv_obj] Object yawed too far. Calling yaw to cv object.')
            yaw_task = await yaw_to_cv_obj(cv_object, search_direction, yaw_threshold, depth_threshold,
                                            depth_level_to_pass, parent=self)
            if not yaw_task:
                logger.info('[cv_tasks.move_to_cv_obj] Failure. Object never found, finishing move_to_cv_obj')
                return False

        logger.info(f'[cv_tasks.move_to_cv_obj] Current Goal Distance: {current_goal_distance}')
        new_pose = geometry_utils.create_pose(current_goal_distance, 0, 0, 0, 0, cv_object_yaw)
        move_task.send(new_pose)
        await util_tasks.sleep(0.05, parent=self)

    logger.info('[cv_tasks.move_to_cv_obj] PID loop complete, finishing move_to_cv_obj')
    Controls().publish_desired_power(Twist())
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
