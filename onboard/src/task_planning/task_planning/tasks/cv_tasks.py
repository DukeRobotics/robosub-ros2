import math

from geometry_msgs.msg import Twist, Vector3
from rclpy.clock import Clock
from rclpy.logging import get_logger
from task_planning.interface.cv import CV, CVObjectType
from task_planning.task import Task, Yield, task
from task_planning.tasks import move_tasks
from task_planning.tasks.move_tasks import move_to_pose_local
from task_planning.utils import geometry_utils
from task_planning.interface.state import State

logger = get_logger('cv_tasks')

@task
async def yaw_until_object_detection(self: Task, cv_object: CVObjectType , search_direction=1,
                                    depth_threshold = 0.2, depth_level=0.5) -> Task[None, str | None, None] | bool:
    """
    Yaws until an object is detected by CV

    Returns when the robot looking at the CV object, within a small tolerance.

    Args:
        self: Task instance.
        cv_object: CV class name of the object to yaw to
        search_direction: If no CV object in view, which direction should it search in. 1 for positive yaw, -1 for negative yaw.
        depth_threshold: The error in depth that will cause a depth-correct call
        depth_level: Desire depth level to hold throughout the task

    Send:
        CV class name of new object to yaw to
    """
    DEPTH_LEVEL = State().orig_depth - depth_level

    logger.info("Beginning yaw_util_object_detection task")
    YAW_PID_STEPS = math.radians(45)

    @task
    async def correct_depth():
        await move_tasks.depth_correction(desired_depth=DEPTH_LEVEL, parent=self)

    @task
    async def object_search_pattern():
        step = 0
        while step <= 7:
            if step <= 2:
                angle = YAW_PID_STEPS
            elif step == 3:
                angle = -3 * YAW_PID_STEPS
            elif step <= 6:
                angle = -1 * YAW_PID_STEPS
            else:
                angle = 3 * YAW_PID_STEPS
                logger.info('Yawed to find object more than 7 times, breaking loop.')
            await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, angle * search_direction),
                                    depth_level=depth_level,
                                    pose_tolerances=Twist(linear=Vector3(x=0.05, y=0.05, z=0.05), angular=Vector3(x=0.2, y=0.3, z=0.3)),
                                    time_limit=10,
                                    parent=self)
            # TODO confirm that this actually steps and doesn't go through the whole thing when we call step out outer function
            step += 1
        return False

    object_search_task = object_search_pattern()

    while not CV().is_receiving_recent_cv_data(cv_object, 10):
        if object_search_task.done:
            return False

        if (abs(State().depth - DEPTH_LEVEL) < depth_threshold):
            await correct_depth()

        object_search_task.step()

    return True

@task
async def yaw_to_cv_obj(self: Task, cv_object: CVObjectType , search_direction=1,
                           yaw_threshold=math.radians(10), depth_threshold = 0.2, depth_level=0.5,
                           pid_timeout = 10) -> Task[None, str | None, None] | None:
    """
    Yaw to an object detected by CV.

    Returns when the robot looking at the CV object, within a small tolerance.

    Args:
        self: Task instance.
        cv_object: CV class name of the object to yaw to
        search_direction: If no CV object in view, which direction should it search in. 1 for positive yaw, -1 for negative yaw.
        yaw_threshold: Tolerance for completing the task
        depth_threshold: The error in depth that will cause a depth-correct call
        depth_level: Desire depth level to hold throughout the task
        pid_timeout: Time within PID loop before automatically breaking out

    Send:
        CV class name of new object to yaw to
    """
    DEPTH_LEVEL = State().orig_depth - depth_level
    SCALE_FACTOR = 0.4 # TODO why this: How much the correction should be scaled down from yaw calculation

    @task
    async def correct_depth():
        await move_tasks.depth_correction(desired_depth=DEPTH_LEVEL, parent=self)

    if not CV().is_receiving_recent_cv_data(cv_object, 10):
        found_object = await yaw_until_object_detection()
        if not found_object:
            return

    logger.info('Starting yaw_to_cv_object')

    cv_object_yaw = CV().angles[cv_object]
    move_to_pose_task = move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, cv_object_yaw),
                            depth_level=depth_level,
                            pose_tolerances=Twist(linear=Vector3(x=0.05, y=0.05, z=0.05),
                                                    angular=Vector3(x=0.2, y=0.3, z=yaw_threshold)),
                            time_limit=10,
                            parent=self)

    starting_time = Clock().now()

    while not move_to_pose_task.done:
        if abs(State().depth - DEPTH_LEVEL) < depth_threshold:
            await correct_depth()

        move_to_pose_task.step()

        cv_object_yaw = CV().angles[cv_object]
        move_to_pose_task.send(cv_object_yaw)

        if Clock.now() - starting_time < pid_timeout:
            logger.info('Timeout elapsed, Finishing Yaw To CV Object')
            return

    logger.info('PID Loop Complete, Finishing Yaw To CV Object')
    return


# TODO: this task will likely be depleted once we complete the refactoring tasks in comp_tasks.py
@task
async def move_to_cv_obj(self: Task, name: CVObjectType) -> Task[None, str | None, None]:
    """
    Move to the pose of an object detected by CV.

    Returns when the robot is at the object's pose with zero velocity, within a small tolerance.

    Args:
        self: Task instance.
        name: CV class name of the object to move to

    Send:
        CV class name of new object to move to
    """
    # Get initial object location and initialize task to move to it
    pose = CV().get_pose(name)
    move_task = move_to_pose_local(pose, parent=self)
    move_task.send(None)

    # Move until the robot has reached the object's pose
    # TODO: Stop when stopped recieving decisions
    # TODO: Stop within stop_distance (param)
    while not move_task.done:
        # Update object to move to
        updated_obj = await Yield(pose)

        if updated_obj is not None:
            name = updated_obj

        pose = CV().get_pose(name)

        # TODO: Add offset
        move_task.send(pose)


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
