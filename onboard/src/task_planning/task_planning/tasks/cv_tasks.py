from rclpy.logging import get_logger
from task_planning.interface.cv import CV, CVObjectType
from task_planning.task import Task, Yield, task
from task_planning.tasks import move_tasks
from task_planning.tasks.move_tasks import move_to_pose_local
from task_planning.utils import geometry_utils

logger = get_logger('cv_tasks')


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
