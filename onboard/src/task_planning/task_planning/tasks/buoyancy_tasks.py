import math

from rclpy.logging import get_logger
from task_planning.interface.state import State
from task_planning.task import Task, task
from task_planning.tasks import move_tasks
from task_planning.utils import geometry_utils

logger = get_logger('buoyancy_tasks')

@task
async def buoyancy_task(self: Task, submerge_depth: float) -> Task[None, None, None]:
    """Move robot forward and backward 1 meter while submerged."""
    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, submerge_depth, 0, 0, 0),
        keep_level=False,
        parent=self,
    )
    logger.info(f'Submerged {submerge_depth} meters')

    x_pose = 1

    while True:
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(x_pose, 0, 0, 0, 0, 0),
            keep_level=False,
            parent=self,
        )
        imu_euler_angles = State().imu_euler_angles
        logger.info(f'Roll: {math.degrees(imu_euler_angles.x):.2f}, Pitch: {math.degrees(imu_euler_angles.y):.2f}')
        x_pose *= -1
