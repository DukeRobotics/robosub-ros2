from rclpy.duration import Duration
from rclpy.logging import get_logger
from task_planning.interface.acouustics import Acoustics 
from task_planning.task import Task, task
from task_planning.tasks import util_tasks

logger = get_logger('acoustics_tasks')

@task
async def move_torward_pinger(self: Task) -> Task[None, None, None]:
    logger.info("[move_torward_pinger] Calling acoustics service.")
    response = Acoustics().request()

    if not response.is_object:
        logger.error(f'[move_torward_pinger] Error from acoustics service')
    
    closest = response.closest

    # TODO ask about directions and move to that direction
