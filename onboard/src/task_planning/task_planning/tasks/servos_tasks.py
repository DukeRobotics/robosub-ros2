from rclpy.duration import Duration
from rclpy.logging import get_logger
from task_planning.interface.servos import MarkerDropperStates, Servos, TorpedoStates
from task_planning.task import Task, task
from task_planning.tasks import util_tasks

logger = get_logger('servos_tasks')


@task
async def fire_torpedo(self: Task, torpedo_side: TorpedoStates) -> Task[None, None, None]:
    """Fire the torpedo on the given side."""
    logger.info(f'[fire_torpedo] Firing torpedo: {torpedo_side}')
    Servos().fire_torpedo(torpedo_side)
    logger.info(f'[fire_torpedo] Fired torpedo: {torpedo_side}')
    await util_tasks.sleep(Duration(seconds=3), parent=self)


@task
async def drop_marker(self: Task, marker_side: MarkerDropperStates) -> Task[None, None, None]:
    """Drop the marker on the given side."""
    logger.info(f'[drop_marker] Dropping marker: {marker_side}')
    Servos().drop_marker(marker_side)
    logger.info(f'[drop_marker] Dropped marker: {marker_side}')
    await util_tasks.sleep(Duration(seconds=2), parent=self)
