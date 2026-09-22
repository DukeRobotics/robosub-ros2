from rclpy.logging import get_logger
from task_planning.interface.acoustics import Acoustics
from task_planning.task import Task, task

logger = get_logger('acoustics_tasks')


@task
async def request_acoustics(self: Task) -> Task[None, None, None]:  # noqa: ARG001 - self required by @task
    """
    Make a call to acoustics, and return the response.

    Future processing can be done here. For now, abstraction will be trusted!
    """
    logger.info('[move_torward_pinger] Calling acoustics service.')
    response = Acoustics().request()

    if not response.is_object:
        logger.error('[move_torward_pinger] Error from acoustics service')

    return response
