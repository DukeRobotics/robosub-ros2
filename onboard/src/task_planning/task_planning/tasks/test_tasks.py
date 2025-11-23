from rclpy.logging import get_logger
from std_msgs.msg import String
from task_planning.task import Task, task
from task_planning.tasks import util_tasks

logger = get_logger('test_tasks')


@task
async def wait_for_seconds(_self: Task, wait_time: float = 1) -> Task[None, None, None]:
    """Wait for a number of seconds."""
    logger.info('Started wait_for_seconds task')
    await util_tasks.sleep(wait_time)
    logger.info('Completed wait_for_seconds task')


@task
async def print_task(_self: Task, data: str = 'data') -> Task[None, None, None]:
    """Print a string."""
    msg = String()
    msg.data = data
    logger.info(f'print_task: {data}')

    return msg


@task
async def wait_then_print(self: Task, wait_time: float = 1) -> Task[None, None, None]:
    """Execute a wait_for_seconds task and then a print_task task."""
    await wait_for_seconds(wait_time, parent=self)
    await print_task(parent=self)



