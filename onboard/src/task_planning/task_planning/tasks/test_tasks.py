from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.logging import get_logger
from std_msgs.msg import String
from task_planning.interface.ivc import IVC
from task_planning.task import Task, Yield, task
from task_planning.utils.coroutine_utils import sleep

logger = get_logger('test_tasks')


@task
async def wait_for_seconds(_self: Task, wait_time: float = 1) -> Task[None, None, None]:
    """Wait for a number of seconds."""
    logger.info('Started wait_for_seconds task')
    await sleep(wait_time)
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


@task
async def test_ivc(self: Task, msg: str) -> Task[None, None, None]:
    """Test inter-vehicle communication."""
    while not IVC().received_modem_status:
        logger.info('Waiting for modem status...')
        await sleep(1)
    logger.info('Received modem status.')

    # Send a IVC message, then wait for a response for 5 seconds, then send another message, and repeat
    messages_received = len(IVC().messages)
    while True:
        logger.info(f'Sending IVC message: {msg}')
        IVC().send_message(msg)

        duration = Duration(seconds=5)
        start_time = Clock().now()
        while start_time + duration > Clock().now():
            if len(IVC().messages) > messages_received:
                logger.info(f'Received IVC message: {IVC().messages[-1].data}')
                messages_received = len(IVC().messages)
            await Yield()
