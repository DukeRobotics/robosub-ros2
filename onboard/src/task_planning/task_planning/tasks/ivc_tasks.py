from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.logging import get_logger
from task_planning.interface.ivc import IVC, IVCMessageType
from task_planning.task import Task, Yield, task
from task_planning.tasks import util_tasks

logger = get_logger('ivc_tasks')

@task
async def wait_for_modem_status(self: Task) -> Task[None, None, None]:
    """Wait for modem status to be received."""
    while not IVC().received_modem_status:
        logger.info('Waiting for modem status...')
        await util_tasks.sleep(1, parent=self)
    logger.info('Received modem status.')

@task
async def test_ivc(self: Task, msg: IVCMessageType) -> Task[None, None, None]:
    """Test inter-vehicle communication."""
    await wait_for_modem_status(parent=self)

    # Send a IVC message, then wait for a response for 5 seconds, then send another message, and repeat
    messages_received = len(IVC().messages)
    while True:
        logger.info(f'Sending IVC message: {msg} = {msg.value}')
        IVC().send_message(msg)

        sleep_task = util_tasks.sleep(5, parent=self)
        while not sleep_task.done:
            if len(IVC().messages) > messages_received:
                logger.info(f'Received IVC message: {IVC().messages[-1]}')
                messages_received = len(IVC().messages)
            sleep_task.step()
