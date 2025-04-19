from typing import TYPE_CHECKING, cast

from rclpy.duration import Duration
from rclpy.logging import get_logger
from task_planning.interface.ivc import IVC, IVCMessageType
from task_planning.task import Task, task
from task_planning.tasks import util_tasks

if TYPE_CHECKING:
    from custom_msgs.srv import SendModemMessage

logger = get_logger('ivc_tasks')

@task
async def wait_for_modem_status(self: Task[None, None, None], timeout: float = 10) -> bool:
    """
    Wait for modem status to be received.

    Args:
        self (Task): The task instance.
        timeout (float): The timeout in seconds. Defaults to 10 seconds.

    Returns:
        bool: True if modem status is received, False otherwise.
    """
    sleep_task = util_tasks.sleep(timeout, parent=self)
    while not IVC().received_modem_status:
        remaining_duration = sleep_task.step()
        if not remaining_duration:
            logger.error('Timeout waiting for modem status.')
            return False

        logger.info('Waiting for modem status...')
        await util_tasks.sleep(min(remaining_duration, Duration(seconds=1)), parent=self)

    sleep_task.close()
    logger.info('Received modem status.')
    return True

@task
async def wait_for_modem_ready(self: Task[None, None, None], timeout: float = 15) -> bool:
    """
    Wait for modem to be ready.

    Args:
        self (Task): The task instance.
        timeout (float): The timeout in seconds. Defaults to 15 seconds.

    Returns:
        bool: True if the modem is ready, False otherwise.
    """
    sleep_task = util_tasks.sleep(timeout, parent=self)
    received_modem_status = await wait_for_modem_status(timeout=timeout, parent=self)
    if not received_modem_status:
        logger.error('Modem status not received so modem is not ready.')
        return False

    while IVC().modem_status.busy:
        remaining_duration = sleep_task.step()
        if not remaining_duration:
            logger.error('Timeout waiting for modem to be ready.')
            return False

        logger.info('Waiting for modem to be ready...')
        await util_tasks.sleep(min(remaining_duration, Duration(seconds=1)), parent=self)

    sleep_task.close()
    logger.info('Modem is ready.')
    return True

@task
async def test_ivc(self: Task[None, None, None], msg: IVCMessageType) -> None:
    """Test inter-vehicle communication."""
    # Send a IVC message, then wait for a response for 5 seconds, then send another message, and repeat
    messages_received = len(IVC().messages)
    while True:
        await wait_for_modem_ready(parent=self)

        logger.info(f'Sending IVC message: {msg.name}')
        future = IVC().send_message(msg)
        if future is None:
            logger.error('Could not call modem send message service.')
        else:
            service_response = cast('SendModemMessage.Response', await future)
            if not service_response.success:
                logger.error(f'Modem failed to send message. Response: {service_response.message}')

        sleep_task = util_tasks.sleep(5, parent=self)
        while not sleep_task.done:
            if len(IVC().messages) > messages_received:
                logger.info(f'Received IVC message: {IVC().messages[-1].msg.name}')
                messages_received = len(IVC().messages)
            sleep_task.step()
