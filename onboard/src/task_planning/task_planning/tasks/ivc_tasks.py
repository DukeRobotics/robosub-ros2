from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, cast
import aiofiles
import pytz
from rclpy.duration import Duration
from rclpy.logging import get_logger
from task_planning.interface.ivc import IVC, IVCMessageType
from task_planning.task import Task, task
from task_planning.tasks import util_tasks
from task_planning.utils.other_utils import ros_timestamp_to_pacific_time

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
    logger.info(f'Messages received: {messages_received}')
    while True:
        await wait_for_modem_ready(parent=self)

        future = IVC().send_message(msg)
        if future is None:
            logger.error('Could not call IVC send message service.')
        else:
            service_response = cast('SendModemMessage.Response', await future)
            if service_response.success:
                logger.info(f'Sent IVC message: {msg.name}')
            else:
                logger.error(f'Modem failed to send message. Response: {service_response.message}')

        sleep_task = util_tasks.sleep(5, parent=self)
        while not sleep_task.done:
            sleep_task.step()


@task
async def ivc_send(self: Task[None, None, None], msg: IVCMessageType) -> None:
    """Send IVC message."""
    await wait_for_modem_ready(parent=self)

    future = IVC().send_message(msg)
    if future is None:
        logger.error('Could not call IVC send message service.')
    else:
        service_response = cast('SendModemMessage.Response', await future)
        if service_response.success:
            logger.info(f'Sent IVC message: {msg.name}')

            # Log to text file
            async with aiofiles.open('ivc_log.txt', 'a') as f:  # noqa: ASYNC230
                timestamp = ros_timestamp_to_pacific_time(
                    IVC().modem_status.header.stamp.sec,
                    IVC().modem_status.header.stamp.nanosec,
                )
                await f.write(f'Sent IVC message: {msg.name} at {timestamp}\n')
        else:
            logger.error(f'Modem failed to send message. Response: {service_response.message}')


@task
async def ivc_receive(self: Task[None, None, None], timeout: float = 10) -> IVCMessageType:
    """Receive IVC message."""
    await wait_for_modem_ready(parent=self)

    messages_received = len(IVC().messages)

    sleep_task = util_tasks.sleep(timeout, parent=self)
    while not (len(IVC().messages) > messages_received):
        remaining_duration = sleep_task.step()
        if not remaining_duration:
            logger.error('Timeout waiting for message.')
            return False

        logger.info('Waiting for message...')
        await util_tasks.sleep(min(remaining_duration, Duration(seconds=1)), parent=self)

    sleep_task.close()

    logger.info(f'Received IVC message: {IVC().messages[-1].msg.name}')
    messages_received = len(IVC().messages)

    if IVC().messages[-1].msg != IVCMessageType.UNKNOWN:
        return IVC().messages[-1].msg

    logger.warning(f'Received message {IVC().messages[-1].msg.name} is unknown.')
    return IVCMessageType.UNKNOWN

@task
async def ivc_send_blocking(self: Task[None, None, None], msg_to_send: IVCMessageType,
                                msg_to_receive: IVCMessageType, num_attempts: int, timeout: float = 60) -> Task[None, None, None]:
    """Send and wait for an IVC message in a blocking manner."""
    await ivc_send(msg_to_send, parent=self)
    count = num_attempts
    while count != 0 and await ivc_receive(timeout=timeout, parent=self) != msg_to_receive:
        logger.info(f'Unexpected message received. Remaining attempts: {count}')
        count -= 1


@task
async def ivc_receive_then_send(self: Task[None, None, None], check_msg: IVCMessageType, send_msg: IVCMessageType,
                                timeout: float = 60) -> Task[None, None, None]:
    """Receive then after receipt send IVC message."""
    if timeout <= 0:
        return

    while await ivc_receive(timeout=timeout, parent=self) != check_msg:
        logger.info('Unexpected message received.')

    await ivc_send(send_msg, parent=self)


@task
async def delineate_ivc_log(self: Task[None, None, None]) -> Task[None, None, None]:  # noqa: ARG001
    """Append a header to the IVC log file."""
    async with aiofiles.open('ivc_log.txt', 'a') as f:
        await f.write('----- NEW RUN STARTED -----\n')


@task
async def add_to_ivc_log(self: Task[None, None, None], message: str) -> Task[None, None, None]:  # noqa: ARG001
    """Add a message to the IVC log file."""
    async with aiofiles.open('ivc_log.txt', 'a') as f:
        await f.write(f'{message}\n')