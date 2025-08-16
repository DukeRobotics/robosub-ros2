from typing import TYPE_CHECKING, cast
from datetime import datetime
import pytz

from rclpy.duration import Duration
from rclpy.logging import get_logger
from task_planning.interface.ivc import IVC, IVCMessageType
from task_planning.task import Task, task
from task_planning.tasks import util_tasks

if TYPE_CHECKING:
    from custom_msgs.srv import SendModemMessage

logger = get_logger('ivc_tasks')

def ros_timestamp_to_pacific_time(sec: int, nanosec: int) -> str:
    """
    Convert ROS timestamp (seconds and nanoseconds) to human-readable Pacific time.

    # TODO: move to utils

    Args:
        sec (int): Seconds since epoch
        nanosec (int): Nanoseconds

    Returns:
        str: Human-readable timestamp in Pacific timezone
    """
    # Convert to datetime object
    timestamp = datetime.fromtimestamp(sec + nanosec / 1e9)

    # Convert to Pacific timezone
    pacific_tz = pytz.timezone('US/Pacific')
    pacific_time = timestamp.astimezone(pacific_tz)

    # Format as human-readable string
    return pacific_time.strftime('%Y-%m-%d %H:%M:%S %Z')

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
    await wait_for_modem_ready(parent=self)

    future = IVC().send_message(msg)
    if future is None:
        logger.error('Could not call IVC send message service.')
    else:
        service_response = cast('SendModemMessage.Response', await future)
        if service_response.success:
            logger.info(f'Sent IVC message: {msg.name}')

            # Log to text file
            with open("ivc_log.txt", "a") as f:
                timestamp = ros_timestamp_to_pacific_time(
                    IVC().modem_status.header.stamp.sec,
                    IVC().modem_status.header.stamp.nanosec
                )
                f.write(f'Sent IVC message: {msg.name} at {timestamp}\n')
                # f.write(f'Sent IVC message: {msg.name} at {IVC().modem_status.header.stamp.sec}.{IVC().modem_status.header.stamp.nanosec}\n')
        else:
            logger.error(f'Modem failed to send message. Response: {service_response.message}')

@task
async def ivc_receive(self: Task[None, None, None], timeout: float = 10) -> IVCMessageType:
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

    logger.warning(f'Received message {IVC().messages[-1].msg.name} does not match expected message {msg.name}.')
    return IVCMessageType.UNKNOWN
