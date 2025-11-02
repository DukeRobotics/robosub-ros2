from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, cast

import pytz
from rclpy.duration import Duration
from rclpy.logging import get_logger
from task_planning.interface.ivc import IVC, IVCMessageType
from task_planning.task import Task, task
from task_planning.tasks import util_tasks

if TYPE_CHECKING:
    from custom_msgs.srv import SendModemMessage

# TODO: rewrite/reorganize/clean up the comp ivc_tasks
# TODO: add docstrings for all tasks

logger = get_logger('ivc_tasks')

def ros_timestamp_to_pacific_time(sec: int, nanosec: int) -> str:
    """
    Convert ROS timestamp (seconds and nanoseconds) to human-readable Pacific time.

    # TODO: move to utils + merge with same function in ivc.py

    Args:
        sec (int): Seconds since epoch
        nanosec (int): Nanoseconds

    Returns:
        str: Human-readable timestamp in Pacific timezone
    """
    # Convert to datetime object
    pacific_tz = pytz.timezone('US/Pacific')
    timestamp = datetime.fromtimestamp(sec + nanosec / 1e9, tz=pacific_tz)

    # Convert to Pacific timezone
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
            with Path('ivc_log.txt').open('a') as f:  # noqa: ASYNC230 TODO switch to async IO
                timestamp = ros_timestamp_to_pacific_time(
                    IVC().modem_status.header.stamp.sec,
                    IVC().modem_status.header.stamp.nanosec,
                )
                f.write(f'Sent IVC message: {msg.name} at {timestamp}\n')
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
async def crush_ivc_spam(self: Task[None, None, None], msg_to_send: IVCMessageType) -> Task[None, None, None]:
    """Spam IVC message for Crush."""
    while True:
        await ivc_send(msg_to_send, parent=self) # Send crush is done with gate
        await util_tasks.sleep(20, parent=self)


@task
async def ivc_send_then_receive(self: Task[None, None, None], msg_to_send: IVCMessageType,
                                msg_to_receive: IVCMessageType, timeout: float = 60) -> Task[None, None, None]:
    """Send then receive IVC message."""
    await ivc_send(msg_to_send, parent=self) # Send crush is done with gate

    count = 2
    # Wait for Oogway to say starting/acknowledge command
    while count != 0 and await ivc_receive(timeout=timeout, parent=self) != msg_to_receive:
        logger.info(f'Unexpected message received. Remaining attempts: {count}')
        count -= 1


@task
async def crush_ivc_send(self: Task[None, None, None], msg_to_send: IVCMessageType,
                         msg_to_receive: IVCMessageType, timeout: float = 60) -> Task[None, None, None]:
    """Send IVC message crush."""
    await ivc_send(msg_to_send, parent=self) # Send crush is done with gate

    count = 2
    # Wait for Oogway to say starting/acknowledge command
    while count != 0 and await ivc_receive(timeout=timeout, parent=self) != msg_to_receive:
        logger.info(f'Unexpected message or no message received. Sending again. Remaining attempts: {count - 1}')
        await ivc_send(msg_to_send, parent=self) # Send crush is done with gate
        count -= 1

    if count == 0:
        logger.info('No acknowledgement, sending one last time')
        await ivc_send(msg_to_send, parent=self) # Send crush is done with gate


@task
async def crush_ivc_receive(self: Task[None, None, None], msg_to_receive: IVCMessageType,
                            msg_to_send: IVCMessageType, timeout: float = 60) -> Task[None, None, None]:
    """Receive IVC message crush."""
    count = 2
    # Wait for Oogway to say starting/acknowledge command
    while count != 0 and await ivc_receive(timeout=timeout, parent=self) != msg_to_receive:
        logger.info(f'Unexpected message or no message received. Remaining attempts: {count - 1}')
        count -= 1

    await ivc_send(msg_to_send, parent=self) # Send crush is done with gate


@task
async def ivc_receive_then_send(self: Task[None, None, None], msg: IVCMessageType,
                                timeout: float = 60) -> Task[None, None, None]:
    """Receive then after receipt send IVC message."""
    if timeout <= 0:
        return

    # Wait until Crush is done with gate
    while await ivc_receive(timeout=timeout, parent=self) != IVCMessageType.CRUSH_GATE:
        logger.info('Unexpected message received.')

    await ivc_send(msg, parent=self) # Oogway says ok and starting


@task
async def delineate_ivc_log(self: Task[None, None, None]) -> Task[None, None, None]:  # noqa: ARG001
    """Append a header to the IVC log file."""
    with Path('ivc_log.txt').open('a') as f:  # noqa: ASYNC230 TODO eventually use async io
        f.write('----- NEW RUN STARTED -----\n')


@task
async def add_to_ivc_log(self: Task[None, None, None], message: str) -> Task[None, None, None]:  # noqa: ARG001
    """Add a message to the IVC log file."""
    with Path('ivc_log.txt').open('a') as f: # noqa: ASYNC230 TODO eventually use async io
        f.write(f'{message}\n')
