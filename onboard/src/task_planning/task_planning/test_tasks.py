from rclpy.logging import get_logger
from rclpy.clock import Clock
from rclpy.duration import Duration

import math
import copy
import numpy as np

from task_planning.task import Task, task
import task_planning.move_tasks as move_tasks
import task_planning.cv_tasks as cv_tasks
from task_planning.utils import geometry_utils
from task_planning.task import Yield

from enum import Enum

from task_planning.interface.state import State
from task_planning.interface.cv import CV
from task_planning.interface.controls import Controls
from task_planning.interface.marker_dropper import MarkerDropper

from task_planning.utils.coroutine_utils import sleep
from std_msgs.msg import String

logger = get_logger('comp_tasks')


@task
async def wait_for_seconds(self: Task, wait_time=3) -> Task[None, None, None]:
    """
    Wait for a number of seconds.
    """

    logger.info("Started wait_for_seconds task")
    await sleep(wait_time)
    logger.info("Completed wait_for_seconds task")

@task
async def print_task(self: Task, data: str = ':)') -> Task[None, None, None]:
    """
    Print a string.
    """
    msg = String()
    msg.data = data
    logger.info(data)

    return msg


@task
async def wait_then_print(self: Task, wait_time=3) -> Task[None, None, None]:
    """
    Execute a wait_for_seconds task and then a print_task task.
    """

    await wait_for_seconds(wait_time, parent=self)
    await print_task(parent=self)
