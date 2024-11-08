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

logger = get_logger('comp_tasks')


@task
async def wait_for_seconds(self: Task, wait_time=3) -> Task[None, None, None]:
    """
    Complete two full barrel rolls
    """

    logger.info("Started wait_for_seconds task")
    await sleep(3)
    logger.info("Completed wait_for_seconds task")

@task
async def print_smiley_face(self: Task) -> Task[None, None, None]:
    """
    Complete two full barrel rolls
    """

    logger.info(":)")

