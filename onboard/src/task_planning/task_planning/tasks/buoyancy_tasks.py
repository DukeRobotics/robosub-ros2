import copy
import math
from collections.abc import Coroutine
from enum import Enum

import numpy as np
from custom_msgs.msg import ControlTypes
from geometry_msgs.msg import Twist
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.logging import get_logger
from transforms3d.euler import quat2euler

from task_planning.interface.controls import Controls
from task_planning.interface.cv import CV
from task_planning.interface.servos import Servos, MarkerDropperStates
from task_planning.interface.state import State
from task_planning.task import Task, Yield, task
from task_planning.tasks import cv_tasks, move_tasks
from task_planning.utils import geometry_utils
from task_planning.utils.coroutine_utils import sleep

logger = get_logger('buoyancy_tasks')

@task
async def buoyancy_task(self: Task, submerge_depth: float) -> Task[None, None, None]:
    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, submerge_depth, 0, 0, 0),
        keep_level=False,
        parent=self,
    )
    logger.info(f'Submerged {submerge_depth} meters')

    while(True):
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, 0),
            keep_level=False,
            parent=self,
        )
        imu_euler_angles = State().imu_euler_angles
        logger.info("Roll: {:.2f}, Pitch: {:.2f}".format(
            math.degrees(imu_euler_angles.x),
            math.degrees(imu_euler_angles.y),
        ))
