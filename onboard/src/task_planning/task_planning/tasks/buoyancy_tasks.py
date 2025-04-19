# ruff: noqa

import copy
import math
from collections.abc import Coroutine
from enum import Enum

import numpy as np
from custom_msgs.msg import ControlTypes
from geometry_msgs.msg import Twist, Vector3
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
async def tune_static_power(self: Task) -> Task[None, None, None]:
    """Find Z position setpoint that makes the robot stay a depth of -1m."""

    DELTA_SCALE_FACTOR = 0.65
    MAX_ERROR = 0.05
    curr_depth = State().depth
    TARGET_DEPTH = State().orig_depth - 1.0
    delta = curr_depth - TARGET_DEPTH
    move_to_depth = -1.0

    while (abs(delta) > MAX_ERROR):
        logger.info(f'Move to depth: {move_to_depth}')
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, 0),
            keep_level=False,
            depth_level=move_to_depth,
            pose_tolerances=Twist(linear=Vector3(x=0.05, y=0.05, z=0.3), angular=Vector3(x=0.2, y=0.3, z=0.1)),
            parent=self,
        )

        curr_depth = State().depth
        delta = curr_depth - TARGET_DEPTH
        move_to_depth = TARGET_DEPTH - DELTA_SCALE_FACTOR * delta

@task
async def buoyancy_task(self: Task, submerge_depth: float) -> Task[None, None, None]:
    """Move robot forward and backward 1 meter while submerged."""
    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, submerge_depth, 0, 0, 0),
        keep_level=False,
        parent=self,
    )
    logger.info(f'Submerged {submerge_depth} meters')

    x_pose = 1

    while True:
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(x_pose, 0, 0, 0, 0, 0),
            keep_level=False,
            parent=self,
        )
        imu_euler_angles = State().imu_euler_angles
        logger.info(f'Roll: {math.degrees(imu_euler_angles.x):.2f}, Pitch: {math.degrees(imu_euler_angles.y):.2f}')
        x_pose *= -1
