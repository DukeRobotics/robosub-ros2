from typing import TYPE_CHECKING, cast

import numpy as np
from custom_msgs.srv import SonarSweepRequest
from rclpy.logging import get_logger
from task_planning.interface.sonar import Sonar
from task_planning.task import Task, task
from task_planning.tasks.move_tasks import create_twist_tolerance, move_to_pose_local
from task_planning.utils import geometry_utils

if TYPE_CHECKING:
   from custom_msgs.srv import SonarSweepRequest


logger = get_logger('sonar_tasks')

MAX_STEPS = 3

@task
async def sonar_test(_self: Task, start_angle: float, end_angle: float, scan_distance: float) -> Task[None, None, None]:
   """Repeatedly perform sonar scans."""
   while True:
       logger.info(f'Sonar scan from {start_angle} to {end_angle} degrees, distance: {scan_distance} m')
       future = Sonar().sweep(
           start_angle=start_angle,
           end_angle=end_angle,
           scan_distance=scan_distance,
       )
       if future is None:
           logger.error('Could not call sonar request service.')
       else:
           service_response = cast('SonarSweepRequest.Response', await future)
           logger.info(f'Sonar scan response: {service_response}')

async def get_normal_angle(start_angle: float, end_angle: float, scan_distance: float) -> float:
    """Get a normal angle from the sonar scan."""
    future = Sonar().sweep(start_angle=start_angle, end_angle=end_angle, scan_distance=scan_distance)
    response = await future
    if not response.is_object:
        logger.error('No object detected — cannot rotate')
        return np.nan
    if response.normal_angle is None:
        logger.error('[Sonar] normal_angle was None — cannot rotate')
        return np.nan
    return response.normal_angle

@task
async def rotate_to_normal(
    self: Task, start_angle: float, end_angle: float, scan_distance: float, yaw_threshold: float,
) -> Task[None, None, None]:
  """Rotates to face a normal angle."""
  logger.info(f'Sonar scan from {start_angle} to {end_angle} degrees, distance: {scan_distance} m')

  normal_angle = await get_normal_angle(start_angle, end_angle, scan_distance)
  await move_to_pose_local(
       geometry_utils.create_pose(0, 0, 0, 0, 0, normal_angle),
       keep_orientation=True,
       pose_tolerances=create_twist_tolerance(angular_yaw=0.1),
       parent=self,
   )
  new_normal_angle = await get_normal_angle(start_angle, end_angle, scan_distance)
  steps = 0
  while(new_normal_angle > yaw_threshold and steps < MAX_STEPS):
      await move_to_pose_local(
           geometry_utils.create_pose(0, 0, 0, 0, 0, new_normal_angle),
           keep_orientation=True,
           pose_tolerances=create_twist_tolerance(angular_yaw=0.1),
           parent=self,
      )
      new_normal_angle = await get_normal_angle(start_angle, end_angle, scan_distance)
      steps += 1

@task
async def rotate_to_angle_from_normal(
    self: Task,
    start_angle: float,
    end_angle: float,
    scan_distance: float,
    yaw_threshold: float,
    rotated_angle: float,
) -> Task[None, None, None]:
  """Rotates to a specified angle using Sonar normal angle."""
  logger.info(f'Sonar scan from {start_angle} to {end_angle} degrees, distance: {scan_distance} m')

  angle = await get_normal_angle(start_angle, end_angle, scan_distance)
  angle = rotated_angle + angle
  await move_to_pose_local(
       geometry_utils.create_pose(0, 0, 0, 0, 0, angle),
       keep_orientation=True,
       pose_tolerances=create_twist_tolerance(angular_yaw=0.1),
       parent=self,
   )
  new_angle = await get_normal_angle(start_angle, end_angle, scan_distance)
  new_angle = rotated_angle + new_angle
  steps = 0
  while(new_angle > yaw_threshold + angle and steps < MAX_STEPS):
      await move_to_pose_local(
           geometry_utils.create_pose(0, 0, 0, 0, 0, new_angle),
           keep_orientation=True,
           pose_tolerances=create_twist_tolerance(angular_yaw=0.1),
           parent=self,
      )
      new_angle = await get_normal_angle(start_angle, end_angle, scan_distance)
      new_angle = rotated_angle + new_angle
      steps += 1
