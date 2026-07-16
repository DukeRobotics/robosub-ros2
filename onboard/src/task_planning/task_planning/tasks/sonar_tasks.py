from typing import cast

import numpy as np
from custom_msgs.srv import SonarSweepRequest
from rclpy.logging import get_logger
from task_planning.interface.sonar import Sonar
from task_planning.task import Task, task
from task_planning.tasks.move_tasks import create_twist_tolerance, move_to_pose_local
from task_planning.utils import geometry_utils

logger = get_logger('sonar_tasks')

MAX_STEPS = 3
CORRECTION_GAIN = 1.35  # Fraction of the measured angle to correct per step; damps noise/overshoot to avoid diverging
ROTATION_YAW_TOLERANCE = 0.03  # rad; must be small relative to typical correction sizes or moves finish before arriving


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


async def scan_normal_angle(start_angle: float, end_angle: float, scan_distance: float) -> float:
    """
    Perform a sonar sweep and return its wall-normal angle.

    Args:
        start_angle (float): The angle (degrees) to start a sweep at.
        end_angle (float): The angle (degrees) for the sweep to finish at.
        scan_distance (float): The distance (meters) the sonar should scan up to.

    Returns:
        float: the normal angle from get_normal_angle, or NaN if the sweep request couldn't be
            made at all (e.g. bypass mode or the sonar service is unavailable).
    """
    logger.info(f'Sonar scan from {start_angle} to {end_angle} degrees, distance: {scan_distance} m')
    future = Sonar().sweep(
        start_angle=start_angle,
        end_angle=end_angle,
        scan_distance=scan_distance,
    )
    if future is None:
        logger.error('Could not call sonar request service (bypass mode or service unavailable).')
        return np.nan
    return get_normal_angle(await future)


@task
async def rotate_to_normal(self: Task,
                           start_angle: float,
                           end_angle: float,
                           scan_distance: float,
                           yaw_threshold: float,
                           tries_until_detection: int = 2) -> Task[None, None, None]:
    """Rotates to face a normal angle.""" 
    attempts_made = 1
    found = False

    while attempts_made <= tries_until_detection:
        logger.info(f'Attempt {attempts_made} of {tries_until_detection} to scan for normal angle')
        normal_angle = await scan_normal_angle(start_angle, end_angle, scan_distance)
        logger.info(f'Initial Normal Angle:  {normal_angle}')
        if np.isnan(normal_angle):
            logger.error(f'Normal angle does not exist.')
            attempts_made += 1
            continue

        found = True
        await move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, CORRECTION_GAIN * normal_angle),
            keep_orientation=True,
            timeout=10,
            pose_tolerances=create_twist_tolerance(angular_yaw=ROTATION_YAW_TOLERANCE),
            parent=self,
        )

        break

    if not found:
        logger.error(f'Normal angle does not exist after {tries_until_detection} attempts.')
        return

    logger.info('Starting confirmation scan')
    normal_angle = await scan_normal_angle(start_angle, end_angle, scan_distance)
    steps = 0
    while abs(normal_angle) > yaw_threshold and steps < MAX_STEPS:
        logger.info(f'Normal angle {normal_angle} at step {steps} is above threshold, rotating')
        await move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, CORRECTION_GAIN * normal_angle),
            keep_orientation=True,
            timeout=10,
            pose_tolerances=create_twist_tolerance(angular_yaw=ROTATION_YAW_TOLERANCE),
            parent=self,
        )
        normal_angle = await scan_normal_angle(start_angle, end_angle, scan_distance)
        if np.isnan(normal_angle):
            logger.error('Lost the wall mid-correction, exiting task.')
            return
        steps += 1
        logger.info(f'Normal Angle {normal_angle} at step {steps}')

@task
async def rotate_to_angle_from_normal(self: Task,
                                      start_angle: float,
                                      end_angle: float,
                                      scan_distance: float,
                                      yaw_threshold: float,
                                      rotated_angle: float) -> Task[None, None, None]:
    """Rotates to a specified angle using Sonar normal angle."""
    angle = await scan_normal_angle(start_angle, end_angle, scan_distance)
    if np.isnan(angle):
        logger.error('Normal angle does not exist, exiting task.')
        return

    angle = rotated_angle + angle
    logger.info(f'Initial Angle: {angle}')

    await move_to_pose_local(
        geometry_utils.create_pose(0, 0, 0, 0, 0, CORRECTION_GAIN * angle),
        keep_orientation=True,
        timeout=10,
        pose_tolerances=create_twist_tolerance(angular_yaw=ROTATION_YAW_TOLERANCE),
        parent=self,
    )

    angle = await scan_normal_angle(start_angle, end_angle, scan_distance)
    if np.isnan(angle):
        logger.error('Normal angle does not exist, exiting task.')
        return
    angle = rotated_angle + angle
    steps = 0
    while abs(angle) > yaw_threshold and steps < MAX_STEPS:
        await move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, CORRECTION_GAIN * angle),
            keep_orientation=True,
            timeout=10,
            pose_tolerances=create_twist_tolerance(angular_yaw=ROTATION_YAW_TOLERANCE),
            parent=self,
        )
        angle = await scan_normal_angle(start_angle, end_angle, scan_distance)
        if np.isnan(angle):
            logger.error('Lost the wall mid-correction, exiting task.')
            return
        angle = rotated_angle + angle
        steps += 1
        logger.info(f'Angle {angle} at step {steps}')

def get_normal_angle(response: SonarSweepRequest.Response) -> float:
    """Get a normal angle from the sonar scan."""
    if not response.is_object:
        logger.error('No object detected — cannot rotate')
        return np.nan
    if np.isnan(response.normal_angle):
        logger.error('[Sonar] normal_angle was NaN — cannot rotate')
        return np.nan

    if response.normal_angle < -np.pi/2.:
        return response.normal_angle + np.pi
    if response.normal_angle > np.pi/2.:
        return response.normal_angle - np.pi

    return response.normal_angle
