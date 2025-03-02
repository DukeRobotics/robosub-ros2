# ruff: noqa: N806

import math
from collections.abc import Coroutine

from rclpy.logging import get_logger

from task_planning import move_tasks
from task_planning.interface.cv import CV
from task_planning.interface.state import State
from task_planning.task import Task, task
from task_planning.utils import geometry_utils

logger = get_logger('prequal_tasks')

# The width of the lane marker in meters (1 foot)
LANE_MARKER_HEIGHT_METERS = 0.3048


@task
async def prequal_task(self: Task) -> Task[None, None, None]:  # noqa: PLR0915
    """Complete the prequalification task by tracking the lane marker."""
    DEPTH_LEVEL = -0.5

    await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, DEPTH_LEVEL, 0, 0, 0),
            parent=self)
    logger.info(f'Moved to (0, 0, {DEPTH_LEVEL})')

    DEPTH_LEVEL = State().depth

    async def rotate_deg(angle: float, log_msg_prefix: str = 'Rotate') -> Coroutine[None, None, None]:
        """
        Rotate the robot by the specified angle in degrees.

        Args:
            angle (float): The angle to rotate the robot in degrees. Positive angles rotate the robot to the left.
            log_msg_prefix (str): The prefix for the message logged after rotating. Defaults to 'Rotate'.
        """
        rad_angle = math.radians(angle)
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, rad_angle),
            parent=self)
        logger.info(f'{log_msg_prefix} {angle}')

    async def track_lane_marker(distance: float, forward: bool, step_size: float = 1.0) -> \
        Coroutine[None, None, None]:
        """
        Move the robot along the X axis the specified distance centering the lane marker in the bottom mono cam's frame.

        Args:
            distance (float): The total distance to move the robot forwards or backwards in meters. Must be positive.
            forward (bool): Whether to move the robot forward (True) or backward (False).
            step_size (float): The distance to move the robot in meters each step.
        """
        direction_term = 'forward' if forward else 'backward'
        logger.info(f'track_lane_marker {distance} {direction_term}')

        direction_sign = 1 if forward else -1

        # The number of times to move the robot along the X axis
        repeats = math.ceil(distance / step_size)

        # The total distance moved so far
        total_dist = 0

        # Whether the lane marker is touching the top or bottom of the frame in the previous step
        prev_touching_top = False
        prev_touching_bottom = False

        for i in range(repeats):
            step = distance - total_dist if i == repeats-1 else step_size
            await move_tasks.move_to_pose_local(
                geometry_utils.create_pose(step * direction_sign, 0, 0, 0, 0, 0),
                parent=self)

            total_dist += step
            logger.info(f'Moved {direction_term} {total_dist}')

            # If the lane marker is partially cut off in the bottom mono camera's frame,
            # then one or both of these flags will be true

            # If lane marker detection is touching the top of the frame
            touching_top = CV().cv_data['lane_marker_touching_top']

             # If lane marker detection is touching the bottom of the frame
            touching_bottom = CV().cv_data['lane_marker_touching_bottom']

            # If the lane marker is touching both the top and bottom of the frame,
            # then the robot is very close to the pool floor, but it is still centered on the lane marker.
            # If the lane marker is touching neither the top nor the bottom of the frame,
            # then the code below will accurately rotate and move the robot to center it on the lane marker.

            # If the lane marker is touching the top but not the bottom,
            # then the robot is too far to the right, so move left
            if touching_top and not touching_bottom:
                await move_tasks.move_to_pose_local(
                    geometry_utils.create_pose(0, 0.2, 0, 0, 0, 0),
                    parent=self)
                logger.info('Touching top correction: Moved 0.2 to the left')

                # If the lane marker touched the top of the frame in the previous step as well despite the yaw
                # correction, then it means the robot's yaw has drifted significantly to the right, so rotate left
                if prev_touching_top and not prev_touching_bottom:
                    await rotate_deg(20)

            # If the lane marker is touching the bottom but not the top,
            # then the robot is too far to the left, so move right
            elif not touching_top and touching_bottom:
                await move_tasks.move_to_pose_local(
                    geometry_utils.create_pose(0, -0.2, 0, 0, 0, 0),
                    parent=self)
                logger.info('Touching bottom correction: Moved 0.2 to the right')

                # If the lane marker touched the bottom of the frame in the previous step as well despite the yaw
                # correction, then it means the robot's yaw has drifted significantly to the left, so rotate right
                if not prev_touching_top and prev_touching_bottom:
                    await rotate_deg(-20)

            # Yaw correction so the robot's X axis is parallel to the lane marker
            # The 'lane_marker_angle' provides the angle between the lane line and the robot's X axis. The coordinate
            # frame of this angle is such that when viewed from the top, if the robot has rotated counterclockwise
            # (left) relative to the lane marker, then the angle is positive. If the robot has rotated clockwise
            # (right) relative to the lane marker, then the angle is negative. To correct the robot's yaw, the robot
            # must rotate in the opposite direction of the angle, hence the angle is negated.
            angle = (CV().cv_data['lane_marker_angle'] * -1)
            if abs(angle) > 0:
                await rotate_deg(angle, 'Yaw correction')

            # Y correction so the robot is centered on the lane marker
            dist_pixels = CV().cv_data['lane_marker_dist']
            height_pixels = CV().cv_data['lane_marker_height']
            dist_meters = dist_pixels * LANE_MARKER_HEIGHT_METERS / height_pixels
            if abs(dist_meters) > 0:
                await move_tasks.move_to_pose_local(
                    geometry_utils.create_pose(0, dist_meters, 0, 0, 0, 0),
                    parent=self)
                logger.info(f'Y correction {dist_meters}')

            # Depth correction so the robot is at the correct depth
            depth_delta = DEPTH_LEVEL - State().depth
            if abs(depth_delta) > 0:
                await move_tasks.move_to_pose_local(
                    geometry_utils.create_pose(0, 0, depth_delta, 0, 0, 0),
                    parent=self)
                logger.info(f'Depth correction {depth_delta}')

            prev_touching_top = touching_top
            prev_touching_bottom = touching_bottom

    # Move up to gate
    await track_lane_marker(2.5, True)

    # Submerge below gate
    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, -0.5, 0, 0, 0),
        parent=self)
    logger.info('Moved to (0, 0, -0.5)')
    DEPTH_LEVEL = State().depth

    # Move through gate
    await track_lane_marker(2, True)

    # Come back up
    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, 0.2, 0, 0, 0),
        parent=self)
    logger.info('Moved to (0, 0, 0.2)')
    DEPTH_LEVEL = State().depth

    # Move to buoy
    await track_lane_marker(7, True)

    # Dead reckon around buoy
    directions = [
        (1, 0, 0),
        (1, 0, 0),
        (1, 0, 0),
        (1, 0, 0),
        (0, 1, 0),
        (0, 1, 0),
        (-1, 0, 0),
        (-1, 0, 0),
        (-1, 0, 0),
        (0, -1, 0),
        (0, -1, 0),
        (0, -1, 0),
        (0, -0.5, 0),
    ]
    for direction in directions:
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(direction[0], direction[1], direction[2], 0, 0, 0),
            parent=self)
        logger.info(f'Moved to {direction}')

    # Come back to gate
    await track_lane_marker(7, False)

    # Robot is already deep enough to go through gate

    # Move back through gate
    await track_lane_marker(2, False)

    # Come back up
    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, 0.2, 0, 0, 0),
        parent=self)
    logger.info('Moved to (0, 0, 0.2)')
    DEPTH_LEVEL = State().depth

    # Return to start
    await track_lane_marker(3, False)
