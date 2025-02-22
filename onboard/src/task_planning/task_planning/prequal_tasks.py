# ruff: noqa
import math
from task_planning import move_tasks

from rclpy.logging import get_logger
from task_planning.interface.cv import CV
from task_planning.interface.state import State
from task_planning.task import Task, task
from task_planning.utils import geometry_utils

logger = get_logger('prequal_tasks')

RECT_HEIGHT_METERS = 0.3048


@task
async def prequal_task(self: Task) -> Task[None, None, None]:
    """
    Complete the prequalification task by moving to a series of local poses. Returns when the robot is at the final pose
    with zero velocity, within a small tolerance, completing the prequalification task.
    """
    DEPTH_LEVEL = -0.5

    await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, DEPTH_LEVEL, 0, 0, 0),
            parent=self)
    logger.info(f'Moved to (0, 0, {DEPTH_LEVEL})')

    DEPTH_LEVEL = State().depth

    async def rotate_deg(angle):
        rad_angle = math.radians(angle)
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, rad_angle),
            parent=self)
        logger.info(f'Rotate {angle}')

    async def track_blue_rectangle(distance, direction):
        logger.info(f'track_blue_rectangle {distance} {direction}')
        repeats = math.ceil(distance)
        total_dist = 0
        prev_touching_top = False
        prev_touching_bottom = False
        for i in range(repeats):
            step = distance - total_dist if i == repeats-1 else 1
            await move_tasks.move_to_pose_local(
                geometry_utils.create_pose(step * direction, 0, 0, 0, 0, 0),
                parent=self)

            total_dist += step
            logger.info(f'Moved forward {total_dist}')

            touching_top = CV().cv_data['lane_marker_touching_top']
            touching_bottom = CV().cv_data['lane_marker_touching_bottom']
            if touching_top and not touching_bottom:
                await move_tasks.move_to_pose_local(
                    geometry_utils.create_pose(0, 0.2, 0, 0, 0, 0),
                    parent=self)
                logger.info('Touching top correction (0, 0.2, 0)')
                if prev_touching_top and not prev_touching_bottom:
                    await rotate_deg(20)
            elif not touching_top and touching_bottom:
                await move_tasks.move_to_pose_local(
                    geometry_utils.create_pose(0, -0.2, 0, 0, 0, 0),
                    parent=self)
                logger.info('Touching bottom correction (0, -0.2, 0)')
                if not prev_touching_top and prev_touching_bottom:
                    await rotate_deg(-20)

            angle = (CV().cv_data['lane_marker_angle'] * -1)
            if abs(angle) > 0:
                rad_angle = math.radians(angle)
                await move_tasks.move_to_pose_local(
                    geometry_utils.create_pose(0, 0, 0, 0, 0, rad_angle),
                    parent=self)
                logger.info(f'Yaw correction {angle}')

            dist_pixels = CV().cv_data['lane_marker_dist']
            height_pixels = CV().cv_data['lane_marker_height']
            dist_meters = dist_pixels * RECT_HEIGHT_METERS / height_pixels
            if abs(dist_meters) > 0:
                await move_tasks.move_to_pose_local(
                    geometry_utils.create_pose(0, dist_meters, 0, 0, 0, 0),
                    parent=self)
                logger.info(f'Correction {dist_meters}')

            depth_delta = DEPTH_LEVEL - State().depth
            if abs(depth_delta) > 0:
                await move_tasks.move_to_pose_local(
                    geometry_utils.create_pose(0, 0, depth_delta, 0, 0, 0),
                    parent=self)
                logger.info(f'Depth correction {depth_delta}')

            prev_touching_top = touching_top
            prev_touching_bottom = touching_bottom

    # Move up to gate
    await track_blue_rectangle(2.5, 1)

    # Submerge below gate
    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, -0.5, 0, 0, 0),
        parent=self)
    logger.info('Moved to (0, 0, -0.5)')
    DEPTH_LEVEL = State().depth

    # Move through gate
    await track_blue_rectangle(2, 1)

    # Come back up
    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, 0.2, 0, 0, 0),
        parent=self)
    logger.info('Moved to (0, 0, 0.2)')
    DEPTH_LEVEL = State().depth

    # Move to buoy
    await track_blue_rectangle(7, 1)

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
        (0, -0.5, 0)
    ]
    for direction in directions:
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(direction[0], direction[1], direction[2], 0, 0, 0),
            parent=self)
        logger.info(f'Moved to {direction}')

    # Come back to gate
    await track_blue_rectangle(7, -1)

    # Submerge below gate
    # await move_tasks.move_to_pose_local(
    #     geometry_utils.create_pose(0, 0, -0.4, 0, 0, 0),
    #     parent=self)
    # logger.info('Moved to (0, 0, -0.4)')
    # DEPTH_LEVEL = State().depth

    # Move back through gate
    await track_blue_rectangle(2, -1)

    # Come back up
    await move_tasks.move_to_pose_local(
        geometry_utils.create_pose(0, 0, 0.2, 0, 0, 0),
        parent=self)
    logger.info('Moved to (0, 0, 0.2)')
    DEPTH_LEVEL = State().depth

    # Return to start
    await track_blue_rectangle(3, -1)
