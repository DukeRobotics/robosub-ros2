from typing import cast

from rclpy.logging import get_logger
from task_planning.interface.sonar import Sonar
from task_planning.task import Task, task
from custom_msgs.srv import SonarSweepRequest

logger = get_logger('sonar_tasks')

@task
async def test_sonar(_self: Task, start_angle: float, end_angle: float, scan_distance: float,
                     target_frame_id: str = 'base_link') -> Task[None, None, None]:
    """Repeatedly perform sonar scans."""
    while True:
        logger.info(f'Sonar scan from {start_angle} to {end_angle} degrees, distance: {scan_distance} m')
        future = Sonar().sweep(
            start_angle=start_angle,
            end_angle=end_angle,
            scan_distance=scan_distance,
            target_frame_id=target_frame_id,
        )
        if future is None:
            logger.error('Could not call sonar request service.')
        else:
            service_response = cast(SonarSweepRequest.Response, await future)
            logger.info(f'Sonar scan response: {service_response}')
