from custom_msgs.srv import SonarSweepRequest
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.task import Future
from task_planning.utils.other_utils import singleton

logger = get_logger('sonar_interface')

@singleton
class Sonar:
    """Interface for sonar."""

    SONAR_SWEEP_REQUEST_SERVICE = '/sonar/request'

    def __init__(self, node: Node, bypass: bool = False) -> None:
        self.node = node
        self.bypass = bypass

        if not self.bypass:
            self._sonar_request = node.create_client(SonarSweepRequest, self.SONAR_SWEEP_REQUEST_SERVICE)
            while not self._sonar_request.wait_for_service(timeout_sec=1.0):
                logger.info(f'{self.SONAR_SWEEP_REQUEST_SERVICE} not ready, waiting...')

    def sweep(self, start_angle: float, end_angle: float, scan_distance: float) -> Future | None:
        """
        Perform an angular sweep using the sonar.

        Args:
            start_angle (float): The angle to start a sweep at.
            end_angle (float): The angle for the sweep to finish at.
            scan_distance (float): The distance the sonar should scan up to.

        Returns:
            Future: The result of the aynschronous service call.
        """
        request = SonarSweepRequest.Request()
        request.start_angle = float(start_angle)
        request.end_angle = float(end_angle)
        request.distance_of_scan = float(scan_distance)

        if not self.bypass:
            return self._sonar_request.call_async(request)

        return None
