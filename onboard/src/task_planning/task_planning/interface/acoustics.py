from custom_msgs.srv import AcousticsRequest
from rclpy.logging import get_logger
from rclpy.node import Node
from task_planning.utils.other_utils import singleton

logger = get_logger('acoustics_interface')

@singleton
class Acoustics:
    ACOUSTICS_REQUEST_SERVICE = '/aoustics/request'

    def __init__(self, node: Node, bypass: bool = False) -> None:
        self.node = node
        self.bypass = bypass

        if not bypass:
            self._acoustics_request = node.create_client(AcousticsRequest, self.ACOUSTICS_REQUEST_SERVICE)
            while not self._acoustics_request.wait_for_service(timeout_sec=1.0):
                logger.info(f'{self.ACOUSTICS_SERVICE_REUQEST} not ready, waiting...')

    def request(self):
        request = AcousticsRequest.Request()

        if not self.bypass:
            return self._acoustics_request.call_async(request)

        return None
