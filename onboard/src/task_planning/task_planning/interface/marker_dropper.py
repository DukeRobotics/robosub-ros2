from rclpy.logging import get_logger
from rclpy.node import Node
from example_interfaces.srv import SetBool
from task_planning.utils.other_utils import singleton

logger = get_logger('marker_dropper')

@singleton
class MarkerDropper:
    SERVO_CONTROL_SERVICE = 'marker_dropper/servo_control'

    def __init__(self, node: Node, bypass: bool = False):
        self.node = node

        self.drop_marker_client = node.create_client(SetBool, self.SERVO_CONTROL_SERVICE)

        if not bypass:
            while not self.drop_marker_client.wait_for_service(timeout_sec=1.0):
                logger.info(f'{self.SERVO_CONTROL_SERVICE} not ready, waiting...')

    def drop_marker(self, data: bool):
        request = SetBool.Request()
        request.data = data

        future = self.drop_marker_client.call_async(request)
        return future
