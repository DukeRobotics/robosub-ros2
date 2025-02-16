from enum import Enum

from custom_msgs.srv import SetDiscreteServo
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.task import Future
from task_planning.utils.other_utils import singleton

logger = get_logger('marker_dropper_interface')

class MarkerDropperStates(Enum):
    """Enum for the states of the marker dropper servo."""
    LEFT = 'left'
    RIGHT = 'right'

@singleton
class MarkerDropper:
    """
    A singleton class to control the marker dropper mechanism using a ROS 2 service.

    This class provides an interface to interact with the `/servos/marker_dropper` service, which controls the servo
    mechanism for dropping markers.

    Attributes:
        MARKER_DROPPER_SERVICE (str): The name of the ROS 2 service for controlling the marker dropper servo.
        node (Node): The ROS 2 node instance used to create the service client.
        drop_marker_client (Client): A client for the service to control the servo.
    """

    MARKER_DROPPER_SERVICE = '/servos/marker_dropper'

    def __init__(self, node: Node, bypass: bool = False) -> None:
        self.node = node

        self.drop_marker_client = node.create_client(SetDiscreteServo, self.MARKER_DROPPER_SERVICE)

        if not bypass:
            while not self.drop_marker_client.wait_for_service(timeout_sec=1.0):
                logger.info(f'{self.MARKER_DROPPER_SERVICE} not ready, waiting...')

    def drop_marker(self, data: MarkerDropperStates) -> Future:
        """
        Rotate the marker dropper servo to the specified state.

        Args:
            data (MarkerDropperStates): The state to set the marker dropper servo to.

        Returns:
            Future: The result of the asynchronous service call.
        """
        request = SetDiscreteServo.Request()
        request.state = data.value

        return self.drop_marker_client.call_async(request)
