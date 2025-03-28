from enum import Enum

from custom_msgs.srv import SetDiscreteServo
from rclpy.client import Client
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.task import Future
from task_planning.utils.other_utils import singleton

logger = get_logger('marker_dropper_interface')

class MarkerDropperStates(Enum):
    """Enum for the states of the marker dropper servo."""
    LEFT = 'left'
    RIGHT = 'right'

class TorpedoStates(Enum):
    """Enum for the states of the torpedo servo."""
    LEFT = 'left'
    RIGHT = 'right'

@singleton
class Servos:
    """
    A singleton class to control servos.

    Attributes:
        MARKER_DROPPER_SERVICE (str): The name of the service for controlling the marker dropper servo.
        TORPEDO_SERVICE (str): The name of the service for controlling the torpedo servo.
        node (Node): The ROS 2 node instance used to create the service client.
        drop_marker_client (Client): A client for the service to control the marker dropper servo.
        torpedo_client (Client): A client for the service to control the torpedo servo.
    """

    MARKER_DROPPER_SERVICE = '/servos/marker_dropper'
    TORPEDO_SERVICE = '/servos/torpedo'

    def __init__(self, node: Node, bypass: bool = False) -> None:
        self.node = node

        self.drop_marker_client = node.create_client(SetDiscreteServo, self.MARKER_DROPPER_SERVICE)
        if not bypass:
            while not self.drop_marker_client.wait_for_service(timeout_sec=1.0):
                logger.info(f'{self.MARKER_DROPPER_SERVICE} not ready, waiting...')

        self.torpedo_client = node.create_client(SetDiscreteServo, self.TORPEDO_SERVICE)
        if not bypass:
            while not self.torpedo_client.wait_for_service(timeout_sec=1.0):
                logger.info(f'{self.TORPEDO_SERVICE} not ready, waiting...')

    def _set_discrete_servo(self, service_client: Client, state: str) -> Future:
        """
        Rotate a discrete servo to the specified state.

        Args:
            service_client (Client): The client for the service to control the servo.
            state (str): The state to set the discrete servo to.

        Returns:
            Future: The result of the asynchronous service call.
        """
        request = SetDiscreteServo.Request()
        request.state = state

        return service_client.call_async(request)

    def drop_marker(self, data: MarkerDropperStates) -> Future:
        """
        Rotate the marker dropper servo to the specified state.

        Args:
            data (MarkerDropperStates): The state to set the marker dropper servo to.

        Returns:
            Future: The result of the asynchronous service call.
        """
        return self._set_discrete_servo(self.drop_marker_client, data.value)

    def fire_torpedo(self, data: TorpedoStates) -> Future:
        """
        Rotate the torpedo servo to the specified state.

        Args:
            data (TorpedoStates): The state to set the torpedo servo to.

        Returns:
            Future: The result of the asynchronous service call.
        """
        return self._set_discrete_servo(self.torpedo_client, data.value)
