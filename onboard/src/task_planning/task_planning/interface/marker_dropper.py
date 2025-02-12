# TODO: Update marker_dropper interface (see ROS 2 offboard_comms package)

from custom_msgs.srv import SetServo
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.task import Future
from task_planning.utils.other_utils import singleton

logger = get_logger('marker_dropper_interface')

@singleton
class MarkerDropper:
    """
    A singleton class to control the marker dropper mechanism using a ROS 2 service.

    This class provides an interface to interact with the `marker_dropper/servo_control` service,
    which controls the servo mechanism for dropping markers.

    Attributes:
        SERVO_CONTROL_SERVICE (str): The name of the ROS 2 service for servo control.
        node (Node): The ROS 2 node instance used to create the service client.
        drop_marker_client (Client): A client for the `SetBool` service to control the servo.
    """

    SERVO_CONTROL_SERVICE = '/servo_control'

    def __init__(self, node: Node, bypass: bool = False) -> None:
        self.node = node

        self.drop_marker_client = node.create_client(SetServo, self.SERVO_CONTROL_SERVICE)

        if not bypass:
            while not self.drop_marker_client.wait_for_service(timeout_sec=1.0):
                logger.info(f'{self.SERVO_CONTROL_SERVICE} not ready, waiting...')

    def drop_marker(self, data: bool) -> Future:
        """
        Send a request to activate or deactivate the marker dropper servo.

        Args:
            data (bool): True to drop the left marker, False to drop the right marker.

        Returns:
            Future: The result of the asynchronous service call.
        """
        request = SetServo.Request()
        request.tag = 'M'
        request.pwm = 1250 if data else 1750

        return self.drop_marker_client.call_async(request)
