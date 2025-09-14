from dataclasses import dataclass
from enum import Enum
from typing import ClassVar

from custom_msgs.srv import SetContinuousServo, SetDiscreteServo
from rclpy.client import Client
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.task import Future
from task_planning.utils.other_utils import get_robot_name, singleton

logger = get_logger('servos_interface')

class MarkerDropperStates(Enum):
    """Enum for the states of the marker dropper servo."""
    LEFT = 'left'
    RIGHT = 'right'

class TorpedoStates(Enum):
    """Enum for the states of the torpedo servo."""
    LEFT = 'left'
    RIGHT = 'right'

@dataclass
class ServoInfo:
    """
    Dataclass for servo information.

    Attributes:
        service_topic (str): The topic for the service.
        service_type (Any): The type of the service.
        robot_names (list[str]): The names of the robots that use this servo.
        service_client (Client | None): The client for the service, if available.
    """
    service_topic: str
    service_type: type[SetDiscreteServo] | type[SetContinuousServo]
    robot_names: list[str]
    service_client: Client | None = None

@singleton
class Servos:
    """
    A singleton class to control servos.

    Attributes:
        SERVOS (ClassVar[dict[str, ServoInfo]]): A dictionary containing servo information.
        node (Node): The ROS 2 node instance used to create the service client.
        service_callers (dict): A dictionary mapping service types to their corresponding service call methods.
    """

    SERVOS: ClassVar[dict[str, ServoInfo]] = {
        'marker_dropper': ServoInfo(
            service_topic='/servos/marker_dropper',
            service_type=SetDiscreteServo,
            robot_names=['oogway', 'oogway_shell'],
        ),
        'torpedo': ServoInfo(
            service_topic='/servos/torpedo',
            service_type=SetDiscreteServo,
            robot_names=['oogway', 'oogway_shell'],
        ),
    }

    def __init__(self, node: Node, bypass: bool = False) -> None:
        self.node = node

        self.service_callers = {
            SetDiscreteServo: self._set_discrete_servo,
            SetContinuousServo: self._set_continuous_servo,
        }

        robot_name = get_robot_name()

        for servo_name, servo_info in self.SERVOS.items():
            if robot_name.value in servo_info.robot_names:
                self.SERVOS[servo_name].service_client = node.create_client(servo_info.service_type,
                                                                            servo_info.service_topic)
                if not bypass:
                    while not self.SERVOS[servo_name].service_client.wait_for_service(timeout_sec=1.0):
                        logger.info(f'{servo_info.service_topic} not ready, waiting...')

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

    def _set_continuous_servo(self, service_client: Client, pwm: int) -> Future:
        """
        Rotate a continuous servo to the specified PWM.

        Args:
            service_client (Client): The client for the service to control the servo.
            pwm (int): The PWM value to set the continuous servo to.

        Returns:
            Future: The result of the asynchronous service call.
        """
        request = SetContinuousServo.Request()
        request.pwm = pwm

        return service_client.call_async(request)

    def drop_marker(self, data: MarkerDropperStates) -> Future | None:
        """
        Rotate the marker dropper servo to the specified state.

        Args:
            data (MarkerDropperStates): The state to set the marker dropper servo to.

        Returns:
            Future | None: The result of the asynchronous service call, or None if the service client is not available.
        """
        servo_info = self.SERVOS['marker_dropper']
        if not servo_info.service_client:
            logger.error('Marker dropper service client is not available.')
            return None
        return self.service_callers[servo_info.service_type](servo_info.service_client, data.value)

    def fire_torpedo(self, data: TorpedoStates) -> Future | None:
        """
        Rotate the torpedo servo to the specified state.

        Args:
            data (TorpedoStates): The state to set the torpedo servo to.

        Returns:
            Future | None: The result of the asynchronous service call, or None if the service client is not available.
        """
        servo_info = self.SERVOS['torpedo']
        if not servo_info.service_client:
            logger.error('Torpedo service client is not available.')
            return None
        return self.service_callers[servo_info.service_type](servo_info.service_client, data.value)
