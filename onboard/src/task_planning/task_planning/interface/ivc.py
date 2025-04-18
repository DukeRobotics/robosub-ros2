from dataclasses import dataclass
from enum import Enum

from custom_msgs.msg import ModemStatus, StringWithHeader
from custom_msgs.srv import SendModemMessage
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from task_planning.utils.other_utils import singleton

logger = get_logger('ivc_interface')

class IVCMessageType(Enum):
    """Enum for the types of IVC messages."""

    UNKNOWN = ''
    OOGWAY_TEST = 'to'
    CRUSH_TEST = 'tc'


@dataclass
class IVCMessage:
    """
    Dataclass to store IVC message properties.

    Attributes:
        timestamp (Time): The timestamp of the message.
        msg (IVCMessageType): The type of the IVC message.
    """
    timestamp: Time
    msg: IVCMessageType


class IVC:
    """Interface for inter-vehicle communication (IVC)."""

    _instance = None

    # ROS topics for the state and resetting the pose
    STATUS_TOPIC = '/sensors/modem/status'
    MESSAGES_TOPIC = '/sensors/modem/messages'
    SEND_MESSAGE_SERVICE = '/sensors/modem/send_message'

    def __new__(cls, node: Node | None = None, bypass: bool = False) -> 'IVC':
        if cls._instance is None:
            if node is None:
                raise ValueError("IVC must be initialized with a Node the first time.")
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self, node: Node | None = None, bypass: bool = False) -> None:
        """
        Initialize the state.

        Args:
            node (Node): The ROS node used for communication and state management.
            bypass (bool): If True, bypass certain checks. Defaults to False.
        """
        if self._initialized:
            return

        self._initialized = True

        self.bypass = bypass

        self._modem_status = ModemStatus()
        self._received_modem_status = False

        self._messages: list[IVCMessage] = []

        node.create_subscription(ModemStatus, self.STATUS_TOPIC, self._on_receive_modem_status, 10)
        node.create_subscription(StringWithHeader, self.MESSAGES_TOPIC, self._on_receive_modem_message, 10)

        self._send_message = node.create_client(SendModemMessage, self.SEND_MESSAGE_SERVICE)
        if not bypass:
            while not self._send_message.wait_for_service(timeout_sec=1.0):
                logger.info(f'{self.SEND_MESSAGE_SERVICE} not ready, waiting...')

    @property
    def modem_status(self) -> ModemStatus:
        """The current modem status. If a status message has not been received, return an empty status."""
        return self._modem_status

    @property
    def received_modem_status(self) -> bool:
        """True if a modem status message has been received, False otherwise."""
        return self._received_modem_status

    @property
    def messages(self) -> list[IVCMessage]:
        """The list of messages received from the other robot."""
        return self._messages

    def _on_receive_modem_status(self, msg: ModemStatus) -> None:
        """
        Store the received modem status message.

        Args:
            msg (ModemStatus): The received modem status message.
        """
        self._modem_status = msg
        self._received_modem_status = True

    def _convert_stringwithheader_to_ivcmessage(self, msg: StringWithHeader) -> IVCMessage:
        """
        Convert a StringWithHeader message to an IVCMessage.

        If the StringWithHeader message contains data that is not a valid IVCMessageType, it will be set to UNKNOWN.

        Args:
            msg (StringWithHeader): The StringWithHeader message to convert.

        Returns:
            IVCMessage: The message converted to an IVCMessage.
        """
        timestamp = Time.from_msg(msg.header.stamp)
        try:
            message_type = IVCMessageType(msg.data)
        except ValueError:
            message_type = IVCMessageType.UNKNOWN
        return IVCMessage(timestamp=timestamp, msg=message_type)

    def _on_receive_modem_message(self, msg: StringWithHeader) -> None:
        """
        Convert the message received from the other robot to an IVCMessage and append it to the list of messages.

        Args:
            msg (StringWithHeader): The message received from the other robot.
        """
        self._messages.append(self._convert_stringwithheader_to_ivcmessage(msg))

    def send_message(self, msg: IVCMessageType) -> Future | None:
        """
        Send a message to the other robot.

        Args:
            msg (IVCMessageType): The message to send.

        Returns:
            Future | None: The future of the service call if not bypassed, None otherwise.
        """
        request = SendModemMessage.Request()
        request.message = msg.value
        if not self.bypass:
            return self._send_message.call_async(request)

        return None
