from dataclasses import dataclass
from datetime import datetime
from enum import Enum

import pytz
from custom_msgs.msg import ModemStatus, StringWithHeader
from custom_msgs.srv import SendModemMessage
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.task import Future
from rclpy.time import Time
from task_planning.utils.other_utils import singleton

logger = get_logger('ivc_interface')

class IVCMessageType(Enum):
    """
    Enum for the types of IVC messages.

    Each message must be a ASCII-encodable string with exactly 2 characters.

    Attributes:
        UNKNOWN (str): Unknown message type. Any message that is not one of the following will be set to this type.
        OOGWAY_TEST (str): Test message for Oogway.
        OOGWAY_GATE (str): Confirm pass through gate by Oogway.
        OOGWAY_ACKNOWLEDGE (str): Acknowledgement message by Oogway
        CRUSH_TEST (str): Test message for Crush.
        CRUSH_GATE (str): Confirm pass through gate by Crush
        CRUSH_ACKNOWLEDGE (str):  Acknowledgement message by Crush
    """
    UNKNOWN = ''
    OOGWAY_TEST = 'to'
    OOGWAY_GATE = 'og'
    OOGWAY_ACKNOWLEDGE = 'oa'
    OOGWAY_TORPEDOES = 'fu'
    CRUSH_TEST = 'tc'
    CRUSH_GATE = 'cg'
    CRUSH_ACKNOWLEDGE = 'ca'
    CRUSH_OCTAGON = 'ky'



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

def ros_timestamp_to_pacific_time(sec: int, nanosec: int) -> str:
    """
    Convert ROS timestamp (seconds and nanoseconds) to human-readable Pacific time.

    # TODO: move to utils + merge with same function in ivc_tasks.py

    Args:
        sec (int): Seconds since epoch
        nanosec (int): Nanoseconds

    Returns:
        str: Human-readable timestamp in Pacific timezone
    """
    # Convert to datetime object
    timestamp = datetime.fromtimestamp(sec + nanosec / 1e9)

    # Convert to Pacific timezone
    pacific_tz = pytz.timezone('US/Pacific')
    pacific_time = timestamp.astimezone(pacific_tz)

    # Format as human-readable string
    return pacific_time.strftime('%Y-%m-%d %H:%M:%S %Z')

@singleton
class IVC:
    """Interface for inter-vehicle communication (IVC)."""

    _instance = None

    STATUS_TOPIC = '/sensors/modem/status'
    MESSAGES_TOPIC = '/sensors/modem/messages'
    SEND_MESSAGE_SERVICE = '/sensors/modem/send_message'

    def __new__(cls, node: Node | None = None, bypass: bool = False) -> 'IVC':  # noqa: ARG004
        """Create a new instance of the IVC class or return the existing instance."""
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False  # noqa: SLF001
        return cls._instance

    def __init__(self, node: Node | None = None, bypass: bool = False) -> None:
        """
        Initialize the IVC interface.

        Args:
            node (Node): The task planning ROS node.
            bypass (bool): If True, bypass certain checks. Defaults to False.
        """
        if self._initialized:
            return

        self._initialized = True

        if node is None:
            error_msg = 'IVC interface must be initialized with a Node the first time.'
            raise ValueError(error_msg)

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
        ivc_message = self._convert_stringwithheader_to_ivcmessage(msg)
        self._messages.append(ivc_message)

        seconds, nanoseconds = ivc_message.timestamp.seconds_nanoseconds()
        timestamp = ros_timestamp_to_pacific_time(
            seconds,
            nanoseconds,
        )
        msg = f'Received IVC message: {ivc_message.msg.name} at {timestamp}\n'

        # Log to text file
        logger.info(msg)
        with open('ivc_log.txt', 'a') as f:
            f.write(msg)

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