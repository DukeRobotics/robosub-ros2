from custom_msgs.msg import ModemStatus, StringWithHeader
from custom_msgs.srv import SendModemMessage
from rclpy.logging import get_logger
from rclpy.node import Node
from task_planning.utils.other_utils import singleton

logger = get_logger('ivc_interface')

@singleton
class IVC:
    """Interface for inter-vehicle communication (IVC)."""

    # ROS topics for the state and resetting the pose
    STATUS_TOPIC = '/sensors/modem/status'
    MESSAGES_TOPIC = '/sensors/modem/messages'
    SEND_MESSAGE_SERVICE = '/sensors/modem/send_message'

    def __init__(self, node: Node, bypass: bool = False) -> None:
        """
        Initialize the state.

        Args:
            node: The ROS node used for communication and state management.
            bypass: If True, bypass certain checks. Defaults to False.
        """
        self.bypass = bypass

        self._modem_status = ModemStatus()
        self._received_modem_status = False

        self._messages = []

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
    def messages(self) -> list[StringWithHeader]:
        """The list of messages received from the other robot."""
        return self._messages

    def _on_receive_modem_status(self, msg: ModemStatus) -> None:
        """
        Store the received modem status message.

        Args:
            msg: The received modem status message.
        """
        self._modem_status = msg
        self._received_modem_status = True

    def _on_receive_modem_message(self, msg: StringWithHeader) -> None:
        """
        Store the message received from the other robot.

        Args:
            msg: The message received from the other robot.
        """
        self._messages.append(msg)

    def send_message(self, msg: str) -> None:
        """
        Send a message to the other robot.

        Args:
            msg: The message to send.
        """
        request = SendModemMessage.Request()
        request.message = msg
        if not self.bypass:
            self._send_message.call_async(request)
