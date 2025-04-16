import os
import struct
from typing import ClassVar

import rclpy
from custom_msgs.msg import ModemDiagnosticReport, ModemStatus
from custom_msgs.srv import SendModemCommand, SendModemMessage
from std_msgs.msg import String

from offboard_comms.serial_node import SerialNode, SerialReadType


class ModemPublisher(SerialNode):
    """Serial publisher to publish and send data from WaterLinked M16 modems."""

    SERIAL_DEVICE_NAME = 'IVC Modem'
    CONFIG_FILE_PATH = f'package://offboard_comms/config/{os.getenv("ROBOT_NAME")}.yaml'

    BAUDRATE = 9600
    NODE_NAME = 'modem'
    CONNECTION_RETRY_PERIOD = 1.0  # seconds
    LOOP_RATE = 20  # Hz

    COMMAND_NAMES: ClassVar[dict[int, str]] = {
        SendModemCommand.Request.TRANSPARENT_MODE: 'TRANSPARENT_MODE',
        SendModemCommand.Request.DIAGNOSTIC_MODE: 'DIAGNOSTIC_MODE',
        SendModemCommand.Request.CHANGE_CHANNEL: 'CHANGE_CHANNEL',
        SendModemCommand.Request.REQUEST_REPORT: 'REQUEST_REPORT',
        SendModemCommand.Request.SET_POWER_LEVEL: 'SET_POWER_LEVEL',
    }
    MIN_COMMAND = min(COMMAND_NAMES.keys())
    MAX_COMMAND = max(COMMAND_NAMES.keys())

    MIN_CHANNEL = 1
    MAX_CHANNEL = 12

    MIN_POWER_LEVEL = 1
    MAX_POWER_LEVEL = 4

    MESSAGE_SIZE = 2  # bytes

    DIAGNOSTIC_REPORT_START_BYTE = 0x24 # '$'
    DIAGNOSTIC_REPORT_END_BYTE = 0x0A # '\n'
    DIAGNOSTIC_REPORT_SIZE = 18  # bytes

    FORBIDDEN_CHARS_IN_MESSAGE: ClassVar[list[str]] = [chr(0),
                                                       chr(DIAGNOSTIC_REPORT_START_BYTE),
                                                       chr(DIAGNOSTIC_REPORT_END_BYTE)]

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME, self.BAUDRATE, self.CONFIG_FILE_PATH, self.SERIAL_DEVICE_NAME,
                         SerialReadType.BYTES_ALL, self.CONNECTION_RETRY_PERIOD, loop_rate=self.LOOP_RATE)
        self.buffer = bytearray()

        self.status = ModemStatus(busy=False)
        self.message = String()
        self.report = ModemDiagnosticReport()

        self.received_report = False

        self.modem_status_publisher = self.create_publisher(ModemStatus, '/sensors/modem/status', 1)
        self.message_publisher = self.create_publisher(String, '/sensors/modem/messages', 1)
        self.diagnostic_reports_publisher = self.create_publisher(ModemDiagnosticReport,
                                                                 '/sensors/modem/diagnostic_reports', 1)
        self.send_command_srv = self.create_service(SendModemCommand, '/sensors/modem/send_command', self.send_command)
        self.send_message_srv = self.create_service(SendModemMessage, '/sensors/modem/send_message', self.send_message)

        self.publish_modem_status_timer = self.create_timer(1.0 / 10.0, self.publish_modem_status)

    def get_ftdi_string(self) -> str:
        """
        Get the FTDI string for the WaterLinked Modem-M16.

        Returns:
            str: FTDI string for the WaterLinked Modem-M16.
        """
        return self._config['modem']['ftdi']

    def after_connect(self) -> None:
        """Request a report from the modem to obtain its initial status."""
        self.send_command(SendModemCommand.Request(command=SendModemCommand.Request.REQUEST_REPORT),
                          SendModemCommand.Response())

    def send_data(self, data: str) -> bool:
        """
        Send data to the modem.

        Args:
            data (str): Data to send.
        """
        return self.writebytes(data.encode('ascii'))

    def publish_modem_status(self) -> None:
        """Publish the modem status if at least one report has been received and the serial port is open."""
        if self.received_report and self._serial and self._serial.is_open:
            self.modem_status_publisher.publish(self.status)

    def process_bytes(self, data: bytes) -> None:
        """
        Process bytes read from serial.

        Args:
            data (bytes): Data to process.
        """
        self.buffer.extend(data)
        self.process_buffer()

    def process_buffer(self) -> None:
        """Process the buffer to extract messages and diagnostic reports."""
        index = 0
        while index < len(self.buffer):
            # Check if a diagnostic report begins at the current index
            if self.buffer[index] == self.DIAGNOSTIC_REPORT_START_BYTE:
                if len(self.buffer) - index < self.DIAGNOSTIC_REPORT_SIZE:
                    # Not enough data for a full diagnostic report
                    break

                report_end_index = index + self.DIAGNOSTIC_REPORT_SIZE - 1
                if self.buffer[report_end_index] == self.DIAGNOSTIC_REPORT_END_BYTE:
                    # Extract the diagnostic report
                    packet = self.buffer[index:report_end_index]
                    self.process_diagnostic_report(bytes(packet))
                else:
                    self.get_logger().warn(f'Invalid diagnostic report end byte at index {report_end_index}.')
                    # Invalid end byte, discard the data

                index = report_end_index + 1
            else:
                # Not a diagnostic report, process as regular data
                # Must be two bytes for a message
                if len(self.buffer) - index < self.MESSAGE_SIZE:
                    break

                packet = self.buffer[index:index + self.MESSAGE_SIZE]
                self.publish_message(bytes(packet).decode('ascii'))
                index += 2

        self.buffer = self.buffer[index:]


    def publish_message(self, message: str) -> None:
        """
        Publish the message received from the other modem.

        Args:
            message (str): Message to publish.
        """
        # Publish this message to a ROS topic
        self.message.data = message
        self.message_publisher.publish(self.message)

    def process_diagnostic_report(self, packet: bytes) -> None:
        r"""
        Process the diagnostic report received from the WaterLinked Modem-M16.

        Args:
            packet (bytes): raw packet (18 bytes) starting with '$' (0x24) and ending with '\n' (0x0A).

        Returns:
            dict[str, any]: A dictionary with decoded values.
        """
        # Get the 16 bytes after the '$' and before the '\n'
        data = packet[1:17]
        decoded = struct.unpack('<HBBBHBBBBBHBB', data)

        message_bytes = decoded[0].to_bytes(self.MESSAGE_SIZE, 'little')

        self.report.header.stamp = self.get_clock().now().to_msg()
        self.report.message = message_bytes.decode('ascii')
        self.report.bit_error_rate = decoded[1]
        self.report.signal_power = decoded[2]
        self.report.noise_power = decoded[3]
        self.report.packet_valid = decoded[4]
        self.report.packet_invalid = decoded[5]
        self.report.git_revision = decoded[6].to_bytes(1, 'little')
        self.report.time_since_boot = (decoded[9] << 16) | (decoded[8] << 8) | decoded[7]
        self.report.chip_id = decoded[10]
        self.report.hardware_revision = (decoded[11] & 0b00000011).to_bytes(1, 'little')
        self.report.channel = (decoded[11] & 0b00111100) >> 2
        self.report.transport_block_valid = bool((decoded[11] & 0b01000000) >> 6)
        self.report.transmission_complete = bool((decoded[11] & 0b10000000) >> 7)
        self.report.diagnostic_mode = bool(decoded[12] & 0b00000001)
        self.report.power_level = (decoded[12] & 0b00001100) >> 2
        self.diagnostic_reports_publisher.publish(self.report)

        # If the report includes a non-empty message, publish it
        if message_bytes != b'\x00' * self.MESSAGE_SIZE:
            self.publish_message(self.report.message)

        # Update the modem status
        self.status.mode = ModemStatus.DIAGNOSTIC_MODE if self.report.diagnostic_mode else \
            ModemStatus.TRANSPARENT_MODE
        self.status.channel = self.report.channel
        self.status.power_level = self.report.power_level

        self.received_report = True


    def send_message(self, request: SendModemMessage.Request, response: SendModemMessage.Response) -> \
            SendModemMessage.Response:
        """
        Send a message to the modem.

        Args:
            request (ModemWrite.Request): Request object containing the message to send.
            response (ModemWrite.Response): Response object to be filled with the result.

        Returns:
            ModemWrite.Response: The response object with the result of the operation.
        """
        if len(request.message) != self.MESSAGE_SIZE:
            error_msg = (f'Message "{request.message}" must be {self.MESSAGE_SIZE} bytes long.')
            self.get_logger().error(error_msg)
            response.success = False
            response.message = error_msg
            return response

        if self.status.busy:
            error_msg = 'Cannot send message. Modem is busy.'
            self.get_logger().error(error_msg)
            response.success = False
            response.message = error_msg
            return response

        if any(char in request.message for char in self.FORBIDDEN_CHARS_IN_MESSAGE):
            error_msg = (f'Message "{request.message}" contains forbidden characters: '
                         f'{self.FORBIDDEN_CHARS_IN_MESSAGE}')
            self.get_logger().error(error_msg)
            response.success = False
            response.message = error_msg
            return response

        if not request.message.isascii():
            error_msg = f'Message "{request.message}" contains non-ASCII characters.'
            self.get_logger().error(error_msg)
            response.success = False
            response.message = error_msg
            return response

        self.status.busy = True
        if self.send_data(request.message):
            self.ready_timer = self.create_timer(2.0, self.set_ready)

            response.success = True
            response.message = 'Sent message successfully'
        else:
            self.status.busy = False

            error_msg = f'Failed to send message "{request.message}".'
            self.get_logger().error(error_msg)
            response.success = False
            response.message = error_msg

        return response

    def send_command(self, request: SendModemCommand.Request, response: SendModemCommand.Response) -> \
            SendModemCommand.Response:
        """
        Set a comand to the modem.

        Args:
            request (SendModemCommand.Request): Request object containing the command to send.
            response (SendModemCommand.Response): Response object to be filled with the result.

        Returns:
            SendModemCommand.Response: The response object with the result of the operation.
        """
        if self.status.busy:
            error_msg = 'Cannot send command. Modem is busy.'
            self.get_logger().error(error_msg)
            response.success = False
            response.message = error_msg
            return response

        self.command = request.command
        raw_value = request.value

        self.command_char = ''
        self.command_value = None

        match self.command:
            case SendModemCommand.Request.TRANSPARENT_MODE:
                self.command_char = 't'
            case SendModemCommand.Request.DIAGNOSTIC_MODE:
                self.command_char = 'd'
            case SendModemCommand.Request.CHANGE_CHANNEL:
                self.command_char = 'c'
                if not (self.MIN_CHANNEL <= raw_value <= self.MAX_CHANNEL):
                    error_msg = (f'Invalid channel {raw_value}. Must be in range [{self.MIN_CHANNEL}, '
                                 f'{self.MAX_CHANNEL}]')
                    self.get_logger().error(error_msg)
                    response.success = False
                    response.message = error_msg
                    return response

                self.command_value = format(raw_value, 'x')
            case SendModemCommand.Request.REQUEST_REPORT:
                self.command_char = 'r'

            # NOTE: The power level command is available only on modems with firmware version 0x56 or recent.
            # If the modem has an older firmware version, this command will have no effect.
            case SendModemCommand.Request.SET_POWER_LEVEL:
                self.command_char = 'l'
                if not (self.MIN_POWER_LEVEL <= raw_value <= self.MAX_POWER_LEVEL):
                    error_msg = (f'Invalid power level {raw_value}. Must be in range [{self.MIN_POWER_LEVEL}, '
                                 f'{self.MAX_POWER_LEVEL}]')
                    self.get_logger().error(error_msg)
                    response.success = False
                    response.message = error_msg
                    return response

                self.command_value = format(raw_value, 'x')
            case _:
                error_msg = f'Invalid command {self.command}. Must be in range [{self.MIN_COMMAND}, {self.MAX_COMMAND}]'
                self.get_logger().error(error_msg)
                response.success = False
                response.message = error_msg
                return response

        self.command_name = self.COMMAND_NAMES[self.command]

        self.status.busy = True
        if self.send_data(self.command_char):
            self.finish_sending_command_timer = self.create_timer(1.0, self.finish_sending_command)

            response.success = True
            response.message = f'Sent command {self.command_name} successfully.'
        else:
            self.status.busy = False

            error_msg = f'Failed to send first character for command {self.command_name}.'
            self.get_logger().error(error_msg)

            response.success = False
            response.message = error_msg

        return response

    def finish_sending_command(self) -> None:
        """Send the second command character and value if applicable."""
        self.finish_sending_command_timer.cancel()

        if self.send_data(self.command_char):
            match self.command:
                case SendModemCommand.Request.TRANSPARENT_MODE:
                    self.status.mode = ModemStatus.TRANSPARENT_MODE
                case SendModemCommand.Request.DIAGNOSTIC_MODE:
                    self.status.mode = ModemStatus.DIAGNOSTIC_MODE

            if self.command_value is not None:
                if self.send_data(self.command_value):
                    match self.command:
                        case SendModemCommand.Request.CHANGE_CHANNEL:
                            self.status.channel = int(self.command_value, 16)
                        case SendModemCommand.Request.SET_POWER_LEVEL:
                            self.status.power_level = int(self.command_value, 16)
                else:
                    error_msg = (f'Failed to send value "{self.command_value}" for command '
                                f'{self.COMMAND_NAMES[self.command]}')
                    self.get_logger().error(error_msg)
        else:
            error_msg = f'Failed to send second character for command {self.command_name}.'
            self.get_logger().error(error_msg)
            self.status.busy = False
            return

        self.ready_timer = self.create_timer(1.0, self.set_ready)

    def set_ready(self) -> None:
        """Set the modem to ready (not busy) state."""
        self.ready_timer.cancel()
        self.status.busy = False


def main(args: list[str] | None = None) -> None:
    """Create and run the modem publisher node."""
    rclpy.init(args=args)
    modem_publisher = ModemPublisher()

    try:
        rclpy.spin(modem_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        modem_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

# Black is left
