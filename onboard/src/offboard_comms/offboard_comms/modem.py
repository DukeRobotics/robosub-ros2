import os
import time
import struct
import time
from typing import ClassVar

import rclpy
from std_msgs.msg import ByteMultiArray
from custom_msgs.srv import SetModemSettings, SendModemMessage


from offboard_comms.serial_node import SerialNode, SerialReadType



class ModemPublisher(SerialNode):
    """Serial publisher to publish and send data from WaterLinked M16 modems."""

    SERIAL_DEVICE_NAME = 'wlmodem-m16'
    CONFIG_FILE_PATH = f'package://offboard_comms/config/{os.getenv("ROBOT_NAME")}.yaml'

    BAUDRATE = 9600
    NODE_NAME = 'wlmodem'
    CONNECTION_RETRY_PERIOD = 1.0  # seconds
    LOOP_RATE = 20  # Hz, loop every 1.6 seconds

    DIAGNOSTIC_PACKET_SIZE = 18

    SETTINGS: ClassVar[list[int]] = [
        SetModemSettings.Request.TRANSPARENT_MODE,
        SetModemSettings.Request.DIAGNOSTIC_MODE,
        SetModemSettings.Request.CHANGE_CHANNEL,
        SetModemSettings.Request.REQUEST_REPORT,
        SetModemSettings.Request.SET_POWER_LEVEL
    ]
    MIN_SETTING = min(SETTINGS)
    MAX_SETTING = max(SETTINGS)

    MIN_CHANNEL = 1
    MAX_CHANNEL = 12

    MIN_POWER_LEVEL = 1
    MAX_POWER_LEVEL = 4

    modem_ready: ClassVar[bool] = True
    change_setting_timer: ClassVar[float] = time.time()

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME, self.BAUDRATE, self.CONFIG_FILE_PATH, self.SERIAL_DEVICE_NAME,
                         SerialReadType.BYTES_ALL, self.CONNECTION_RETRY_PERIOD, loop_rate=self.LOOP_RATE)
        self._publisher = self.create_publisher(ByteMultiArray, '/offboard/ivc/messages', 1)
        self.change_setting_srv = self.create_service(SetModemSettings, '/offboard/ivc/settings', self.change_setting)
        self.send_message_srv = self.create_service(SendModemMessage, 'offboard/ivc/send_message', self.send_message)

        # Create a timer that checks if a second byte needs to be sent

        # Define a custom ROS service type for changing the setting
        # Create a ROS service that calls the change_setting function

    def get_ftdi_string(self) -> str:
        """
        Get the FTDI string for the WaterLinked Modem-M16.

        Returns:
            str: FTDI string for the WaterLinked Modem-M16.
        """
        return self._config['modem']['ftdi']

    def send_data(self, data: str) -> bool:
        """
        Send data to the modem.

        Args:
            data (str): Data to send.
        """
        return self.writebytes(data.encode('ascii'))

    def process_bytes(self, data: bytes) -> None:
        """
        Process bytes read from serial.

        Args:
            data (bytes): Data to process.
        """
        print(f"Received {len(data)} bytes: {data}")
        if len(data) != self.DIAGNOSTIC_PACKET_SIZE or data[0] != ord('$') or data[-1] != ord('\n'):
            self.process_data(data)
        else:
            self.process_diagnostic_report(data)

    def process_data(self, packet: bytes) -> bytes:
        """
        Process the message report received from the WaterLinked Modem-M16.

        This will be further implemented later on when a data transfer protocol is established.

        Args:
            packet (bytes): raw data packet.

        Returns:
            byte: the raw data.
        """
        # Publish this message to a ROS topic
        msg_to_send = ByteMultiArray()
        msg_to_send.data = packet
        self._publisher.publish(msg_to_send)
        return packet

    def process_diagnostic_report(self, packet: bytes) -> dict[str, any]:
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

        return {
            'TR_BLOCK': decoded[0],
            'BER': decoded[1],
            'SIGNAL_POWER': decoded[2],
            'NOISE_POWER': decoded[3],
            'PACKET_VALID': decoded[4],
            'PACKET_INVALID': decoded[5],
            'GIT_REV': decoded[6].to_bytes(1, 'little'),
            'TIME': (decoded[9] << 16) | (decoded[8] << 8) | decoded[7],
            'CHIP_ID': decoded[10],
            'HW_REV': decoded[11] & 0b00000011,
            'CHANNEL': (decoded[11] & 0b00111100) >> 2,
            'TB_VALID': (decoded[11] & 0b01000000) >> 6,
            'TX_COMPLETE': (decoded[11] & 0b10000000) >> 7,
            'DIAGNOSTIC_MODE': decoded[12] & 0b00000001,
            'LEVEL': (decoded[12] & 0b00001100) >> 2,
        }

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
        if self.send_data(request.message):
            response.success = True
            response.message = 'Sent message successfully'
        else:
            response.success = False
            response.message = 'Failed to send message.'

        return response

    def change_setting(self, request: SetModemSettings.Request, response: SetModemSettings.Response) -> \
            SetModemSettings.Response:
        """
        Change the channel the modem is on if the modem is ready for commands.

        Args:
            channel (int): New channel value for the modem to operate on.
            setting (int): Setting to change. 1 - Comm Channel, 2 - Op Mode, 3 - Report Request, 4 - Power Level
            char_to_send (str): Value to change setting to. i.e. channel #, power level
        """
        if not self.modem_ready:
            error_msg = 'Modem not ready for commands.'
            self.get_logger().error(error_msg)
            response.success = False
            response.message = error_msg
            return response

        setting = request.setting
        char_to_send = request.char_to_send

        setting_char = ''
        true_char_to_send = ''

        match request.setting:
            case SetModemSettings.Request.TRANSPARENT_MODE:
                setting_char = 't'
            case SetModemSettings.Request.DIAGNOSTIC_MODE:
              setting_char = 'd'

            case SetModemSettings.Request.CHANGE_CHANNEL:
                setting_char = 'c'
                channel = char_to_send
                if not (self.MIN_CHANNEL <= channel <= self.MAX_CHANNEL):
                    error_msg = f'Invalid channel {channel}. Must be in range [{self.MIN_CHANNEL}, {self.MAX_CHANNEL}]'
                    self.get_logger().error(error_msg)
                    response.success = False
                    response.message = error_msg
                    return response

                true_char_to_send = format(channel, 'x')

            case SetModemSettings.Request.REQUEST_REPORT:
                setting_char = 'r'

            case SetModemSettings.Request.SET_POWER_LEVEL:
                setting_char = 'l'
                power_level = char_to_send
                if not (self.MIN_POWER_LEVEL <= power_level <= self.MAX_POWER_LEVEL):
                    error_msg = (f'Invalid power level {power_level}. Must be in range [{self.MIN_POWER_LEVEL}, '
                                 f'{self.MAX_POWER_LEVEL}]')
                    self.get_logger().error(error_msg)
                    response.success = False
                    response.message = error_msg
                    return response

                true_char_to_send = str(char_to_send)
            case _:
                error_msg = f'Invalid setting {setting}. Must be in range [{self.MIN_SETTING}, {self.MAX_SETTING}]'
                self.get_logger().error(error_msg)
                response.success = False
                response.message = error_msg
                return response

        self.change_setting_timer = time.time()
        print(f'Sending {setting_char}')
        self.send_data(setting_char)
        self.setting_char = setting_char
        self.true_char_to_send = true_char_to_send
        self.modem_ready = False
        self.setting = setting
        self.send_timer = self.create_timer(1.0, self.delayed_write)
        response.success = True
        return response

    def delayed_write(self) -> None:
        """Send the second byte after a delay."""
        if not self.modem_ready:
            print(f'Sending {self.setting_char} {self.true_char_to_send} {time.time() - self.change_setting_timer}')
            self.send_data(self.setting_char)
            if self.setting in {1, 4}:
                self.send_data(self.true_char_to_send)
            self.modem_ready = True
            self.send_timer.cancel()


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