import os
import time
import struct
import time

import rclpy


from offboard_comms.serial_node import SerialNode


class ModemPublisher(SerialNode):
    """Serial publisher to publish and send data from WaterLinked M16 modems."""

    SERIAL_DEVICE_NAME = 'wlmodem-m16'
    CONFIG_FILE_PATH = f'package://offboard_comms/config/{os.getenv("ROBOT_NAME")}.yaml'

    BAUDRATE = 9600
    NODE_NAME = 'wlmodem'
    CONNECTION_RETRY_PERIOD = 1.0  # seconds
    LOOP_RATE = 0.625  # Hz, loop every 1.6 seconds

    DIAGNOSTIC_PACKET_SIZE = 18

    modem_ready = True
    change_setting_timer = time.time()

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME, self.BAUDRATE, self.CONFIG_FILE_PATH, self.SERIAL_DEVICE_NAME, True,
                         self.CONNECTION_RETRY_PERIOD, self.LOOP_RATE, use_nonblocking=True, return_byte=True)

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

    def process_line(self, line: bytes) -> None:
        """
        Process a line of serial data from the WaterLinked Modem-M16.

        Args:
            line (byte): line to process
        """
        if len(line) != self.DIAGNOSTIC_PACKET_SIZE or line[0] != ord('$') or line[-1] != ord('\n'):
            self.process_data(line)
        else:
            self.process_diagnostic_report(line)

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

    def change_setting(self, setting: int, char_to_send: str = '') -> None:
        """
        Change the channel the modem is on if the modem is ready for commands.

        Args:
            channel (int): New channel value for the modem to operate on.
            setting (int): Setting to change. 1 - Comm Channel, 2 - Op Mode, 3 - Report Request, 4 - Power Level
            char_to_send (str): Value to change setting to. i.e. channel #, power level
        """
        setting_char = ''
        true_char_to_send = ''

        if ((setting == 1 or setting == 4) and not char_to_send.isdigit()){
            self.get_logger().error('Invalid char_to_send, must be a number')
            return
        }

        if (setting == 1 and char_to_send.isdigit()):
            setting_char = 'c'
            channel = int(char_to_send)
            if channel < 1 or channel > 12:
                self.get_logger().error('Invalid channel, please input an int within [1, 12]')
                return
            if (channel < 10):
                true_char_to_send = str(channel)
            else:
                true_char_to_send = ['a','b','c'][channel-10]
        elif (setting == 2):
            setting_char = 'm'
        elif (setting == 3):
            setting_char = 'r'
        elif (setting == 4 and char_to_send.isdigit()):
            setting_char = 'l'
            power_lvl = int(char_to_send)
            true_char_to_send = char_to_send
            if power_lvl < 1 or power_lvl > 4:
                self.get_logger().error('Invalid power level, please input an int within [1, 4]')
                return

        if self.modem_ready and setting_char != '':
            self.change_setting_timer = time.time()
            self.writebytes(ord(setting_char))
            self.setting_char = setting_char
            # Set a flag to true, record timestamp of the first byte, and record the content of the byte
            while time.time() < self.change_setting_timer + 1:
                continue
            self.writebytes(ord(setting_char))
            if setting == 1 or setting == 4:
                self.writebytes(ord(true_char_to_send))


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
