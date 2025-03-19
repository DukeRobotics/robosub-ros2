import os

import rclpy
from custom_msgs.msg import DVLRaw

from offboard_comms.serial_node import SerialNode, SerialReadType


class DVLRawPublisher(SerialNode):
    """A class to read and publish raw DVL data from a serial port."""

    SERIAL_DEVICE_NAME = 'DVL'
    CONFIG_FILE_PATH = f'package://offboard_comms/config/{os.getenv("ROBOT_NAME")}.yaml'

    BAUDRATE = 115200
    NODE_NAME = 'dvl_raw_pub'
    TOPIC_NAME = 'sensors/dvl/raw'
    LINE_DELIM = ','

    CONNECTION_RETRY_PERIOD = 1.0 #S
    LOOP_RATE = 100.0 #Hz

    def __init__(self) -> None:

        super().__init__(self.NODE_NAME, self.BAUDRATE, self.CONFIG_FILE_PATH, self.SERIAL_DEVICE_NAME,
                         SerialReadType.LINE_BLOCKING, self.CONNECTION_RETRY_PERIOD, self.LOOP_RATE)

        self._dvl_line_parsers = {
            'SA': self._parse_SA,
            'TS': self._parse_TS,
            'BI': self._parse_BI,
            'BS': self._parse_BS,
            'BE': self._parse_BE,
            'BD': self._parse_BD,
            'RA': self._parse_RA,
        }

        self._pub = self.create_publisher(DVLRaw, self.TOPIC_NAME, 10)

        self._current_msg = DVLRaw()

    def get_ftdi_string(self) -> str:
        """
        Get the FTDI string for the DVL.

        Returns:
            str: FTDI string for the DVL.
        """
        return self._config['dvl']['ftdi']

    def _extract_floats(self, num_string: str, start: int, stop: int) -> list[float]:
        """Return a list of floats from a given string, using LINE_DELIM and going from start to stop."""
        return [float(num) for num in num_string.split(self.LINE_DELIM)[start:stop]]

    def process_line(self, line: str) -> None:
        """
        Process a line of serial data from the DVL.

        Args:
            line (str): line to process
        """
        try:
            data_type = line[1:3]
        except IndexError:
            self.get_logger().warn(f'Failed to parse data type from line: {line}')

        if data_type in self._dvl_line_parsers:
            self._dvl_line_parsers[data_type](self._clean_line(line))
        else:
            self.get_logger().warn(f'Unknown data type: {data_type}')

    def _clean_line(self, line: str) -> str:
        """
        Remove whitespace from line.

        Args:
            line (str): line to clean
        """
        return line[4:].replace('\r\n', '')

    def _parse_SA(self, line: str) -> None:
        """
        Parse system attitude data.

        Args:
            line (str): line to parse
        """
        fields = self._extract_floats(line, 0, None)
        self._current_msg.sa_roll = fields[0]
        self._current_msg.sa_pitch = fields[1]
        self._current_msg.sa_heading = fields[2]

    def _parse_TS(self, line: str) -> None:
        """
        Parse timing and scaling data.

        Args:
            line (str): line to parse
        """
        fields = self._extract_floats(line, 1, None)
        self._current_msg.ts_salinity = fields[0]
        self._current_msg.ts_temperature = fields[1]
        self._current_msg.ts_depth = fields[2]
        self._current_msg.ts_sound_speed = fields[3]
        self._current_msg.ts_built_in_test = int(fields[4])

    def _parse_BI(self, line: str) -> None:
        """
        Parse bottom-track, instrument-referenced velocity data.

        Args:
            line (str): line to parse
        """
        fields = self._extract_floats(line, 0, 4)
        self._current_msg.bi_x_axis = fields[0]
        self._current_msg.bi_y_axis = fields[1]
        self._current_msg.bi_z_axis = fields[2]
        self._current_msg.bi_error = fields[3]
        self._current_msg.bi_status = line.split(self.LINE_DELIM)[4]

    def _parse_BS(self, line: str) -> None:
        """
        Parse bottom-track ship-referenced velocity data.

        Args:
            line (str): line to parse
        """
        fields = self._extract_floats(line, 0, 3)

        # Filter out error values
        error_threshold = 32000
        if abs(fields[0]) > error_threshold or abs(fields[1]) > error_threshold or abs(fields[2]) > error_threshold:
            return

        self._current_msg.bs_transverse = fields[0]
        self._current_msg.bs_longitudinal = fields[1]
        self._current_msg.bs_normal = fields[2]
        self._current_msg.bs_status = line.split(self.LINE_DELIM)[3]

    def _parse_BE(self, line: str) -> None:
        """
        Parse bottom-track Earth-referenced velocity data.

        Args:
            line (str): The line to parse.
        """
        fields = self._extract_floats(line, 0, 3)
        self._current_msg.be_east = fields[0]
        self._current_msg.be_north = fields[1]
        self._current_msg.be_upwards = fields[2]
        self._current_msg.be_status = line.split(self.LINE_DELIM)[3]

    def _parse_BD(self, line: str) -> None:
        """
        Parse bottom-track Earth-referenced distance data.

        Args:
            line (str): The line to parse.
        """
        fields = self._extract_floats(line, 0, None)
        self._current_msg.bd_east = fields[0]
        self._current_msg.bd_north = fields[1]
        self._current_msg.bd_upwards = fields[2]
        self._current_msg.bd_range = fields[3]
        self._current_msg.bd_time = fields[4]

        self._pub.publish(self._current_msg)
        self._current_msg = DVLRaw()

    def _parse_RA(self, line: str) -> None:
        """
        Parse range and altitude data. *This data is not currently used*.

        Args:
            line (str): The line to parse.
        """


def main(args: list[str] | None = None) -> None:
    """Create and run the DVL raw data publisher node."""
    rclpy.init(args=args)
    dvl_raw = DVLRawPublisher()

    try:
        rclpy.spin(dvl_raw)
    except KeyboardInterrupt:
        pass
    finally:
        dvl_raw.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

