import time
from abc import ABC, abstractmethod
from contextlib import suppress
from pathlib import Path

import resource_retriever as rr
import serial
import yaml
from rclpy.node import Node
from serial.tools import list_ports


class SerialNode(Node, ABC):
    """Abstract ROS node to read and write to serial."""

    def __init__(self, node_name: str, baudrate: int, config_file_path: str, serial_device_name: str,
                 read_from_serial: bool, connection_retry_period: int=1, loop_rate: int=10,
                 use_nonblocking: bool = False, max_num_consecutive_empty_lines: int = 5) -> None:
        """
        Initialize SerialNode.

        Args:
            node_name (str): Name of the ROS node.
            baudrate (int): Baudrate for serial communication.
            config_file_path (str): Path to the config file. Loaded into self._config.
            serial_device_name (str): Human-readable name of the serial device. Used only for logging.
            read_from_serial (bool): Whether to read from serial. If True, subclasses must implement process_line.
            connection_retry_period (int): Time in seconds to wait before trying again to connect to serial.
            loop_rate (int): Rate in Hz to read from serial.
            use_nonblocking (bool): Whether to use non-blocking read from serial.
            max_num_consecutive_empty_lines (int): Maximum number of consecutive empty lines to read before resetting
                serial connection.
        """
        self._node_name = node_name
        self._baud = baudrate
        self._serial_device_name = serial_device_name
        self._read_from_serial = read_from_serial
        self._connection_retry_period = connection_retry_period
        self._loop_rate = loop_rate
        self._use_nonblocking = use_nonblocking
        self._max_num_consecutive_empty_lines = max_num_consecutive_empty_lines

        with Path(rr.get_filename(config_file_path, use_protocol=False)).open() as f:
            self._config = yaml.safe_load(f)

        super().__init__(node_name)

        self.connect_timer = self.create_timer(self._connection_retry_period, self.connect)
        self.read_timer = self.create_timer(1.0/loop_rate, self.read, autostart=False)

        self._serial_port = None
        self._serial = None
        self._num_consecutive_empty_lines = 0

    @abstractmethod
    def get_ftdi_string(self) -> str:
        """
        Get the FTDI string for the serial device.

        Returns:
            str: FTDI string for the serial device.
        """

    def connect(self) -> None:
        """Find and connect to the serial port."""
        try:
            self._serial_port = next(list_ports.grep(self.get_ftdi_string())).device.strip()
        except StopIteration:
            self.get_logger().error(f'Could not find {self._serial_device_name} in serial ports, trying again in '
                                    f'{self._connection_retry_period} seconds.')
            return

        try:
            self._serial = serial.Serial(self._serial_port, self._baud, timeout=1, write_timeout=None,
                                         bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                         stopbits=serial.STOPBITS_ONE)
            self.connect_timer.cancel()
            self.read_timer.reset()
            self.get_logger().info(f'Connected to {self._serial_device_name} at {self._serial_port}.')
        except StopIteration:
            self.get_logger().error(f'Error in connecting to {self._serial_device_name} over serial, trying again in '
                                    f'{self._connection_retry_period} seconds.')

    def readline_nonblocking(self, tout: int = 1) -> str:
        """
        Read line from serial port without blocking.

        Args:
            tout (int): timeout in seconds

        Returns:
            str: The line read from the serial port.
        """
        start = time.time()
        buff = b''
        while ((time.time() - start) < tout) and (b'\r\n' not in buff):
            with suppress(serial.SerialException):
                buff += self._serial.read(1)

        return buff.decode('utf-8', errors='ignore')

    def writebytes(self, data: bytes) -> bool:
        """
        Write bytes to serial port.

        Args:
            data (bytes): Data to write.

        Returns:
            bool: True if write was successful, False otherwise
        """
        if self._serial and self._serial.is_open:
            try:
                self._serial.write(data)
            except serial.SerialException:
                self.get_logger().error(f'Error in writing to {self._serial_device_name} serial port, trying to '
                                        'reconnect.')
                self.reset_serial()
                return False
        else:
            self.get_logger().error(f'Error in writing to {self._serial_device_name} serial port; not connected.')
            return False

        return True

    def writeline(self, line: str) -> bool:
        """
        Write line to serial port.

        Args:
            line (str): Line to write.

        Returns:
            bool: True if write was successful, False otherwise.
        """
        return self.writebytes((line + '\r\n').encode('utf-8'))

    def process_line(self, _: str) -> None:
        """
        Process line read from serial.

        Args:
            _ (str): Line to process.
        """
        if self._read_from_serial:
            error_msg = 'Subclasses must implement this method if read_from_serial is True.'
            raise NotImplementedError(error_msg)

    def reset_serial(self) -> None:
        """Reset the serial connection."""
        self.read_timer.cancel()
        self._serial.close()
        self._serial = None
        self._serial_port = None
        self.connect_timer.reset()

    def read(self) -> None:
        """Read from serial port and process the line."""
        try:
            if self._read_from_serial:
                if self._use_nonblocking:
                    line = self.readline_nonblocking().strip()
                else:
                    line = self._serial.readline().decode('utf-8', errors='ignore').strip()

                if line:
                    self._num_consecutive_empty_lines = 0
                    self.process_line(line)
                else:
                    self.get_logger().info(f'Empty line read from {self._serial_device_name}.')
                    self._num_consecutive_empty_lines += 1

                    if self._num_consecutive_empty_lines >= self._max_num_consecutive_empty_lines:
                        self.get_logger().error(f'{self._num_consecutive_empty_lines} consecutive empty lines read from'
                                                f' {self._serial_device_name}. Resetting serial connection.')
                        self._num_consecutive_empty_lines = 0
                        self.reset_serial()

        except serial.SerialException:
            self.get_logger().error(f'Error in reading {self._serial_device_name} from serial, trying to reconnect.')
            self.reset_serial()

    def destroy_node(self) -> None:
        """Clean up resources when node is destroyed."""
        if self._serial and self._serial.is_open:
            self._serial.close()
        super().destroy_node()
