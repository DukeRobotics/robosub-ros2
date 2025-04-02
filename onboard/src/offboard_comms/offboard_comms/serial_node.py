import time
from abc import ABC, abstractmethod
from contextlib import suppress
from enum import Enum
from pathlib import Path

import resource_retriever as rr
import serial
import yaml
from rclpy.node import Node
from serial.tools import list_ports


class SerialReadType(Enum):
    """Enum for the different methods of reading from serial."""
    NONE = 0  # Don't read from serial
    BYTES_FIXED = 1  # Read a fixed number of bytes
    BYTES_ALL = 2  # Read all bytes available
    LINE_BLOCKING = 3  # Read each line, blocking
    LINE_NONBLOCKING = 4  # Read each line, non-blocking


class SerialNode(Node, ABC):
    """Abstract ROS node to read and write to serial."""

    def __init__(self, node_name: str, baudrate: int, config_file_path: str, serial_device_name: str,
                 read_type: SerialReadType, connection_retry_period: float = 1, loop_rate: float = 10,
                 max_num_consecutive_empty_lines: int = 5, parity: str = serial.PARITY_NONE,
                 read_timeout: float = 1.0, num_bytes_to_read: int = 1, flush_input_after_read: bool = False) -> None:
        """
        Initialize SerialNode.

        Args:
            node_name (str): Name of the ROS node.
            baudrate (int): Baudrate for serial communication.
            config_file_path (str): Path to the config file. Loaded into self._config.
            serial_device_name (str): Human-readable name of the serial device. Used only for logging.
            read_type (SerialReadType): Type of read from serial.
            connection_retry_period (int): Time in seconds to wait before trying again to connect to serial.
            loop_rate (int): Rate in Hz to read from serial.
            use_nonblocking (bool): Whether to use non-blocking read from serial.
            max_num_consecutive_empty_lines (int): Maximum number of consecutive empty lines to read before resetting
                serial connection. Defaults to 5.
            parity (str): Parity for serial communication. Defaults to serial.PARITY_NONE.
            read_timeout (float): Timeout in seconds for reading bytes from serial. Defaults to 1.0.
            num_bytes_to_read (int): Number of bytes to read from serial at a given time. Used only if read_type is
                SerialReadType.BYTES_FIXED. Defaults to 1.
            flush_input_after_read (bool): Whether to flush the input buffer after reading. Defaults to False.
        """
        self._node_name = node_name
        self._baud = baudrate
        self._serial_device_name = serial_device_name
        self._read_type = read_type
        self._connection_retry_period = connection_retry_period
        self._loop_rate = loop_rate
        self._max_num_consecutive_empty_lines = max_num_consecutive_empty_lines
        self._parity = parity
        self._read_timeout = read_timeout
        self._num_bytes_to_read = num_bytes_to_read
        self._flush_input_after_read = flush_input_after_read

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
            self._serial = serial.Serial(self._serial_port, self._baud, timeout=self._read_timeout, write_timeout=None,
                                         bytesize=serial.EIGHTBITS, parity=self._parity, stopbits=serial.STOPBITS_ONE)
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

    def process_line(self, line: str) -> None:
        """
        Process line read from serial.

        Args:
            line (str): Line to process.
        """
        error_msg = 'Subclasses must implement this method to process the line read from serial.'
        raise NotImplementedError(error_msg)

    def process_bytes(self, _: bytes) -> None:
        """
        Process bytes read from serial.

        Args:
            _ (bytes): Data to process.
        """
        error_msg = 'Subclasses must implement this method to process the bytes read from serial.'
        raise NotImplementedError(error_msg)

    def reset_serial(self) -> None:
        """Reset the serial connection."""
        self.read_timer.cancel()
        self._serial.close()
        self._serial = None
        self._serial_port = None
        self.connect_timer.reset()

    def _read_bytes(self) -> bytes:
        """
        Read bytes from serial. Flush input buffer after reading if required.

        Returns:
            bytes: The bytes read from serial.
        """
        if self._read_type == SerialReadType.BYTES_FIXED:
            num_bytes = self._num_bytes_to_read
        elif self._read_type == SerialReadType.BYTES_ALL:
            num_bytes = self._serial.in_waiting
        else:
            error_msg = f'Invalid read type for reading bytes from serial: {self._read_type}'
            raise ValueError(error_msg)

        data = self._serial.read(num_bytes)

        if self._flush_input_after_read:
            self._serial.flushInput()

        return data

    def _read_line(self) -> str:
        """
        Read line from serial.

        Returns:
            str: The line read from serial.
        """
        if self._read_type == SerialReadType.LINE_BLOCKING:
            return self._serial.readline().decode('utf-8', errors='ignore').strip()
        if self._read_type == SerialReadType.LINE_NONBLOCKING:
            return self.readline_nonblocking().strip()

        error_msg = f'Invalid read type for reading line from serial: {self._read_type}'
        raise ValueError(error_msg)

    def _handle_line(self, line: str) -> None:
        """Handle the line read from serial."""
        if line:
            self._num_consecutive_empty_lines = 0
            self.process_line(line)
        else:
            self.get_logger().info(f'Empty line read from {self._serial_device_name}.')
            self._num_consecutive_empty_lines += 1

            if self._num_consecutive_empty_lines >= self._max_num_consecutive_empty_lines:
                self.get_logger().error(f'{self._num_consecutive_empty_lines} consecutive empty lines read from '
                                        f'{self._serial_device_name}. Resetting serial connection.')
                self._num_consecutive_empty_lines = 0
                self.reset_serial()

    def read(self) -> None:
        """Read from serial port and process the line."""
        try:
            if self._serial and self._serial.is_open and self._read_type != SerialReadType.NONE:
                match self._read_type:
                    case SerialReadType.BYTES_FIXED | SerialReadType.BYTES_ALL:
                        data = self._read_bytes()
                        if data:
                            self.process_bytes(data)
                        return
                    case SerialReadType.LINE_BLOCKING | SerialReadType.LINE_NONBLOCKING:
                        line = self._read_line()
                        self._handle_line(line)

        except serial.SerialException:
            self.get_logger().error(f'Error in reading {self._serial_device_name} from serial, trying to reconnect.')
            self.reset_serial()

    def destroy_node(self) -> None:
        """Clean up resources when node is destroyed."""
        if self._serial and self._serial.is_open:
            self._serial.close()
        super().destroy_node()
