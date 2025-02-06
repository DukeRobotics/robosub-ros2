import time
import traceback
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
                 use_nonblocking: bool = False) -> None:
        self._node_name = node_name
        self._baud = baudrate
        self._serial_device_name = serial_device_name
        self._read_from_serial = read_from_serial
        self._connection_retry_period = connection_retry_period
        self._loop_rate = loop_rate
        self._use_nonblocking = use_nonblocking

        with Path(rr.get_filename(config_file_path, use_protocol=False)).open() as f:
            self._config = yaml.safe_load(f)

        super().__init__(node_name)

        self.connect_timer = self.create_timer(self._connection_retry_period, self.connect)
        self.run_timer = self.create_timer(1.0/loop_rate, self.run)
        self.run_timer.cancel()

        self._serial_port = None
        self._serial = None

    @abstractmethod
    def get_ftdi_string(self) -> str:
        """Get the FTDI string for the serial device."""

    def connect(self) -> None:
        """Read FTDI strings of all ports in list_ports.grep."""
        try:
            self._serial_port = next(list_ports.grep(self.get_ftdi_string())).device.strip()
            self._serial = serial.Serial(self._serial_port, self._baud,
                                            timeout=1, write_timeout=None,
                                            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                            stopbits=serial.STOPBITS_ONE)
            self.connect_timer.cancel()
            self.run_timer.reset()
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
            str: the line read from the serial port
        """
        start = time.time()
        buff = b''
        while ((time.time() - start) < tout) and (b'\r\n' not in buff):
            with suppress(serial.SerialException):
                buff += self._serial.read(1)

        return buff.decode('utf-8', errors='ignore')

    def writebytes(self, data: bytes) -> None:
        """
        Write bytes to serial port.

        Args:
            data (bytes): data to write
        """
        if self._serial and self._serial.is_open:
            try:
                self._serial.write(data)
            except serial.SerialException:
                self.get_logger().error(f'Error in writing to {self._serial_device_name} serial port, trying to '
                                        'reconnect.')
                self.get_logger().error(traceback.format_exc())
                self._serial.close()
                self._serial = None
                self._serial_port = None
                self.run_timer.cancel()
                self.connect_timer.reset()
        else:
            self.get_logger().error(f'Error in writing to {self._serial_device_name} serial port; not connected.')

    def writeline(self, line: str) -> None:
        """
        Write line to serial port.

        Args:
            line (str): line to write
        """
        self.writebytes((line + '\r\n').encode('utf-8'))

    def process_line(self, _: str) -> None:
        """
        Process line read from serial.

        Args:
            _ (str): line to process
        """
        if self._read_from_serial:
            error_msg = 'Subclasses must implement this method if read_from_serial is True.'
            raise NotImplementedError(error_msg)

    def run(self) -> None:
        """
        Run the serial publisher.

        Initializes ROS node
        Connects to serial device
        Processes and publishes the serial data to ROS
        """
        try:
            if self._read_from_serial:
                if self._use_nonblocking:
                    line = self.readline_nonblocking().strip()
                else:
                    line = self._serial.readline().decode('utf-8').strip()

                if line:
                    self.process_line(line)

        except serial.SerialException:
            self.get_logger().error(f'Error in reading {self._serial_device_name} from serial, trying to reconnect.')
            self.get_logger().error(traceback.format_exc())
            self._serial.close()
            self._serial = None
            self._serial_port = None
            self.run_timer.cancel()
            self.connect_timer.reset()

    def destroy_node(self) -> None:
        """Clean up resources when node is destroyed."""
        if self._serial and self._serial.is_open:
            self._serial.close()
        super().destroy_node()
