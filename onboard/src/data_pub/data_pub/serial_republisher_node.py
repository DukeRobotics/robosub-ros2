#!/usr/bin/env python3

from abc import ABC, abstractmethod
import time
import serial
import serial.tools.list_ports as list_ports
import yaml
import resource_retriever as rr
import traceback

import rclpy
from rclpy.node import Node

class SerialReublisherNode(Node, ABC):
    """
    Abstract serial publisher for a ROS node
    """

    def __init__(self, node_name, baudrate, config_file_path, config_name, connection_retry_period=1, loop_rate=10, use_nonblocking = False):
        """
        @param node_name: The ROS node name
        @param topic_name: The ROS topic name
        @param baud_rate: the baud rate
        @config_file_path: location of serial config
        @config_name: name of Arduino to reference from config file
        @param connection_retry_period: time to wait before retrying connection
        @param loop_rate: rate at which to run the node
        @param use_nonblocking: whether to use nonblocking serial read
        """

        self._node_name = node_name
        self._baud = baudrate
        self._config_name = config_name
        self._connection_retry_period = connection_retry_period
        self._loop_rate = loop_rate
        self._use_nonblocking = use_nonblocking

        with open(rr.get_filename(config_file_path, use_protocol=False)) as f:
            self._config_data = yaml.safe_load(f)

        super().__init__(node_name)

        self.connect_timer = self.create_timer(self._connection_retry_period, self.connect)
        self.run_timer = self.create_timer(1.0/loop_rate, self.run)
        self.run_timer.cancel()

        self._serial_port = None
        self._serial = None

    def connect(self):
        """
        Read FTDI strings of all ports in list_ports.grep
        """
        try:
            self._serial_port = next(list_ports.grep(self._config_data[self._config_name]['ftdi'])).device.strip()
            self._serial = serial.Serial(self._serial_port, self._baud,
                                            timeout=1, write_timeout=None,
                                            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                            stopbits=serial.STOPBITS_ONE)
            self.connect_timer.cancel()
            self.run_timer.reset()
            self.get_logger().info(f"Connected to {self._config_name} sensor at {self._serial_port}.")
        except StopIteration:
            self.get_logger().error(f"Error in connecting to serial device in {self._node_name}, trying again in {self._connection_retry_period} seconds.")

    def readline_nonblocking(self, tout=1):
        """
        Read line from serial port without blocking

        @param tout: timeout, default = 1 sec
        """
        start = time.time()
        buff = b''
        while ((time.time() - start) < tout) and (b'\r\n' not in buff):
            try:
                buff += self._serial.read(1)
            except serial.SerialException:
                pass

        return buff.decode('utf-8', errors='ignore')

    def writeline(self, line):
        """
        Write line to serial port

        @param line: the line to write
        """
        self._serial.write(line.encode('utf-8') + b'\r\n')

    @abstractmethod
    def process_line(self, line):
        """
        Abstract method to implement how serial input is formatted.

        @param line: the line to be processed
        """
        pass

    def run(self):
        """
        Runs the serial publisher

        Initializes ROS node
        Connects to serial device
        Processes and publishes the serial data to ROS
        """

        try:
            if self._use_nonblocking:
                line = self.readline_nonblocking().strip()
            else:
                line = self._serial.readline().decode('utf-8').strip()

            if line:
                self.process_line(line)

        except Exception:
            self.get_logger().error(f"Error in reading {self._config_name} serial read, trying again in 1 second.")
            self.get_logger().error(traceback.format_exc())
            self._serial.close()
            self._serial = None
            self._serial_port = None
            self.run_timer.cancel()
            self.connect_timer.reset()