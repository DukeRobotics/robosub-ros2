#!/usr/bin/env python3

import os
import serial
import serial.tools.list_ports as list_ports
import traceback
import yaml
import resource_retriever as rr

import rclpy
from rclpy.node import Node

from custom_msgs.msg import DVLRaw

class DVLRawPublisher(Node):

    CONFIG_FILE_PATH = f'package://data_pub/config/{os.getenv("ROBOT_NAME", "oogway")}.yaml'

    BAUDRATE = 115200
    NODE_NAME = 'dvl_raw_pub'
    TOPIC_NAME = 'sensors/dvl/raw'
    LINE_DELIM = ','

    CONNECTION_RETRY_PERIOD = 0.2 #S
    LOOP_RATE = 10 #Hz

    def __init__(self):
        with open(rr.get_filename(self.CONFIG_FILE_PATH, use_protocol=False)) as f:
            self._config_data = yaml.safe_load(f)

        super().__init__(self.NODE_NAME)
        self._pub = self.create_publisher(DVLRaw, self.TOPIC_NAME, 10)

        self._current_msg = DVLRaw()

        self._serial_port = None
        self._serial = None

        self._dvl_line_parsers = {
            'SA': self._parse_SA,
            'TS': self._parse_TS,
            'BI': self._parse_BI,
            'BS': self._parse_BS,
            'BE': self._parse_BE,
            'BD': self._parse_BD,
            'RA': self._parse_RA
        }

        self.connection_timer = self.create_timer(self.CONNECTION_RETRY_PERIOD, self.connect)
        self.run_timer = self.create_timer(1.0/self.LOOP_RATE, self.run)
        self.run_timer.cancel()

    def connect(self):
        while self._serial_port is None and rclpy.ok():
            try:
                dvl_ftdi_string = self._config_data['dvl']['ftdi']
                self._serial_port = next(list_ports.grep(dvl_ftdi_string)).device
                self._serial = serial.Serial(self._serial_port, self.BAUDRATE,
                                             timeout=0.1, write_timeout=1.0,
                                             bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                             stopbits=serial.STOPBITS_ONE)
                self.run_timer.reset()
                self.connection_timer.cancel()
            except StopIteration:
                self.get_logger().error(f"DVL not found, trying again in {self.CONNECTION_RETRY_PERIOD} seconds.")

    def run(self):
        try:
            line = self._serial.readline().decode('utf-8')
            if line and line.strip() and line[0] == ':':
                self._parse_line(line)
        except Exception:
            self.get_logger().info("Error in reading and extracting information. Reconnecting.")
            self.get_logger().info(traceback.format_exc())
            self._serial.close()
            self._serial = None
            self._serial_port = None
            self.run_timer.cancel()
            self.connection_timer.reset()

    def _parse_line(self, line):
        data_type = line[1:3]
        self._dvl_line_parsers[data_type](self._clean_line(line))

    def _clean_line(self, line):
        return line[4:].replace('\r\n', '')

    def _parse_SA(self, line):
        fields = self._extract_floats(line, 0, None)
        self._current_msg.sa_roll = fields[0]
        self._current_msg.sa_pitch = fields[1]
        self._current_msg.sa_heading = fields[2]

    def _parse_TS(self, line):
        fields = self._extract_floats(line, 1, None)
        self._current_msg.ts_salinity = fields[0]
        self._current_msg.ts_temperature = fields[1]
        self._current_msg.ts_depth = fields[2]
        self._current_msg.ts_sound_speed = fields[3]
        self._current_msg.ts_built_in_test = int(fields[4])

    def _parse_BI(self, line):
        fields = self._extract_floats(line, 0, 4)
        self._current_msg.bi_x_axis = fields[0]
        self._current_msg.bi_y_axis = fields[1]
        self._current_msg.bi_z_axis = fields[2]
        self._current_msg.bi_error = fields[3]
        self._current_msg.bi_status = line.split(self.LINE_DELIM)[4]

    def _parse_BS(self, line):
        fields = self._extract_floats(line, 0, 3)

        # Filter out error values
        if abs(fields[0]) > 32000 or abs(fields[1]) > 32000 or abs(fields[2]) > 32000:
            return

        self._current_msg.bs_transverse = fields[0]
        self._current_msg.bs_longitudinal = fields[1]
        self._current_msg.bs_normal = fields[2]
        self._current_msg.bs_status = line.split(self.LINE_DELIM)[3]

    def _parse_BE(self, line):
        fields = self._extract_floats(line, 0, 3)
        self._current_msg.be_east = fields[0]
        self._current_msg.be_north = fields[1]
        self._current_msg.be_upwards = fields[2]
        self._current_msg.be_status = line.split(self.LINE_DELIM)[3]

    def _parse_BD(self, line):
        fields = self._extract_floats(line, 0, None)
        self._current_msg.bd_east = fields[0]
        self._current_msg.bd_north = fields[1]
        self._current_msg.bd_upwards = fields[2]
        self._current_msg.bd_range = fields[3]
        self._current_msg.bd_time = fields[4]

        self._publish_current_msg()

    # Pressure and range to bottom data, currently being ignored
    def _parse_RA(self, line):
        pass

    def _extract_floats(self, num_string, start, stop):
        """Return a list of floats from a given string,
        using LINE_DELIM and going from start to stop
        """
        return [float(num) for num in num_string.split(self.LINE_DELIM)[start:stop]]

    def _publish_current_msg(self):
        """Publish the current DVL message and set the message to empty
        """
        self._pub.publish(self._current_msg)
        self._current_msg = DVLRaw()


def main(args=None):
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