#!/usr/bin/env python3

import os
import serial
import serial.tools.list_ports as list_ports
import traceback
import yaml
import resource_retriever as rr

import rclpy

from custom_msgs.msg import DVLRaw
from data_pub.serial_republisher_node import SerialReublisherNode

class DVLRawPublisher(SerialReublisherNode):

    CONFIG_FILE_PATH = f'package://data_pub/config/{os.getenv("ROBOT_NAME", "oogway")}.yaml'

    BAUDRATE = 115200
    NODE_NAME = 'dvl_raw_pub'
    TOPIC_NAME = 'sensors/dvl/raw'
    LINE_DELIM = ','

    CONNECTION_RETRY_PERIOD = 1.0 #S
    LOOP_RATE = 100.0 #Hz

    def __init__(self):

        super().__init__(self.NODE_NAME, self.BAUDRATE, self.CONFIG_FILE_PATH, 'dvl', self.CONNECTION_RETRY_PERIOD, self.LOOP_RATE)

        self._dvl_line_parsers = {
            'SA': self._parse_SA,
            'TS': self._parse_TS,
            'BI': self._parse_BI,
            'BS': self._parse_BS,
            'BE': self._parse_BE,
            'BD': self._parse_BD,
            'RA': self._parse_RA
        }

        self._pub = self.create_publisher(DVLRaw, self.TOPIC_NAME, 10)

        self._current_msg = DVLRaw()

    def _extract_floats(self, num_string, start, stop):
        """Return a list of floats from a given string,
        using LINE_DELIM and going from start to stop
        """
        return [float(num) for num in num_string.split(self.LINE_DELIM)[start:stop]]

    def process_line(self, line):
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

        self._pub.publish(self._current_msg)
        self._current_msg = DVLRaw()

    def _parse_RA(self, line):
        pass


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

