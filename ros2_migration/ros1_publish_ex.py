#!/usr/bin/env python3
# ruff: noqa

import os
import serial
import serial.tools.list_ports as list_ports
import traceback
import yaml

import resource_retriever as rr
import rospy

from custom_msgs.msg import DVLRaw


class DvlRawPublisher:

    CONFIG_FILE_PATH = f'package://data_pub/config/{os.getenv("ROBOT_NAME", "oogway")}.yaml'

    BAUDRATE = 115200
    TOPIC_NAME = 'sensors/dvl/raw'
    NODE_NAME = 'dvl_raw_publisher'
    LINE_DELIM = ','

    def __init__(self):
        with open(rr.get_filename(self.CONFIG_FILE_PATH, use_protocol=False)) as f:
            self._config_data = yaml.safe_load(f)

        self._pub = rospy.Publisher(self.TOPIC_NAME, DVLRaw, queue_size=10)

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

    def connect(self):
        while self._serial_port is None and not rospy.is_shutdown():
            try:
                dvl_ftdi_string = self._config_data['dvl']['ftdi']
                self._serial_port = next(list_ports.grep(dvl_ftdi_string)).device
                self._serial = serial.Serial(self._serial_port, self.BAUDRATE,
                                             timeout=0.1, write_timeout=1.0,
                                             bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                             stopbits=serial.STOPBITS_ONE)
            except StopIteration:
                rospy.logerr("DVL not found, trying again in 0.1 seconds.")
                rospy.sleep(0.1)

    def run(self):
        rospy.init_node(self.NODE_NAME)
        self.connect()

        while not rospy.is_shutdown():
            try:
                line = self._serial.readline().decode('utf-8')
                if not line or line == '':
                    rospy.sleep(0.1)
                    continue  # Skip and retry
                if line.strip() and line[0] == ':':
                    self._parse_line(line)
            except Exception:
                rospy.logerr("Error in reading and extracting information. Reconnecting.")
                rospy.logerr(traceback.format_exc())
                self._serial.close()
                self._serial = None
                self._serial_port = None
                self.connect()

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

        # BD type is the last message received, so publish
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
        # We stopped resetting the current message because we want to use the past value in case of an error
        # See _parse_BS for relevant code
        # self._current_msg = DVLRaw()


if __name__ == '__main__':
    try:
        DvlRawPublisher().run()
    except rospy.ROSInterruptException:
        pass