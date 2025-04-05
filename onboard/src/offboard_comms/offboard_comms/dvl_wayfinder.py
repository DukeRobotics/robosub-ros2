import datetime as dt
import math
import os
import time
from pathlib import Path

import numpy as np
import rclpy
import resource_retriever as rr
import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from serial.tools import list_ports
from transforms3d.euler import euler2mat

from offboard_comms.dvl_wayfinder_lib.dvl import Dvl
from offboard_comms.dvl_wayfinder_lib.system import OutputData


class DVLWayfinderPublisher(Node):
    """A class to read and publish Teledyne Wayfinder DVL data from a serial port."""
    CONFIG_FILE_PATH = f'package://offboard_comms/config/{os.getenv("ROBOT_NAME")}.yaml'

    WAYFINDER_ROLL = np.radians(180)
    WAYFINDER_PITCH = np.radians(0)
    WAYFINDER_YAW = np.radians(135)

    NODE_NAME = 'dvl_wayfinder'

    DVL_ODOM_TOPIC = 'sensors/dvl/odom'

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME)

        with Path(rr.get_filename(self.CONFIG_FILE_PATH, use_protocol=False)).open() as f:
            self._config_data = yaml.safe_load(f)

        ftdi = self._config_data['dvl']['ftdi']
        self.get_logger().info(f'FTDI: {ftdi}')

        try:
            self._serial_port = next(list_ports.grep(ftdi)).device
        except StopIteration:
            self.get_logger().error('Wayfinder DVL not found.')
            rclpy.shutdown()

        self._sensor = Dvl()
        self._sensor.connect(self._serial_port, 115200)

        self.get_logger().info(f'Connected to DVL Wayfinder at {self._serial_port}.')

        # Precisely set the time on the DVL
        self._sensor.enter_command_mode()
        two_seconds = dt.datetime.now(dt.UTC) + dt.timedelta(seconds=2)
        time_target = dt.datetime(two_seconds.year, two_seconds.month, two_seconds.day, two_seconds.hour,
                                  two_seconds.minute, two_seconds.second, tzinfo=dt.UTC)
        now = dt.datetime.now(dt.UTC)
        time.sleep((time_target - now).total_seconds())
        self._sensor.set_time(dt.datetime.now(dt.UTC))
        self._sensor.exit_command_mode()

        # Set up rotation matrix
        self._rotMatrix = euler2mat(self.WAYFINDER_ROLL, self.WAYFINDER_PITCH, self.WAYFINDER_YAW, axes='sxyz')

        self._pub = self.create_publisher(Odometry, self.DVL_ODOM_TOPIC, 50)

        # Set up the odometry message
        self.odom = Odometry()
        self.odom.header.frame_id = 'odom'
        self.odom.child_frame_id = 'dvl'

        # Only set the covariance for (linear x, linear x), (linear y, linear y), (linear z, linear z)
        self.odom.twist.covariance[0] = 0.01
        self.odom.twist.covariance[7] = 0.01
        self.odom.twist.covariance[14] = 0.01

        #enable data callback
        self._sensor.register_ondata_callback(self.wayfinder_data_callback, None)


    def wayfinder_data_callback(self, data_obj: OutputData, *_) -> None:
        """
        Process Wayfinder data.

        Args:
            data_obj (OutputData): WayFinder output data
        """
        # Check velocities are valid
        if any(math.isnan(val) for val in [data_obj.vel_x, data_obj.vel_y, data_obj.vel_z, data_obj.vel_err]):
            return

        vels = np.array([data_obj.vel_x, data_obj.vel_y, data_obj.vel_z])
        vels =  np.matmul(self._rotMatrix, vels)

        # Confidence calculation
        vel_err_max = 1
        vel_err_min = 0.001
        pct_err_vel = 0.005
        vel_mean = np.mean(vels)
        confidence = 1 - (vel_mean - vel_err_min) / (vel_err_max - vel_err_min)

        dvl_time = dt.datetime(data_obj.year, data_obj.month, data_obj.day, data_obj.hour, data_obj.minute,
                                data_obj.second, tzinfo=dt.UTC)
        formatted_date = dvl_time.strftime('%Y-%m-%d %H:%M:%S')

        self.get_logger().info(f'{vels[0]:9.3f} {vels[1]:9.3f} {vels[2]:9.3f} | {data_obj.vel_err:9.3f} | '
                                f'{data_obj.range_beam1:9.3f} {data_obj.range_beam2:9.3f} '
                                f'{data_obj.range_beam3:9.3f} {data_obj.range_beam4:9.3f} | {data_obj.bit_code} | '
                                f'{data_obj.coordinate_system} | {formatted_date}')

        # Convert to Unix timestamp (as float in seconds)
        unix_timestamp = dvl_time.timestamp()

        # Convert to ROS2 Header stamp (sec and nanosec)
        sec = int(unix_timestamp)
        nanosec = int((unix_timestamp - sec) * 1e9)

        # Create the ROS2 Time message
        header_stamp = Time()
        header_stamp.sec = sec
        header_stamp.nanosec = nanosec

        vx = data_obj.vel_x
        vy = data_obj.vel_y
        vz = data_obj.vel_z

        if self._config_data['dvl']['negate_x_vel']:
            vx = -vx
        if self._config_data['dvl']['negate_y_vel']:
            vy = -vy
        if self._config_data['dvl']['negate_z_vel']:
            vz = -vz

        # Publish the data
        self.odom.header.stamp = header_stamp
        self.odom.twist.twist.linear = Vector3(x=vx, y=vy, z=vz)
        self._pub.publish(self.odom)

def main(args: list[str] | None = None) -> None:
    """Initialize and run the Sonar node."""
    rclpy.init(args=args)
    dvl_wayfinder = DVLWayfinderPublisher()

    try:
        rclpy.spin(dvl_wayfinder)
    except KeyboardInterrupt:
        pass
    finally:
        dvl_wayfinder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

