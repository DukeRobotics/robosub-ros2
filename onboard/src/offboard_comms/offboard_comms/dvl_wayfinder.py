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

    # DVL orientation in degrees
    WAYFINDER_ROLL = 0
    WAYFINDER_PITCH = 0
    WAYFINDER_YAW = 225

    NODE_NAME = 'dvl_wayfinder'

    DVL_ODOM_TOPIC = 'sensors/dvl/odom'

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME)

        with Path(rr.get_filename(self.CONFIG_FILE_PATH, use_protocol=False)).open() as f:
            self._config_data = yaml.safe_load(f)

        ftdi = self._config_data['dvl']['ftdi']

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
        self._rotMatrix = euler2mat(math.radians(self.WAYFINDER_ROLL), math.radians(self.WAYFINDER_PITCH),
                                    math.radians(self.WAYFINDER_YAW))

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
        # Make sure the data is valid
        if any(math.isnan(val) for val in [data_obj.vel_x, data_obj.vel_y, data_obj.vel_z, data_obj.vel_err]):
            return

        # Apply rotation matrix to velocities
        vels = np.matmul(self._rotMatrix, [data_obj.vel_x, data_obj.vel_y, data_obj.vel_z])

        # Convert DVL time to Unix timestamp (as float in seconds)
        dvl_time = dt.datetime(data_obj.year, data_obj.month, data_obj.day, data_obj.hour, data_obj.minute,
                                data_obj.second, tzinfo=dt.UTC)
        unix_timestamp = dvl_time.timestamp()

        # Create the ROS2 Time message
        header_stamp = Time()
        header_stamp.sec = int(unix_timestamp)
        header_stamp.nanosec = int(int(unix_timestamp) * 1e9)

        # Negate velocities if configured
        if self._config_data['dvl']['negate_x_vel']:
            vels[0] = -vels[0]
        if self._config_data['dvl']['negate_y_vel']:
            vels[1] = -vels[1]
        if self._config_data['dvl']['negate_z_vel']:
            vels[2] = -vels[2]

        # Publish the data
        self.odom.header.stamp = header_stamp
        self.odom.twist.twist.linear = Vector3(x=vels[0], y=vels[1], z=vels[2])
        self._pub.publish(self.odom)

def main(args: list[str] | None = None) -> None:
    """Initialize and run the DVL Wayfinder node."""
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

