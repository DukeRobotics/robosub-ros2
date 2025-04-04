import datetime as dt
import math
import time
import traceback
from pathlib import Path
import os
import yaml

import numpy as np
import rclpy
from rclpy.node import Node
from serial.tools import list_ports
import resource_retriever as rr

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from builtin_interfaces.msg import Time

from offboard_comms.wayfinder_dvl.dvl import Dvl
from offboard_comms.wayfinder_dvl.system import OutputData


class Wayfinder_DVL(Node):
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

        try:
            self._serial_port = next(list_ports.grep('DN04A9W9')).device
        except StopIteration:
            self.get_logger().error('Wayfinder DVL not found.')
            rclpy.shutdown()

        self._sensor = Dvl()
        self._sensor.connect(self._serial_port, 115200)

        self.get_logger().info(f'Wayfinder connected at {self._serial_port}')

        #set time of device
        self._sensor.enter_command_mode()
        two_seconds = dt.datetime.now() + dt.timedelta(seconds=2)
        time_target = dt.datetime(two_seconds.year, two_seconds.month, two_seconds.day, two_seconds.hour, two_seconds.minute, two_seconds.second)
        now = dt.datetime.now()
        time.sleep((time_target - now).total_seconds())
        self._sensor.set_time(dt.datetime.now())
        self._sensor.exit_command_mode()

        #setup rotation matrix
        self._rotMatrix = np.array([[np.cos(self.WAYFINDER_YAW) * np.cos(self.WAYFINDER_PITCH), np.cos(self.WAYFINDER_YAW) * np.sin(self.WAYFINDER_PITCH) * np.sin(self.WAYFINDER_ROLL) - np.sin(self.WAYFINDER_YAW) * np.cos(self.WAYFINDER_ROLL), np.cos(self.WAYFINDER_YAW) * np.sin(self.WAYFINDER_PITCH) * np.cos(self.WAYFINDER_ROLL) + np.sin(self.WAYFINDER_YAW) * np.sin(self.WAYFINDER_ROLL)],
                            [np.sin(self.WAYFINDER_YAW) * np.cos(self.WAYFINDER_PITCH), np.sin(self.WAYFINDER_YAW) * np.sin(self.WAYFINDER_PITCH) * np.sin(self.WAYFINDER_ROLL) + np.cos(self.WAYFINDER_YAW) * np.cos(self.WAYFINDER_ROLL), np.sin(self.WAYFINDER_YAW) * np.sin(self.WAYFINDER_PITCH) * np.cos(self.WAYFINDER_ROLL) - np.cos(self.WAYFINDER_YAW) * np.sin(self.WAYFINDER_ROLL)],
                            [ -np.sin(self.WAYFINDER_PITCH), np.cos(self.WAYFINDER_PITCH) * np.sin(self.WAYFINDER_ROLL), np.cos(self.WAYFINDER_PITCH) * np.cos(self.WAYFINDER_ROLL)]])

        self._pub = self.create_publisher(Odometry, self.DVL_ODOM_TOPIC, 50)

        #enable data callback
        self._sensor.register_ondata_callback(self.wayfinderDataCallback, None)


    def wayfinderDataCallback(self, dataObj: OutputData, *args):
        """
        WayFinder Data Callback Function

        WayFinder Data Callback - this processed the data from the WayFinder and
        passes it to the PixHawk autopilot

        Arguments:
            dataObj {OutputData} -- WayFinder output data
        """
        try:
            # Check velocities are valid
            if math.isnan(dataObj.vel_x) or math.isnan(dataObj.vel_y) or math.isnan(dataObj.vel_z) or math.isnan(dataObj.vel_err):
                return

            vels = np.array([dataObj.vel_x, dataObj.vel_y, dataObj.vel_z])
            vels =  np.matmul(self._rotMatrix, vels)

            # Confidence calculation
            vel_err_max = 1
            vel_err_min = 0.001
            pct_err_vel = 0.005
            vel_mean = np.mean(vels)
            confidence = 1 - (vel_mean - vel_err_min) / (vel_err_max - vel_err_min)

            dvl_time = dt.datetime(dataObj.year, dataObj.month, dataObj.day, dataObj.hour, dataObj.minute, dataObj.second)
            formatted_date = dvl_time.strftime("%Y-%m-%d %H:%M:%S")

            self.get_logger().info('%9.3f %9.3f %9.3f | %9.3f | %9.3f %9.3f %9.3f %9.3f | %s %s' % (vels[0], vels[1], vels[2], dataObj.vel_err, dataObj.range_beam1, dataObj.range_beam2, dataObj.range_beam3, dataObj.range_beam4, dataObj.bit_code, formatted_date))

            # Convert to Unix timestamp (as float in seconds)
            unix_timestamp = dvl_time.timestamp()

            # Convert to ROS2 Header stamp (sec and nanosec)
            sec = int(unix_timestamp)
            nanosec = int((unix_timestamp - sec) * 1e9)

            # Create the ROS2 Time message
            header_stamp = Time()
            header_stamp.sec = sec
            header_stamp.nanosec = nanosec

            # Publish the data
            odom = Odometry()
            odom.header.stamp = header_stamp
            odom.header.frame_id = 'odom'

            # set pose
            odom.child_frame_id = 'dvl'

            # set twist (set angular velocity to (0, 0, 0), should not be used)
            odom.twist.twist = Twist(linear=Vector3(x=dataObj.vel_x, y=dataObj.vel_y, z=dataObj.vel_z),
                                     angular=Vector3(x=0.0, y=0.0, z=0.0))
            odom.twist.covariance[0] = 0.01
            odom.twist.covariance[7] = 0.01
            odom.twist.covariance[14] = 0.01
            self._pub.publish(odom)
        except Exception as e:
            self.get_logger().info(e)
            traceback.print_exc()

def main(args: list[str] | None = None) -> None:
    """Initialize and run the Sonar node."""
    rclpy.init(args=args)
    wayfinder_dvl = Wayfinder_DVL()

    try:
        rclpy.spin(wayfinder_dvl)
    except KeyboardInterrupt:
        pass
    finally:
        wayfinder_dvl.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

