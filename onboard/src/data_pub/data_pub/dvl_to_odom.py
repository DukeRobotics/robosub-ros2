#!/usr/bin/env python3

import math
import numpy as np
import yaml
import os
import resource_retriever as rr

import rclpy
from rclpy.node import Node

from custom_msgs.msg import DVLRaw
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler

class DVLOdomPublisher(Node):
    CONFIG_FILE_PATH = f'package://data_pub/config/{os.getenv("ROBOT_NAME", "oogway")}.yaml'

    NODE_NAME = 'dvl_odom_pub'
    DVL_RAW_TOPIC = 'sensors/dvl/raw'
    DVL_ODOM_TOPIC = 'sensors/dvl/odom'

    DVL_BAD_STATUS_MSG = 'V'

    def __init__(self) -> None:
        with open(rr.get_filename(self.CONFIG_FILE_PATH, use_protocol=False)) as f:
            self._config_data = yaml.safe_load(f)

        super().__init__(self.NODE_NAME)
        self._pub = self.create_publisher(Odometry, self.DVL_ODOM_TOPIC, 50)
        self._sub = self.create_subscription(DVLRaw, self.DVL_RAW_TOPIC, self.convert_to_odom, 10)

    def convert_to_odom(self, msg):
        # check if the data is good
        # for now, only check bs and sa status as they are the only two data that we are currently using
        # there is no status for sa
        # for status: A = good, V = bad
        if msg.bs_status == self.DVL_BAD_STATUS_MSG:
            return
        # handle message here
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'

        # bs velocity, normalized to meters (given in mm)
        # parentheses denote new negative signs
        vx = np.float64(msg.bs_transverse) / 1000
        vy = np.float64(msg.bs_longitudinal) / 1000
        vz = np.float64(msg.bs_normal) / 1000

        if self._config_data["dvl"]["negate_x_vel"]:
            vx = -vx
        if self._config_data["dvl"]["negate_y_vel"]:
            vy = -vy
        if self._config_data["dvl"]["negate_z_vel"]:
            vz = -vz

        # quat
        roll = math.radians(np.float64(msg.sa_roll))
        pitch = math.radians(np.float64(msg.sa_pitch))
        yaw = math.radians(np.float64(msg.sa_heading))
        odom_quat = quaternion_from_euler(roll, pitch, yaw)

        # set pose
        odom.pose.pose = Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=odom_quat[1], y=odom_quat[2], z=odom_quat[3], w=odom_quat[0]))
        odom.child_frame_id = "dvl_link"

        # set twist (set angular velocity to (0, 0, 0), should not be used)
        odom.twist.twist = Twist(linear=Vector3(x=vx, v=vy, z=vz), angular=Vector3(x=0.0, y=0.0, z=0.0))
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[7] = 0.01
        odom.twist.covariance[14] = 0.01
        self._pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    dvl_odom = DVLOdomPublisher()

    try:
        rclpy.spin(dvl_odom)
    except KeyboardInterrupt:
        pass
    finally:
        dvl_odom.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()