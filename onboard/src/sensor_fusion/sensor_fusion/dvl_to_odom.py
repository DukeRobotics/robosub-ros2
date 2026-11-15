import math

import rclpy
from custom_msgs.msg import DVLRaw
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf_transformations import quaternion_from_euler


class DVLOdomPublisher(Node):
    """A class to convert raw DVL data to odometry messages."""

    NODE_NAME = 'dvl_odom_pub'
    DVL_RAW_TOPIC = '/sensors/dvl/raw'
    DVL_ODOM_TOPIC = '/sensors/dvl/odom'

    # Starting values for an uncharacterized vehicle installation. Bottom-track
    # quality and error velocity can only increase this variance.
    BASE_VELOCITY_VARIANCE = 0.0225
    WATER_TRACK_VARIANCE = 0.25  # Initial value: includes unmeasured water motion.
    MAX_WATER_TRACK_AGE = 1.0  # Age of a fix, not a limit on fallback duration.
    DVL_ATTITUDE_VARIANCE = 0.25
    INVALID_ATTITUDE_VARIANCE = 1e6
    MIN_VALID_BEAMS = 3
    GOOD_CORRELATION = 64

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME)
        self._pub = self.create_publisher(Odometry, self.DVL_ODOM_TOPIC, 50)
        self._sub = self.create_subscription(DVLRaw, self.DVL_RAW_TOPIC, self.convert_to_odom, 10)

    def convert_to_odom(self, msg: DVLRaw) -> None:
        """
        Convert raw DVL data to an odometry message and publish it.

        Args:
            msg (DVLRaw): Raw DVL data.
        """
        velocity = [msg.bs_transverse, msg.bs_longitudinal, msg.bs_normal]
        bottom_lock = msg.bs_status.strip() == 'A' and all(math.isfinite(value) and value != -32768
                                                         for value in velocity)
        valid_beams = [i for i, distance in enumerate(msg.bt_beam_ranges)
                       if math.isfinite(distance) and distance > 0]
        if msg.bt_ranges_valid or msg.bt_quality_valid:
            bottom_lock = bottom_lock and len(valid_beams) >= self.MIN_VALID_BEAMS

        # Prefer bottom lock. Use a fresh ship-frame water fix when bottom lock
        # is unavailable; never feed invalid bottom data or hold the last fix.
        if not bottom_lock:
            velocity = [msg.ws_transverse, msg.ws_longitudinal, msg.ws_normal]
            if (msg.ws_status.strip() != 'A'
                    or not all(math.isfinite(value) and value != -32768 for value in velocity)
                    or not math.isfinite(msg.wd_time) or not 0 <= msg.wd_time <= self.MAX_WATER_TRACK_AGE):
                return

        quality_scale = 1.0
        if bottom_lock and msg.bt_quality_valid:
            mean_percent_good = sum(msg.bt_percent_good[index] for index in valid_beams) / len(valid_beams)
            mean_correlation = sum(msg.bt_correlation[index] for index in valid_beams) / len(valid_beams)
            # Three valid beams, low percent-good values, and weak correlation
            # all reduce confidence. This is deliberately bounded: diagnostics
            # should soften a valid measurement, not unexpectedly suppress DVL
            # data before the thresholds have been characterized in the pool.
            quality = ((len(valid_beams) / 4)
                       * (mean_percent_good / 100)
                       * min(mean_correlation / self.GOOD_CORRELATION, 1.0))
            quality_scale = 1 / max(quality, 0.1) ** 2

        # handle message here
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'

        # bs velocity, normalized to meters (given in mm)
        # parentheses denote new negative signs
        vx, vy, vz = [float(value) / 1000 for value in velocity]

        # set pose
        attitude_variance = self.INVALID_ATTITUDE_VARIANCE
        orientation = Quaternion(w=1.0)
        if msg.sa_valid and all(math.isfinite(value) for value in [msg.sa_roll, msg.sa_pitch, msg.sa_heading]):
            roll = math.radians(msg.sa_roll)
            pitch = math.radians(msg.sa_pitch)
            yaw = math.radians(msg.sa_heading)
            orientation_quaternion = quaternion_from_euler(roll, pitch, yaw)
            orientation = Quaternion(x=orientation_quaternion[0], y=orientation_quaternion[1],
                                     z=orientation_quaternion[2], w=orientation_quaternion[3])
            # Attitude fusion remains disabled pending frame validation.
            attitude_variance = self.DVL_ATTITUDE_VARIANCE

        odom.pose.pose = Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=orientation)
        odom.pose.covariance[21] = attitude_variance
        odom.pose.covariance[28] = attitude_variance
        odom.pose.covariance[35] = attitude_variance
        odom.child_frame_id = msg.header.frame_id

        # set twist (set angular velocity to (0, 0, 0), should not be used)
        odom.twist.twist = Twist(linear=Vector3(x=vx, y=vy, z=vz), angular=Vector3(x=0.0, y=0.0, z=0.0))
        # Error velocity is the DVL's per-sample residual. Treat it as an
        # additional standard deviation: large residuals make this sample less
        # influential without discarding an otherwise valid bottom-track fix.
        error_velocity_variance = 0.0
        error = msg.bi_error if bottom_lock else msg.wi_error
        if math.isfinite(error):
            error_velocity_variance = min(abs(error / 1000), 100.0) ** 2
        baseline = self.BASE_VELOCITY_VARIANCE if bottom_lock else self.WATER_TRACK_VARIANCE
        velocity_variance = (baseline + error_velocity_variance) * quality_scale
        if not math.isfinite(error):  # Three-beam fixes may lack error velocity.
            velocity_variance *= 4
        odom.twist.covariance[0] = velocity_variance
        odom.twist.covariance[7] = velocity_variance
        odom.twist.covariance[14] = velocity_variance
        self._pub.publish(odom)


def main(args: list[str] | None = None) -> None:
    """Create and run the DVL odometry publisher node."""
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
