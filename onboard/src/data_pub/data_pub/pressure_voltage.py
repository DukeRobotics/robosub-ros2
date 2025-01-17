import os

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64

from data_pub.serial_republisher_node import SerialRepublisherNode


class PressureVoltagePublisher(SerialRepublisherNode):
    """Serial publisher to publish voltage and pressure data to ROS."""

    CONFIG_FILE_PATH = f'package://data_pub/config/{os.getenv("ROBOT_NAME", "oogway")}.yaml'

    BAUDRATE = 9600
    NODE_NAME = 'pressure_voltage_pub'
    DEPTH_DEST_TOPIC = 'sensors/depth'
    VOLTAGE_DEST_TOPIC = 'sensors/voltage'

    CONNECTION_RETRY_PERIOD = 1.0 #S
    LOOP_RATE = 50.0 #Hz
    MEDIAN_FILTER_SIZE = 3
    MAX_ABS_PRESSURE = 7

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME, self.BAUDRATE, self.CONFIG_FILE_PATH, 'pv', self.CONNECTION_RETRY_PERIOD,
                         self.LOOP_RATE, use_nonblocking=False)

        self._pressure = None  # Pressure to publish
        self._previous_pressure = None  # Previous pressure readings for median filter

        self._pub_depth = self.create_publisher(PoseWithCovarianceStamped, self.DEPTH_DEST_TOPIC, 50)
        self._pub_voltage = self.create_publisher(Float64, self.VOLTAGE_DEST_TOPIC, 50)

        self._current_pressure_msg = PoseWithCovarianceStamped()
        self._current_voltage_msg = Float64()

    def process_line(self, line: str) -> None:
        """
        Read and publish individual lines of data from the serial port.

        Assumes data comes in the following format:
            P:0.22
            P:0.23
            P:0.22
            P:0.22
            V:15.85
            P:0.24
            ...

        Args:
            line (str): A line of data from the serial port
        """
        tag = line[0:2]  # P for pressure and V for voltage
        data = line[2:]
        if 'P:' in tag:
            self._update_pressure(float(data))  # Filter out bad readings
            self._parse_pressure()  # Parse pressure data
            self._publish_current_pressure_msg()  # Publish pressure data
        if 'V:' in tag:
            self._current_voltage_msg = Float64(data=float(data))
            self._publish_current_voltage_msg()

    def _update_pressure(self, new_reading: float) -> None:
        """
        Update pressure reading to publish and filter out bad readings.

        Args:
            new_reading (float): New pressure value to be published
        """
        # Ignore readings that are too large
        if abs(new_reading) > self.MAX_ABS_PRESSURE:
            return

        # First reading
        if self._pressure is None:
            self._pressure = new_reading
            self._previous_pressure = [new_reading] * self.MEDIAN_FILTER_SIZE

        # Median filter
        else:
            self._previous_pressure.append(new_reading)
            self._previous_pressure.pop(0)
            self._pressure = sorted(self._previous_pressure)[int(self.MEDIAN_FILTER_SIZE / 2)]

    def _parse_pressure(self) -> None:
        """Parse the pressure into an odom message."""
        self._current_pressure_msg.pose.pose.position.x = 0.0
        self._current_pressure_msg.pose.pose.position.y = 0.0
        self._current_pressure_msg.pose.pose.position.z = -1 * float(self._pressure)

        self._current_pressure_msg.pose.pose.orientation.x = 0.0
        self._current_pressure_msg.pose.pose.orientation.y = 0.0
        self._current_pressure_msg.pose.pose.orientation.z = 0.0
        self._current_pressure_msg.pose.pose.orientation.w = 1.0

        # Only the z,z covariance
        self._current_pressure_msg.pose.covariance[14] = 0.01

    def _publish_current_pressure_msg(self) -> None:
        """Publish current pressure to ROS node."""
        if abs(self._current_pressure_msg.pose.pose.position.z) > self.MAX_ABS_PRESSURE:
            self._current_pressure_msg = PoseWithCovarianceStamped()
            return

        self._current_pressure_msg.header.stamp = self.get_clock().now().to_msg()
        self._current_pressure_msg.header.frame_id = 'odom'  # World frame

        self._pub_depth.publish(self._current_pressure_msg)
        self._current_pressure_msg = PoseWithCovarianceStamped()

    def _publish_current_voltage_msg(self) -> None:
        """Publish current voltage to ROS node."""
        self._pub_voltage.publish(self._current_voltage_msg)


def main(args: list[str] | None = None) -> None:
    """Create and run the pressure and voltage publisher node."""
    rclpy.init(args=args)
    pressure_voltage_pub = PressureVoltagePublisher()

    try:
        rclpy.spin(pressure_voltage_pub)
    except KeyboardInterrupt:
        pass
    finally:
        pressure_voltage_pub.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
