import os

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64
from std_srvs.srv import SetBool

from data_pub.serial_republisher_node import SerialRepublisherNode

CONFIG_FILE_PATH = f'package://offboard_comms/config/{os.getenv("ROBOT_NAME", "oogway")}.yaml'
CONFIG_NAME = 'peripheral'

BAUDRATE = 9600
NODE_NAME = 'peripheral_pub'
DEPTH_DEST_TOPIC = 'sensors/depth'
VOLTAGE_DEST_TOPIC = 'sensors/voltage'
HUMIDITY_DEST_TOPIC = 'sensors/humidity'
TEMPERATURE_DEST_TOPIC = 'sensors/temperature'
SERVO_SERVICE = 'servo_control'


class PeripheralPublisher(SerialRepublisherNode):
    """Serial publisher to publish data from peripheral arduino and send servo commands."""

    MAX_ABS_PRESSURE = 7
    MAX_ABS_TEMPERATURE = 200
    MAX_ABS_HUMIDITY = 200
    MEDIAN_FILTER_SIZE = 3

    def __init__(self) -> None:
        super().__init__(NODE_NAME, BAUDRATE, CONFIG_FILE_PATH, CONFIG_NAME)

        # Latest readings, after filtering
        self._pressure = None
        self._voltage = None
        self._temperature = None
        self._humidity = None

        # Previous readings; used in median filter
        self._previous_pressure = None
        self._previous_temperature = None
        self._previous_humidity = None

        # Messages
        self._current_pressure_msg = PoseWithCovarianceStamped()
        self._current_voltage_msg = Float64()
        self._current_temperature_msg = Float64()
        self._current_humidity_msg = Float64()

        # Publishers
        self._pub_depth = self.create_publisher(PoseWithCovarianceStamped, DEPTH_DEST_TOPIC, 50)
        self._pub_voltage = self.create_publisher(Float64, VOLTAGE_DEST_TOPIC, 10)
        self._pub_temperature = self.create_publisher(Float64, TEMPERATURE_DEST_TOPIC, 10)
        self._pub_humidity = self.create_publisher(Float64, HUMIDITY_DEST_TOPIC, 10)

        # Service for servo control
        self._servo_service = self.create_service(SetBool, SERVO_SERVICE, self.servo_control)

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
            T:69.1
            H:30.1
            T:69.2
            H:27.8
            T:69.1
            H:27.8
            T:69.8
            ...

        Args:
            line (str): A line of data from the serial port
        """
        tag = line[0:2]
        data = line[2:]
        if data == '':
            return
        if 'P:' in tag:
            self._update_pressure(float(data))  # Filter out bad readings
            self._parse_pressure()  # Parse pressure data
            self._publish_current_pressure_msg()  # Publish pressure data
        if 'V:' in tag:
            self._update_voltage(float(data))
            self._publish_current_voltage_msg()
        if 'T:' in tag:
            self._update_temperature(float(data))  # Filter out bad readings
            self._publish_current_temperature_msg()  # Publish temperature data
        if 'H:' in tag:
            self._update_humidity(float(data))  # Filter out bad readings
            self._publish_current_humidity_msg()  # Publish humidity data

    def _update_pressure(self, new_reading: float) -> None:
        """
        Update self._pressure with new reading and filter out bad readings.

        Args:
            new_reading (float): New pressure value recieved.
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
        """Publish current pressure."""
        self._current_pressure_msg.header.stamp = self.get_clock().now().to_msg()
        self._current_pressure_msg.header.frame_id = 'odom'  # World frame

        self._pub_depth.publish(self._current_pressure_msg)
        self._current_pressure_msg = PoseWithCovarianceStamped()

    def _update_voltage(self, new_reading: float) -> None:
        """
        Update self._voltage with new reading.

        Args:
            new_reading (float): New voltage value recieved.
        """
        self._voltage = new_reading

    def _publish_current_voltage_msg(self) -> None:
        """Publish current voltage."""
        self._current_voltage_msg.data = self._voltage
        self._pub_voltage.publish(self._current_voltage_msg)

    def _update_temperature(self, new_reading: float) -> None:
        """
        Update self._temperature with new reading and filter out bad readings.

        Args:
            new_reading (float): New temperature value recieved.
        """
        # Ignore readings that are too large
        if abs(new_reading) > self.MAX_ABS_TEMPERATURE:
            return

        # First reading
        if self._temperature is None:
            self._temperature = new_reading
            self._previous_temperature = [new_reading] * self.MEDIAN_FILTER_SIZE

        # Median filter
        else:
            self._previous_temperature.append(new_reading)
            self._previous_temperature.pop(0)
            self._temperature = sorted(self._previous_temperature)[int(self.MEDIAN_FILTER_SIZE / 2)]

    def _update_humidity(self, new_reading: float) -> None:
        """
        Update self._humidity with new reading and filter out bad readings.

        Args:
            new_reading (float): New humidity value recieved.
        """
        # Ignore readings that are too large
        if abs(new_reading) > self.MAX_ABS_HUMIDITY:
            return

        # First reading
        if self._humidity is None:
            self._humidity = new_reading
            self._previous_humidity = [new_reading] * self.MEDIAN_FILTER_SIZE

        # Median filter
        else:
            self._previous_humidity.append(new_reading)
            self._previous_humidity.pop(0)
            self._humidity = sorted(self._previous_humidity)[int(self.MEDIAN_FILTER_SIZE / 2)]

    def _publish_current_temperature_msg(self) -> None:
        """Publish current temperature."""
        self._current_temperature_msg.data = self._temperature
        self._pub_temperature.publish(self._current_temperature_msg)

    def _publish_current_humidity_msg(self) -> None:
        """Publish current humidity."""
        self._current_humidity_msg.data = self._humidity
        self._pub_humidity.publish(self._current_humidity_msg)

    def servo_control(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        """
        Service callback to control the servo.

        Args:
            request (SetBool.Request): The request to control the servo.
            response (SetBool.Response): The response message.
        """
        if request.data:
            self.writeline('L')
        else:
            self.writeline('R')

        # Set response attributes explicitly (correct way in ROS 2)
        response.success = True
        response.message = f'Successfully set servo to {"left" if request.data else "right"}.'
        return response


def main(args: list[str] | None = None) -> None:
    """Create and run the pressure and voltage publisher node."""
    rclpy.init(args=args)
    pressure_voltage_pub = PeripheralPublisher()

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
