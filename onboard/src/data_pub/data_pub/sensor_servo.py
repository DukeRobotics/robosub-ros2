import os

import rclpy
from std_msgs.msg import Float64
from std_srvs.srv import SetBool

from data_pub.serial_republisher_node import SerialRepublisherNode


class SensorServoPublisher(SerialRepublisherNode):
    """Serial publisher to publish temperature and humidity data to ROS."""

    CONFIG_FILE_PATH = f'package://data_pub/config/{os.getenv("ROBOT_NAME", "oogway")}.yaml'

    BAUDRATE = 9600
    NODE_NAME = 'servo_sensors_pub'
    HUMIDITY_DEST_TOPIC = 'sensors/humidity'
    TEMPERATURE_DEST_TOPIC = 'sensors/temperature'
    SERVO_SERVICE = 'servo_control'

    CONNECTION_RETRY_PERIOD = 1.0 #S
    LOOP_RATE = 2.0 #Hz
    MEDIAN_FILTER_SIZE = 3
    MAX_ABS_TEMPERATURE = 200
    MAX_ABS_HUMIDITY = 200

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME, self.BAUDRATE, self.CONFIG_FILE_PATH, 'servo', self.CONNECTION_RETRY_PERIOD,
                         self.LOOP_RATE, use_nonblocking=True)

        self._temperature = None  # Temperature to publish
        self._humidity = None
        self._previous_temperature = None  # Previous temperature readings for median filter

        self._pub_temperature = self.create_publisher(Float64, self.TEMPERATURE_DEST_TOPIC, 50)
        self._pub_humidity = self.create_publisher(Float64, self.HUMIDITY_DEST_TOPIC, 50)

        self._servo_service = self.create_service(SetBool, self.SERVO_SERVICE, self.servo_control)

        self._current_temperature_msg = Float64()
        self._current_humidity_msg = Float64()

    def servo_control(self, req: SetBool.Request) -> SetBool.Response:
        """
        Respond to servo control service request.

        @param req: the request to control the servo
        """
        self.get_logger().info(f'Setting servo to {"left" if req.data else "right"}.')
        if req.data:
            self.writeline('L')
        else:
            self.writeline('R')
        return SetBool.Response(True, f'Successfully set servo to {"left" if req.data else "right"}.')

    def process_line(self, line: str) -> None:
        """
        Read and publish individual lines.

        Assumes data comes in the following format:
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
        tag = line[0:2]  # T for temperature and H for humidity
        data = line[2:]
        if data == '':
            return
        if 'T:' in tag:
            self._update_temperature(float(data))  # Filter out bad readings
            self._publish_current_temperature_msg()  # Publish temperature data
        if 'H:' in tag:
            self._update_humidity(float(data))  # Filter out bad readings
            self._publish_current_humidity_msg()  # Publish humidity data

    def _update_temperature(self, new_reading: float) -> None:
        """
        Update temperature reading to publish and filter out bad readings.

        Args:
            new_reading (float): New temperature value to be printed
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
        Update humidity reading to publish and filter out bad readings.

        Args:
            new_reading (float): New humidity value to be printed
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
        """Publish current temperature to ROS node."""
        self._current_temperature_msg.data = self._temperature
        self._pub_temperature.publish(self._current_temperature_msg)

    def _publish_current_humidity_msg(self) -> None:
        """Publish current humidity to ROS node."""
        self._current_humidity_msg.data = self._humidity
        self._pub_humidity.publish(self._current_humidity_msg)


def main(args: list[str] | None = None) -> None:
    """Create and run the sensor servo publisher node."""
    rclpy.init(args=args)
    sensor_servo = SensorServoPublisher()

    try:
        rclpy.spin(sensor_servo)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_servo.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()
