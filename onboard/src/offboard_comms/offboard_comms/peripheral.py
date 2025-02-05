import os
from dataclasses import dataclass
from typing import ClassVar

import rclpy
from custom_msgs.srv import SetServo

from offboard_comms.peripheral_sensors import (
    HumiditySensor,
    PeripheralSensor,
    PressureSensor,
    TemperatureSensor,
    VoltageSensor,
)
from offboard_comms.serial_republisher_node import SerialRepublisherNode


@dataclass
class PeripheralServo:
    """
    Dataclass to store servo information.

    Attributes:
        name (str): Human-readable name of the servo.
        tag (str): The tag to identify the servo.
        min_pwm (int): The minimum servo PWM.
        max_pwm (int): The maximum servo PWM.
    """
    name: str
    tag: str
    min_pwm: int
    max_pwm: int


class PeripheralPublisher(SerialRepublisherNode):
    """Serial publisher to publish data from peripheral arduino and send servo commands."""

    NAME = 'peripheral arduino'
    CONFIG_FILE_PATH = f'package://offboard_comms/config/{os.getenv("ROBOT_NAME", "oogway")}.yaml'

    BAUDRATE = 9600
    NODE_NAME = 'peripheral'
    ARDUINO_NAME = 'peripheral'
    SERVO_SERVICE = 'servo_control'
    CONNECTION_RETRY_PERIOD = 1.0  # seconds
    LOOP_RATE = 50.0  # Hz
    SENSOR_CLASSES: ClassVar[dict[str, PeripheralSensor]] = {
        'pressure': PressureSensor,
        'voltage': VoltageSensor,
        'temperature': TemperatureSensor,
        'humidity': HumiditySensor,
    }

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME, self.BAUDRATE, self.CONFIG_FILE_PATH, self.NAME,
                         self.CONNECTION_RETRY_PERIOD, self.LOOP_RATE, use_nonblocking=True)

        self.sensors: dict[str, PeripheralSensor] = {}
        self.setup_sensors()

        self.servos: dict[str, PeripheralServo] = {}
        self.setup_servos()

        if self.servos:
            self._servo_service = self.create_service(SetServo, self.SERVO_SERVICE, self.servo_control)

    def get_ftdi_string(self) -> str:
        """Get the FTDI string for the Peripheral Arduino."""
        return self._config['arduino'][self.ARDUINO_NAME]['ftdi']

    def setup_sensors(self) -> None:
        """Initialize sensor classes based on the config file."""
        for sensor in self._config['arduino'][self.ARDUINO_NAME]['sensors']:
            sensor_class = self.SENSOR_CLASSES.get(sensor['type'])
            if sensor_class:
                self.sensors[sensor['tag']] = sensor_class(self, sensor['tag'], sensor['topic'])
            else:
                self.get_logger().error(f'Invalid sensor type: {sensor["type"]}')

    def setup_servos(self) -> None:
        """Initialize servo classes based on the config file."""
        for servo in self._config['arduino'][self.ARDUINO_NAME]['servos']:
            self.servos[servo['tag']] = PeripheralServo(servo['name'], servo['tag'], servo['min_pwm'], servo['max_pwm'])

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
        tag, data = line.split(':', 1)
        if data == '':
            return
        if tag in self.sensors:
            self.sensors[tag].update_and_publish_value(float(data))
        else:
            self.get_logger().error(f'Invalid tag: {tag}')

    def servo_control(self, request: SetServo.Request, response: SetServo.Response) -> SetServo.Response:
        """
        Service callback to control the servo.

        Args:
            request (SetServo.Request): The request to control the servo.
            response (SetServo.Response): The response message.
        """
        if request.tag in self.servos:
            pwm = request.pwm
            servo = self.servos[request.tag]

            if servo.min_pwm <= pwm <= servo.max_pwm:
                self.writeline(f'{request.tag}:{pwm}')

                response.success = True
                response.message = f'Successfully set {servo.name} servo to PWM: {pwm}'
            else:
                error_msg = (f'Invalid PWM value {pwm} for {servo.name} servo. Must be between {servo.min_pwm} and '
                             f'{servo.max_pwm}.')

                response.success = False
                response.message = error_msg

                self.get_logger().error(error_msg)
        else:
            error_msg = f'Invalid servo tag: {request.tag}'

            response.success = False
            response.message = error_msg

            self.get_logger().error(error_msg)

        return response


def main(args: list[str] | None = None) -> None:
    """Create and run the peripheral publisher node."""
    rclpy.init(args=args)
    peripheral = PeripheralPublisher()

    try:
        rclpy.spin(peripheral)
    except KeyboardInterrupt:
        pass
    finally:
        peripheral.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
