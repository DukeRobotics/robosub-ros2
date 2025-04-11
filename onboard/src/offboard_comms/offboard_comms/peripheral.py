import functools
import os
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, ClassVar

import rclpy
from custom_msgs.srv import SetContinuousServo, SetDiscreteServo
from rclpy.service import Service

from offboard_comms.peripheral_sensors import (
    HumiditySensor,
    PeripheralSensor,
    PressureSensor,
    TemperatureSensor,
    VoltageSensor,
)
from offboard_comms.serial_node import SerialNode, SerialReadType


@dataclass
class PeripheralDiscreteServo:
    """
    Dataclass to store discrete servo information.

    Attributes:
        name (str): Human-readable name of the servo.
        tag (str): The tag to identify the servo.
        service_name (str): The name of the service to control the servo.
        states (dict[str, int]): Mapping of servo states to PWM values.
        min_delay (float): Minimum delay between servo commands, in seconds.
        service (Service): The service to control the servo.
        last_called_time (float): The time the servo was last called, in seconds since epoch.
    """
    name: str
    tag: str
    service_name: str
    states: dict[str, int]
    min_delay: float
    service: Service
    last_called_time: float = 0.0

@dataclass
class PeripheralContinuousServo:
    """
    Dataclass to store continuous servo information.

    Attributes:
        name (str): Human-readable name of the servo.
        tag (str): The tag to identify the servo.
        service_name (str): The name of the service to control the servo.
        min_pwm (int): Minimum PWM value for the servo.
        max_pwm (int): Maximum PWM value for the servo.
        min_delay (float): Minimum delay between servo commands, in seconds.
        service (Service): The service to control the servo.
        last_called_time (float): The time the servo was last called, in seconds since epoch.
    """
    name: str
    tag: str
    service_name: str
    min_pwm: int
    max_pwm: int
    min_delay: float
    service: Service
    last_called_time: float = 0.0

@dataclass
class ServoTypeInfo:
    """
    Dataclass to store types and functions to use based on servo type.

    Attributes:
        servo_dataclass (type[PeripheralDiscreteServo] | type[PeripheralContinuousServo]): The servo data class to use.
        service_msg_type (type[SetDiscreteServo] | type[SetContinuousServo]): The service message type to use.
        callback (callable): The service callback function to use.
    """
    servo_dataclass: type[PeripheralDiscreteServo] | type[PeripheralContinuousServo]
    service_msg_type: type[SetDiscreteServo] | type[SetContinuousServo]
    callback: Callable[[SetDiscreteServo.Request, SetDiscreteServo.Response, str], SetDiscreteServo.Response] | \
                Callable[[SetContinuousServo.Request, SetContinuousServo.Response, str], SetContinuousServo.Response]


class PeripheralPublisher(SerialNode):
    """Serial publisher to publish data from peripheral arduino and send servo commands."""

    SERIAL_DEVICE_NAME = 'peripheral arduino'
    CONFIG_FILE_PATH = f'package://offboard_comms/config/{os.getenv("ROBOT_NAME")}.yaml'

    BAUDRATE = 9600
    NODE_NAME = 'peripheral'
    ARDUINO_NAME = 'peripheral'
    CONNECTION_RETRY_PERIOD = 1.0  # seconds
    LOOP_RATE = 50.0  # Hz
    SENSOR_CLASSES: ClassVar[dict[str, type[PeripheralSensor]]] = {
        'pressure': PressureSensor,
        'voltage': VoltageSensor,
        'temperature': TemperatureSensor,
        'humidity': HumiditySensor,
    }

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME, self.BAUDRATE, self.CONFIG_FILE_PATH, self.SERIAL_DEVICE_NAME,
                         SerialReadType.LINE_NONBLOCKING, self.CONNECTION_RETRY_PERIOD, self.LOOP_RATE)

        self.sensors: dict[str, PeripheralSensor] = {}
        self.setup_sensors()

        self.SERVO_TYPES = {
            'discrete': ServoTypeInfo(PeripheralDiscreteServo, SetDiscreteServo, self.discrete_servo),
            'continuous': ServoTypeInfo(PeripheralContinuousServo, SetContinuousServo, self.continuous_servo),
        }
        self.servos: dict[str, PeripheralDiscreteServo | PeripheralContinuousServo] = {}
        self.setup_servos()

    def get_ftdi_string(self) -> str:
        """
        Get the FTDI string for the Peripheral Arduino.

        Returns:
            str: FTDI string for the Peripheral Arduino.
        """
        return self._config['arduino'][self.ARDUINO_NAME]['ftdi']

    def setup_sensors(self) -> None:
        """Initialize sensor classes based on the config file."""
        for sensor in self._config['arduino'][self.ARDUINO_NAME].get('sensors', []):
            if sensor['tag'] in self.sensors:
                error_msg = f'Duplicate sensor tag: {sensor["tag"]}'
                raise ValueError(error_msg)

            sensor_class = self.SENSOR_CLASSES.get(sensor['type'])
            if sensor_class:
                self.sensors[sensor['tag']] = sensor_class(self, sensor['tag'], sensor['topic'])
            else:
                error_msg = f'Invalid sensor type: {sensor["type"]}'
                raise ValueError(error_msg)

    def setup_servos(self) -> None:
        """Initialize servo classes based on the config file."""
        for servo in self._config['arduino'][self.ARDUINO_NAME].get('servos', []):
            if servo['tag'] in self.servos:
                error_msg = f'Duplicate servo tag: {servo["tag"]}'
                raise ValueError(error_msg)

            servo_type_info = self.SERVO_TYPES.get(servo['type'])
            if servo_type_info:
                del servo['type']  # Remove type from servo dict as it is not stored in the servo dataclass
                servo_callback_with_tag = functools.partial(servo_type_info.callback, tag=servo['tag'])
                service = self.create_service(servo_type_info.service_msg_type, servo['service_name'],
                                              servo_callback_with_tag)
                self.servos[servo['tag']] = servo_type_info.servo_dataclass(**servo, service=service)
            else:
                error_msg = f'Invalid servo type: {servo["type"]}'
                raise ValueError(error_msg)

    def process_line(self, line: str) -> None:
        """
        Read and publish individual lines of data from the serial port.

        Requires lines to come in the format 'tag:data', with one data point per line, where data is a float.

        Args:
            line (str): A line of data from the serial port.
        """
        if ':' not in line:
            self.get_logger().error(f'Invalid data format: "{line}"')
            return

        tag, data = line.split(':', 1)
        if data == '':
            self.get_logger().error(f'Empty data for tag: "{tag}"')
            return
        if tag in self.sensors:
            try:
                data_float = float(data)
            except ValueError:
                self.get_logger().error(f'Could not convert data to float: "{data}" for tag "{tag}"')
                return

            self.sensors[tag].update_and_publish_value(data_float)
        else:
            self.get_logger().error(f'Invalid tag: "{tag}"')

    def discrete_servo(self, request: SetDiscreteServo.Request, response: SetDiscreteServo.Response, tag: str) \
            -> SetDiscreteServo.Response:
        """
        Service callback to control discrete servos.

        Args:
            request (SetDiscreteServo.Request): The request to control the servo.
            response (SetDiscreteServo.Response): The response message.
            tag (str): The tag of the servo to control.
        """
        if tag not in self.servos:
            response.success = False
            response.message = f'Invalid tag: {tag}'
            return response

        state = request.state
        servo: PeripheralDiscreteServo = self.servos[tag]

        if time.time() - servo.last_called_time < servo.min_delay:
            error_msg = (f'Minimum delay of {servo.min_delay} seconds not met since last call to '
                         f'{self.servos[tag].name} servo.')

            response.success = False
            response.message = error_msg

            self.get_logger().error(error_msg)
            return response

        if state in servo.states:
            pwm = servo.states[state]
            if self.writeline(f'{tag}:{pwm}'):
                servo.last_called_time = time.time()
                response.success = True
                response.message = f'Successfully set {servo.name} servo to state: "{state}"'
            else:
                response.success = False
                response.message = f'Failed to set {servo.name} servo to state: "{state}". Error in writing to serial.'
        else:
            # Create a string of possible states for the error message; format: ["state1", "state2", ...]
            possible_states = '[' + ', '.join(f'"{possible_state}"' for possible_state in servo.states) + ']'
            error_msg = f'Invalid state "{state}" for {servo.name} servo. Must be one of {possible_states}.'

            response.success = False
            response.message = error_msg

            self.get_logger().error(error_msg)

        return response

    def continuous_servo(self, request: SetContinuousServo.Request, response: SetContinuousServo.Response, tag: str) \
            -> SetContinuousServo.Response:
        """
        Service callback to control continuous servos.

        Args:
            request (SetContinuousServo.Request): The request to control the servo.
            response (SetContinuousServo.Response): The response message.
            tag (str): The tag of the servo to control.
        """
        if tag not in self.servos:
            response.success = False
            response.message = f'Invalid tag: {tag}'
            return response

        pwm = request.pwm
        servo: PeripheralContinuousServo = self.servos[tag]

        if time.time() - servo.last_called_time < servo.min_delay:
            error_msg = (f'Minimum delay of {servo.min_delay} seconds not met since last call to '
                         f'{self.servos[tag].name} servo.')

            response.success = False
            response.message = error_msg

            self.get_logger().error(error_msg)
            return response

        if servo.min_pwm <= pwm <= servo.max_pwm:
            if self.writeline(f'{tag}:{pwm}'):
                servo.last_called_time = time.time()
                response.success = True
                response.message = f'Successfully set {servo.name} servo to PWM: {pwm}'
            else:
                response.success = False
                response.message = f'Failed to set {servo.name} servo to PWM: {pwm}. Error in writing to serial.'
        else:
            error_msg = (f'Invalid PWM value {pwm} for {servo.name} servo. Must be between {servo.min_pwm} and '
                            f'{servo.max_pwm} (both inclusive).')

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
