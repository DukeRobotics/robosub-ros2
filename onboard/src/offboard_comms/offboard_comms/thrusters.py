import csv
import os
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import ClassVar

import rclpy
import resource_retriever as rr
import yaml
from ament_index_python.packages import get_package_share_directory
from custom_msgs.msg import PWMAllocs, ThrusterAllocs
from std_msgs.msg import Float64

from offboard_comms.serial_node import SerialNode


@dataclass
class VoltageTable:
    """
    Container for voltage and its corresponding PWM lookup table.

    Attributes:
        voltage (float): Voltage level for this table
        table (list[int]): PWM lookup values for this voltage level
    """
    voltage: float
    table: list[int]


class Thrusters(SerialNode):
    """
    ROS2 node for thruster control with voltage-based PWM allocation and serial communication.

    This node handles:
    1. Voltage-based PWM lookup and interpolation
    2. Serial communication with thruster hardware
    3. Subscription to thruster allocations and voltage
    4. Publishing of PWM values
    """
    NODE_NAME = 'thrusters'
    BAUDERATE = 57600
    SERIAL_DEVICE_NAME = 'thruster arduino'
    CONNECTION_RETRY_PERIOD = 1.0  # seconds

    CONTROLS_CONFIG_FILE_PATH = f'package://controls/config/{os.getenv("ROBOT_NAME", "oogway")}.yaml'
    OFFBOARD_COMMS_CONFIG_FILE_PATH = f'package://offboard_comms/config/{os.getenv("ROBOT_NAME", "oogway")}.yaml'
    ARDUINO_NAME = 'thruster'
    NUM_LOOKUP_ENTRIES = 201  # -1.0 to 1.0 in 0.01 increments
    VOLTAGE_FILES: ClassVar[list[tuple[int, str]]] = [
        (14.0, '14.csv'),
        (16.0, '16.csv'),
        (18.0, '18.csv'),
    ]
    START_FLAG = bytearray([0xFF, 0xFF])
    MIN_ALLOC = -1.0
    MAX_ALLOC = 1.0
    STOP_PWM = 1500

    def __init__(self) -> None:
        """Initialize the thruster node with all necessary components."""
        super().__init__(self.NODE_NAME, self.BAUDERATE, self.OFFBOARD_COMMS_CONFIG_FILE_PATH, self.SERIAL_DEVICE_NAME,
                         False, self.CONNECTION_RETRY_PERIOD)

        with Path(rr.get_filename(self.CONTROLS_CONFIG_FILE_PATH, use_protocol=False)).open() as f:
            controls_config = yaml.safe_load(f)
            self.num_thrusters = len(controls_config['thrusters'])

        # Initialize voltage, voltage bounds, and lookup tables
        self.voltage: float = 15.5
        self.voltage_min = min(v for v, _ in self.VOLTAGE_FILES)
        self.voltage_max = max(v for v, _ in self.VOLTAGE_FILES)
        self.voltage_tables: list[VoltageTable] = []

        # Load lookup tables
        self.load_lookup_tables()

        # Create subscribers
        self.create_subscription(Float64, '/sensors/voltage', self.voltage_callback, 1)

        self.create_subscription(ThrusterAllocs, '/controls/thruster_allocs', self.thruster_allocs_callback, 1)

        # Create publisher
        self.pwm_publisher = self.create_publisher(PWMAllocs, '/offboard/pwm', 1)

    def get_ftdi_string(self) -> str:
        """
        Get the FTDI string for the Thruster Arduino.

        Returns:
            str: FTDI string for the Thruster Arduino.
        """
        return self._config['arduino'][self.ARDUINO_NAME]['ftdi']

    def load_lookup_tables(self) -> None:
        """Load voltage-based PWM lookup tables from CSV files."""
        try:
            package_path = Path(get_package_share_directory('offboard_comms'))
            data_path = package_path / 'data'

            # Load all voltage tables and sort by voltage
            self.voltage_tables = [
                VoltageTable(voltage=v, table=self._read_lookup_table_csv(data_path / fname))
                for v, fname in self.VOLTAGE_FILES
            ]
            self.voltage_tables.sort(key=lambda x: x.voltage)

        except Exception as e:
            self.get_logger().error(f'Failed to load lookup tables: {e}')
            raise

    def _read_lookup_table_csv(self, filepath: Path) -> list[int]:
        """
        Read a lookup table from a CSV file.

        Args:
            filepath (Path): Path to the CSV file containing the lookup table.

        Returns:
            list[int]: Array containing the PWM values.
        """
        lookup_table = [0] * self.NUM_LOOKUP_ENTRIES

        with filepath.open() as file:
            next(file)  # Skip header
            reader = csv.reader(file)
            for row in reader:
                force = float(row[0])
                pwm = int(row[1])
                index = self._round_to_two_decimals(force)
                if 0 <= index < self.NUM_LOOKUP_ENTRIES:
                    lookup_table[index] = pwm

        return lookup_table

    def voltage_callback(self, msg: Float64) -> None:
        """
        Handle incoming voltage messages.

        Args:
            msg (Float64): Voltage message containing the current system voltage.
        """
        self.voltage = min(max(msg.data, self.voltage_min), self.voltage_max)
        if self.voltage != msg.data:
            self.get_logger().warn(
                f'Voltage {msg.data} out of bounds. Clamped to [{self.voltage_min}, {self.voltage_max}]',
            )

    def thruster_allocs_callback(self, msg: ThrusterAllocs) -> None:
        """
        Handle incoming thruster allocation messages.

        Args:
            msg (ThrusterAllocs): Thruster allocations message containing desired thrust values.
        """
        if len(msg.allocs) != self.num_thrusters:
            self.get_logger().error(
                f'Incorrect number of thruster allocations. Expected {self.num_thrusters}, got {len(msg.allocs)}.',
            )
            return

        # Convert allocations to PWM values
        pwm_values = [self._lookup(alloc) for alloc in msg.allocs]

        # Send to serial if connection is available
        self._write_pwms_to_serial(pwm_values)

        # Create and publish PWM message
        pwm_msg = PWMAllocs()
        pwm_msg.header.stamp = self.get_clock().now().to_msg()
        pwm_msg.allocs = pwm_values
        self.pwm_publisher.publish(pwm_msg)

    def _lookup(self, alloc: float) -> int:
        """
        Look up PWM value for a given thruster allocation and current voltage.

        Args:
            alloc (float): Thruster allocation value.

        Returns:
            int: Interpolated PWM value.
        """
        if not (self.MIN_ALLOC <= alloc <= self.MAX_ALLOC):
            self.get_logger().error(f'Thruster allocation {alloc} out of bounds [{self.MIN_ALLOC}, {self.MAX_ALLOC}]')
            return self.STOP_PWM  # Stop thruster if alloc is out of bounds

        # Ensure index is within bounds
        index = min(max(self._round_to_two_decimals(alloc), 0), self.NUM_LOOKUP_ENTRIES - 1)

        # Find the voltage tables to interpolate between
        for i in range(len(self.voltage_tables) - 1):
            v1, v2 = self.voltage_tables[i], self.voltage_tables[i + 1]
            if v1.voltage <= self.voltage < v2.voltage:
                return int(self._interpolate(
                    v1.voltage, v1.table[index],
                    v2.voltage, v2.table[index],
                    self.voltage,
                ))

        # If voltage is exactly at the highest level, use the last table
        return self.voltage_tables[-1].table[index]

    def _interpolate(self, x1: float, y1: int, x2: float, y2: int, x: float) -> float:
        """
        Perform linear interpolation.

        Args:
            x1 (float): First x value
            y1 (int): First y value
            x2 (float): Second x value
            y2 (int): Second y value
            x (float): X value to interpolate at

        Returns:
            float: Interpolated value
        """
        return y1 + ((y2 - y1) * (x - x1)) / (x2 - x1)

    def _round_to_two_decimals(self, num: float) -> int:
        """
        Round a number to two decimal places and convert to lookup table index.

        Args:
            num (float): Number to round

        Returns:
            int: Index for lookup table
        """
        return int(round(num * 100) + 100)

    def _write_pwms_to_serial(self, pwm_values: list[int]) -> None:
        """
        Write PWM values to serial port with checksum.

        Args:
            pwm_values (list[int]): Array of PWM values to write
        """
        # Start flag
        data = self.START_FLAG.copy()

        # Add PWM values in big-endian format
        data.extend(struct.pack(f'>{len(pwm_values)}H', *pwm_values))

        # Write to serial
        self.writebytes(bytes(data))


def main(args: list[str] | None = None) -> None:
    """
    Run main entry point for the thrusters node.

    Args:
        args (list[str] | None): Command-line arguments.
    """
    rclpy.init(args=args)
    node = Thrusters()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
