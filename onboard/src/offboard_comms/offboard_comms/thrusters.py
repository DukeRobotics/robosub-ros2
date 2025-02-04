import csv
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import ClassVar

import rclpy
import serial
from ament_index_python.packages import get_package_share_directory
from custom_msgs.msg import PWMAllocs, ThrusterAllocs
from rclpy.node import Node
from std_msgs.msg import Float64


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


class Thrusters(Node):
    """
    ROS2 node for thruster control with voltage-based PWM allocation and serial communication.

    This node handles:
    1. Voltage-based PWM lookup and interpolation
    2. Serial communication with thruster hardware
    3. Subscription to thruster allocations and voltage
    4. Publishing of PWM values
    """

    VOLTAGE_MIN: float = 14.0
    VOLTAGE_MAX: float = 18.0
    NUM_LOOKUP_ENTRIES: int = 201  # -1.0 to 1.0 in 0.01 increments
    VOLTAGE_FILES: ClassVar[list[tuple[int, str]]] = [
        (14.0, '14.csv'),
        (16.0, '16.csv'),
        (18.0, '18.csv'),
    ]
    START_FLAG: bytearray = bytearray([0xFF, 0xFF])

    def __init__(self) -> None:
        """Initialize the thruster node with all necessary components."""
        super().__init__('thrusters')

        self.serial_port = self.declare_parameter('serial_port', 'device not found').value

        # Initialize voltage and lookup tables
        self.voltage: float = 15.5
        self.voltage_tables: list[VoltageTable] = []

        # Load lookup tables
        self.load_lookup_tables()

        # Initialize serial connection
        self.setup_serial_connection()

        # Create subscribers
        self.create_subscription(Float64, '/sensors/voltage', self.voltage_callback, 1)

        self.create_subscription(ThrusterAllocs, '/controls/thruster_allocs', self.thruster_allocs_callback, 1)

        # Create publisher
        self.pwm_publisher = self.create_publisher(PWMAllocs, '/offboard/pwm', 1)

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

    def setup_serial_connection(self) -> None:
        """Set up the serial connection with the thruster hardware."""
        try:
            device = self.declare_parameter('device', self.serial_port).value
            self.ser = serial.Serial(port=device, baudrate=57600, timeout=1.0)
            self.get_logger().info(f'Connected to thruster arduino at {device}.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to initialize serial connection: {e}')
            self.ser = None
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
        self.voltage = min(max(msg.data, self.VOLTAGE_MIN), self.VOLTAGE_MAX)
        if self.voltage != msg.data:
            self.get_logger().warn(
                f'Voltage {msg.data} out of bounds. Clamped to [{self.VOLTAGE_MIN}, {self.VOLTAGE_MAX}]',
            )

    def thruster_allocs_callback(self, msg: ThrusterAllocs) -> None:
        """
        Handle incoming thruster allocation messages.

        Args:
            msg (ThrusterAllocs): Thruster allocations message containing desired thrust values.
        """
        # Convert allocations to PWM values
        pwm_values = [self._lookup(force) for force in msg.allocs]

        # Send to serial if connection is available
        if self.ser and self.ser.is_open:
            self._write_to_serial(pwm_values)
        else:
            self.get_logger().warn('Serial connection not available. PWM values not sent.')

        # Create and publish PWM message
        pwm_msg = PWMAllocs()
        pwm_msg.header.stamp = self.get_clock().now().to_msg()
        pwm_msg.allocs = pwm_values
        self.pwm_publisher.publish(pwm_msg)

    def _lookup(self, force: float) -> int:
        """
        Look up PWM value for a given force and current voltage.

        Args:
            force (float): Desired force value in range [-1.0, 1.0].

        Returns:
            int: Interpolated PWM value.
        """
        if not (-1.0 <= force <= 1.0):
            self.get_logger().error(f'Force {force} out of bounds [-1.0, 1.0]')
            return 1500  # Safe middle value

        # Ensure index is within bounds
        index = min(max(self._round_to_two_decimals(force), 0), self.NUM_LOOKUP_ENTRIES - 1)

        # Find the voltage tables to interpolate between
        for i in range(len(self.voltage_tables) - 1):
            v1, v2 = self.voltage_tables[i], self.voltage_tables[i + 1]
            if v1.voltage <= self.voltage <= v2.voltage:
                return int(self._interpolate(
                    v1.voltage, v1.table[index],
                    v2.voltage, v2.table[index],
                    self.voltage,
                ))

        # If we're exactly at the highest voltage, use that table
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

    def _write_to_serial(self, pwm_values: list[int]) -> None:
        """
        Write PWM values to serial port with checksum.

        Args:
            pwm_values (list[int]): Array of PWM values to write
        """
        try:
            # Start flag
            data = self.START_FLAG.copy()

            # Add PWM values in big-endian format
            data.extend(struct.pack(f'>{len(pwm_values)}H', *pwm_values))

            # Write to serial
            self.ser.write(data)

        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Error writing to serial: {e}')

    def destroy_node(self) -> None:
        """Clean up resources when node is destroyed."""
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


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
