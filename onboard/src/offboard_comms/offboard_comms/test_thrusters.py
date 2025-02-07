import argparse
import os
from pathlib import Path

import rclpy
import resource_retriever as rr
import yaml
from custom_msgs.msg import ThrusterAllocs
from rclpy.node import Node

MIN_SPEED = -1.0
MAX_SPEED = 1.0

DEFAULT_SPEED = 0.05
DEFAULT_RATE = 20.0

def validate_speed(value: str) -> float:
    """Validate that the speed is between MIN_SPEED and MAX_SPEED."""
    try:
        speed = float(value)
        if MIN_SPEED <= speed <= MAX_SPEED:
            return speed
        error_msg = f'Speed must be between {MIN_SPEED} and {MAX_SPEED}.'
        raise argparse.ArgumentTypeError(error_msg)
    except ValueError:
        error_msg = 'Invalid speed value.'
        raise argparse.ArgumentTypeError(error_msg) from None


class ThrusterTester(Node):
    """Node to publish thruster allocations. Publishes a constant speed for all thrusters at 20 Hz."""

    CONFIG_PATH = f'package://controls/config/{os.getenv("ROBOT_NAME", "oogway")}.yaml'

    def __init__(self, speed: float, rate: float, no_log_allocs: bool) -> None:
        """
        Initialize the thruster tester node.

        Args:
            speed (float): Speed to publish to all thrusters.
            rate (float): Publishing rate in Hz.
            no_log_allocs (bool): Disable logging of thruster allocs messages.
        """
        super().__init__('thruster_tester')
        self.publisher_ = self.create_publisher(ThrusterAllocs, '/controls/thruster_allocs', 10)
        self.num_thrusters = self.get_thruster_count()
        self.thrust_speed = speed
        self.allocs = [self.thrust_speed] * self.num_thrusters
        self.no_log_allocs = no_log_allocs
        self.timer = self.create_timer(1.0 / rate, self.publish_allocs)
        self.get_logger().info(f'Publishing {self.num_thrusters} thrusters at speed {self.thrust_speed} with rate '
                               f'{rate} Hz.')

    def get_thruster_count(self) -> int:
        """
        Read the controls config file to determine the number of thrusters on the robot.

        Returns:
            int: Number of thrusters.
        """
        with Path(rr.get_filename(self.CONFIG_PATH, use_protocol=False)).open() as f:
            controls_config = yaml.safe_load(f)
            return len(controls_config['thrusters'])

    def publish_allocs(self) -> None:
        """Publish the thruster allocations."""
        msg = ThrusterAllocs()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.allocs = self.allocs
        self.publisher_.publish(msg)
        if not self.no_log_allocs:
            self.get_logger().info(f'Published thruster allocs: {self.allocs}')


def main(args: list[str] | None = None) -> None:
    """
    Run main entry point for the thrusters node.

    Args:
        args (list[str] | None): Command-line arguments.
    """
    parser = argparse.ArgumentParser(description='Thruster Tester Node')
    parser.add_argument('-s', '--speed', type=validate_speed, default=DEFAULT_SPEED,
                        help=f'Thruster speed (default: {DEFAULT_SPEED})')
    parser.add_argument('-r', '--rate', type=float, default=DEFAULT_RATE,
                        help=f'Publishing rate in Hz (default: {DEFAULT_RATE})')
    parser.add_argument('--no-log-allocs', action='store_true', help='Disable logging of thruster allocs messages')
    parsed_args = parser.parse_args()

    rclpy.init(args=args)
    node = ThrusterTester(parsed_args.speed, parsed_args.rate, parsed_args.no_log_allocs)
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
