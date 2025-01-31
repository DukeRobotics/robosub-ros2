import rclpy
from abc import ABC, abstractmethod

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64

class PeripheralSensor(ABC):
    """Abstract class for peripheral sensors."""

    def __init__(self, node: rclpy.node.Node, tag: str, topic: str) -> None:
        self.node = node
        self.tag = tag
        self.topic = topic

    @abstractmethod
    def publish_value(self, new_reading: float) -> None:
        """
        Update sensor value with new reading, filter out bad readings, and publish the value.

        Args:
            new_reading (float): New sensor value recieved.
        """


class PressureSensor(PeripheralSensor):
    """Pressure sensor class."""

    MIN_VALUE = -7
    MAX_VALUE = 7
    MEDIAN_FILTER_SIZE = 3

    def __init__(self, node: rclpy.node.Node, tag: str, topic: str) -> None:
        super().__init__(node, tag, topic)

        self._pressure = None
        self._previous_pressure = None
        self.current_pressure_msg = PoseWithCovarianceStamped()

        self.publisher = self.node.create_publisher(PoseWithCovarianceStamped, self.topic, 10)

    def publish_value(self, new_reading: float) -> None:
        """Update pressure value with new reading, filter out bad readings, and publish the value."""
        self.update_value(new_reading)
        self.parse_press

    def update_value(self, new_reading: float) -> None:
        """
        Update self._pressure with new reading and filter out bad readings.

        Args:
            new_reading (float): New pressure value recieved.
        """
        # Ignore readings that are too large
        if new_reading < self.MIN_VALUE or new_reading > self.MAX_VALUE:
            return

        if self.MEDIAN_FILTER_SIZE > 0:
            # First reading
            if self._pressure is None:
                self._pressure = new_reading
                self._previous_pressure = [new_reading] * self.MEDIAN_FILTER_SIZE

            # Median filter
            else:
                self._previous_pressure.append(new_reading)
                self._previous_pressure.pop(0)
                self._pressure = sorted(self._previous_pressure)[int(self.MEDIAN_FILTER_SIZE / 2)]

        self._publish_current_pressure_msg()

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

