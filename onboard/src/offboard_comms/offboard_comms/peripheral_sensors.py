from abc import ABC, abstractmethod

from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import Float64


class PeripheralSensor(ABC):
    """Abstract class for peripheral sensors."""

    def __init__(self, node: Node, tag: str, topic: str, min_value: float, max_value: float,
                 median_filter_size: int) -> None:
        self.node = node
        self.tag = tag
        self.topic = topic
        self.min_value = min_value
        self.max_value = max_value
        self.median_filter_size = median_filter_size

        # Current sensor value, after filtering
        self._value: float = None

        # Most recent readings for median filter. Stored in order of arrival from oldest to newest.
        self._previous_values: list[float] = None

    def update_and_publish_value(self, new_reading: float) -> None:
        """
        Update sensor value with new reading, filter out bad readings, and publish the value.

        Args:
            new_reading (float): New sensor value recieved.
        """
        # Don't update value when new reading is too large
        if self.min_value < new_reading < self.max_value:
            if self.median_filter_size > 0:
                # First reading
                if self._value is None:
                    self._value = new_reading
                    self._previous_values = [new_reading] * self.median_filter_size

                # Median filter
                else:
                    self._previous_values.append(new_reading)
                    self._previous_values.pop(0)
                    self._value = sorted(self._previous_values)[int(self.median_filter_size / 2)]
            else:
                self._value = new_reading

        self._publish_current_value()

    @abstractmethod
    def _publish_current_value(self) -> None:
        """Publish current sensor value."""

class VoltageSensor(PeripheralSensor):
    """Voltage sensor class."""

    MIN_VALUE = 0
    MAX_VALUE = 30
    MEDIAN_FILTER_SIZE = 0

    def __init__(self, node: Node, tag: str, topic: str) -> None:
        super().__init__(node, tag, topic, self.MIN_VALUE, self.MAX_VALUE, self.MEDIAN_FILTER_SIZE)

        self._current_voltage_msg = Float64()
        self._publisher = self.node.create_publisher(Float64, self.topic, 10)

    def _publish_current_value(self) -> None:
        """Publish current voltage."""
        self._current_voltage_msg.data = self._value
        self._publisher.publish(self._current_voltage_msg)

class PressureSensor(PeripheralSensor):
    """Pressure sensor class."""

    MIN_VALUE = -7
    MAX_VALUE = 7
    MEDIAN_FILTER_SIZE = 3

    def __init__(self, node: Node, tag: str, topic: str) -> None:
        super().__init__(node, tag, topic, self.MIN_VALUE, self.MAX_VALUE, self.MEDIAN_FILTER_SIZE)

        self._current_pressure_msg = PoseWithCovarianceStamped()
        self._current_pressure_msg.header.frame_id = 'odom'  # World frame
        self._current_pressure_msg.pose.pose.position.x = 0.0
        self._current_pressure_msg.pose.pose.position.y = 0.0
        self._current_pressure_msg.pose.pose.orientation.x = 0.0
        self._current_pressure_msg.pose.pose.orientation.y = 0.0
        self._current_pressure_msg.pose.pose.orientation.z = 0.0
        self._current_pressure_msg.pose.pose.orientation.w = 1.0
        self._current_pressure_msg.pose.covariance[14] = 0.01   # Only the z,z covariance

        self._publisher = self.node.create_publisher(PoseWithCovarianceStamped, self.topic, 10)

    def _publish_current_value(self) -> None:
        """Convert pressure to odometry message and publish."""
        self._current_pressure_msg.header.stamp = self.node.get_clock().now().to_msg()
        self._current_pressure_msg.pose.pose.position.z = -1 * float(self._value)
        self._publisher.publish(self._current_pressure_msg)

class TemperatureSensor(PeripheralSensor):
    """Temperature sensor class."""

    MIN_VALUE = -200
    MAX_VALUE = 200
    MEDIAN_FILTER_SIZE = 3

    def __init__(self, node: Node, tag: str, topic: str) -> None:
        super().__init__(node, tag, topic, self.MIN_VALUE, self.MAX_VALUE, self.MEDIAN_FILTER_SIZE)

        self._current_temperature_msg = Float64()
        self._publisher = self.node.create_publisher(Float64, self.topic, 10)

    def _publish_current_value(self) -> None:
        """Publish current temperature."""
        self._current_temperature_msg.data = self._value
        self._publisher.publish(self._current_temperature_msg)

class HumiditySensor(PeripheralSensor):
    """Humidity sensor class."""

    MIN_VALUE = 0
    MAX_VALUE = 200
    MEDIAN_FILTER_SIZE = 3

    def __init__(self, node: Node, tag: str, topic: str) -> None:
        super().__init__(node, tag, topic, self.MIN_VALUE, self.MAX_VALUE, self.MEDIAN_FILTER_SIZE)

        self._current_humidity_msg = Float64()
        self._publisher = self.node.create_publisher(Float64, self.topic, 10)

    def _publish_current_value(self) -> None:
        """Publish current humidity."""
        self._current_humidity_msg.data = self._value
        self._publisher.publish(self._current_humidity_msg)
