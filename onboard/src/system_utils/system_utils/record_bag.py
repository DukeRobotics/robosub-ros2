import os
import signal
import subprocess
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float64


class RecordBag(Node):

    # Duration of time to wait for a voltage message before stopping recording
    TIMEOUT_DURATION = Duration(seconds=5)
    NODE_NAME = 'record_bag'

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME)

        # Initialize variables
        self.process = None
        bypass = self.declare_parameter('bypass', False).value
        self.enable_recording = self.declare_parameter('enable_recording', False).value

        if not self.enable_recording:
            self.get_logger().info('Recording is disabled.')
            self.destroy_node()
            return

        # Initialize last message time to current time
        self.last_msg_time = Clock().now()

        # Subscribe to the voltage topic
        if bypass:
            self.start_recording()
        else:
            self.create_subscription(Float64, 'sensors/voltage', self.voltage_callback, 10)

        # Create a timer to check for timeout; calls check_timeout every second
        self.timer = self.create_timer(1, self.check_timeout)

        self.get_logger().info('Record bag node started.')

    def voltage_callback(self, data: Float64) -> None:
        """
        Execute callback function for the voltage topic.

        If the voltage is above 5V and the node is not already recording,
        start recording. If the voltage drops below 5V and the node is currently recording, stop recording.

        Args:
            data: The voltage value published to the topic.
        """
        # Update last received message time
        self.last_msg_time = Clock().now()

        # If voltage is below 5V and the node is currently recording, stop recording
        min_voltage = 5
        if data.data < min_voltage:
            if self.process:
                self.get_logger().info('Voltage is below 5V. Stopping recording due to low voltage.')
                self.stop_recording()
                self.shutdown_node()

        # If voltage is above 5V and the node is not currently recording, start recording
        elif self.process is None:
            self.start_recording()

    def start_recording(self) -> None:
        """
        Start recording all topics to a bag file by executing the `rosbag record` command in the shell.

        The bag file is
        saved in the bag_files directory in the robosub-ros package. The file name is the current date and time in a
        human-readable format.
        """
        # Get the current time in seconds since the Unix epoch
        current_time_sec = Clock().now().seconds_nanoseconds()[0]

        # Convert to a human-readable format
        human_readable_time = datetime.fromtimestamp(current_time_sec).strftime('%Y.%m.%d_%I-%M-%S_%p')

        # Start recording all topics to a bag file
        Path('/root/dev/robosub-ros2/bag_files/').mkdir(parents=False, exist_ok=True)

        command = f'ros2 bag record -a -o /root/dev/robosub-ros2/bag_files/{human_readable_time}.bag'
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, shell=True,
                                        preexec_fn=os.setsid)

        self.get_logger().info('Started recording all topics.')

    def stop_recording(self) -> None:
        """Stop recording the bag file, if it is currently recording."""
        if self.process:
            os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            self.process = None
            self.get_logger().info('Recording stopped.')

    def check_timeout(self) -> None:
        """
        Check if the last voltage message received was more than TIMEOUT_DURATION ago.

        If so, and if the node is
        currently recording, stop recording and shutdown the node.

        Args:
            event: The timer event that triggered this function.
        """
        current_time = Clock().now()
        if (current_time - self.last_msg_time) > self.TIMEOUT_DURATION and self.process is not None:
            self.get_logger().info(f'No voltage messages received for '
                                   f'{self.TIMEOUT_DURATION.nanoseconds / 1e9} seconds. '
                                   'Stopping recording.')
            self.stop_recording()
            self.shutdown_node()

    def shutdown_node(self) -> None:
        """Shutdown the node and stop recording."""
        self.get_logger().info('Stopping node due to recording stop.')
        rclpy.shutdown()


def main(args: list[str] | None = None) -> None:
    """Execute main entry point for the record_Bag node."""
    rclpy.init(args=args)
    recorder = RecordBag()

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.stop_recording()
        if rclpy.ok():
            recorder.get_logger().info('Shutting down. Stopping recording.')

        recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
