import os
from typing import Any, ClassVar

import rclpy
import tf2_ros
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from task_planning.interface.controls import Controls
from task_planning.interface.cv import CV
from task_planning.interface.servos import Servos
from task_planning.interface.sonar import Sonar
from task_planning.interface.state import State
from task_planning.robot import crush, oogway, oogway_shell
from task_planning.task import Task, TaskStatus, TaskUpdatePublisher


class TaskPlanning(Node):
    """Node for running task planning."""
    NODE_NAME = 'task_planning'
    TASK_RATE_SECS = 1 / 30
    COUNTDOWN_SECS = 10

    ROBOT_NAME_TO_MODULE: ClassVar[dict] = {
        'oogway': oogway,
        'oogway_shell': oogway_shell,
        'crush': crush,
    }

    def __init__(self) -> None:
        """Set up the task planning node."""
        super().__init__(self.NODE_NAME)
        self.get_logger().info('Task planning node initialized')

        self.bypass = self.declare_parameter('bypass', False).value
        self.autonomous = self.declare_parameter('autonomous', False).value

        # Initialize transform buffer and listener
        tf_buffer = tf2_ros.Buffer()
        _ = tf2_ros.TransformListener(tf_buffer, self)

        # Initialize interfaces
        Controls(self, bypass=self.bypass)
        State(self, tf_buffer=tf_buffer, bypass=self.bypass)
        CV(self, bypass=self.bypass)
        Servos(self, bypass=self.bypass)
        Sonar(self, bypass=self.bypass)

        # Initialize the task update publisher
        TaskUpdatePublisher(self)

        if not self.bypass:

            # Ensure transform from odom to base_link is available
            min_transform_time = Clock(clock_type=ClockType.ROS_TIME).now()
            last_message_time = Clock().now()
            while rclpy.ok():
                # Log message every second
                if Clock().now() - last_message_time >= Duration(seconds=1):
                    self.get_logger().info('Waiting for transform from base_link to odom...')
                    last_message_time = Clock().now()

                try:
                    # Get latest available transform
                    transform = tf_buffer.lookup_transform('odom', 'base_link', Time())
                    transform_time = Time.from_msg(transform.header.stamp)

                    # Ensure transform is recent
                    if transform_time >= min_transform_time:
                        break
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    pass

                rclpy.spin_once(self, timeout_sec=0.1)

            # Ensure state is available
            last_message_time = Clock().now()
            while rclpy.ok() and not State().received_state:

                # Log message every second
                if Clock().now() - last_message_time >= Duration(seconds=1):
                    self.get_logger().info('Waiting to receive state...')
                    last_message_time = Clock().now()

                rclpy.spin_once(self, timeout_sec=0.1)

        # Determine the robot name
        robot_name = os.getenv('ROBOT_NAME')

        # Get the task for the robot
        if robot_name in self.ROBOT_NAME_TO_MODULE:
            self.task: Task[Any, Any, Any] = self.ROBOT_NAME_TO_MODULE[robot_name].main(parent=Task.MAIN_ID)
        else:
            msg = f'Unknown robot name: {robot_name}'
            raise ValueError(msg)

        self.started_running_task = False

        if self.autonomous:
            self.countdown_value = self.COUNTDOWN_SECS
            input(f'Press enter to start {self.countdown_value} second countdown...')
            self.get_logger().info('Countdown started...')
            self.countdown_timer = self.create_timer(1.0, self.countdown)
        else:
            input('Press enter to run tasks...')

        self.task_timer = self.create_timer(self.TASK_RATE_SECS, self.run_task, autostart=not self.autonomous)

    def countdown(self) -> None:
        """Countdown to start tasks."""
        self.get_logger().info(f'Countdown: {self.countdown_value}')
        if self.countdown_value <= 0:
            self.get_logger().info('Countdown complete!')
            self.countdown_timer.cancel()
            Controls().call_enable_controls(True)
            self.task_timer.reset()

        self.countdown_value -= 1

    def run_task(self) -> None:
        """Step through the task, or end the node if the task is complete."""
        try:
            if not self.started_running_task:
                # Main has finished initializing and has started running task
                self.started_running_task = True
                TaskUpdatePublisher().publish_update(Task.MAIN_ID, Task.MAIN_ID, 'main', TaskStatus.INITIALIZED, None)
                self.get_logger().info('Running tasks...')

            if self.task.done or not rclpy.ok():
                if self.autonomous:
                    Controls().call_enable_controls(False)
                self.task_timer.cancel()
                self.get_logger().info('Task planning completed!')

                # Main has finished gracefully
                TaskUpdatePublisher().publish_update(Task.MAIN_ID, Task.MAIN_ID, 'main', TaskStatus.RETURNED, None)
                self.destroy_node()
                rclpy.shutdown()
            elif not self.task.done:
                self.task.step()

        except BaseException as e:
            # Main has errored
            TaskUpdatePublisher().publish_update(Task.MAIN_ID, Task.MAIN_ID, 'main', TaskStatus.ERRORED, e)
            raise

def main(args: list[str] | None = None) -> None:
    """Spin up the task planning node."""
    rclpy.init(args=args)
    task_planning = TaskPlanning()

    try:
        rclpy.spin(task_planning)
    except KeyboardInterrupt:
        pass
    finally:
        task_planning.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
