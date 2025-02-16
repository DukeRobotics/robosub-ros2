# ruff: noqa: ERA001
import time

import rclpy
import tf2_ros
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.node import Node

from task_planning import comp_tasks, move_tasks, prequal_tasks, test_tasks  # noqa: F401
from task_planning.interface.controls import Controls
from task_planning.interface.cv import CV
from task_planning.interface.marker_dropper import MarkerDropper
from task_planning.interface.state import State
from task_planning.task import Task, TaskStatus, TaskUpdatePublisher


class TaskPlanning(Node):
    """Node for running tasks."""
    NODE_NAME = 'task_planning'
    TASK_RATE_SECS = 1 / 30
    COUNTDOWN_SECS = 10

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
        MarkerDropper(self, bypass=self.bypass)

        # Initialize the task update publisher
        TaskUpdatePublisher(self)

        # Wait one second for all publishers and subscribers to start
        time.sleep(1)

        # Ensure transform from odom to base_link is available
        if not self.bypass:
            try:
                _ = tf_buffer.lookup_transform('odom', 'base_link', Clock().now(), Duration(seconds=15))
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                self.get_logger().error('Failed to get transform')
                return

            # Ensure state is available
            while not State().state:
                pass

        # Tasks to run
        self.tasks = [
            # comp_tasks.initial_submerge(-0.7, parent=Task.MAIN_ID),
            # move_tasks.move_with_directions([(1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, -1, 0)], parent=Task.MAIN_ID),

            prequal_tasks.prequal_task(parent=Task.MAIN_ID),

            # comp_tasks.coin_flip(parent=Task.MAIN_ID),
            # comp_tasks.yaw_to_cv_object('gate_red_cw', direction=1, yaw_threshold=math.radians(10),
            #                             latency_threshold=1, depth_level=0.7, parent=Task.MAIN_ID),
            # comp_tasks.gate_task(offset=-0.1, direction=-1, parent=Task.MAIN_ID),
            # comp_tasks.gate_style_task(depth_level=0.9, parent=Task.MAIN_ID),
            # comp_tasks.yaw_to_cv_object('buoy', direction=1, depth_level=0.7, parent=Task.MAIN_ID),
            # comp_tasks.buoy_task(turn_to_face_buoy=False, depth=0.7, parent=Task.MAIN_ID),
            # comp_tasks.after_buoy_task(parent=Task.MAIN_ID)

            # comp_tasks.align_path_marker(direction=-1, parent=Task.MAIN_ID),
            # comp_tasks.center_path_marker(parent=Task.MAIN_ID),
            # comp_tasks.yaw_to_cv_object('path_marker', yaw_threshold=math.radians(5), direction=-1,
            #                             depth_level=0.5, parent=Task.MAIN_ID),
            # comp_tasks.dead_reckoning_path_marker_to_bin(maximum_distance=4, parent=Task.MAIN_ID),
            # comp_tasks.path_marker_to_pink_bin(maximum_distance=6, parent=Task.MAIN_ID),
            # comp_tasks.buoy_to_octagon(direction=1, move_forward=0, parent=Task.MAIN_ID),
            # comp_tasks.let_search_for_bin_turtlesim_style_because_why_not(parent=Task.MAIN_ID),
            # comp_tasks.bin_task(parent=Task.MAIN_ID),
            # comp_tasks.yaw_to_cv_object('bin_pink_front', direction=-1, yaw_threshold=math.radians(15),
            #                             depth_level=1.0, parent=Task.MAIN_ID),
            # comp_tasks.octagon_task(direction=1, parent=Task.MAIN_ID),
        ]

        self.current_task = 0
        self.started_running_tasks = False

        if self.autonomous:
            self.countdown_value = self.COUNTDOWN_SECS
            input(f'Press enter to start {self.countdown_value} second countdown...')
            self.get_logger().info('Countdown started...')
            self.countdown_timer = self.create_timer(1.0, self.countdown)
        else:
            input('Press enter to start tasks...')

        self.task_timer = self.create_timer(self.TASK_RATE_SECS, self.run_tasks, autostart=not self.autonomous)

    def countdown(self) -> None:
        """Count down to start tasks."""
        self.get_logger().info(f'Countdown: {self.countdown_value}')
        if self.countdown_value <= 0:
            self.get_logger().info('Countdown complete!')
            self.countdown_timer.cancel()
            Controls().call_enable_controls(True)
            self.task_timer.reset()

        self.countdown_value -= 1

    def run_tasks(self) -> None:
        """Step through the tasks, or end the node if all tasks are completed."""
        try:
            if not self.started_running_tasks:
                # Main has finished initializing and has started running tasks
                self.started_running_tasks = True
                TaskUpdatePublisher().publish_update(Task.MAIN_ID, Task.MAIN_ID, 'main', TaskStatus.INITIALIZED, None)
                self.get_logger().info('Running tasks...')

            if self.current_task >= len(self.tasks) or not rclpy.ok():
                if self.autonomous:
                    Controls().call_enable_controls(False)
                self.task_timer.cancel()
                self.get_logger().info('All tasks completed!')

                # Main has finished gracefully
                TaskUpdatePublisher().publish_update(Task.MAIN_ID, Task.MAIN_ID, 'main', TaskStatus.RETURNED, None)
                self.destroy_node()
                rclpy.shutdown()
            elif not self.tasks[self.current_task].done:
                    self.tasks[self.current_task].step()
            else:
                self.current_task += 1

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
