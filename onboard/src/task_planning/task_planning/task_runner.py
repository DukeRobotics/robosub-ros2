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

TASK_RATE_SECS = 1 / 30


class TaskPlanning(Node):
    """Node for running tasks."""
    NODE_NAME = 'task_planning'

    def __init__(self) -> None:
        """Set up the task planning node."""
        super().__init__(self.NODE_NAME)
        self.get_logger().info('task_planning node initialized')

        bypass = self.declare_parameter('bypass', False).value
        untethered = self.declare_parameter('untethered', False).value

        # Initialize transform buffer and listener
        tf_buffer = tf2_ros.Buffer()
        _ = tf2_ros.TransformListener(tf_buffer, self)

        # Initialize interfaces
        controls = Controls(self, bypass=bypass)
        state = State(self, tf_buffer=tf_buffer, bypass=bypass)
        CV(self, bypass=bypass)
        MarkerDropper(self, bypass=bypass)

        # Initialize the task update publisher
        TaskUpdatePublisher(self)

        # Wait one second for all publishers and subscribers to start
        time.sleep(1)

        # Ensure transform from odom to base_link is available
        if not bypass:
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
            while not state.state:
                pass

        # Main has initialized
        TaskUpdatePublisher().publish_update(Task.MAIN_ID, Task.MAIN_ID, 'main', TaskStatus.INITIALIZED, None)

        self.run_tasks(controls=controls, untethered=untethered)

    def run_tasks(self, controls: Controls, untethered: bool = False) -> None:
        """Countdown and run tasks."""
        try:
            # Tasks to run
            tasks = [
                # test_tasks.wait_for_seconds(1, parent=Task.MAIN_ID),
                # test_tasks.print_task(parent=Task.MAIN_ID),
                # test_tasks.wait_then_print(parent=Task.MAIN_ID),
                # test_tasks.print_task('Finished!', parent=Task.MAIN_ID),
                # comp_tasks.initial_submerge(-0.7, parent=Task.MAIN_ID),
                # move_tasks.move_with_directions([(1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, -1, 0)], parent=Task.MAIN_ID),

                prequal_tasks.prequal_task(parent=Task.MAIN_ID),

                # prequal_tasks.prequal_task(parent=Task.MAIN_ID),

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

            input('Press enter to run tasks...')

            self.current_task = 0

            def countdown_callback() -> None:
                self.get_logger().info(f'Countdown: {self.countdown_value}')
                if self.countdown_value <= 0:
                    self.get_logger().info('Countdown complete!')
                    self.countdown_timer.cancel()
                    controls.call_enable_controls(True)
                    self.task_timer.reset()

                self.countdown_value -= 1

            self.entered_task_callback = False
            def task_callback() -> None:
                if not self.entered_task_callback:
                    self.entered_task_callback = True
                    self.get_logger().info('Running tasks...')

                if self.current_task >= len(tasks) or not rclpy.ok():
                    if untethered:
                        controls.call_enable_controls(False)
                    self.task_timer.cancel()

                    TaskUpdatePublisher().publish_update(Task.MAIN_ID, Task.MAIN_ID, 'main', TaskStatus.CLOSED, None)

                    self.destroy_node()
                    rclpy.shutdown()

                    self.get_logger().info('All tasks completed!')
                    return

                if not tasks[self.current_task].done:
                    tasks[self.current_task].step()
                else:
                    self.current_task += 1

            if untethered:
                self.countdown_value = 10
                self.get_logger().info('Countdown started...')
                self.countdown_timer = self.create_timer(1.0, countdown_callback)

            self.task_timer = self.create_timer(TASK_RATE_SECS, task_callback, autostart=not untethered)

        except BaseException as e:
            # Main has errored
            TaskUpdatePublisher().publish_update(Task.MAIN_ID, Task.MAIN_ID, 'main', TaskStatus.ERRORED, e)
            raise

        else:
            # Main has returned
            TaskUpdatePublisher().publish_update(Task.MAIN_ID, Task.MAIN_ID, 'main', TaskStatus.RETURNED, None)


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
