from task_planning.interface.controls import Controls
from task_planning.interface.cv import CV
from task_planning.interface.state import State
from task_planning.interface.marker_dropper import MarkerDropper

import rclpy
from rclpy.node import Node
import time
from rclpy.duration import Duration
from rclpy.clock import Clock

from task_planning.task import Task, TaskStatus, TaskUpdatePublisher
import task_planning.comp_tasks as comp_tasks
import task_planning.test_tasks as test_tasks

import tf2_ros
import math


class TaskPlanning(Node):
    NODE_NAME = 'task_planning'

    def __init__(self):
        super().__init__(self.NODE_NAME)

        # TODO: ros2 set bypass=false
        bypass = self.declare_parameter('bypass', True).value
        main_initialized = False
        untethered = self.declare_parameter('untethered', True).value
        self.get_logger().info('task_planning node initialized')

        # When ros is shutdown, if main finished initializing, publish that it has closed
        def publish_close():
            if main_initialized:
                TaskUpdatePublisher().publish_update(Task.MAIN_ID, Task.MAIN_ID, 'main', TaskStatus.CLOSED, None)

        rclpy.get_default_context().on_shutdown(publish_close)

        # Initialize transform buffer and listener
        tfBuffer = tf2_ros.Buffer()
        _ = tf2_ros.TransformListener(tfBuffer, self)

        # Initialize interfaces
        controls = Controls(self, bypass=bypass)
        state = State(self, tfBuffer=tfBuffer, bypass=bypass)
        CV(self, bypass=bypass)
        MarkerDropper(self, bypass=bypass)

        # Initialize the task update publisher
        TaskUpdatePublisher(self)

        # Wait one second for all publishers and subscribers to start
        time.sleep(1)

        # Ensure transform from odom to base_link is available
        if not bypass:
            try:
                _ = tfBuffer.lookup_transform('odom', 'base_link', Clock().now(), Duration(seconds=15))
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
        main_initialized = True

        # Run tasks
        try:
            # Tasks to run
            tasks = [
                test_tasks.wait_for_seconds(3, parent=Task.MAIN_ID),
                test_tasks.print_smiley_face(parent=Task.MAIN_ID),
                test_tasks.wait_then_print(parent=Task.MAIN_ID)
                # comp_tasks.initial_submerge(-0.7, parent=Task.MAIN_ID),
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

            input('Press enter to run tasks...')  # TODO:ros2 this doesn't work when you use ros2 launch, but does when you do ros2 run because launch doesn't create an interactive terminal. Is this intended behaviour?

            self.current_task = 0

            def countdown_callback():
                self.get_logger().info(f'Countdown: {self.countdown_value}')
                if self.countdown_value <= 0:
                    self.countdown_timer.cancel()  # Stop the timer
                    controls.call_enable_controls(True)  # TODO:ros2 runtime error
                    self.get_logger().info('Countdown complete!')
                    self.task_runner_timer.reset()
                    self.get_logger().info('Running tasks.')
                    self.current_task = 0

                self.countdown_value -= 1

            def run_tasks():
                if self.current_task >= len(tasks) or not rclpy.ok():
                    return
                if not tasks[self.current_task].done:
                    tasks[self.current_task].step()
                else:
                    self.current_task += 1
                if untethered:
                    controls.call_enable_controls(False)

            if untethered:
                self.countdown_value = 10
                self.get_logger().info('Countdown started...')
                self.countdown_timer = self.create_timer(1.0, countdown_callback)
                self.task_runner_timer = self.create_timer(0.03, run_tasks)  # TODO:ros2 what should this time be?
                self.task_runner_timer.cancel()

        except BaseException as e:

            # Main has errored
            TaskUpdatePublisher().publish_update(Task.MAIN_ID, Task.MAIN_ID, 'main', TaskStatus.ERRORED, e)
            raise

        else:

            # Main has returned
            TaskUpdatePublisher().publish_update(Task.MAIN_ID, Task.MAIN_ID, 'main', TaskStatus.RETURNED, None)


def main(args=None):
    """
    Spin up the task planning node.
    """
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
