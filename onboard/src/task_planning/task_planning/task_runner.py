# import comp_tasks
from task_planning.interface.controls import Controls
from task_planning.interface.cv import CV
from task_planning.interface.state import State
from task_planning.interface.marker_dropper import MarkerDropper

import rclpy
from rclpy.node import Node
import time
from builtin_interfaces.msg import Time
from rclpy.time import Time as rclpy_time
from rclpy.duration import Duration
from rclpy.clock import Clock

from task_planning.task import Task, TaskStatus, TaskUpdatePublisher
import task_planning.comp_tasks as comp_tasks

import tf2_ros


class TaskPlanning(Node):
    NODE_NAME = 'task_planning'

    def __init__(self):
        """
        Initialize the task planning node.

        Including intializing the interfaces and the task update publisher.
        """
        main_initialized = False
        super().__init__(self.NODE_NAME)
        bypass = self.declare_parameter('bypass', False).value
        untethered = self.declare_parameter('untethered', False).value
        self.get_logger().info('task_planning node initialized')

        # When rospy is shutdown, if main finished initializing, publish that it has closed
        # TODO:ros2
        # def publish_close():
        #     if main_initialized:
        #         TaskUpdatePublisher().publish_update(Task.MAIN_ID, Task.MAIN_ID, 'main', TaskStatus.CLOSED, None)

        # rclpy.shutdown_callback(publish_close)

        # Initialize transform buffer and listener
        tfBuffer = tf2_ros.Buffer()
        _ = tf2_ros.TransformListener(tfBuffer, self)

        # Initialize interfaces
        # TODO:ros2 set bypass=false
        Controls(self, bypass=True)
        state = State(self, tfBuffer=tfBuffer, bypass=True)
        CV(self, bypass=True)
        MarkerDropper(self, bypass=True)



        # Initialize the task update publisher
        TaskUpdatePublisher(self)


        # Wait one second for all publishers and subscribers to start
        # time.sleep(1) TODO:ros2 uncomment

        # Ensure transform from odom to base_link is available
        try:
            clock = Clock()
            duration = Duration(seconds=timeout_sec)
            _ = tfBuffer.lookup_transform('odom', 'base_link', clock.now(), duration)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().info("str(e)")
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
                comp_tasks.initial_submerge(-0.7, parent=Task.MAIN_ID),
                comp_tasks.coin_flip(parent=Task.MAIN_ID),
                comp_tasks.yaw_to_cv_object('gate_red_cw', direction=1, yaw_threshold=math.radians(10),
                                            latency_threshold=1, depth_level=0.7, parent=Task.MAIN_ID),
                comp_tasks.gate_task(offset=-0.1, direction=-1, parent=Task.MAIN_ID),
                comp_tasks.gate_style_task(depth_level=0.9, parent=Task.MAIN_ID),
                comp_tasks.yaw_to_cv_object('buoy', direction=1, depth_level=0.7, parent=Task.MAIN_ID),
                comp_tasks.buoy_task(turn_to_face_buoy=False, depth=0.7, parent=Task.MAIN_ID),
                comp_tasks.after_buoy_task(parent=Task.MAIN_ID)



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
            input('Press enter to run tasks...\n')

            # TODO:ros2 migrate this correctly using timers

            def countdown_callback():
                self.get_logger().info(f'Countdown: {self.countdown_value}')

                if self.countdown_value <= 0:
                    self.countdown_timer.cancel()  # Stop the timer
                    Controls().call_enable_controls(True)
                    self.get_logger().info('Countdown complete!')

                self.countdown_value -= 1

            if untethered:
                # self.get_logger().info('\nCountdown started...\n')
                # self.countdown_timer = self.create_timer(1, self.countdown)
                # for i in tqdm(range(10, 0, -1)):
                #     time.sleep(1)
                #     if not rclpy.ok():
                #         break
                # Controls().call_enable_controls(True)
                self.countdown_value = 10  # Start from 10
                self.get_logger().info('\nCountdown started...\n')
                self.countdown_timer = self.create_timer(1.0, countdown_callback)
                self.countdown_timer.cancel()


            self.get_logger().info('\nRunning tasks.\n')

            # TODO:ros2 migrate this correctly using timers
            # Step through tasks, stopping if rpy is shutdown

            current_task = 0
            def run_tasks():
                if current_task >= len(tasks) or not rclpy.ok():
                    return
                if not tasks[self.current_task].done:
                    tasks[self.current_task].step()
                else:
                    self.current_task += 1

            self.task_runner_timer = self.create_timer(30, run_tasks)


            # rate = self.create_rate(30)
            # for t in tasks:
            #     while not t.done and rclpy.ok():
            #         t.step()
            #         rate.sleep()
            #     if not rclpy.ok():
            #         break

            if untethered:
                Controls().call_enable_controls(False)

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
