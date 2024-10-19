#!/usr/bin/env python3
"""
Task Planning Node. This node is responsible for running tasks in the task planning package.
Tasks include:
- CV
- Controls
- State
"""

import math
import time

# import comp_tasks
from task_planning.interface.controls import Controls
from task_planning.interface.cv import CV
from task_planning.interface.state import State

import rclpy
from rclpy.node import Node

# from task_planning.task import Task, TaskStatus, TaskUpdatePublisher

import tf2_ros

# from tqdm import tqdm


class TaskPlanning(Node):
    """TODO."""

    NODE_NAME = 'task_planning'

    def __init__(self):
        """
        Initialize the task planning node.

        Including intializing the interfaces and the task update publisher.
        """
        super().__init__(self.NODE_NAME)
        self.declare_parameter('bypass', False)
        self.declare_parameter('untethered', False)

        main_initialized = False
        bypass = self.get_parameter('bypass').get_parameter_value().bool_value
        untethered = self.get_parameter('untethered').get_parameter_value().bool_value

        self.get_logger().info('Task Planning Node Initialized')

        # # When rospy is shutdown, if main finished initializing, publish that it has closed
        # def publish_close():
        #     if main_initialized:
        #         TaskUpdatePublisher().publish_update(Task.MAIN_ID, Task.MAIN_ID, 'main', TaskStatus.CLOSED, None)

        # rclpy.shutdown_callback(publish_close)

        # TODO: FIX THIS, as we are trying to bring this code up from the main method
        # # # Initialize transform buffer and listener
        # tfBuffer = tf2_ros.Buffer()
        # _ = tf2_ros.TransformListener(tfBuffer)

        # # Initialize interfaces
        # Controls(self, bypass)
        # state = State(self, bypass, tfBuffer)
        # CV(self, bypass)


    def main(self):
        """
        Initialize interfaces and publishers:
        - Controls
        - State
        - CV
        - TaskUpdatePublisher

        Run tasks (requires user input to start):
        - initial_submerge
        - coin_flip
        - yaw_to_cv_object
        - gate_task
        - gate_style_task
        - yaw_to_cv_object
        - buoy_task
        - after_buoy_task



        """
        main_initialized = False
        bypass = self.get_parameter('bypass').get_parameter_value().bool_value
        untethered = self.get_parameter('untethered').get_parameter_value().bool_value


        #TODO: FIX shutdown_callback
        # self.get_logger().info('-1')
        # # When rospy is shutdown, if main finished initializing, publish that it has closed
        # def publish_close():
        #     if main_initialized:
        #         self.get_logger().info('main has closed')
        #         TaskUpdatePublisher().publish_update(Task.MAIN_ID, Task.MAIN_ID, 'main', TaskStatus.CLOSED, None)

        # self.get_logger().info('0')
        # self.shutdown_callback(publish_close)

        # Initialize transform buffer and listener
        '''
        TODO: Get Rid of tfBuffer in main and bring up to the init method
        '''
        tfBuffer = tf2_ros.Buffer()
        _ = tf2_ros.TransformListener(tfBuffer, self)

        # Initialize interfaces that were imported
        # TODO:ros2 se
        self.get_logger().info('1')
        Controls(self, bypass=True)
        self.get_logger().info('2')
        state = State(self, bypass=True, tfBuffer)
        self.get_logger().info('3')
        CV(self, bypass=True)
        self.get_logger().info('4')

        # Initialize the task update publisher
        TaskUpdatePublisher()

        # Wait one second for all publishers and subscribers to start
        time.sleep(1)

        # Ensure transform from odom to base_link is available
        try:
            _ = tfBuffer.lookup_transform('odom', 'base_link', rclpy.time.Time(), rclpy.duration(seconds=15))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
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

            if untethered:
                self.get_logger().info('\nCountdown started...\n')
                for i in tqdm(range(10, 0, -1)):
                    time.sleep(1)
                    if not rclpy.ok():
                        break
                Controls().call_enable_controls(True)

            self.get_logger().info('\nRunning tasks.\n')

            # TODO: migrate this correctly
            # Step through tasks, stopping if rpy is shutdown
            rate = self.create_rate(30)
            for t in tasks:
                while not t.done and rclpy.ok():
                    t.step()
                    rate.sleep()
                if not rclpy.ok():
                    break

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
    task_planning.main()

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
