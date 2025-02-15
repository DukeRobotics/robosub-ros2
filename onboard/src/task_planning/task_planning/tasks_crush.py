from task_planning.task import Task, TaskStatus, TaskUpdatePublisher
import task_planning.comp_tasks as comp_tasks
import task_planning.test_tasks as test_tasks
import task_planning.prequal_tasks as prequal_tasks
from math import radians

def get_tasks():
    return [
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