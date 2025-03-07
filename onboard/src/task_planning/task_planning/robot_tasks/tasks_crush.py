# ruff: noqa: ERA001, F401
from math import radians

from task_planning.other_tasks import comp_tasks, move_tasks, prequal_tasks, test_tasks
from task_planning.task import Task


def get_tasks() -> list[Task]:
    """Define the task sequence to be performed by Crush."""
    return [
            # comp_tasks.initial_submerge(-0.7, parent=Task.MAIN_ID),
            # move_tasks.move_with_directions([(1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, -1, 0)], parent=Task.MAIN_ID),

            prequal_tasks.prequal_task(parent=Task.MAIN_ID),

            # comp_tasks.coin_flip(parent=Task.MAIN_ID),
            # comp_tasks.yaw_to_cv_object('gate_red_cw', direction=1, yaw_threshold=radians(10),
            #                             latency_threshold=1, depth_level=0.7, parent=Task.MAIN_ID),
            # comp_tasks.gate_task(offset=-0.1, direction=-1, parent=Task.MAIN_ID),
            # comp_tasks.gate_style_task(depth_level=0.9, parent=Task.MAIN_ID),
            # comp_tasks.yaw_to_cv_object('buoy', direction=1, depth_level=0.7, parent=Task.MAIN_ID),
            # comp_tasks.buoy_task(turn_to_face_buoy=False, depth=0.7, parent=Task.MAIN_ID),
            # comp_tasks.after_buoy_task(parent=Task.MAIN_ID),

            # comp_tasks.align_path_marker(direction=-1, parent=Task.MAIN_ID),
            # comp_tasks.center_path_marker(parent=Task.MAIN_ID),
            # comp_tasks.yaw_to_cv_object('path_marker', yaw_threshold=radians(5), direction=-1,
            #                             depth_level=0.5, parent=Task.MAIN_ID),
            # comp_tasks.path_marker_to_pink_bin(maximum_distance=6, parent=Task.MAIN_ID),
            # comp_tasks.buoy_to_octagon(direction=1, move_forward=0, parent=Task.MAIN_ID),
            # comp_tasks.spiral_bin_search(parent=Task.MAIN_ID),
            # comp_tasks.bin_task(parent=Task.MAIN_ID),
            # comp_tasks.yaw_to_cv_object('bin_pink_front', direction=-1, yaw_threshold=radians(15),
            #                             depth_level=1.0, parent=Task.MAIN_ID),
            # comp_tasks.octagon_task(direction=1, parent=Task.MAIN_ID),
        ]
