# ruff: noqa: ERA001, F401
from math import radians
import numpy as np
from task_planning.interface.ivc import IVCMessageType
from task_planning.interface.cv import CVObjectType
from task_planning.task import Task, task
from task_planning.tasks import buoyancy_tasks, comp_tasks, ivc_tasks, move_tasks, prequal_tasks, sonar_tasks
from task_planning.utils import geometry_utils


@task
async def main(self: Task) -> Task[None, None, None]:
    """Run the tasks to be performed by Crush."""
    tasks = [
        # comp_tasks.gate_task_dead_reckoning(),
        comp_tasks.initial_submerge(-0.5, z_tolerance=0.15, enable_controls_flag=True, parent=self),
        comp_tasks.first_robot_ivc(IVCMessageType.CRUSH_GATE, parent = self),
        # ivc_tasks.test_ivc(IVCMessageType.CRUSH_TEST, parent=self),
        # buoyancy_tasks.tune_static_power(parent=self),
        # comp_tasks.initial_submerge(-0.5, z_tolerance=0.15, enable_controls_flag=True, parent=self),
        # move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, -np.pi/2),
                                      #pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.2),
                                    #   parent = self),
        # comp_tasks.octagon_task(direction=1, parent=self),
        # comp_tasks.coin_flip(parent=self, enable_same_direction=False),
        # move_tasks.move_with_directions([(1, 0, 0)], depth_level=-0.7, correct_depth=True, correct_yaw=True, parent=self),
        # move_tasks.move_with_directions([(3, 0, 0)], depth_level=-0.9, correct_depth=True, correct_yaw=True, parent=self),
        # move_tasks.move_with_directions([(3, 0, 0)], depth_level=-0.7, correct_depth=True, correct_yaw=True, parent=self),
        # move_tasks.move_with_directions([(3, 0, 0)], depth_level=-0.7, correct_depth=True, correct_yaw=True, parent=self),
        # move_tasks.move_with_directions([(3, 0, 0)], depth_level=-0.7, correct_depth=True, correct_yaw=True, parent=self),
        # move_tasks.move_with_directions([(3, 0, 0)], depth_level=-0.7, correct_depth=True, correct_yaw=True, parent=self),
        # move_tasks.move_with_directions([(3, 0, 0)], depth_level=-0.7, correct_depth=True, correct_yaw=True, parent=self),
        # move_tasks.move_with_directions([(3, 0, 0)], depth_level=-0.7, correct_depth=True, correct_yaw=True, parent=self),
        # move_tasks.move_with_directions([(3, 0, 0)], depth_level=-0.7, correct_depth=True, correct_yaw=True, parent=self),
        # # move_tasks.move_with_directions([(3, 0, 0)], depth_level=-0.7, correct_depth=True, correct_yaw=True, parent=self),
        # move_tasks.move_with_directions([(3, 0, 0)], depth_level=-0.7, correct_depth=True, correct_yaw=True, parent=self),
        # move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, -np.pi / 2), parent = self),
        # move_tasks.move_with_directions([(0, 2, 0)], depth_level=-0.9, correct_depth=True, parent=self),
        # move_tasks.move_with_directions([(0, 2, 0)], depth_level=-0.9, correct_depth=True, parent=self),
        # move_tasks.move_with_directions([(0.5, 0, 0), (0, 0.5, 0), (-0.5, 0, 0), (0, -0.5, 0)], parent=self),
        # move_tasks.move_with_directions([(1, 0, 0), (1, 0, 0), (1, 0, 0), (1, 0, 0)], parent=self),
        # move_tasks.move_with_directions([(1, 0, 0), (-1, 0, 0)], depth_level=-0.5, parent=self),
        # move_tasks.move_with_directions([(0, 0, -0.5), (0, 0, 0.5)], parent=self),
        # prequal_tasks.prequal_task(parent=self),
        # comp_tasks.yaw_to_cv_object(CVObjectType.GATE_SAWFISH, direction=1, yaw_threshold=radians(10),
        #                             latency_threshold=1, depth_level=0.7, parent=self),
        # comp_tasks.gate_task(offset=-0.1, direction=-1, parent=self),
        # comp_tasks.gate_style_task(depth_level=0.7, parent=self),
        # comp_tasks.yaw_to_cv_object(CVObjectType.BUOY, direction=1, depth_level=0.7, parent=self),
        # comp_tasks.buoy_task(turn_to_face_buoy=False, depth=0.7, parent=self),
        # comp_tasks.after_buoy_task(parent=self),

        # sonar_tasks.test_sonar(-60, 60, 5, parent=self),

        # comp_tasks.first_robot_ivc(self, IVCMessageType.CRUSH_GATE)
        # comp_tasks.align_path_marker(direction=-1, parent=self),
        # comp_tasks.center_path_marker(parent=self),
        # comp_tasks.yaw_to_cv_object(CVObjectType.PATH_MARKER, yaw_threshold=radians(5), direction=-1,
        #                             depth_level=0.5, parent=self),
        # comp_tasks.path_marker_to_pink_bin(maximum_distance=6, parent=self),
        # comp_tasks.buoy_to_octagon(direction=1, move_forward=0, parent=self),
        # comp_tasks.spiral_bin_search(parent=self),
        # comp_tasks.bin_task(parent=self),
        # comp_tasks.yaw_to_cv_object('bin_pink_front', direction=-1, yaw_threshold=radians(15),
        #                             depth_level=1.0, parent=self),
        # comp_tasks.octagon_task(direction=1, parent=self),
    ]

    for task_to_run in tasks:
        await task_to_run
