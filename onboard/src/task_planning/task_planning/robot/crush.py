# ruff: noqa: ERA001, F401
from math import radians

import numpy as np
from task_planning.interface.cv import CVObjectType
from task_planning.interface.ivc import IVCMessageType
from task_planning.task import Task, task
from task_planning.tasks import buoyancy_tasks, comp_tasks, ivc_tasks, move_tasks, prequal_tasks, sonar_tasks
from task_planning.utils import geometry_utils


@task
async def main(self: Task) -> Task[None, None, None]:
    """Run the tasks to be performed by Crush."""
    tasks = [
        ######## Main competition tasks ########
        ivc_tasks.delineate_ivc_log(parent=self),
        comp_tasks.initial_submerge(-0.5, z_tolerance=0.15, enable_controls_flag=True, timeout=10, parent=self),
        comp_tasks.coin_flip(enable_same_direction=False, parent=self),
        comp_tasks.gate_task_dead_reckoning(depth_level=0.7, parent=self),  # Move through gate via 2,2; right strafe via 1.5  # noqa: E501
        comp_tasks.gate_style_task(depth_level=0.975, parent=self),  # Spin
        comp_tasks.slalom_task_dead_reckoning(depth_level=0.975, parent=self),  # Move through slalom via 2,2,2
        # Move to octagon front via 2,2; left strafe via 0.75
        comp_tasks.slalom_to_octagon_dead_reckoning(depth_level=0.975, parent=self),
        ivc_tasks.crush_ivc_spam(msg_to_send=IVCMessageType.CRUSH_OCTAGON, parent=self),

        ######## Unused competition tasks ########
        ## Gate
        # comp_tasks.gate_task(offset=-0.1, direction=-1, parent=self),

        ## Path marker
        # comp_tasks.align_path_marker(direction=-1, parent=self),
        # comp_tasks.path_marker_to_pink_bin(maximum_distance=6, parent=self),

        ## Bins/Marker dropper
        # comp_tasks.spiral_bin_search(parent=self),
        # comp_tasks.bin_task(parent=self),

        ## Octagon
        # comp_tasks.octagon_task(direction=1, parent=self),
        # comp_tasks.orient_to_wall(parent=self),
        # comp_tasks.gate_to_octagon(direction=1, move_forward=0, timeout=15, parent=self),

        ## Return home
        # comp_tasks.return_task_dead_reckoning(depth_level=1.1, parent=self),

        ## IVC
        # ivc_tasks.crush_ivc_send(msg_to_send=IVCMessageType.CRUSH_GATE,
        #     msg_to_receive=IVCMessageType.OOGWAY_ACKNOWLEDGE, timeout=90, parent=self),
        # ivc_tasks.crush_ivc_receive(msg_to_receive=IVCMessageType.OOGWAY_GATE,
        #     msg_to_send=IVCMessageType.CRUSH_ACKNOWLEDGE, timeout=90, parent=self),

        ## Movement/CV tasks
        # move_tasks.yaw_from_local_pose(np.pi / 2, parent=self),
        # move_tasks.move_with_directions([(1, 0, 0)], depth_level=-0.7, correct_depth=True, correct_yaw=True,
        #                                   parent=self),

        # comp_tasks.yaw_to_cv_object(CVObjectType.GATE_SAWFISH, direction=1, yaw_threshold=radians(10),
        #                             latency_threshold=1, depth_level=0.7, parent=self),

        # move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, 0, 0, -np.pi/2),
        #                               pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.2),
        #                               parent=self),

        ######## Setup/Testing tasks ########
        # sonar_tasks.sonar_test(-60, 60, 5, parent=self),
        # ivc_tasks.test_ivc(IVCMessageType.CRUSH_TEST, parent=self),
        # buoyancy_tasks.tune_static_power(parent=self),

        ######## Prequal tasks ########
        # prequal_tasks.prequal_task(parent=self),
    ]

    for task_to_run in tasks:
        await task_to_run
