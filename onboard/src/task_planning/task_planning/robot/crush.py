# ruff: noqa: ERA001, F401
from math import radians

import numpy as np
from task_planning.interface.cv import CVObjectType
from task_planning.interface.ivc import IVCMessageType
from task_planning.interface.state import State
from task_planning.task import Task, task
from task_planning.tasks import buoyancy_tasks, comp_tasks, move_tasks, prequal_tasks, sonar_tasks, ivc_tasks, util_tasks
from task_planning.utils import geometry_utils
import time
import math


@task
async def main(self: Task) -> Task[None, None, None]:
    """Run the tasks to be performed by Crush."""
    # keep_depth target during long moves (matches coin_flip default depth)

    """
        Forwards:
        2: ~8
        2.5: ~13
        3.0: ~18 (I extrapolated this lmao)

        Strafe:
        0.5: ~5
    """
    DEPTH_LEVEL = State().orig_depth - 0.8
    tasks = [

        ######## True competition plan, Lane D ########
        # Gate
        ivc_tasks.delineate_ivc_log(parent=self),
        comp_tasks.initial_submerge(0.8, enable_controls_flag=True, timeout=15, parent=self),
        comp_tasks.coin_flip(enable_same_direction=True, parent=self),
        move_tasks.move_with_directions(
            [(2.5, 0, 0), (2, 0, 0)],
            depth_level=DEPTH_LEVEL,
            correct_yaw=True,
            correct_depth=True,
            keep_orientation=True,
            keep_depth=True,
            timeout=12,
            pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.03),
            parent=self,
        ),
        ivc_tasks.ivc_send(IVCMessageType.CRUSH_GATE, timeout=7, parent=self),
        comp_tasks.gate_style_task(depth_level=1.0, parent=self),
        ivc_tasks.ivc_send(IVCMessageType.CRUSH_STYLE, timeout=7, parent=self),
        # Turn 120 degrees CCW toward the wall
        move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, 11 * math.pi / 12),
            keep_orientation=True,
            timeout=10,
            pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.03),
            parent=self,
        ),
        sonar_tasks.rotate_to_normal(
            start_angle=-25,
            end_angle=25,
            scan_distance=8,
            yaw_threshold=math.pi / 24,
            parent=self,
        ),
        sonar_tasks.rotate_to_normal(
            start_angle=-45,
            end_angle=45,
            scan_distance=7,
            yaw_threshold=math.pi / 36,
            parent=self,
        ),
        # Turn 90 degrees CW toward the slalom.
        move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, -math.pi/2),
            keep_orientation=True,
            timeout=10,
            pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.03),
            parent=self,
        ),
        # Continue through the gate, then move left.

        # Slalom
        move_tasks.move_with_directions(
            [(0, 0.65, 0)],
            depth_level=DEPTH_LEVEL,
            correct_yaw=True,
            correct_depth=True,
            keep_orientation=True,
            keep_depth=True,
            timeout=7,
            pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.03),
            parent=self,
        ),
        move_tasks.move_with_directions(
            [
                (3.3, 0, 0),
            ],
            depth_level=DEPTH_LEVEL,
            correct_yaw=True,
            correct_depth=True,
            keep_orientation=True,
            keep_depth=True,
            timeout=15,
            pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.03),
            parent=self,
        ),
        move_tasks.move_with_directions(
            [
                (0, 0.65, 0),
            ],
            depth_level=DEPTH_LEVEL,
            correct_yaw=True,
            correct_depth=True,
            keep_orientation=True,
            keep_depth=True,
            timeout=7,
            pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
            parent=self,
        ),
        move_tasks.move_with_directions(
            [
                (1.85, 0, 0),
            ],
            depth_level=DEPTH_LEVEL,
            correct_yaw=True,
            correct_depth=True,
            keep_orientation=True,
            keep_depth=True,
            timeout=12,
            pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.03),
            parent=self,
        ),
        move_tasks.move_with_directions(
            [
                (0, -0.75, 0),
            ],
            depth_level=DEPTH_LEVEL,
            correct_yaw=True,
            correct_depth=True,
            keep_orientation=True,
            keep_depth=True,
            timeout=7,
            pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.03),
            parent=self,
        ),
        move_tasks.move_with_directions(
            [
                (2.1, 0, 0),
            ],
            depth_level=DEPTH_LEVEL,
            correct_yaw=True,
            correct_depth=True,
            keep_orientation=True,
            keep_depth=True,
            timeout=12,
            pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.03),
            parent=self,
        ),
        ivc_tasks.ivc_send(IVCMessageType.CRUSH_SLALOM, timeout=7, parent=self),
        # Slalom to octagon
        move_tasks.move_with_directions(
            [(0, -0.75, 0)],
            depth_level=DEPTH_LEVEL,
            correct_yaw=True,
            correct_depth=True,
            keep_orientation=True,
            keep_depth=True,
            timeout=7,
            pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.03),
            parent=self,
        ),
        move_tasks.move_with_directions(
            [(2.6, 0, 0), (2.6, 0, 0)],
            depth_level=DEPTH_LEVEL,
            correct_yaw=True,
            correct_depth=True,
            keep_orientation=True,
            keep_depth=True,
            timeout=12,
            pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.03),
            parent=self,
        ),
        sonar_tasks.rotate_to_normal(
            start_angle=-20,
            end_angle=20,
            scan_distance=6,
            yaw_threshold=math.pi / 6,
            parent=self,
        ),
        sonar_tasks.rotate_to_normal(
            start_angle=-45,
            end_angle=45,
            scan_distance=6,
            yaw_threshold=math.pi / 12,
            parent=self,
        ),
        # Turn 45 degrees CCW for the octagon image
        move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, math.pi / 4),
            keep_orientation=True,
            timeout=10,
            pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
            parent=self,
        ),
        comp_tasks.surface_task(parent=self),
        ivc_tasks.ivc_send(IVCMessageType.CRUSH_OCTAGON, timeout=7, parent=self),

        # # Return home: descend, undo the net 2.88 m left offset, turn around,
        # # and drive 16.67 m (matching outbound forward: 2+2.5+3.47+1.85+1.85+2.5+2.5).
        # comp_tasks.initial_submerge(0.8, timeout=15, parent=self),
        # move_tasks.move_to_pose_local(
        #     geometry_utils.create_pose(0, 0, 0, 0, 0, -math.pi / 4),
        #     keep_orientation=True,
        #     timeout=10,
        #     pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.03),
        #     parent=self,
        # ),
        # move_tasks.move_with_directions(
        #     [(0, -3.9, 0)],
        #     depth_level=DEPTH_LEVEL,
        #     correct_yaw=True,
        #     correct_depth=True,
        #     keep_orientation=True,
        #     keep_depth=True,
        #     timeout=15,
        #     pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.03),
        #     parent=self,
        # ),
        # move_tasks.move_to_pose_local(
        #     geometry_utils.create_pose(0, 0, 0, 0, 0, math.pi),
        #     keep_orientation=True,
        #     pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.03),
        #     parent=self,
        # ),
        # move_tasks.move_with_directions(
        #     [(2.5, 0, 0), (2.5, 0, 0), (2.5, 0, 0), (2.5, 0, 0), (1.7, 0, 0), (0, -2.75, 0), (2, 0, 0), (2, 0, 0)],
        #     depth_level=DEPTH_LEVEL,
        #     correct_yaw=True,
        #     correct_depth=True,
        #     keep_orientation=True,
        #     keep_depth=True,
        #     timeout=12,
        #     pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.03),
        #     parent=self,
        # ),
        # ivc_tasks.ivc_send(IVCMessageType.CRUSH_HOME, timeout=7, parent=self),
        # util_tasks.sleep(1000, parent=self),


        ######## True competition plan, Lane D ########
        # Gate
        # comp_tasks.initial_submerge(0.8, enable_controls_flag=True, timeout=15, parent=self),
        # comp_tasks.coin_flip(enable_same_direction=True, parent=self),
        # move_tasks.move_with_directions(
        #     [(2, 0, 0)],
        #     depth_level=DEPTH_LEVEL,
        #     correct_yaw=True,
        #     correct_depth=True,
        #     keep_orientation=True,
        #     keep_depth=True,
        #     timeout=15,
        #     pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
        #     parent=self,
        # ),
        # comp_tasks.gate_style_task(depth_level=1.0, parent=self),
        # # Turn 90 degrees CW toward the wall.
        # move_tasks.move_to_pose_local(
        #     geometry_utils.create_pose(0, 0, 0, 0, 0, -math.pi / 2),
        #     keep_orientation=True,
        #     pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
        #     parent=self,
        # ),
        # sonar_tasks.rotate_to_normal(
        #     start_angle=-20,
        #     end_angle=20,
        #     scan_distance=6,
        #     yaw_threshold=math.pi / 6,
        #     parent=self,
        # ),
        # sonar_tasks.rotate_to_normal(
        #     start_angle=-45,
        #     end_angle=45,
        #     scan_distance=6,
        #     yaw_threshold=math.pi / 12,
        #     parent=self,
        # ),
        # # Turn 180 degrees toward the slalom.
        # move_tasks.move_to_pose_local(
        #     geometry_utils.create_pose(0, 0, 0, 0, 0, math.pi),
        #     keep_orientation=True,
        #     pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
        #     parent=self,
        # ),
        # # Continue through the gate, then move left.
        # move_tasks.move_with_directions(
        #     [(2.5, 0, 0), (0, 0.25, 0)],
        #     depth_level=DEPTH_LEVEL,
        #     correct_yaw=True,
        #     correct_depth=True,
        #     keep_orientation=True,
        #     keep_depth=True,
        #     timeout=15,
        #     pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
        #     parent=self,
        # ),
        # # Slalom
        # move_tasks.move_with_directions(
        #     [
        #         (3.25, 0, 0),
        #         (0, 0.45, 0),
        #         (1.75, 0, 0),
        #         (0, -0.6, 0),
        #         (1.75, 0, 0),
        #     ],
        #     depth_level=DEPTH_LEVEL,
        #     correct_yaw=True,
        #     correct_depth=True,
        #     keep_orientation=True,
        #     keep_depth=True,
        #     timeout=15,
        #     pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
        #     parent=self,
        # ),
        # # Gate to octagon
        # move_tasks.move_with_directions(
        #     [(0, 0.45, 0), (2.25, 0, 0), (2.5, 0, 0)],
        #     depth_level=DEPTH_LEVEL,
        #     correct_yaw=True,
        #     correct_depth=True,
        #     keep_orientation=True,
        #     keep_depth=True,
        #     timeout=15,
        #     pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
        #     parent=self,
        # ),
        # sonar_tasks.rotate_to_normal(
        #     start_angle=-20,
        #     end_angle=20,
        #     scan_distance=6,
        #     yaw_threshold=math.pi / 6,
        #     parent=self,
        # ),
        # sonar_tasks.rotate_to_normal(
        #     start_angle=-45,
        #     end_angle=45,
        #     scan_distance=6,
        #     yaw_threshold=math.pi / 12,
        #     parent=self,
        # ),
        # # Entering on the right: turn 45 degrees CCW for the octagon image
        # move_tasks.move_to_pose_local(
        #     geometry_utils.create_pose(0, 0, 0, 0, 0, math.pi / 4),
        #     keep_orientation=True,
        #     pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
        #     parent=self,
        # ),
        # comp_tasks.surface_task(parent=self),

        # # Return home: descend, undo the net 0.55 m left offset, turn around,
        # # and drive 17.5 m to slightly overshoot the starting position.
        # comp_tasks.initial_submerge(0.8, timeout=15, parent=self),
        # move_tasks.move_with_directions(
        #     [(0, -0.55, 0)],
        #     depth_level=DEPTH_LEVEL,
        #     correct_yaw=True,
        #     correct_depth=True,
        #     keep_orientation=True,
        #     keep_depth=True,
        #     timeout=15,
        #     pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
        #     parent=self,
        # ),
        # move_tasks.move_to_pose_local(
        #     geometry_utils.create_pose(0, 0, 0, 0, 0, math.pi),
        #     keep_orientation=True,
        #     pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
        #     parent=self,
        # ),
        # move_tasks.move_with_directions(
        #     [(3, 0, 0), (3, 0, 0), (3, 0, 0), (3, 0, 0), (3, 0, 0), (2.5, 0, 0)],
        #     depth_level=DEPTH_LEVEL,
        #     correct_yaw=True,
        #     correct_depth=True,
        #     keep_orientation=True,
        #     keep_depth=True,
        #     timeout=15,
        #     pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
        #     parent=self,
        # ),
        # comp_tasks.surface_task(parent=self),

        ######## True competition plan, Lane A ########
        # comp_tasks.initial_submerge(0.8, enable_controls_flag=True, timeout=15, parent=self),
        # comp_tasks.coin_flip(enable_same_direction=True, parent=self),
        # # Go through gate + style (confirmed Monday, works)
        # move_tasks.move_with_directions([(2, 0, 0), (2.5, 0, 0)],
        #                                 depth_level=DEPTH_LEVEL,
        #                                 correct_yaw=True,
        #                                 correct_depth=True,
        #                                 keep_orientation=True,
        #                                 keep_depth=True,
        #                                 timeout=15,
        #                                 pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
        #                                 parent=self),
        # comp_tasks.gate_style_task(depth_level=1.0, parent=self),
        # # Turn 45 degrees CW
        # move_tasks.move_to_pose_local(
        #     geometry_utils.create_pose(0, 0, 0, 0, 0, -math.pi / 4),
        #     keep_orientation=True,
        #     pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
        #     parent=self,
        # ),
        # sonar_tasks.rotate_to_normal(start_angle=-20, end_angle=20, scan_distance=6, yaw_threshold=math.pi/6, parent=self),
        # sonar_tasks.rotate_to_normal(start_angle=-45, end_angle=45, scan_distance=6, yaw_threshold=math.pi/12, parent=self),
        # # Turn 90 degrees CCW
        # move_tasks.move_to_pose_local(
        #     geometry_utils.create_pose(0, 0, 0, 0, 0, math.pi / 2),
        #     keep_orientation=True,
        #     pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
        #     parent=self,
        # ),
        # # Gate to Slalom to Octagon
        # move_tasks.move_with_directions(
        #     [(0, 1.6, 0), (3.1, 0, 0)] + [(2, 0, 0), (0, 0.5, 0), (2, 0, 0)] + [(2.5, 0, 0), (0, 0.75, 0), (2, 0, 0), (1.75, 0, 0)],
        #     depth_level=DEPTH_LEVEL,
        #     correct_yaw=True,
        #     correct_depth=True,
        #     keep_orientation=True,
        #     keep_depth=True,
        #     timeout=15,
        #     pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
        #     parent=self,
        # ),
        # sonar_tasks.rotate_to_normal(start_angle=-20, end_angle=20, scan_distance=6, yaw_threshold=math.pi/6, parent=self),
        # sonar_tasks.rotate_to_normal(start_angle=-45, end_angle=45, scan_distance=6, yaw_threshold=math.pi/12, parent=self),
        # # Turn for image in octagon 45 degrees CW
        # move_tasks.move_to_pose_local(
        #     geometry_utils.create_pose(0, 0, 0, 0, 0, -math.pi / 4),
        #     keep_orientation=True,
        #     pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
        #     parent=self,
        # ),
        # comp_tasks.surface_task(parent=self),

        ### END COMP TASKS ###

        ### Sonar stuffings
        # comp_tasks.initial_submerge(0.4, parent=self),
        # # move_tasks.move_with_directions([(0.5, 0, 0)], parent=self),
        # # comp_tasks.gate_style_task(depth_level=0.9, parent=self),
        # # Turn 60 degrees CW
        # move_tasks.move_to_pose_local(
        #     geometry_utils.create_pose(0, 0, 0, 0, 0, -math.pi / 3),
        #     keep_orientation=True, 
        #     pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
        #     parent=self,
        # ),
        # sonar_tasks.rotate_to_normal(start_angle=-20, end_angle=20, scan_distance=4, yaw_threshold= math.pi/6, parent=self),
        # sonar_tasks.rotate_to_normal(start_angle=-45, end_angle=45, scan_distance=4, yaw_threshold= math.pi/12, parent=self),
        # # Turn 90 degrees CCW
        # move_tasks.move_to_pose_local(
        #     geometry_utils.create_pose(0, 0, 0, 0, 0, math.pi / 2),
        #     keep_orientation=True, 
        #     pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
        #     parent=self,
        # ),

        #ivc_tasks.delineate_ivc_log(parent=self),
        # comp_tasks.initial_submerge(0.5, parent=self),
        # util_tasks.sleep(15, parent=self)

        ######## Main competition tasks ########
        # ivc_tasks.delineate_ivc_log(parent=self),
        # comp_tasks.initial_submerge(0.8, parent=self),
        # comp_tasks.gate_style_task(depth_level=0.9, parent=self),
        # move_tasks.move_with_directions([(1, 0, 0)], parent=self),
        # move_tasks.move_to_pose_local(geometry_utils.create_pose(12, 0, 0, 0, 0, 0),
        #                               depth_level=DEPTH_LEVEL,
        #                               keep_orientation=True,
        #                               keep_depth=True,
        #                               pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
        #                               parent=self),
        # move_tasks.move_to_pose_local(geometry_utils.create_pose(6, 0, 0, 0, 0, 0),
        #                               depth_level=DEPTH_LEVEL,
        #                               keep_orientation=True,
        #                               keep_depth=True,
        #                               pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
        #                               parent=self),
        # move_tasks.move_to_pose_local(geometry_utils.create_pose(8, 0, 0, 0, 0, 0),
        #                               depth_level=DEPTH_LEVEL,
        #                               keep_orientation=True,
        #                               keep_depth=True,
        #                               pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
        #                               parent=self),
        # move_tasks.move_to_pose_local(geometry_utils.create_pose(3, 0, 0, 0, 0, 0),
        #                               depth_level=DEPTH_LEVEL,
        #                               keep_orientation=True,
        #                               keep_depth=True,
        #                               pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
        #                               parent=self),
        # move_tasks.move_with_directions([(6, 0, 0), (8, 0, 0)],
        #                             depth_level=DEPTH_LEVEL,
        #                             correct_yaw=True,
        #                             keep_orientation=False,
        #                             keep_depth=True,
        #                             pose_tolerances=move_tasks.create_twist_tolerance(angular_yaw=0.05),
        #                             parent=self),

        # comp_tasks.initial_submerge(0.5, enable_controls_flag=True, timeout=10, parent=self),
        # comp_tasks.coin_flip(parent=self),
        # comp_tasks.gate_task_dead_reckoning(depth_level=0.7, parent=self),  # Move through gate via 2,2; right strafe via 1.5  # noqa: E501
        # comp_tasks.slalom_task_dead_reckoning(depth_level=0.975, parent=self),  # Move through slalom via 2,2,2
        # Move to octagon front via 2,2; left strafe via 0.75
        # comp_tasks.slalom_to_octagon_dead_reckoning(depth_level=0.975, parent=self),
        # ivc_tasks.crush_ivc_spam(msg_to_send=IVCMessageType.CRUSH_OCTAGON, parent=self),

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
        #ivc_tasks.test_ivc(IVCMessageType.CRUSH_TEST, parent=self),
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
        # ivc_tasks.ivc_receive(parent = self, timeout=45)
        # buoyancy_tasks.tune_static_power(parent=self),

        ######## Prequal tasks ########
        # prequal_tasks.prequal_task(parent=self),
    ]
    for task_to_run in tasks:
        await task_to_run
