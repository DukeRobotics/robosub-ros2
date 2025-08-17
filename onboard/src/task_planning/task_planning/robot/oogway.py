# ruff: noqa: ERA001, F401
from math import radians
import math

from rclpy.duration import Duration
from task_planning.interface.cv import CVObjectType
from task_planning.task import Task, task
from task_planning.tasks import buoyancy_tasks, comp_tasks, ivc_tasks, sonar_tasks, move_tasks, prequal_tasks
from task_planning.utils import geometry_utils
from task_planning.interface.ivc import IVCMessageType
from task_planning.interface.servos import Servos, MarkerDropperStates, TorpedoStates
from task_planning.tasks import util_tasks
from rclpy.clock import Clock
from rclpy.logging import get_logger

logger = get_logger('oogway')

@task
async def main(self: Task) -> Task[None, None, None]:
    """Run the tasks to be performed by Oogway."""

    ### START OF CONSTANTS ###
    DIRECTION_OF_TORPEDO_BANNER = 1
    DEPTH = 1
    START_TIME = 1755374899 # Run secs alias
    FIRST_TARGET = CVObjectType.TORPEDO_REEF_SHARK_TARGET # CVObjectType.TORPEDO_REEF_SHARK_TARGET or CVObjectType.TORPEDO_SAWFISH_TARGET
    ### END OF CONSTANTS ###

    # Don't modify
    END_OF_IVC = START_TIME + (10 * 60)  # Add 10 minutes to START_TIME
    IVC_TIMEOUT = END_OF_IVC - Clock().now().seconds_nanoseconds()[0]
    logger.info(f'IVC timeout: {IVC_TIMEOUT} seconds')

    tasks = [
        # comp_tasks.initial_submerge(-1.2, parent=self),
        # test(parent=self),
        # comp_tasks.initial_submerge(-0.4, parent=self),
        # comp_tasks.oogway_ivc_start(IVCMessageType.OOGWAY_ACKNOWLEDGE, parent=self),
        # move_tasks.move_to_pose_local(
        #     geometry_utils.create_pose(0, 0, 0, 0, 0, 1.6),
        #     parent=self,
        # ),

        # comp_tasks.yaw_to_cv_object(CVObjectType.TORPEDO_BANNER, direction=1, yaw_threshold=math.radians(10), parent=self)

        # Home pool test 8/13
        # comp_tasks.initial_submerge(-DEPTH, parent=self),
        # comp_tasks.oogway_ivc_start(IVCMessageType.OOGWAY_ACKNOWLEDGE, parent=self),
        # comp_tasks.gate_task_dead_reckoning(depth_level=-DEPTH, parent=self),
        # comp_tasks.torpedo_task(depth_level=DEPTH, direction=DIRECTION_OF_TORPEDO_BANNER, parent=self),

        # IVC 8/13
        # comp_tasks.oogway_ivc_start(IVCMessageType.OOGWAY_ACKNOWLEDGE, parent=self), # Crush thru gate, Oogway acknowledges
        # comp_tasks.initial_submerge(-0.8, parent=self), # Oogway gate
        # Oogway does stuff
        # Oogway sends Crush done
        # Crush returns to gate


        ##### COMP
        comp_tasks.delineate_ivc_log(parent=self),
        comp_tasks.initial_submerge(-DEPTH, parent=self),
        comp_tasks.gate_task_dead_reckoning(depth_level=-DEPTH, parent=self),
        comp_tasks.torpedo_task(first_target=FIRST_TARGET, depth_level=DEPTH, direction=DIRECTION_OF_TORPEDO_BANNER, parent=self),
        comp_tasks.send_torpedo_ivc(parent=self),
        comp_tasks.octagon_task(direction=1, parent=self),
        ##### END COMP

        # comp_tasks.initial_submerge(-1.2, parent=self),


        # comp_tasks.yaw_to_cv_object(CVObjectType.TORPEDO_BANNER, direction=1, yaw_threshold=math.radians(10),
        #     depth_level=0.7, parent=self),


        # comp_tasks.coin_flip(parent=self),
        # comp_tasks.gate_task(steps=3, parent=self),
        # comp_tasks.gate_style_task(parent=self),
        # comp_tasks.gate_task(offset=-0.1, direction=-1, parent=self),
        # comp_tasks.gate_style_task(depth_level=0.9, parent=self),
        # sonar_tasks.test_sonar(-45, 45, 5, parent=self),
        # ivc_tasks.test_ivc(IVCMessageType.OOGWAY_TEST, parent=self),
        # buoyancy_tasks.buoyancy_task(-0.5, parent=self),  # Submerge and stabilize buoyancy
        # move_tasks.move_with_directions([(1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, -1, 0)], parent=self),
        # move_tasks.move_with_directions([(2, 0, 0), (0, 2, 0), (-2, 0, 0), (0, -2, 0)], parent=self),

        # prequal_tasks.prequal_task(parent=self),

        # comp_tasks.coin_flip(parent=self),
        # comp_tasks.second_robot_ivc(self, IVCMessageType.OOGWAY_ACKNOWLEDGE)

        # comp_tasks.yaw_to_cv_object(CVObjectType.GATE_SAWFISH, direction=1, yaw_threshold=radians(10),
        #                             latency_threshold=1, depth_level=0.7, parent=self),
        # comp_tasks.yaw_to_cv_object(CVObjectType.BUOY, direction=1, depth_level=0.7, parent=self),
        # comp_tasks.buoy_task(turn_to_face_buoy=False, depth=0.7, parent=self),
        # comp_tasks.after_buoy_task(parent=self),

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

@task
async def test(self: Task) -> Task[None, None, None]:
    """
    Put test stuff here.
    """
    await Servos().fire_torpedo(TorpedoStates.LEFT)
    await util_tasks.sleep(Duration(seconds=5), parent=self)
    await Servos().fire_torpedo(TorpedoStates.RIGHT)