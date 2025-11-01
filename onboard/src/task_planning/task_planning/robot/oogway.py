# ruff: noqa: ERA001, F401, N806, E501, F841
from math import radians

from task_planning.interface.cv import CVObjectType
from task_planning.interface.ivc import IVCMessageType
from task_planning.interface.servos import TorpedoStates
from task_planning.task import Task, task
from task_planning.tasks import buoyancy_tasks, comp_tasks, ivc_tasks, move_tasks, prequal_tasks, sonar_tasks
from task_planning.utils import geometry_utils


@task
async def main(self: Task) -> Task[None, None, None]:
    """Run the tasks to be performed by Oogway."""
    # Constants
    DIRECTION_OF_TORPEDO_BANNER = 1
    DEPTH = 1
    FIRST_TARGET = CVObjectType.TORPEDO_REEF_SHARK_TARGET  # CVObjectType.TORPEDO_REEF_SHARK_TARGET or CVObjectType.TORPEDO_SAWFISH_TARGET

    tasks = [
        ######## Main competition tasks ########
        ivc_tasks.delineate_ivc_log(parent=self),
        # comp_tasks.initial_submerge(-DEPTH, parent=self),
        # comp_tasks.gate_task_dead_reckoning(depth_level=-DEPTH, parent=self),
        #comp_tasks.torpedo_task(first_target=FIRST_TARGET, direction=DIRECTION_OF_TORPEDO_BANNER, parent=self),
        # # TODO: task not found???
        # comp_tasks.send_torpedo_ivc(parent=self),
        # comp_tasks.octagon_task(direction=1, parent=self),

        ######## Unused competition tasks ########
        ## Coin flip
        # comp_tasks.coin_flip(parent=self),

        ## Gate
        # comp_tasks.gate_task(offset=-0.1, direction=-1, parent=self),
        # comp_tasks.gate_style_task(depth_level=0.9, parent=self),

        ## Path marker
        # comp_tasks.align_path_marker(direction=-1, parent=self),
        # comp_tasks.path_marker_to_pink_bin(maximum_distance=6, parent=self),

        ## Bins/Marker dropper
        # comp_tasks.spiral_bin_search(parent=self),
        # comp_tasks.bin_task(parent=self),

        ## Test
        comp_tasks.fire_torpedoes(TorpedoStates.LEFT, parent=self),
        comp_tasks.fire_torpedoes(TorpedoStates.RIGHT, parent=self),

        ## Octagon
        # comp_tasks.octagon_task(direction=1, parent=self),

        ## IVC
        # TODO: task not found???
        # comp_tasks.oogway_ivc_start(IVCMessageType.OOGWAY_ACKNOWLEDGE, parent=self),

        ## Movement/CV tasks
        # move_tasks.move_with_directions([(1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, -1, 0)], parent=self),
        # move_tasks.move_to_pose_local(
        #     geometry_utils.create_pose(0, 0, 0, 0, 0, 1.6),
        #     parent=self,
        # ),
        # comp_tasks.yaw_to_cv_object(CVObjectType.GATE_SAWFISH, direction=1, yaw_threshold=radians(10),
        #                             latency_threshold=1, depth_level=0.7, parent=self),

        ######## Setup/Testing tasks ########
        # sonar_tasks.sonar_test(-45, 45, 5, parent=self),
        # ivc_tasks.test_ivc(IVCMessageType.OOGWAY_TEST, parent=self),
        # buoyancy_tasks.buoyancy_task(-0.5, parent=self),  # Submerge and stabilize buoyancy

        ######## Prequal tasks ########
        # prequal_tasks.prequal_task(parent=self),
    ]

    for task_to_run in tasks:
        await task_to_run
