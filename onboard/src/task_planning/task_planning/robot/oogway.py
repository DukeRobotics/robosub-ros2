# ruff: noqa: ERA001, F401
from math import radians

from task_planning.interface.cv import CVObjectType
from task_planning.task import Task, task
from task_planning.tasks import buoyancy_tasks, comp_tasks, ivc_tasks, sonar_tasks, move_tasks, prequal_tasks
from task_planning.utils import geometry_utils
from task_planning.interface.ivc import IVCMessageType


@task
async def main(self: Task) -> Task[None, None, None]:
    """Run the tasks to be performed by Oogway."""
    tasks = [
        # sonar_tasks.test_sonar(-45, 45, 5, parent=self),
        # ivc_tasks.test_ivc(IVCMessageType.OOGWAY_TEST, parent=self),
        # buoyancy_tasks.buoyancy_task(-0.5, parent=self),  # Submerge and stabilize buoyancy
        comp_tasks.initial_submerge(-0.5, parent=self),
        # move_tasks.move_with_directions([(1, 0, 0), (0, 1, 0), (-1, 0, 0), (0, -1, 0)], parent=self),
        # move_tasks.move_with_directions([(2, 0, 0), (0, 2, 0), (-2, 0, 0), (0, -2, 0)], parent=self),

        # prequal_tasks.prequal_task(parent=self),

        # comp_tasks.coin_flip(parent=self),
        # comp_tasks.second_robot_ivc(self, IVCMessageType.OOGWAY_ACKNOWLEDGE)

        # comp_tasks.yaw_to_cv_object(CVObjectType.GATE_SAWFISH, direction=1, yaw_threshold=radians(10),
        #                             latency_threshold=1, depth_level=0.7, parent=self),
        # comp_tasks.gate_task(offset=-0.1, direction=-1, parent=self),
        # comp_tasks.gate_style_task(depth_level=0.9, parent=self),
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
        comp_tasks.torpedo_task(depth_level=0.5, parent=self),
    ]

    for task_to_run in tasks:
        await task_to_run
