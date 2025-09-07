from abc import ABC
from collections.abc import Callable, Coroutine

from rclpy.logging import get_logger
from task_planning.interface.cv import CVObjectType
from task_planning.interface.state import State
from task_planning.task import ReturnType, SendType, Task, YieldType
from task_planning.tasks import cv_tasks, move_tasks
from task_planning.utils import geometry_utils
from transforms3d.euler import quat2euler

logger = get_logger('comp_tasks.base_comp_task')


class CompTask(Task[YieldType, SendType, ReturnType], ABC):
    """Base class for RoboSub competition tasks with default implementations."""

    async def move_x(self, step: float = 1) -> None:
        """
        Move robot along the x-axis by the specified step size.

        Args:
            step (float, optional): Distance to move in meters. Defaults to 1.
                Positive values move forward, negative values move backward.
        """
        await move_tasks.move_x(step=step, parent=self)

    async def move_y(self, step: float = 1) -> None:
        """
        Move robot along the y-axis by the specified step size.

        Args:
            step (float, optional): Distance to move in meters. Defaults to 1.
                Positive values move left, negative values move right.
        """
        await move_tasks.move_y(step=step, parent=self)

    async def correct_depth(self, desired_depth: float, skip_threshold: float = 0.05) -> None:
        """
        Correct robot depth to desired level if depth difference is beyond threshold.

        Args:
            desired_depth (float): Target depth in meters from surface
            skip_threshold (float, optional): Minimum depth difference required to trigger correction.
                Defaults to 0.05 meters.
        """
        if abs(State().depth - desired_depth) < skip_threshold:
            logger.info('Depth delta is below threshold, skipping.')
            return
        await move_tasks.depth_correction(desired_depth=desired_depth, parent=self)

    async def correct_y_to_cv_obj(self, cv_target: CVObjectType, add_factor: float = 0, mult_factor: float = 1) -> None:
        """
        Correct y-position relative to detected CV object.

        Args:
            cv_target (CVObjectType): The CV object type to align with
            add_factor (float, optional): Offset to add to correction. Defaults to 0.
            mult_factor (float, optional): Multiplier for correction magnitude. Defaults to 1.
        """
        await cv_tasks.correct_y(prop=cv_target, add_factor=add_factor, mult_factor=mult_factor, parent=self)

    async def correct_z_to_cv_obj(self, cv_target: CVObjectType, add_factor: float = 0, mult_factor: float = 1) -> None:
        """
        Correct z-position relative to detected CV object.

        Args:
            cv_target (CVObjectType): The CV object type to align with
            add_factor (float, optional): Offset to add to correction. Defaults to 0.
            mult_factor (float, optional): Multiplier for correction magnitude. Defaults to 1.
        """
        await cv_tasks.correct_z(prop=cv_target, add_factor=add_factor, mult_factor=mult_factor, parent=self)

    async def correct_yaw(self, yaw_correction: float,
                          add_factor: float = 0, mult_factor: float = 1, timeout: int = 30) -> None:
        """
        Correct yaw orientation by the specified angle with optional adjustments.

        Args:
            yaw_correction (float): Base yaw correction in radians.
                Positive values rotate counter-clockwise, negative values rotate clockwise.
            add_factor (float, optional): Additional angle to add to correction. Defaults to 0.
            mult_factor (float, optional): Multiplier for correction magnitude. Defaults to 1.
            timeout (int, optional): The maximum number of seconds to attempt reaching the pose
                before timing out. Defaults to 30.
        """
        logger.info(f'Correcting yaw by {(yaw_correction + add_factor) * mult_factor} radians')
        await move_tasks.move_to_pose_local(
            geometry_utils.create_pose(0, 0, 0, 0, 0, (yaw_correction + add_factor) * mult_factor),
            keep_orientation=True,
            parent=self,
            timeout=timeout,
        )
        logger.info('Corrected yaw')

    async def correct_roll_and_pitch(self, mult_factor: float = 1) -> None:
        """
        Correct the roll and pitch of the robot so that it is level in the water.

        Args:
            mult_factor (float, optional): Multiplier for the correction magnitudes. Defaults to 1.
        """
        imu_orientation = State().imu.orientation
        euler_angles = quat2euler([imu_orientation.w, imu_orientation.x, imu_orientation.y, imu_orientation.z])
        roll_correction = -euler_angles[0] * mult_factor
        pitch_correction = -euler_angles[1] * mult_factor

        logger.info(f'Roll, pitch correction: {roll_correction, pitch_correction}')
        await move_tasks.move_to_pose_local(geometry_utils.create_pose(0, 0, 0, roll_correction, pitch_correction, 0),
                                            parent=self)


def comp_task[YieldType, SendType, ReturnType](func: Callable[..., Coroutine[YieldType, SendType, ReturnType]]) -> \
                                                     Callable[..., CompTask[YieldType, SendType, ReturnType]]:
    """Wrap a coroutine within CompTask."""

    def wrapper(*args, **kwargs) -> CompTask[YieldType, SendType, ReturnType]:
        return CompTask(func, *args, **kwargs)
    return wrapper
