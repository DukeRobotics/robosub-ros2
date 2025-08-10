import os
from collections.abc import Callable
from enum import Enum
from typing import Any


class RobotName(Enum):
    """Enum for valid robot names."""
    OOGWAY = 'oogway'
    OOGWAY_SHELL = 'oogway_shell'
    CRUSH = 'crush'


def get_robot_name() -> RobotName:
    """
    Get the robot name from the ROBOT_NAME environment variable.

    Returns:
        RobotName: The robot name from the environment variable.
    """
    robot_name = os.getenv('ROBOT_NAME', '')
    try:
        return RobotName(robot_name)
    except ValueError:
        raise ValueError(f'Invalid robot name: {robot_name}. Must be one of: oogway, oogway_shell, crush')


def singleton(cls: type) -> Callable[[Any, Any], type]:
    """Return a function that ensures two instances of a class are not created with the same args/kwargs."""
    instances = {}

    def getinstance(*args: tuple, **kwargs: dict) -> type:
        """Return an existing instance of the class with given args/kwargs or generates a new one."""
        if cls not in instances:
            instances[cls] = cls(*args, **kwargs)
        return instances[cls]
    return getinstance
