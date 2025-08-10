import os
from collections.abc import Callable
from typing import Any, Literal

# TODO: strongly type return (does python support this?)
def get_robot_name() -> Literal['oogway', 'oogway_shell', 'crush']:
    """
    Get the robot name from the ROBOT_NAME environment variable.

    Returns:
        Literal['oogway', 'oogway_shell', 'crush']: The robot name from the environment variable.
    """
    robot_name = os.getenv('ROBOT_NAME', '')
    if robot_name in ('oogway', 'oogway_shell', 'crush'):
        return robot_name
    else:
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
