from collections.abc import Callable
from typing import Any


def singleton(cls: type) -> Callable[[Any, Any], type]:
    """Return a function that ensures two instances of a class are not created with the same args/kwargs."""
    instances = {}

    def getinstance(*args: tuple, **kwargs: dict) -> type:
        """Return an existing instance of the class with given args/kwargs or generates a new one."""
        if cls not in instances:
            instances[cls] = cls(*args, **kwargs)
        return instances[cls]
    return getinstance
