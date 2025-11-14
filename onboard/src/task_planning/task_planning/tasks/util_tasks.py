from rclpy.clock import Clock
from rclpy.duration import Duration
from task_planning.task import Task, Yield, task


@task
async def sleep(_self: Task[Duration, None, None], duration: float | Duration) -> None:
    """
    Sleep for a given number of seconds.

    Args:
        _self (Task): The task instance.
        duration (float | Duration): The duration to sleep. Can be a float (number of seconds) or a Duration object.

    Yields:
        Duration: The remaining time until the sleep is complete.
    """
    duration = duration if isinstance(duration, Duration) else Duration(seconds=duration)
    start_time = Clock().now()
    end_time = start_time + duration
    while end_time > Clock().now():
        await Yield(end_time - Clock().now())
