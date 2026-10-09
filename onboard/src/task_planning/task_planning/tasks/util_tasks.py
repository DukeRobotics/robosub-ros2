from rclpy.duration import Duration
from task_planning.task import Task, TaskUpdatePublisher, Yield, task


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
    start_time = TaskUpdatePublisher().node.get_clock().now()
    end_time = start_time + duration
    while end_time > TaskUpdatePublisher().node.get_clock().now():
        await Yield(end_time - TaskUpdatePublisher().node.get_clock().now())
