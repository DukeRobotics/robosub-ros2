from rclpy.clock import Clock
from rclpy.duration import Duration
from task_planning.task import Task, Yield, task


@task
async def sleep(_self: Task, secs: float) -> Task[None, None, None]:
    """Sleep for a given number of seconds. Yields frequently, then returns when the time has elapsed."""
    duration = Duration(seconds=secs)
    start_time = Clock().now()
    while start_time + duration > Clock().now():
        await Yield()
