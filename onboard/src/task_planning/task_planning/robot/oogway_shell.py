# ruff: noqa: ERA001, F401
from math import radians

from task_planning.task import Task, task
from task_planning.tasks import comp_tasks, move_tasks, prequal_tasks, test_tasks


@task
async def main(self: Task) -> Task[None, None, None]:
    """Run the tasks to be performed by Oogway Shell."""
    tasks = []
    for task_to_run in tasks:
        await task_to_run
