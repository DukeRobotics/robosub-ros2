# ruff: noqa: F401
from task_planning.task import Task, task
from task_planning.tasks import comp_tasks, move_tasks, prequal_tasks, test_tasks


@task
async def main(self: Task) -> Task[None, None, None]:
    """Run the tasks to be performed by Oogway Shell."""
    tasks = [
        test_tasks.print_task('Hello from Oogway Shell!', parent=self),
    ]
    for task_to_run in tasks:
        await task_to_run
