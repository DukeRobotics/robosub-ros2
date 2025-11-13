from collections.abc import Callable, Coroutine
from typing import TypeVar

from task_planning.task import Task, Yield

SendType = TypeVar('SendType')
TransformedSendType = TypeVar('TransformedSendType')
YieldType = TypeVar('YieldType')
TransformedYieldType = TypeVar('TransformedYieldType')
ReturnType = TypeVar('ReturnType')
TransformedReturnType = TypeVar('TransformedReturnType')


async def transform(task: Task[YieldType, TransformedSendType, ReturnType],
                    send_transformer: Callable[[SendType], TransformedSendType] | None = None,
                    yield_transformer: Callable[[YieldType], TransformedYieldType] | None = None,
                    return_transformer: Callable[[ReturnType], TransformedReturnType] | None = None) -> \
                        Coroutine[TransformedYieldType, SendType, TransformedReturnType]:
    """
    Transform the input and output of a task.

    Args:
        task: The task to transform
        send_transformer: A function to transform the values sent into the task. If None, the values are not transformed
        (SendType and TransformedSendType are the same).
        yield_transformer: A function to transform the values yielded by the task. If None, the values are not
        transformed (YieldType and TransformedYieldType are the same).
        return_transformer: A function to transform the value returned by the task. If None, the value is not
        transformed (ReturnType and TransformedReturnType are the same).

    Yields:
        The transformed yielded value of the task

    Returns:
        The transformed returned value of the task
    """
    task_input = None
    task_output = None
    while not task.done:
        # Send non-None input only if the task has been started
        if task.started:

            # Yield transformed output and accept input
            task_input = await Yield(task_output)

            # Transform the input
            if send_transformer:
                task_input = send_transformer(task_input)

        # Send transformed input
        task_output = task.send(task_input)

        # If the task is done, output is the return value, so don't transform it with yield_transformer
        # Instead, break the loop and transform the return value
        if task.done:
            break

        # Transform the yielded value
        if yield_transformer:
            task_output = yield_transformer(task_output)

    # Transform the return value
    if return_transformer:
        task_output = return_transformer(task_output)

    return task_output
