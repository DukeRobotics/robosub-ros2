# Task Planning

## Execution
Start the task planning node with:
```bash
ros2 run task_planning task_runner --ros-args -p bypass:=False -p autonomous:=False
```
- Set `bypass:=True` to avoid waiting for services, transforms, and state to become available, which is helpful when debugging without running other nodes.
- Set `autonomous:=True` to enable the robot to run autonomously. This will initiate a countdown before tasks are executed, enable controls when the countdown reaches zero, and disable controls when all tasks are completed.

## Troubleshooting missing Python modules
If startup fails with `ModuleNotFoundError: No module named 'task_planning.interface'`, the installed package may be missing its subpackages. The package uses namespace package discovery to include directories such as `interface`, `robot`, and `tasks`, which do not contain `__init__.py` files.

After updating the source, rebuild from `/home/ubuntu/robosub-ros2` inside the configured development container:
```bash
source build.sh task_planning
```

Check the installed module locations without starting the node or commanding hardware:
```bash
python3 -c "from importlib.util import find_spec; print(find_spec('task_planning.interface.controls').origin)"
```
This should print the path to `controls.py` in the installed package (or the source tree for a symlink install).

## Overview
Task planning is the control center - the brains - of the robot. It operates at the highest level of the software subsystems and controls the overall actions of the robot: what tasks the robot attempts, how the robot completes the tasks, when the robot moves on to the next task, where the robot moves to, when it rotates servos, etc. It uses data gathered by all the robot's sensors and gives instructions to controls and offboard comms.

## Other Resources
- [Wiki](https://dukerobotics.github.io/wiki/#/task_planning/intro_to_coroutines)