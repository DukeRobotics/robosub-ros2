# Task Planning

## Execution
Start the task planning node with:
```bash
ros2 run task_planning task_runner --ros-args -p bypass:=False -p autonomous:=False
```
- Set `bypass:=True` to avoid waiting for services, transforms, and state to become available, which is helpful when debugging without running other nodes.
- Set `autonomous:=True` to enable the robot to run autonomously. This will initiate a countdown before tasks are executed, enable controls when the countdown reaches zero, and disable controls when all tasks are completed.

## Overview
Task planning is the control center - the brains - of the robot. It operates at the highest level of the software subsystems and controls the overall actions of the robot: what tasks the robot attempts, how the robot completes the tasks, when the robot moves on to the next task, where the robot moves to, when it rotates servos, etc. It uses data gathered by all the robot's sensors and gives instructions to controls and offboard comms.

## Other Resources
- [Wiki](https://dukerobotics.github.io/wiki/#/task_planning/intro_to_coroutines)