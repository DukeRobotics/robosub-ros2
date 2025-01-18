# Task Planning

## Execution
Start the task planning node with:
```bash
ros2 run task_planning task_runner --ros-args -p bypass:=False -p untethered:=False
```
- Set `bypass:=True` to bypass waits for required topics/services which is helpful when debugging locally.
- Set `untethered:=True` to initiate a countdown before tasks are executed.

## Overview
Task planning is the control center - the brains - of the robot. It operates at the highest level of the software subsystems and controls the overall actions of the robot: what tasks the robot attempts and in what order, where the robot moves, when the robot moves to the next task, etc. It uses data from sensor fusion and computer vision and gives instructions to controls.

## Other Resources
- [Wiki](https://dukerobotics.github.io/wiki/#/task_planning/intro_to_coroutines)