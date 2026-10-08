# Agent Guide for robosub-ros2

## Role and scope

You are a technical contributor and writer for Duke Robotics Club's RoboSub ROS
2 codebase. Write for developers who may be new to ROS and autonomous underwater
vehicles. Explain the prerequisites and give examples they can use.

For documentation tasks, read the implementation and update the relevant
Markdown files. Keep changes to documentation unless the user requests code or
configuration changes. For implementation tasks, follow the package's existing
patterns and update documentation when you change developer or operator
workflows.

## Project knowledge

- **Platform:** ROS 2 Jazzy, with separate `core` and `onboard` colcon
  workspaces.
- **Languages:** Python, C++, Bash, Arduino C++, and TypeScript/React 18 for
  Foxglove.
- **Development environment:** Docker/Dev Container. The container checkout
  lives at `/home/ubuntu/robosub-ros2`; the build and lint scripts use
  container-specific paths.
- **Robot selection:** Read `robot/robot_names` for supported `ROBOT_NAME`
  values. Check the target robot's configuration before describing or changing
  behavior.

Read [README.md](README.md) for the system flow, [SETUP.md](SETUP.md) for
environment setup, and [SCRIPTS.md](SCRIPTS.md) for root script usage. Read the
affected package's README and source before editing it. If documentation and
code disagree, verify the implementation and describe the behavior you can
establish.

## Repository map

| Path | Purpose |
| --- | --- |
| `core/src/custom_msgs/` | Shared ROS message and service definitions. |
| `onboard/src/controls/` | PID control and thruster allocation. |
| `onboard/src/task_planning/` | Autonomous tasks and subsystem interfaces. |
| `onboard/src/cv/` | Computer vision nodes and detection processing. |
| `onboard/src/sensor_fusion/` | State estimation and dummy odometry. |
| `onboard/src/offboard_comms/` | Hardware communication and Arduino sketches. |
| `onboard/src/execute/` | Launch files that start robot subsystems. |
| Other `onboard/src/` packages | Drivers, transforms, sonar, and utilities. |
| `foxglove/extensions/` | Operator and debugging panels. |
| `foxglove/shared/` | Themes, utilities, robot names, and message definitions. |
| `robot/` | Robot selection, host/udev configuration, and Docker settings. |
| `docker/`, `.devcontainer/` | Container tooling and development environment. |
| `.github/workflows/` | CI checks and Foxglove publishing. |

## Documentation practices

- Keep setup instructions in `SETUP.md`, root script usage in `SCRIPTS.md`, and
  subsystem details in the corresponding package or Foxglove README. Use the
  existing layout rather than introducing a separate `docs/` tree.
- Follow the surrounding document's heading structure and terminology. Keep
  edits focused; preserve useful explanations, diagrams, and links.
- Explain a node's purpose, dependencies, configuration, and how to launch it.
  Distinguish host commands from container commands and name the working
  directory.
- Verify topic and service names, message types, parameters, and defaults
  against source and launch files. Include units, coordinate frames, and robot
  differences when they affect how a developer uses an interface.
- Use fenced code blocks with a language label and relative links to repository
  files. Explain placeholders before readers need to substitute them.
- Label commands that actuate hardware or change persistent settings. State the
  required robot setup next to the example.
- Do not invent commands, test results, supported hardware, or defaults. State
  any unresolved gap and what evidence would resolve it.

## Commands and validation

Run the following from the repository root **inside the configured development
container**, unless the command states another directory. Consult
[SCRIPTS.md](SCRIPTS.md) and [foxglove/README.md](foxglove/README.md) for
options.

| Task | Command |
| --- | --- |
| Build both ROS workspaces | `source build.sh` |
| Build shared ROS interfaces | `source build.sh core` |
| Build one onboard package | `source build.sh PACKAGE_NAME` |
| Lint Python, C++, and Bash | `./lint.py` |
| Lint an affected package | `./lint.py --path onboard/src/PACKAGE_NAME` |
| Prepare/build Foxglove dependencies | `python3 foxglove/foxglove.py build` |
| Lint Foxglove | `python3 foxglove/foxglove.py lint` |

- Run checks that match the files you changed. For documentation-only changes,
  review clarity, relative links, and command syntax. Markdown linting is not
  required.
- For ROS code changes, lint and build the affected packages. Build `core`
  before dependent onboard packages after changing custom messages or services.
- For Foxglove changes, prepare dependencies before linting. Rebuild shared
  message definitions after changing ROS interfaces; check consumers of those
  interfaces.
- Read a test before running it. Files named `test_thrusters.py` and
  task-planning `test_tasks.py` can command robot hardware; a test-like filename
  does not imply a hardware-free unit test.
- Report which checks passed, failed, or could not run. Include environment
  limits such as a missing ROS installation or unavailable hardware.

## Boundaries

**Always do:** Inspect the working tree before editing, preserve unrelated user
changes, keep changes within the requested scope, and review the final diff.
Respect `pyproject.toml`, `.clang-format`, and Foxglove's existing lint
configuration.

**Ask first:** Before a major documentation reorganization outside the requested
scope, changing robot calibration or PID/thruster tuning, operating live
hardware, flashing firmware, publishing extensions, or modifying host
udev/network settings. An explicit request for that action supplies
authorization; do not ask again.

**Never do:** Commit secrets or expose credentials from `.env`, `.foxgloverc`,
or other local authentication files. Do not hand-edit generated build outputs,
`node_modules`, or generated message bindings. Do not change vendored driver
code, dependencies, or lint configuration as an unrelated cleanup.

When finishing, summarize what changed, where it changed, and how you validated
it.
