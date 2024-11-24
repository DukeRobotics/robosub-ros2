# Duke Robotics Club - RoboSub ROS 2
The Duke Robotics Club is a student organization of [Duke University](https://duke.edu) that develops Autonomous Underwater Vehicles (AUVs) for the [RoboSub](https://robosub.org) competition. Check out [our website](https://duke-robotics.com) for more information about our club and projects.

This repository contains all code required for running and testing our AUVs. The code is built on the [Robot Operating System (ROS) 2](https://github.com/ros2) framework. We use the [Jazzy Jalisco](https://docs.ros.org/en/jazzy) distribution of ROS 2.

We are currently in the process of migrating our codebase from ROS 1 to ROS 2. The ROS 1 codebase can be found in the [robosub-ros repository](https://github.com/DukeRobotics/robosub-ros).

## Set Up the Repository and Development Environment
Setting up the repository and development enviornment is an involved process. The full process is documented in the [SETUP.md](SETUP.md) file.

## Build Packages
1. Open a terminal in the Docker container.
2. Navigate to the root of the repository `/root/dev/robosub-ros2`.
3. Run the following command to build all packages:
    ```bash
    source build.sh
    ```
    - This command builds all packages in the `core` and `onboard` workspaces.
    - See the [Build Script Options](#build-script-options) section for more options.
4. You are now ready to run the code!

See [SCRIPTS.md](SCRIPTS.md) for more information about how to use `build.sh` and other scripts at the root of the repository.
