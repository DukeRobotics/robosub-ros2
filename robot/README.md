# Robot Configuration

This directory contains configuration related to the robots that run the code in this repository.

## Udev Rules

The `udev` directory contains files that define udev rules, which symlink ports of some devices mounted to `/dev` to ports with easily identifiable names that don't change when the devices or robot itself are rebooted. It contains one udev rules file per robot.

The udev rules are applied on the robot's host machine, not the Docker container. The name of all files starts with "99-" to ensure that they are applied after other udev rules, giving these rules highest priority. They also start with `ros2` to easily distinguish them as being related to this repository.

On each robot, its udev rules file file must be symlinked to `/etc/udev/rules.d/99-ros2-ROBOT_NAME.rules`. To do this, run:
```bash
sudo ln -s /absolute/path/to/robosub-ros2/robot/udev/99-ros2-ROBOT_NAME.rules /etc/udev/rules.d/99-ros2-ROBOT_NAME.rules
```
where `/absolute/path/to/robosub-ros2/robot/udev/99-ros2-ROBOT_NAME.rules` is the absolute path to the udev rules file for the robot and `ROBOT_NAME` is the name of the robot.

If the rules in the udev rules file are changed, run:
```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```
on the host to apply the changes.

If you don't see the changes applied:
1. Try unplugging and replugging the device(s) the rule applies to.
2. Try rebooting the host machine.
3. If you still don't see the changes applied, the rules may be incorrect.

## `docker-compose.yml`
The `docker-compose.yml` file in the `robot` directory extends the `docker-compose.yml` file in the repository root. It includes additional configurations for the robot's Docker container that enable the container to access devices connected to the robot.

## `robot_config.sh`
The `robot_config.sh` script defines enviornment variables, aliases, and other configuration needed on the robot. It should be sourced in the robot's `.bashrc` file to make the configuration available in the robot's shell.
- `onboard2`: Alias for running `docker-build.sh`.
- `bashon2`: Alias for opening a bash shell in the Docker container.
- `dkill`: Alias for stopping all running Docker containers.
- `ROBOSUB_ROS2_COMPOSE_FILE_PATH`: Environment variable that specifies the path to the Docker compose file used by the robot. Used in `devcontainer.json`.

## `robot_names`
The `robot_names` file contains a list of valid values for the `ROBOT_NAME` environment variable. It has one valid value per line must include a blank line at the end. It should not include any other blank lines, leading/trailing whitespace, or comments.
