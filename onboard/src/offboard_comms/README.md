# Offboard Communications Package

This package provides communications and functionality for serial devices to be integrated with our main ROS system. The thruster Arduino handles thruster controls. The peripheral Arduino provides pressure (depth), voltage, temperature, and humidity readings, and also enables control of servos. The DVL (Doppler Velocity Log) provides velocity measurements.

Each serial device is paired with a ROS node that interfaces with it. The nodes are responsible for parsing the data from the serial device and publishing it to ROS topics. The nodes also subscribe to ROS topics and/or advertise ROS services to receive commands, which they then send to the serial device.

Multiple Arduinos serve to support hardware that require different serial baud rates. This was necessitated by the thruster Arduino requiring 57600 baud while the pressure sensor is factory-optimized for 9600 baud.

## Structure
### Directory Structure
The directory structure of the offboard_comms package is as follows:

#### Config
The `config` directory contains the [robot config files](#robot-config-file), which are YAML files that contain configuration information for this system that is unique for each robot.

#### Data
The `data` directory contains CSV files that are used to store lookup tables for the thrusters. These tables are used to convert thruster allocations to PWM signals, given the current system voltage.

#### Launch
The `launch` directory contains the ROS launch files for the offboard_comms package. These are used to start the ROS nodes for the thruster Arduino and the peripheral Arduino. See the [Launch Files](#launch-config) section for more information.

#### Offboard_comms
The `offboard_comms` directory contains the source code for the ROS nodes that interface with the serial devices.

#### Resource
The `resource` directory contains an empty file that is used to register the package with the ament_index system, allowing ROS tools to discover and locate the package.

#### Sketches
The `sketches` directory contains the Arduino sketches – the code that runs on the Arduinos. The `peripheral` directory contains the code for the peripheral Arduino, and `thruster` directory contains the code for the thruster Arduino.

#### Package.xml, Setup.cfg, and Setup.py
The `package.xml`, `setup.cfg`, and `setup.py` files are used to define the package's dependencies and other metadata. They are used by the `colcon build` command to build the package.

#### README.md
The `README.md` file contains the documentation for the offboard_comms package.

## Config

### Robot Config File
The robot config file is a YAML file that contains configuration information for this system that is unique for each robot. One config file is required for each robot. They are located in the `config` directory. They should be named `<robot_name>.yaml`, where `<robot_name>` is the value of the `$ROBOT_NAME` environment variable associated with the robot.

The robot config file contains the following fields
```yaml
arduino:
    arduino_name_1:
        ftdi: arduino_1_ftdi_string
        fqbn: arduino_1_fqbn
        core: arduino_1_core
        sketch: arduino_1/path_to_sketch/relative_to_offboard_comms
        libraries (optional):
            - library_1
            - library_2
            - ...
        pre_compile (optional): arduino_1 pre_compile command
        post_compile (optional): arduino_1 post_compile command
    arduino_name_2:
        ...
dvl:
  ftdi: DVL_FTDI_string
  negate_x_vel: true or false
  negate_y_vel: true or false
  negate_z_vel: true or false
ping1D:
  ftdi: Ping1D_FTDI_string
```
- `arduino`
    - `arduino_name_1`, `arduino_name_2`, etc. are the names of the Arduinos. These names are used to refer to the Arduinos in the [CLI](#command-line-interface) and in other files. They can be any string, but should be descriptive of the Arduino. They must be unique. `all` is a special name used by the CLI to refer to all Arduinos; do **_not_** use it as an Arduino name in this file. The names do not necessarily correspond to any names recognized by the Arduino CLI or operating system.
    - `ftdi` is the FTDI string of the Arduino. This is a unique identifier for the Arduino and is used to find the port that the Arduino is connected to. To find the FTDI string, see the [Obtain FTDI String](#obtain-ftdi-string) section.
    - `fqbn` is the fully qualified board name of the Arduino. This is used when compiling and uploading the Arduino code. It is the string that appears in the output of `arduino-cli board list` under the FQBN column.
    > [!NOTE]
    > Not all Arduino devices are returned by the `arduino-cli board list` command (see the [Arduino CLI FAQ](https://arduino.github.io/arduino-cli/latest/FAQ)). In that case, first install the core for the Arduino device using `arduino-cli core install <core_name>`. Then, you can run `arduino-cli board listall` to get a list of all boards supported by the core(s) installed and find the FQBN for your board.
    - `core` is the name of the Arduino core that the Arduino uses; the core library will be installed before compiling and uploading code. It is the string that appears in the output of `arduino-cli board list` under the core column.
    - `sketch` is the path to the directory containing the Arduino sketch, relative to the `offboard_comms` package. The specified directory must contain a `.ino` file. This is the sketch that is compiled and uploaded to the Arduino. It must **_not_** include a leading `/` or `./`.
    - `libraries` is an optional list of libraries that the Arduino requires that are installed through the Arduino CLI. These libraries will be installed before compiling the Arduino code. If an Arduino does not require any libraries, this key must **_not_** be present under the Arduino's dictionary.
    - `pre_compile` is an optional bash command to run _immediately before_ compiling the Arduino code. It is useful for installing libraries or modifying files that will affect the compilation. If an Arduino does not require a pre compile command, this key must **_not_** be present under the Arduino's dictionary.
    - `post_compile` is an optional bash command to run _immediately after_ compiling the Arduino code. It is useful for deleting temporary files or performing any other cleanup after compilation. If an Arduino does not require a post compile command, this key must **_not_** be present under the Arduino's dictionary.
- `dvl`
    - `ftdi` is the FTDI string of the DVL. This is a unique identifier for the DVL and is used to find the port that the DVL is connected to. To find the FTDI string, see the [Obtain FTDI String](#obtain-ftdi-string) section.
    - `negate_x_vel`, `negate_y_vel`, and `negate_z_vel` are boolean values that determine whether the DVL's velocity readings should be negated. These values are used to correct for the orientation of the DVL on the robot. If the DVL is mounted in a way that causes the velocity readings along one or more axes to have an incorrect sign, set the corresponding value(s) to `true`. Otherwise, set them to `false`.

### CSV Files
The `data` directory contains CSV files that are used to store lookup tables for the thrusters. These tables are used to convert thruster allocations to PWM signals, given the current system voltage. They contain two columns: `force` and `pwm`. The `force` column has values ranging from `-1.00` to `1.00` in increments of `0.01`, and the `pwm` column has the corresponding PWM values needed to exert the given force at the given voltage. The CSV files are named `<voltage>.csv`, where `<voltage>` is the voltage the thrusters need to receive for the table to be accurate. The tables are generated using the Blue Robotics T200 Thruster performance data, found on [this page](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/) under "Technical Details".

### Launch Config
The `launch` directory contains the following launch files:
- `dvl.xml`: Launches the `dvl_raw` node that interfaces with the DVL and the `dvl_odom` node that converts the DVL velocity readings to odometry messages.
- `offboard_comms.launch`: Launches the `dvl_raw`, `dvl_odom`, `thrusters`, and `peripheral` nodes. This is the primary launch file for the offboard_comms package.
- `peripheral.xml`: Launches the `peripheral` node that interfaces with the peripheral Arduino.
- `thrusters.xml`: Launches the `thrusters` node that interfaces with the thruster Arduino.

### Obtain FTDI String
To obtain the FTDI string of a serial device, run
```bash
ls /dev/serial/by-id
```
and find the string corresponding to the desired device. The FTDI string is the set of characters after the last underscore `_` but before the hyphen `-`. For example, in the following output,
```
usb-FTDI_FT232R_USB_UART_B0004VDI-if00-port0
```
the FTDI string is `B0004VDI`.

If it's not obvious which FTDI string corresponds with which device, unplug the device, run the command, and note which string disappears. Then plug the device back in and run the command again to find the string that reappears. That string corresponds to the device.


## Arduino CLI
On Linux hosts, with the container running in privileged mode, use the CLI provided by `arduino.py` to install libraries, find ports, compile, and upload Arduino code. The CLI is a wrapper around the Arduino CLI and other commands, and is used to simplify the process of uploading code to the Arduino.

To run the CLI, first make sure the offboard_comms package is built with `build.sh`. To run the CLI you can use `ros2 run offboard_comms arduino` or its alias `arduino` (declared in `ros_bashrc.sh`).

The CLI is run as follows:
```
arduino <COMMAND> <ARDUINO_NAME_1> <ARDUINO_NAME_2> ... <OPTIONAL_FLAGS>
```

`<COMMAND>` must be one of the following:
- `install-libs`: Install the Arduino cores & libraries for the specified Arduino(s).
- `find-ports`: Find the serial port(s) for the specified Arduino(s).
- `compile`: Install all required cores & libraries and compile the sketch(es) for the specified Arduino(s).
- `upload`: Install all required cores & libraries, compile the sketch(es), and upload the sketch(es) to the specified Arduino(s).

Each `<ARDUINO_NAME>` must be a top-level key in `config/arduino.yaml`, or `all`. If `all` is specified, the command will be run on all Arduinos in `config/arduino.yaml`.

The following optional flags can be added to the end of the command:
- `-p`, `--print-output`: Print the output of the commands being run by the CLI. This is useful for debugging if the script throws an error. It is available with the `install-libs`, `compile`, and `upload` commands.
- `-nl`, `--no-linebreaks`: Do not print any line breaks, labels, or prefixes; only print the port(s) found. This is available _only_ with the `find-ports` command. It is useful for running the command in a script or for piping the output to another command.

For example, to upload the sketches for all Arduinos, run:
```bash
arduino upload all
```
To find the ports for all Arduinos, run:
```bash
arduino find-ports all
```
To install the libraries for `arduino_name_1` and `arduino_name_2`, run:
```bash
arduino install-libs arduino_name_1 arduino_name_2
```
To compile the sketch for `arduino_name_1` and print output run:
```bash
arduino compile arduino_name_1 -p
```

Note that uploading to the Arduino might require restarting the Docker container, especially when one or more Arduinos have been disconnected and reconnected since the Docker container was started.

All output from the CLI printed to the console will be prefixed with `Arduino.py:`. This is useful for distinguishing the output of the CLI from the output of other commands.

Additionally, the CLI also prints all commands being run to the console. These commands are prefixed with `CMD:` (in addition to the general prefix above). This is useful for distinguishing the commands being run by the CLI from other output.

If an error occurs, it is recommended to run the command with the `-p` flag to print the output of the subcommands being run. This will help to identify the cause of the error.

> [!IMPORTANT]
> When compiling the Arduino sketches, the CLI includes a `--build-property` flag that defines the `ROBOT_NAME` preprocessor directive. The value of this directive is the value of the `ROBOT_NAME` environment variable, with all letters capitalized, referring to the preprocessor directive for the current robot.
>
> For example, if the sketches are compiled on Oogway, the `ROBOT_NAME` enviornment variable would be `oogway`, and the inclusion of the build property by the CLI is equivalent to adding the following statement to the top of the `.ino` files:
> ```cpp
> #define ROBOT_NAME OOGWAY
> ```
> In this case, `OOGWAY` refers to another preprocessor directive. Thus, all Arduino sketches must include preprocessor derectives that define values for all possible robot names. For example, the preprocessor directives
> ```cpp
> #define OOGWAY 0
> #define OOGWAY_SHELL 1
> #define CRUSH 2
> ```
> define the values for the `oogway`, `oogway_shell`, and `crush` robot names. The `ROBOT_NAME` preprocessor directive is used in the Arduino sketches to modify their behavior based on the capabilities of the robot.

## Thruster Allocations to PWMs
The node `thrusters.py` subscribes to `/controls/thruster_allocs` of type `custom_msgs/msg/ThrusterAllocs`. This is an array of 64-bit floats, and they must be in range [-1, 1]. It also subscribes to `/sensors/voltage` of type `std_msgs/msg/Float64`. This is a 64-bit float that is clamped to the range [14.0, 18.0].

The node maps the thruster allocations to pulse widths, accounting for the current system voltage, and sends them to the thruster Arduino. Note that this node runs _on the robot computer_, not the Arduino.

The node first loads the [CSV Files](#csv-files) that relate force (in range [-1.0, 1.0]) to PWM outputs at a given voltage (fixed by the lookup table, either 14.0v, 16.0v, or 18.0v).

For each `ThrusterAllocs` message it receives, the node first validates the message by checking that the number of thruster allocations is equal to the number of thrusters. If the message is invalid, the node does not send any PWMs to the thruster Arduino and prints an error message to the console, but it will process the next message. If the message is valid, the node computes the PWMs for each thruster.

To compute the PWMs, the node first finds the closest force value (rounded to 2 decimal precision) in the lookup tables for the two voltages that bound the current voltage reading. Then, it performs linear interpolation between those two values using the current voltage to find the PWM that will result in the thruster exerting the desired force at the current voltage.

The PWMs are then sent to the thruster Arduino via serial. See the [thruster Arduino](#thruster-arduino) section for more information on the serial communication format. The PWMs are also published to `/offboard/pwm` of type `custom_msgs/msg/PWMAllocs`; this is for debugging purposes only.

## Thruster Arduino
The thruster Arduino reads the PWM values sent by `thrusters.py` over serial. It expects values to be in the following format: a start flag of two bytes `0xFFFF` followed by pairs of bytes representing the PWM values in big-endian format. The Arduino treats each PWM value as a 16-bit unsigned integer. The Arduino validates the data by making sure that the number of PWM values received matches the number of thrusters, and that each value is within the range [1100, 1900]. If the data are valid, the Arduino sets the PWM values for the thrusters. If the data are invalid, the Arduino does not change the PWM values; the last valid values are retained.

If it has been over 500 miliseconds since the last message was recieved, the thruster Arduino will stop all thrusters. This is to prevent the robot from continuing to move if controls is disabled or if the connection to the main computer is lost.

### ESC Offset
The Blue Robotics Basic ESCs are designed to accept PWM values in the range of [1100, 1900], with a stop signal occurring within a range of values centered around 1500. While 1500 is the intended midpoint for stopping, the exact range of stop values varies depending on the supplied voltage.

Some Blue Robotics Basic ESCs have a defect where the midpoint of their stop range is offset by a certain amount, causing their entire PWM range to shift accordingly. To determine the offset, identify the endpoints of the stop range — the largest and smallest PWM values at which the thrusters stop spinning. The higher endpoint is the value where a slight increase in PWM causes the thrusters to start spinning, and the lower endpoint is where a slight decrease in PWM causes the thrusters to start spinning. Once these two values are found, calculate their midpoint and subtract 1500 from it to determine the offset.

In the thruster Arduino code, the `THRUSTER_PWM_OFFSET` constant is set to the calculated offset value. This offset is then added to the PWM signals sent to the ESCs to compensate for the shifted range.

## Testing Thrusters
First start the ROS nodes for the thruster Arduino and thruster allocs to PWMs:
```
roslaunch offboard_comms offboard_comms.launch
```
Now to test, start sending thruster allocs messages. For instance, to set 0 allocs for all thrusters:
```
ros2 topic pub -r 20 /controls/thruster_allocs custom_msgs/msg/ThrusterAllocs 'allocs: [0, 0, 0, 0, 0, 0, 0, 0]'
```
For testing on land, it is recommended to set 0.05 allocs for all thrusters:
```
ros2 topic pub -r 20 /controls/thruster_allocs custom_msgs/msg/ThrusterAllocs 'allocs: [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]'
```

## Pressure Arduino

The Pressure Arduino primarily publishes depth data over serial. It also is meant to support additional sensors, as is the case with the voltage sensor.

The `data_pub` package is responsible for parsing the data published to serial and publishing the data to ROS.

The different readings are intermixed from the different sensors. Each sensor type has a corresponding header:

- `P`: Pressure
- `V`: Voltage

Example data from the Arduino is as follows:
```
P:0.73
V:15.63
P:0.73
P:0.73
P:0.73
P:0.73
P:0.73
P:0.73
P:0.73
P:0.73
P:0.73
P:0.74
P:0.74
P:0.74
P:0.74
P:0.74
P:0.73
P:0.74
P:0.74
P:0.73
P:0.74
P:0.74
V:15.67
P:0.73
P:0.74
P:0.74
```

### Pressure

The pressure arduino interprets the Blue Robotics Pressure Sensor using the MS5837 library. It sends the data over a serial line using each time the sensor gets a new reading, so the rate is not defined other than "as fast as possible." All of the processing for the depth data can be found in the `data_pub` package.

After extensive testing, it was found that proper functioning of the pressure sensor requires two key things:
- The most up-to-date Arduino Wire library, which includes the [`setWireTimeout` function](https://www.arduino.cc/reference/en/language/functions/communication/wire/setwiretimeout/). As of February 2024, this is **_not_** available on Arduinos using MegaAVR 1.8.3. Therefore, an AVR-based Arduino is required (e.g. Nano or Uno).
- Consistent voltage of 5V. The sensor is factory-optimized for 5V, so readings will be inaccurate if the voltage is not 5V. Experimental results have shown that when the voltage is dropped to at or below 2.5V, the sensor will stop responding to I2C requests.

If these conditions are met, the sensor will return accurate readings and will not become unresponsive. If the sensor does become unresponsive, a 500 microsecond timeout has been set to prevent the Arduino from hanging.

The `MS5837` library has been modified to handle errors in reading the sensor.

The `MS5837::read` function has been modified to return a byte indicating the success/error of the reading. If any call to `Wire::endTransmission` returns a non-zero value, the function will return that value. If any call to `Wire::requestFrom` times out, the function will return 5. If the function returns 0, the reading was successful.

Below is the list of error codes and their meanings:
- 0: success.
- 1: data too long to fit in transmit buffer.
- 2: received NACK on transmit of address.
- 3: received NACK on transmit of data.
- 4: other error.
- 5: timeout

The list is identical to the [one provided by the `Wire::endTransmission` function](https://www.arduino.cc/reference/en/language/functions/communication/wire/endtransmission/).

If a non-timeout error occurs, no data is published to serial. On the next iteration of the loop, reading will be attempted again. If a timeout error occurs, an attempt to reinitialize the sensor will be made in every iteration of the loop until it is successful.

Errors in reading the pressure sensor do not affect the voltage sensor. Voltage readings will continue to be published even if the pressure sensor is unresponsive.

### Voltage

Voltage publishing over serial is also handled by the Pressure Arduino. The voltage is published as a float over serial, and is published at 1 Hz.

The votage is calibrated based on the onboard Arduino's voltage. This is important as the used voltage sensor requires knowledge of its own voltage as it uses a voltage divider to measure the voltage. The voltage sensor used is the a generic voltage sensor.