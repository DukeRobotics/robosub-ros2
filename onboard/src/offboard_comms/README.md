# Offboard Communications Package
This package provides communications and functionality for serial devices to be integrated with our main ROS system. The Thruster Arduino handles thruster controls. The Peripheral Arduino provides voltage, pressure (depth), temperature, and humidity readings, and also enables control of servos. The DVL (Doppler Velocity Log) provides velocity measurements.

Each serial device is paired with a ROS node that interfaces with it. The nodes are responsible for parsing the data from the serial device and publishing it to ROS topics. The nodes also subscribe to ROS topics and/or advertise ROS services to receive commands, which they then send to the serial device.

Multiple Arduinos serve to support hardware that require different serial baud rates. This was necessitated by the Thruster Arduino requiring 57600 baud while the pressure sensor is factory-optimized for 9600 baud.

## Directory Structure
The directory structure of the offboard_comms package is as follows:

### Config
The `config` directory contains the [robot config files](#robot-config-file), which are YAML files that contain configuration information for this system that is unique for each robot.

### Data
The `data` directory contains CSV files that are used to store lookup tables for the thrusters. These tables are used to convert thruster allocations to PWM signals, given the current system voltage.

### Launch
The `launch` directory contains the ROS launch files for the offboard_comms package. These are used to start the ROS nodes for the Thruster Arduino and the Peripheral Arduino. See the [Launch Files](#launch-config) section for more information.

### Offboard_comms
The `offboard_comms` directory contains the source code for the ROS nodes that interface with the serial devices.

### Resource
The `resource` directory contains an empty file that is used to register the package with the ament_index system, allowing ROS tools to discover and locate the package.

### Sketches
The `sketches` directory contains the Arduino sketches – the code that runs on the Arduinos. The `peripheral` directory contains the code for the Peripheral Arduino, and `thruster` directory contains the code for the Thruster Arduino.

### Package.xml, Setup.cfg, and Setup.py
The `package.xml`, `setup.cfg`, and `setup.py` files are used to define the package's dependencies and other metadata. They are used by the `colcon build` command to build the package.

### README.md
The `README.md` file contains the documentation for the offboard_comms package.

## Config

### Robot Config File
The robot config file is a YAML file that contains configuration information for this system that is unique for each robot. One config file is required for each robot. They are located in the `config` directory. They should be named `<robot_name>.yaml`, where `<robot_name>` is the value of the `$ROBOT_NAME` environment variable associated with the robot.

The robot config file contains the following fields:
```yaml
arduino:
    arduino_1:
        ftdi: arduino_1_ftdi_string
        fqbn: arduino_1_fqbn
        core: arduino_1_core
        sketch: sketches/arduino_1
        libraries (optional):
            - library_1
            - library_2
            - ...
        pre_compile (optional): arduino_1 pre_compile command
        post_compile (optional): arduino_1 post_compile command
        sensors (optional):
            - type: sensor_1
              tag: sensor_1_tag
              topic: sensor_1_topic
            - type: sensor_2
              ...
        servos (optional):
            - name: servo_1
              tag: servo_1_tag
              min_pwm: servo_1_min_pwm
              max_pwm: servo_1_max_pwm
            - name: servo_2
              ...
    arduino_2:
        ...
dvl:
  ftdi: DVL_FTDI_string
  negate_x_vel: true or false
  negate_y_vel: true or false
  negate_z_vel: true or false
```
- `arduino`
    - `arduino_1`, `arduino_2`, etc. are the names of the Arduinos. These names are used to refer to the Arduinos in the [CLI](#command-line-interface) and in other files. They can be any string, but should be descriptive of the Arduino. They must be unique. `all` is a special name used by the CLI to refer to all Arduinos; do **_not_** use it as an Arduino name in this file. The names do not necessarily correspond to any names recognized by the Arduino CLI or operating system.
    - `ftdi` is the FTDI string of the Arduino. This is a unique identifier for the Arduino and is used to find the port that the Arduino is connected to. To find the FTDI string, see the [Obtain FTDI String](#obtain-ftdi-string) section.
    - `fqbn` is the fully qualified board name of the Arduino. This is used when compiling and uploading the Arduino code. It is the string that appears in the output of `arduino-cli board list` under the FQBN column.
    > [!NOTE]
    > Not all Arduino devices are returned by the `arduino-cli board list` command (see the [Arduino CLI FAQ](https://arduino.github.io/arduino-cli/latest/FAQ)). In that case, first install the core for the Arduino device using `arduino-cli core install <core_name>`. Then, you can run `arduino-cli board listall` to get a list of all boards supported by the core(s) installed and find the FQBN for your board.
    - `core` is the name of the Arduino core that the Arduino uses; the core library will be installed before compiling and uploading code. It is the string that appears in the output of `arduino-cli board list` under the core column.
    - `sketch` is the path to the directory containing the Arduino sketch, relative to the `offboard_comms` package. The specified directory must contain a `.ino` file. This is the sketch that is compiled and uploaded to the Arduino. It must **_not_** include a leading `/` or `./`.
    - `libraries` is an optional list of libraries that the Arduino requires that are installed through the Arduino CLI. These libraries will be installed before compiling the Arduino code. If an Arduino does not require any libraries, this key must **_not_** be present under the Arduino's dictionary.
    - `pre_compile` is an optional bash command to run _immediately before_ compiling the Arduino code. It is useful for installing libraries or modifying files that will affect the compilation. If an Arduino does not require a pre compile command, this key must **_not_** be present under the Arduino's dictionary.
    - `post_compile` is an optional bash command to run _immediately after_ compiling the Arduino code. It is useful for deleting temporary files or performing any other cleanup after compilation. If an Arduino does not require a post compile command, this key must **_not_** be present under the Arduino's dictionary.
    - `sensors` is an optional list of sensors that the Arduino supports. Each sensor is a dictionary with the following keys:
        - `type` is the type of sensor. This is a string that describes the sensor. It is used to determine how to process the sensor data.
        - `tag` is the tag that the sensor uses to identify its data. The serial data is sent in the form `tag:data`. It is used to identify the sensor that produced the data.
        - `topic` is the ROS topic to publish the sensor data to.
    - `servos` is an optional list of servos that the Arduino supports. Each servo is a dictionary with the following keys:
        - `name` is a human-readable name for the servo. This is used to idenfity the servo in messages and logs.
        - `tag` is the tag that the servo uses to identify its data. The servo commands are sent in the form `tag:pwm`. It is used by the Arduino to identify the servo that the command is intended for.
        - `min_pwm` is the minimum PWM value that the servo accepts.
        - `max_pwm` is the maximum PWM value that the servo accepts.
- `dvl`
    - `ftdi` is the FTDI string of the DVL. This is a unique identifier for the DVL and is used to find the port that the DVL is connected to. To find the FTDI string, see the [Obtain FTDI String](#obtain-ftdi-string) section.
    - `negate_x_vel`, `negate_y_vel`, and `negate_z_vel` are boolean values that determine whether the DVL's velocity readings should be negated. These values are used to correct for the orientation of the DVL on the robot. If the DVL is mounted in a way that causes the velocity readings along one or more axes to have an incorrect sign, set the corresponding value(s) to `true`. Otherwise, set them to `false`.

### CSV Files
The `data` directory contains CSV files that are used to store lookup tables for the thrusters. These tables are used to convert thruster allocations to PWM signals, given the current system voltage. They contain two columns: `force` and `pwm`. The `force` column has values ranging from `-1.00` to `1.00` in increments of `0.01`, and the `pwm` column has the corresponding PWM values needed to exert the given force at the given voltage. The CSV files are named `<voltage>.csv`, where `<voltage>` is the voltage the thrusters need to receive for the table to be accurate. The tables are generated using the Blue Robotics T200 Thruster performance data, found on [this page](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/) under "Technical Details".

### Launch Config
The `launch` directory contains the following launch files:
- `dvl.xml`: Launches the `dvl_raw` node that interfaces with the DVL and the `dvl_odom` node that converts the DVL velocity readings to odometry messages.
- `offboard_comms.xml`: Launches the `dvl_raw`, `dvl_odom`, `thrusters`, and `peripheral` nodes. This is the primary launch file for the offboard_comms package.
- `peripheral.xml`: Launches the `peripheral` node that interfaces with the Peripheral Arduino.
- `thrusters.xml`: Launches the `thrusters` node that interfaces with the Thruster Arduino.

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
arduino {install-libs,find-ports,compile,upload} [{peripheral,thruster,all} ...] ... <OPTIONAL_FLAGS>
```

The first argument is the command to run. The following commands are available:
- `install-libs`: Install the Arduino cores & libraries for the specified Arduino(s).
- `find-ports`: Find the serial port(s) for the specified Arduino(s).
- `compile`: Install all required cores & libraries and compile the sketch(es) for the specified Arduino(s).
- `upload`: Install all required cores & libraries, compile the sketch(es), and upload the sketch(es) to the specified Arduino(s).

After the command, you can specify one or more Arduinos to run the command on. Each Arduino must be a top-level key in `config/arduino.yaml`, or `all`. If `all` is specified, the command will be run on all Arduinos in `config/arduino.yaml`.

The following optional flags can be added to the end of the command:
- `-p`, `--print-output`: Print the output of the commands being run by the CLI. This is useful for debugging if the script throws an error. This is available with the `install-libs`, `compile`, and `upload` commands.
- `-nl`, `--no-linebreaks`: Do not print any line breaks, labels, or prefixes; only print the port(s) found. This is available _only_ with the `find-ports` command. It is useful for running the command in a script or for piping the output to another command.

For example, to upload the sketches for all Arduinos, run:
```bash
arduino upload all
```
To find the ports for all Arduinos, run:
```bash
arduino find-ports all
```
To install the libraries for `arduino_1` and `arduino_2`, run:
```bash
arduino install-libs arduino_1 arduino_2
```
To compile the sketch for `arduino_1` and print output run:
```bash
arduino compile arduino_1 -p
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
The `thrusters.py` node subscribes to `/controls/thruster_allocs` of type `custom_msgs/msg/ThrusterAllocs`. This is an array of 64-bit floats, and they must be in range [-1, 1]. It also subscribes to `/sensors/voltage` of type `std_msgs/msg/Float64`. This is a 64-bit float that is clamped to the range [14.0, 18.0].

The node maps the thruster allocations to pulse widths, accounting for the current system voltage, and sends them to the Thruster Arduino. Note that this node runs _on the robot computer_, not the Arduino.

The node first loads the [CSV Files](#csv-files) that relate force (in range [-1.0, 1.0]) to PWM outputs at a given voltage (fixed by the lookup table, either 14.0v, 16.0v, or 18.0v).

For each `ThrusterAllocs` message it receives, the node first validates the message by checking that the number of thruster allocations is equal to the number of thrusters. If the message is invalid, the node does not send any PWMs to the Thruster Arduino and prints an error message to the console, but it will process the next message. If the message is valid, the node computes the PWMs for each thruster.

To compute the PWMs, the node first finds the closest force value (rounded to 2 decimal precision) in the lookup tables for the two voltages that bound the current voltage reading. Then, it performs linear interpolation between those two values using the current voltage to find the PWM that will result in the thruster exerting the desired force at the current voltage.

The PWMs are then sent to the Thruster Arduino via serial. See the [Thruster Arduino](#thruster-arduino) section for more information on the serial communication format. The PWMs are also published to `/offboard/pwm` of type `custom_msgs/msg/PWMAllocs`; this is for debugging purposes only.

## Thruster Arduino
The Thruster Arduino reads the PWM values sent by `thrusters.py` over serial. It expects values to be in the following format: a start flag of two bytes `0xFFFF` followed by pairs of bytes representing the PWM values in big-endian format. The Arduino treats each PWM value as a 16-bit unsigned integer. The Arduino validates the data by making sure that the number of PWM values received matches the number of thrusters, and that each value is within the range [1100, 1900]. If the data are valid, the Arduino sets the PWM values for the thrusters. If the data are invalid, the Arduino does not change the PWM values; the last valid values are retained.

If it has been over 500 miliseconds since the last message was recieved, the Thruster Arduino will stop all thrusters. This is to prevent the robot from continuing to move if controls is disabled or if the connection to the main computer is lost.

### ESC Offset
The Blue Robotics Basic ESCs are designed to accept PWM values in the range of [1100, 1900], with a stop signal occurring within a range of values centered around 1500. While 1500 is the intended midpoint for stopping, the exact range of stop values varies depending on the supplied voltage.

Some Blue Robotics Basic ESCs have a defect where the midpoint of their stop range is offset by a certain amount, causing their entire PWM range to shift accordingly. To determine the offset, identify the endpoints of the stop range — the largest and smallest PWM values at which the thrusters stop spinning. The higher endpoint is the value where a slight increase in PWM causes the thrusters to start spinning, and the lower endpoint is where a slight decrease in PWM causes the thrusters to start spinning. Once these two values are found, calculate their midpoint and subtract 1500 from it to determine the offset.

In the Thruster Arduino code, the `THRUSTER_PWM_OFFSET` constant is set to the calculated offset value. This offset is then added to the PWM signals sent to the ESCs to compensate for the shifted range.

## Testing Thrusters
First start the `thrusters.py` ROS node:
```
ros2 launch offboard_comms thrusters.xml
```
Now to test, use the `test_thrusters.py` script. This script starts a ROS node that publishes thruster allocations to the `/controls/thruster_allocs` topic. To run this script you can use `ros2 run offboard_comms test_thrusters.py` or its alias `test-thrusters`.

The script provides the following CLI:
```bash
test_thrusters [-s SPEED] [-r RATE] [--no-log-allocs]
```
- `-s`, `--speed`: The speed at which the thrusters should spin. This is a float in the range [-1, 1]. The default is 0.05.
- `-r`, `--rate`: The rate at which the thruster allocations should be published. This is a float in Hz. The default is 20.
- `--no-log-allocs`: Do not log the thruster allocations to the console. By default, the allocations are logged.

For example, to test the thrusters at a speed of 0.1 and a rate of 10 Hz, run:
```bash
test_thrusters -s 0.1 -r 10
```

## DVL
We use the [Teledyne Pathfinder DVL](https://www.teledynemarine.com/brands/rdi/pathfinder-dvl) for velocity measurements. The DVL is connected to the robot's main computer via a USB serial converter.

The `dvl_raw` script publishes the raw DVL data to the `/sensors/dvl/raw` topic with type `custom_msgs/msg/DVLRaw`. It obtains the DVL's FTDI string from the [robot config file](#robot-config-file) and uses it to find the DVL's serial port.

The `dvl_to_odom` script converts the raw DVL data and publishes it to `/sensors/dvl/odom` with type `nav_msgs/msg/Odometry` for use in `sensor_fusion`. It obtains the DVL's negation values from the [robot config file](#robot-config-file) and uses them to negate the DVL's X, Y, and Z velocities before publishing the odometry message.

You can launch both scripts using the `dvl.xml` launch file.

## Peripheral Arduino
The Peripheral Arduino obtains data from the following sensors:
- Generic voltage sensor
- [Blue Robotics Bar02 Pressure Sensor](https://bluerobotics.com/store/sensors-cameras/sensors/bar02-sensor-r1-rp/)
- [DHT11 sensor](https://www.adafruit.com/product/386) for temperature and humidity readings

The data from the sensors is sent as raw serial messages to the robot's main computer, in the following format: `tag:value`, where each sensor has a unique tag, and the values are the readings from the sensors, of float type. Sensors of the same type have tags that start with the same letter, but may include additional characters to differentiate between them. The tags are as follows:
- `V`: Voltage
- `P`: Pressure
- `T`: Temperature
- `H`: Humidity

The Peripheral Arduino also enables control of servos. Servos are controlled by sending a message to the Arduino over serial in the following format: `servo_tag:pwm`, where `servo_tag` is the unique identifier of the servo, and `pwm` is the PWM value to set the servo to.

The code for the Peripheral Arduino contains a set of classes, one for each type of sensor, that initialize the sensor, read data from it, and send the data over serial. The code also contains a class for the servos that sets the PWM values for the servos.

Additionally, the code contains the `Robot` class, which provides the `process()` function that is called by the main `loop()` function. The `process()` function instructs all sensors to read data and send it over serial, and also reads serial messages sent to the Arduino to control the servos.

The `Robot` class is subclassed by classes for each robot, which initialize the sensor and servo classes for the specific sensors and servos used by the robot.

### Voltage
The Peripheral Arduino publishes voltage at 10 Hz.

The votage is calibrated based on the Peripheral Arduino's voltage. This is important as the generic voltage sensor we use uses a voltage divider to measure the voltage, so it requires knowledge of its input voltage to provide an accurate reading.

### Pressure
The Peripheral Arduino interprets the Blue Robotics Pressure Sensor using the MS5837 library. It sends the data over serial each time the sensor gets a new reading, so the rate is not defined other than "as fast as possible."

After extensive testing, it was found that proper functioning of the pressure sensor requires two key things:
- The most up-to-date Arduino Wire library, which includes the [`setWireTimeout` function](https://docs.arduino.cc/language-reference/en/functions/communication/wire/setWireTimeout). As of February 2025, this is **_not_** available on Arduinos using MegaAVR 1.8.3. Therefore, an AVR-based Arduino is required (e.g. Nano or Uno).
- Consistent voltage of 5V. The sensor is factory-optimized for 5V, so readings will be inaccurate if the voltage is not 5V. Experimental results have shown that when the voltage is dropped to at or below 2.5V, the sensor will stop responding to I2C requests.

If these conditions are met, the sensor will return accurate readings and will not become unresponsive. If the sensor does become unresponsive, a 500 microsecond timeout has been set to prevent the Arduino from hanging.

The `MS5837` library has been modified to handle errors in reading the sensor.

The `MS5837::read` function has been modified to return a byte indicating the success/error of the reading. If any call to `Wire::endTransmission` returns a non-zero value, the function will return that value. If any call to `Wire::requestFrom` times out, the function will return 5. If the function returns 0, the reading was successful.

Below is the list of error codes and their meanings, as provided by the [`Wire::endTransmission` function](https://docs.arduino.cc/language-reference/en/functions/communication/wire/endTransmission):
- 0: success.
- 1: data too long to fit in transmit buffer.
- 2: received NACK on transmit of address.
- 3: received NACK on transmit of data.
- 4: other error.
- 5: timeout

If a non-timeout error occurs, no data is published to serial. On the next iteration of the loop, reading will be attempted again. If a timeout error occurs, an attempt to reinitialize the sensor will be made in every iteration of the loop until it is successful.

### Temperature/Humidity
The temperature and humidity readings are gathered by the [DHT11 sensor](https://www.adafruit.com/product/386) using the DHT11 library. The temperature is measured in degrees Fahrenheit, and the humidity is measured as a percentage. The data is sent over serial at 1 Hz.

A copy of the library is included in the Peripheral Arduino sketch, as one line in the library was modified to prevent the Arduino from hanging if the sensor becomes unresponsive – the `TIMEOUT_DURATION` constant defined in `DHT11.h` was changed from 1000 milliseconds to 10 milliseconds. This change was made because the sensor can become unresponsive and hang the Arduino for up to 1 second if the sensor is disconnected or if the sensor is not functioning properly, preventing other sensors from being read. With the change, the Arduino will only hang for 10 milliseconds before moving on to the next sensor.

### Servos
The Peripheral Arduino supports any servo compatible with the Arduino Servo library.

The servo is controlled by sending a message to the Arduino over serial in the following format: `servo_tag:pwm`, where `servo_tag` is the unique identifier of the servo, and `pwm` is the PWM value to set the servo to. The Arduino validates the data by checking that the tag is a valid servo tag and the PWM is within the range defined for that servo. If the data is valid, the Arduino sets the PWM value for the servo, rotating it to the desired position. After a predefined time (usually 1 second), the servo will return to its original position.

If a servo command is sent, then the servo will not accept any other commands until the servo has returned to its original position. Thus, if you wish to send multiple commands to the same servo in quick succession, you must wait for at least 2 seconds between commands.

## Peripheral Publisher
The `peripheral.py` script starts a ROS node that interfaces with the Peripheral Arduino. It publishes all sensor data received from the Arduino to the appropriate ROS topics. It also advertises the `/servo_control` service, with message type `custom_msgs/srv/SetServo`, which enables control over the servos.

### Sensors
The script uses the `sensors` section under the Peripheral Arduino section of the [robot config file](#robot-config-file) to determine the types of sensors connected to the current robot, the tags that the sensors use to identify their data, and the ROS topics to publish the sensor data to.

Each sensor type is associated with a subclass of `PeripheralSensor` in the `peripheral_sensors.py` script. The `PeripheralSensor` class receives data from the Arduino and filters it. The subclasses are responsible for publishing the filtered data to the appropriate ROS topic with the appropriate message type.

The sensor types, classes, and the message types they publish are as follows:
- `voltage`: `VoltageSensor`, `std_msgs/msg/Float64`
- `pressure`: `PressureSensor`, `geometry_msgs/msg/PoseWithCovarianceStamped`
- `temperature`: `TemperatureSensor`, `std_msgs/msg/Float64`
- `humidity`: `HumiditySensor`, `std_msgs/msg/Float64`

To add support for a new sensor type, create a new subclass of `PeripheralSensor` in the `peripheral_sensors.py` script and add it to the `SENSOR_CLASSES` dictionary in the `peripheral.py` script.

> [!IMPORTANT]
> The Peripheral Arduino sketch does **not** read the [robot config file](#robot-config-file). So, if a new sensor is added to the robot and connected to the Peripheral Arduino, simply adding the new sensor to the `sensors` section of the [robot config file](#robot-config-file) will **not** be sufficient to enable the sensor. The subclass for the robot in the Peripheral Arduino sketch must be updated to initialize the new sensor.

### Servos
The script uses the `servos` section under the Peripheral Arduino section of the [robot config file](#robot-config-file) to determine the servos connected to the current robot, the tags that the servos use to identify their data, and the PWM range for each servo.

When `peripheral.py` receives a service request to control a servo, it validates the request by checking that the tag is a valid servo tag and the PWM is within the range defined for that servo. If the request is valid, it sends the command to the Peripheral Arduino to control the servo.

> [!CAUTION]
> The `peripheral.py` script and the Peripheral Arduino only check if the PWM value is within the min and max PWM values defined for the servo. They do **not** check if setting the PWM value will result in the servo moving to an unsafe position. It is the responsibility of the user to ensure that the PWM values sent to the servos are safe.

> [!WARNING]
> The Peripheral Arduino will not accept any other commands for a servo until the servo has returned to its original position. If you send multiple service requests for the same servo in quick succession, the service will respond with a success message, but the servo will not perform any actions except the first one until it has returned to its original position. To avoid this, wait for at least 2 seconds between service requests for the same servo.

> [!IMPORTANT]
> The Peripheral Arduino sketch does **not** read the [robot config file](#robot-config-file). So, if a new servo is added to the robot and connected to the Peripheral Arduino, simply adding the new servo to the `servos` section of the [robot config file](#robot-config-file) will **not** be sufficient to enable the servo. The subclass for the robot in the Peripheral Arduino sketch must be updated to initialize the new servo.