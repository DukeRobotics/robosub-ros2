# Sonar

The sonar package listens from sonar ping data from the Blue Robotics Ping360. We then run a denoising pipeline which primarily utilizes 2D FFTs to allow for object detection within the scan. The package publishes sonar scans as they are run.

## Structure
The following are the folders and files in the sonar package:
`config`: Robot-specific config files for the sonar package.
`launch`: Contains the launch files for the Sonar package.
`resource`: Empty ROS2 resource directory.
`sonar`:
- `decode_ping_python_360.py`: Directly interfaces with the Ping360 to decode messages.
- `sonar_image_processing.py`: Contains utility methods to process Sonar images.
- `sonar_object_detection.py`: Contains pipeline for sonar denoising and object detection.
- `sonar_utils.py`: Contains utility sonar methods for sonar calculations.
- `sonar_test_client`: Creates client node to send sonar sweep requests.
- `sonar.py`: Contains logic to initialize and run the sonar node.
`sweep_data`: Sample raw sonar scan data.

## Config
The `config` directory contains robot-specific `.yaml` files. The format is as follows:
```yaml
ftdi: <string> FTDI device serial number of the USB-to-serial adapter used by the Ping360
center_gradians: <float> Referencing heading for center direction of the sonar
increase_ccw: <bool> Whether angle values increases counterclockwise or not
```

## Topics

### Published
- `/sonar/image/raw`
  - When the sonar pipeline runs, it publishes the raw sonar image to this topic
  - Type: `sensor_msgs/CompressedImage`
- `/sonar/image/compressed`
  - When the sonar pipeline runs, it publishes the denoised image to this topic
  - Type: `sensor_msgs/CompressedImage`
- `/sonar/wall/angle`
  - When the sonar pipeline runs, it publishes the relative angle of a wall (if found) to the robot
  - When it faces directly at a wall: 0 radians, if it is parallel with the wall on the right side: pi/2 radians, if it is parallel with the wall on the left side: -pi/2 radians.
  - Type: `sensor_msgs/Float32`
