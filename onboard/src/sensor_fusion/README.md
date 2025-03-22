# Sensor Fusion

This package runs ROS's [robot_localization](http://wiki.ros.org/robot_localization) package, which "Provides nonlinear state estimation through sensor fusion of an arbitrary number of sensors."

The parameters for sensor fusion are set in the [params/main.yaml](params/main.yaml) file.

## Dummy State Publisher
This package also includes `dummy_state_publisher.py`, which publishes dummy odometry and transform messages between the `odom` and `base_link` frames to the `/state` and `/tf` topics. This is useful for testing scripts that require odometry and transform messages when `robot_localization` cannot be run (i.e. on a local machine).