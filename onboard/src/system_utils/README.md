# System Utils

This package provides various ROS interfaces to assist with evaluating system performance and also provides various system utilities. To launch all the nodes in this package, use the command:
```bash
ros2 launch system_utils system_utils.xml
```

## Record Bag
The `RecordBag` node automates the recording of all current topics into bag files if `enable_recording` is set to `True`. It monitors the `sensors/voltage` topic and begins recording all topics when the voltage exceeds 5V, automatically creating timestamped bag files in the `bag_files` directory. Recording stops if the voltage drops below 5V or if no voltage messages are received for 5 seconds. An optional `bypass` mode allows recording to start immediately and continue independent of the voltage topic.

To launch the node, use the command:
```bash
ros2 launch system_utils record_bag.xml [bypass:=False] [enable_recording:=True]
```
This takes the following parameters:
- `bypass` (default: `False`): If set to `True`, recording takes place independent of the voltage topic.
- `enable_recording` (default: `True`): Whether recording is enabled.

## System Info Publisher
The system info publisher publishes a message of type `custom_msgs/SystemUsage.msg`. It contains data corresponding to the CPU usage (percentage and speed), the GPU usage (percentage, speed, usage, and memory), RAM usage (percentage, total used, total available), and disk usage (percentage, total used, total available). All memory items and speeds and in GB and GHz respectively. See the message declaration in the `custom_msgs` package for more information.

To launch the system info publisher, use the command:
```bash
ros2 launch system_utils system_info_pub.xml
```

GPU telemetry comes from NVIDIA Jetson's `tegrastats`, which must be available on
`PATH` **inside the development container** with access to the Jetson's device
and sysfs interfaces. It replaces GPUtil/`nvidia-smi`. From the repository root
inside the container, check access with `tegrastats --interval 1000` (Ctrl+C to
stop), then launch the publisher using the command above.

If `tegrastats` is not found in the container, the ROS image does not include
this JetPack host utility. On **Crush's Jetson host, outside Docker**, first check:

```bash
command -v tegrastats
tegrastats --interval 1000
```

Stop the check with Ctrl+C. If it works on the host, copy that host's executable
into the existing container (also run these commands on the host):

```bash
docker cp "$(command -v tegrastats)" onboard2:/usr/local/bin/tegrastats
docker exec onboard2 tegrastats --interval 1000
```

Stop the container check with Ctrl+C, then restart the system info publisher.
This copy lasts only until the container is recreated. For a permanent host
binary mount, follow [Crush's Jetson setup](../../../SETUP.md#enable-the-jetson-gpu-on-crush).
`docker-build.sh` selects the Jetson Compose file on robot hosts configured with
`ROBOT_DOCKER_RUNTIME=nvidia`; Oogway keeps the shared robot configuration.
If the host command is also missing, restore the utility from the matching
JetPack/L4T installation before retrying. If the copied executable reports
missing libraries or omits GPU readings, copying alone is insufficient: inspect
its library dependencies and the container's Jetson device/sysfs access.

The node keeps one `tegrastats` process running at a one-second sampling interval
and reads it outside the ROS executor. `gpu_percent` reports `GR3D_FREQ`
utilization; `gpu_speed` reports the maximum listed GPU clock in GHz, or zero
when no clock is reported. GPU memory fields remain zero because `tegrastats`
does not provide GPU-only memory usage; system RAM remains available in `ram`.
See [NVIDIA's field descriptions](https://docs.nvidia.com/jetson/archives/r36.3/DeveloperGuide/AT/JetsonLinuxDevelopmentTools/TegrastatsUtility.html).

If the utility is missing, exits, produces no valid GPU readings, or the latest
reading is over five seconds old, GPU fields remain zero and warnings explain
that telemetry is unavailable. These zeros are not measurements. CPU, RAM, and
disk publishing continues. Restore `tegrastats` access and restart the node to
retry starting the reader. Non-Jetson machines also follow this fallback; there
is no discrete NVIDIA GPU backend. Shutdown stops only this node's child process.

After changing this package, run `source build.sh system_utils` from the repository
root inside the container and restart the publisher.

## Topic Transforms

The topic transforms node will allow the user to transform a topic from one message type to another. This is useful primarily for debugging purposes, or to make certain topics more human-readable. To run the node, use the command:
```bash
ros2 launch system_utils topic_transforms.xml
```

To transform a topic, add a `TopicTransformData` object to the `TOPIC_TRANSFORM_DATA` in the `TopicTransforms` class.

The purpose of the `input_type_conversion` function is to convert the topic's message type to a common message type that can be input to the `output_type_conversion` function. This avoids code repetition when several input topics have different input types but have the same output type.

For example, suppose we have two topics: one with input type `nav_msgs/Odometry` and another with input type `geometry_msgs/Pose`. The transformed output of both is of type `geometry_msgs/Twist`, representing the `odometry.pose.pose` and `pose`, respectively, converted to twists with Euler angles in place of quaternions. In this case, creating two separate functions, one to convert odometry to twist, and one to convert pose to twist would be redundant, since in both cases the actual transform being done is pose to twist. Therefore, to avoid code repetition, use `input_type_conversion` to simply return the pose component of the odometry message, then use the pose to twist function as the `output_type_conversion` function for both.

The `input_type_conversion` function should almost always be a `lambda` function, as its purpose is to only return a specific componenent of the input message. The `output_type_conversion` function does all the heavy lifting and applies the desired transform.
