import os
from pathlib import Path

import resource_retriever as rr
import yaml
from custom_msgs.msg import ControlTypes, ThrusterAllocs
from custom_msgs.srv import SetControlTypes
from geometry_msgs.msg import Pose, Twist
from rclpy.logging import get_logger
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from task_planning.utils.other_utils import singleton

logger = get_logger('controls_interface')

@singleton
class Controls:
    """
    A singleton class for the controls interface.

    Attributes:
        _instance: The singleton instance of this class. Is a static attribute.
        _set_control_types: The service proxy for setting control types
        _all_axes_control_type: The control type for all axes
        _reset_pid_loop: The service proxy for resetting the PID loops
        _desired_position_pub: The publisher for the desired position
        _desired_velocity_pub: The publisher for the desired velocity
        _desired_power_pub: The publisher for the desired power
        _read_config: The config file
        num_thrusters: The number of thrusters
        thruster_dict: The thruster dictionary
        _thruster_pub: The publisher for thruster allocations
    """

    # ROS service name for setting control types
    CONTROL_TYPES_SERVICE = 'controls/set_control_types'
    RESET_PID_LOOPS_SERVICE = 'controls/reset_pid_loops'
    ENABLE_CONTROLS_SERVICE = 'controls/enable'
    DESIRED_POSITION_TOPIC = 'controls/desired_position'
    DESIRED_VELOCITY_TOPIC = 'controls/desired_velocity'
    DESIRED_POWER_TOPIC = 'controls/desired_power'
    THRUSTER_ALLOCS_TOPIC = 'controls/thruster_allocs'
    CONTROL_TYPES_TOPIC = 'controls/control_types'

    def __init__(self, node: Node, bypass: bool = False) -> None:
        self.node = node

        self._set_control_types = node.create_client(SetControlTypes, self.CONTROL_TYPES_SERVICE)
        if not bypass:
            while not self._set_control_types.wait_for_service(timeout_sec=1.0):
                logger.info(f'{self.CONTROL_TYPES_SERVICE} not ready, waiting...')

        # NOTE: if this variable gets out of sync with the actual control types, bad things may happen
        self._all_axes_control_type = None

        self.control_types: ControlTypes = None
        node.create_subscription(
            ControlTypes,
            self.CONTROL_TYPES_TOPIC,
            self._update_control_types,
            10,
        )

        self._reset_pid_loops = node.create_client(Trigger, self.RESET_PID_LOOPS_SERVICE)
        if not bypass:
            while not self._reset_pid_loops.wait_for_service(timeout_sec=1.0):
                logger.info(f'{self.RESET_PID_LOOPS_SERVICE} not ready, waiting...')

        self._enable_controls = node.create_client(SetBool, self.ENABLE_CONTROLS_SERVICE)

        self._desired_position_pub = node.create_publisher(Pose, self.DESIRED_POSITION_TOPIC, 1)
        self._desired_velocity_pub = node.create_publisher(Twist, self.DESIRED_VELOCITY_TOPIC, 1)
        self._desired_power_pub = node.create_publisher(Twist, self.DESIRED_POWER_TOPIC, 1)

        self._read_config = None

        self.num_thrusters = None
        self.thruster_dict = None

        self.get_thruster_dict()
        self._thruster_pub = node.create_publisher(ThrusterAllocs, self.THRUSTER_ALLOCS_TOPIC, 1)
        self.bypass = bypass

    def _update_control_types(self, control_types: ControlTypes) -> None:
        self.control_types = control_types

    def get_thruster_dict(self) -> None:
        """
        Get thruster dictionary.

        Returns:
            The thruster dictionary
        """
        config_file_path = f'package://controls/config/{os.getenv('ROBOT_NAME')}.yaml'
        filename = Path(rr.get_filename(config_file_path, use_protocol=False))
        with filename.open() as f:
            full_thruster_dict = yaml.safe_load(f)

        thruster_dict = {}
        for index, t_dict in enumerate(full_thruster_dict['thrusters']):
            thruster_name = t_dict['name']
            thruster_dict[thruster_name] = index

        self.num_thrusters = len(full_thruster_dict['thrusters'])
        self.thruster_dict = thruster_dict
        return thruster_dict

    def call_enable_controls(self, enable: bool) -> None:
        """
        Enable or disable controls.

        Args:
            enable: Whether to enable (true) or disable (false).
        """
        request = SetBool.Request()
        request.data = enable
        self._enable_controls.call_async(request)

    def _set_all_axes_control_type(self, control_type: ControlTypes) -> None:
        """
        Set the control type for all axes.

        Args:
            control_type (ControlType): The control type to set
        """
        if self._all_axes_control_type == control_type:
            return
        # TODO: what if this doesn't return success?
        if not self.bypass:
            request = SetControlTypes.Request()
            request.control_types = ControlTypes(
                x=control_type,
                y=control_type,
                z=control_type,
                roll=control_type,
                pitch=control_type,
                yaw=control_type,
            )
            self._set_control_types.call_async(request)
        self._all_axes_control_type = control_type
        self.start_new_move()

    def set_axis_control_type(self, x: ControlTypes | None, y: ControlTypes | None,
                              z: ControlTypes | None, roll: ControlTypes | None,
                              pitch: ControlTypes | None, yaw: ControlTypes | None) -> None:
        """
        Set the control type for each axis individually or use the default values if not specified.

        This method allows setting the control type for the x, y, z, roll, pitch, and yaw axes. If a
        control type for a particular axis is not provided, the method uses the current control type
        stored in `self.control_types`.

        Args:
            x (Optional): Control type for the x-axis. Defaults to the current value in `self.control_types.x`.
            y (Optional): Control type for the y-axis. Defaults to the current value in `self.control_types.y`.
            z (Optional): Control type for the z-axis. Defaults to the current value in `self.control_types.z`.
            roll (Optional): Control type for the roll axis. Defaults to the current value in `self.control_types.roll`.
            pitch (Optional): Control type for the pitch axis.
            Defaults to the current value in `self.control_types.pitch`.
            yaw (Optional): Control type for the yaw axis. Defaults to the current value in `self.control_types.yaw`.

        Returns:
            None
        """
        x = self.control_types.x if x is None else x
        y = self.control_types.y if y is None else y
        z = self.control_types.z if z is None else z
        roll = self.control_types.roll if roll is None else roll
        pitch = self.control_types.pitch if pitch is None else pitch
        yaw = self.control_types.yaw if yaw is None else yaw

        self._all_axes_control_type = None

        self._set_control_types(ControlTypes(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw))

    # Resets the PID loops. Should be called for every "new" movement
    def start_new_move(self) -> None:
        """Start a new movement."""
        if not self.bypass:
            self._reset_pid_loops.call_async(Trigger.Request())

    # In global coordinates
    def publish_desired_position(self, pose: Pose, set_control_types: bool = True) -> None:
        """
        Publish the desired position.

        Args:
            pose: The desired position.
            set_control_types: Whether all axes should be set to DESIRED_POSITION.
        """
        if set_control_types:
            self._set_all_axes_control_type(ControlTypes.DESIRED_POSITION)
        self._desired_position_pub.publish(pose)

    # In local coordinates
    def publish_desired_velocity(self, twist: Twist, set_control_types: bool = True) -> None:
        """
        Publish the desired velocity.

        Args:
            twist: The desired velocity
            set_control_types: Whether all axes should be set to DESIRED_VELOCITY
        """
        if set_control_types:
            self._set_all_axes_control_type(ControlTypes.DESIRED_VELOCITY)
        self._desired_velocity_pub.publish(twist)

    def publish_desired_power(self, power: Twist, set_control_types: bool = True) -> None:
        """
        Publish the desired power.

        Args:
            power: The desired power
            set_control_types: Whether all axes should be set to DESIRED_POWER
        """
        if set_control_types:
            self._set_all_axes_control_type(ControlTypes.DESIRED_POWER)
        self._desired_power_pub.publish(power)

    def publish_thruster_allocs(self, **kwargs) -> None:
        """
        Publish the thruster allocations.

        Args:
            kwargs: The thruster allocations

        Raises:
            ValueError: If the thruster name is not in thruster_dict
            ValueError: If the thruster alloc is not between -1 and 1 inclusive
        """
        thruster_allocs = [0] * self.num_thrusters

        for kwarg_name, kwarg_value in kwargs.items():

            if kwarg_name not in self.thruster_dict:
                msg = f'Thruster name not in thruster_dict {kwarg_name}'
                raise ValueError(msg)

            if kwarg_value > 1 or kwarg_value < -1:
                msg = (
                    f'Received {kwarg_value} for thruster {kwarg_name}. Thruster alloc must be between -1 and 1 '
                    'inclusive.'
                )
                raise ValueError(msg)

            thruster_allocs[self.thruster_dict[kwarg_name]] = kwarg_value

        thruster_allocs_msg = ThrusterAllocs()
        thruster_allocs_msg.header.stamp = self.node.get_clock().now()
        thruster_allocs_msg.allocs = thruster_allocs

        self._thruster_pub.publish(thruster_allocs_msg)
