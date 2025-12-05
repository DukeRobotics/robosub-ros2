# Toggle keyboard Panel
This is a panel to toggle keyboard control on/off, as well as to publish transformed keyboard inputs as a desired power.

Keyboard inputs consist of movement in the x, y, z, roll, pitch, and yaw axes.

When keyboard control is toggled on, a request is sent to `/controls/set_control_types` toggling each control to `DESIRED_POWER` mode. Keyboard inputs are now read and interpreted as power in each direction.

When keyboard control is toggled off, a request is sent to `/controls/set_control_types` toggling each control to `DESIRED_POSITION` mode. Keyboard inputs are now ignored.

When keyboard control is enabled, keyboard inputs are published to `/controls/desired_power`.

## Usage

Ensure that controls is running:
```bash
roslaunch controls controls.launch
```

Click the button to toggle keyboard control. If the call fails, an alert is displayed.
