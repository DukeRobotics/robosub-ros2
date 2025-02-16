# Toggle Controls Panel
This is a panel to toggle controls on/off.

## Usage
Ensure that the `/controls/enable` service is running. Click the button to toggle controls.
If the call fails, an alert is displayed.

## Configuration
The following constants can be modified:
- `ENABLE_CONTROLS_SERVICE`: The service that handles enabling/disabling controls.
- `CONTROLS_STATUS_TOPIC`: The topic that publishes the controls status.