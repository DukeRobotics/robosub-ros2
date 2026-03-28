# Robot Name
This package exports the `Robot` enum and related utilities used for implementing robot-specific behavior.

Users should set the `ROBOT_NAME` [Foxglove variable](https://docs.foxglove.dev/docs/visualization/variables) to the name of the current robot.

## Usage
To access the robot name, decorate the panel's `onRender` handler with `withRobotName`.
```js
import { Robot, useRobotName } from "@duke-robotics/robot-name";

function ExamplePanel({ context }: { context: PanelExtensionContext }): React.JSX.Element {
    const { robotName, withRobotName } = useRobotName(context);

    useLayoutEffect(() => {
        context.onRender = withRobotName((renderState: Immutable<RenderState>, done) => {
            // Panel-specific work...
            done();
        });
    }, [context, withRobotName]);

    if (robotName === Robot.Crush) {
        // Do Crush-specific work...
    }

    return (
        <Box>
            {/* Alert user if no robot name is set */}
            <NoRobotNameAlert robotName={robotName} context={context} />
        </Box>
    );
}
```
