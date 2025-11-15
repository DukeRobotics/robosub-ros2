# Robot Name

This package exports the `Robot` enum and related utilities used for implementing robot-specific behavior.


## Usage

Users should set the `ROBOT_NAME` [Foxglove variable](https://docs.foxglove.dev/docs/visualization/variables) to the name of the current robot.

To access the robot name, pass the render state to `getRobotName`.
```js
import { Robot, getRobotName } from "@duke-robotics/robot-name";

function ExamplePanel({ context }: { context: PanelExtensionContext }): React.JSX.Element {
    useLayoutEffect(() => {
        context.onRender = (renderState: Immutable<RenderState>, done) => {
            // undefined means the ROBOT_NAME value is not recognized
            const robotName: Robot | undefined = getRobotName(renderState);
            ...
        };
    }, [context]);
}
```
