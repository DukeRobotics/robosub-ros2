# Foxglove Definitions
This package exports datatype maps and TypeScript interfaces/enums for both ROS 1 and Duke Robotics custom message definitions.
Datatype maps are needed to advertise topics. TypeScript interfaces are used for static type checking.

## Usage
### Datatype Maps
Foxglove requires a [datatype map](https://docs.foxglove.dev/docs/visualization/extensions/api/panel-api#native-ros-1)
when advertising a topic in an extension. This package generates the dataype maps of all ROS 1 and Duke Robotics custom definitions.

Import with:
```js
import { allDatatypeMaps } from "@duke-robotics/defs/datatype_maps";
```
Then, advertise using the appropriate map. For example, to advertise thruster allocs, use:
```js
context.advertise(`<Topic>`, "custom_msgs/ThrusterAllocs", {
    datatypes: allDatatypeMaps.custom_msgs["custom_msgs/ThrusterAllocs"],
});
```

### TypeScript Interfaces/Enums
To enforce static type checking of ROS messages in Foxglove extensions, an interface must be specified
that describes the message's shape. This package uses [ros-typescript-generator](https://github.com/Greenroom-Robotics/ros-typescript-generator) to translate all ROS 2 and Duke Robotics custom definitions
to TypeScript interfaces.

Import with:
```js
import { <Namespace> } from "@duke-robotics/defs/types";
```

To view all available types, see `foxglove/shared/defs/types/dist/types.ts`.

#### Interface Translation
Interface names are translated from **snake_case** to **PascalCase** and are organized into namespaces according to the package name.

##### Messages
[ROS messages](http://wiki.ros.org/msg) are translated directly to one interface (with no suffix).

##### Services
[ROS services](http://wiki.ros.org/Services) are split into two interfaces.
- The *request* (before the `---`) has the suffix `request`.
- The *reply* (after the `---`) has the suffix `response`.

##### Actions
[ROS actions](http://wiki.ros.org/actionlib) are split into three interfaces.
- The *goal* has the suffix `ActionGoal`
- The *feedback* has the suffix `ActionFeedback`
- The *result* has the suffix `ActionResult`

##### Constants
Constants are translated into an enum with the suffix `Const`.

## Acknowledgements
- [rosmsg-msgs-common](https://github.com/foxglove/rosmsg-msgs-common)
- [ros-typescript-generator](https://github.com/Greenroom-Robotics/ros-typescript-generator)
