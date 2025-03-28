import { ros2jazzy } from "@foxglove/rosmsg-msgs-common";

import { writeAllDatatypeMaps } from "./datatypeMaps";
import { generateRosTsGeneratorConfig } from "./generateRosTsGeneratorConfig";
import { writeMessageDefinitions } from "./messageDefinitions";

// The following paths are relative to the src directory containing this script
// For this script to work correctly, this script must be run from the src directory
const CUSTOM_MSGS_DEFS_PATH = "../../../core/src/custom_msgs/msg";
const CUSTOM_MSGS_SAVE_DIR = "custom_msg_defs/dist";

const DATATYPE_MAPS_SAVE_DIR = "datatype_maps/dist";
const MSG_DEFS_RELATIVE_PATH = "../../custom_msg_defs/dist";

async function main() {
  generateRosTsGeneratorConfig(
    "/opt/ros/jazzy/share/",
    "ros_ts_generator_configs/dist/ros-ts-generator-config.json",
    "../../types/dist/types.ts",
    [{ namespace: "custom_msgs", path: "../../../../../core/src/custom_msgs" }],
  );

  // Get message definitions for custom msgs and export them to files
  const definitionsByGroup = await writeMessageDefinitions(CUSTOM_MSGS_DEFS_PATH, CUSTOM_MSGS_SAVE_DIR);

  // Add definitions of ROS 1 built-in message types to the ros2jazzy group
  definitionsByGroup.set("ros2jazzy", ros2jazzy);

  // Generate and export datatype maps for all message definitions, including custom msgs and ROS 1 built-in msgs
  await writeAllDatatypeMaps(definitionsByGroup, DATATYPE_MAPS_SAVE_DIR, MSG_DEFS_RELATIVE_PATH);
}

void main();
