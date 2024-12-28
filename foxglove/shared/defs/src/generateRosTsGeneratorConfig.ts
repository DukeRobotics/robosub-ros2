import { readdirSync, statSync, writeFileSync } from "fs";
import { join } from "path";

/**
 * Generate a ROS 2 TypeScript generator configuration file.
 * The generated file is used as input to the ros-typescript-generator package.
 * See https://github.com/Greenroom-Robotics/ros-typescript-generator for more information.
 * @param rosSharePath - Path to the ROS 2 share directory.
 * @param configPath - Path where the JSON configuration file will be written.
 * @param generatedTypesPath - Path where the generated TypeScript types will be written.
 * @param additionalInputs - Array of additional inputs to include in the configuration.
 */
export function generateRosTsGeneratorConfig(
  rosSharePath: string,
  configPath: string,
  generatedTypesPath: string,
  additionalInputs: { namespace: string; path: string }[] = [],
): void {
  // Get all directories in the ROS share path
  const allDirectories = readdirSync(rosSharePath)
    .map((name) => join(rosSharePath, name))
    .filter((path) => statSync(path).isDirectory());

  // Filter directories to only include those ending in '_srvs' or '_msgs'
  const filteredDirectories = allDirectories.filter((dir) => dir.endsWith("_srvs") || dir.endsWith("_msgs"));

  // Create the JSON configuration object
  const outputData = {
    output: generatedTypesPath,
    rosVersion: 2,
    useNamespaces: true,
    smartEnums: true,
    input: [
      ...additionalInputs,
      ...filteredDirectories.map((dirName) => ({
        namespace: dirName.split("/").pop(),
        path: dirName,
      })),
    ],
  };

  // Write the JSON to the specified path
  writeFileSync(configPath, JSON.stringify(outputData, null, 2));
}
