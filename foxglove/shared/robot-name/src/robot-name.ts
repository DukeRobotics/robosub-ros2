import { Immutable, RenderState } from "@foxglove/extension";

/**
 * The values correspond to the recognized values of the `ROBOT_NAME` Foxglove variable.
 */
export enum Robot {
  Oogway = "oogway",
  Crush = "crush",
}
export const allRobots: Robot[] = Object.values(Robot);

export function isRobotName(name: unknown): name is Robot {
  if (typeof name !== "string") {
    return false;
  }
  return allRobots.includes(name.toLowerCase() as Robot);
}

const ROBOT_NAME_VAR = "ROBOT_NAME";
export function getRobotName(renderState: Immutable<RenderState>): Robot | undefined {
  const robotNameVarValue = renderState.variables?.get(ROBOT_NAME_VAR);
  return isRobotName(robotNameVarValue) ? robotNameVarValue : undefined;
}
