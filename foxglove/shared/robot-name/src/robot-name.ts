import { Immutable, PanelExtensionContext, RenderState } from "@foxglove/extension";
import { useCallback, useLayoutEffect, useState } from "react";

/**
 * The values correspond to the recognized values of the `ROBOT_NAME` Foxglove variable.
 */
export enum Robot {
  Oogway = "oogway",
  Crush = "crush",
}
export const ROBOTS: Robot[] = Object.values(Robot);

export function isRobotName(name: unknown): name is Robot {
  if (typeof name !== "string") {
    return false;
  }
  return ROBOTS.includes(name as Robot);
}

const ROBOT_NAME_VAR = "ROBOT_NAME";
export function getRobotName(renderState: Immutable<RenderState>): Robot | undefined {
  const robotNameVarValue = renderState.variables?.get(ROBOT_NAME_VAR);
  return isRobotName(robotNameVarValue) ? robotNameVarValue : undefined;
}

type PanelRender = NonNullable<PanelExtensionContext["onRender"]>;

/**
 * Returns the current robot name (if set) and a context.onRender decorator to update the robot name.
 */
export function useRobotName(context: PanelExtensionContext): {
  robotName: Robot | undefined;
  withRobotName: (handler: PanelRender) => PanelRender;
} {
  const [robotName, setRobotName] = useState<Robot | undefined>(undefined);

  const withRobotName = useCallback(
    (handler: PanelRender): PanelRender =>
      (renderState, done) => {
        setRobotName(getRobotName(renderState));
        handler(renderState, done);
      },
    [],
  );

  useLayoutEffect(() => {
    context.watch("variables");
  }, [context]);

  return { robotName, withRobotName };
}
