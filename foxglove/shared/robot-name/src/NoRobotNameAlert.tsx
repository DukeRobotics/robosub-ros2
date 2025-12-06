import { PanelExtensionContext } from "@foxglove/extension";
import { Box, Link } from "@mui/material";
import Alert from "@mui/material/Alert";
import React from "react";

import { Robot, ROBOTS } from "./robot-name";

export function NoRobotNameAlert({
  robotName,
  context,
}: {
  robotName: Robot | undefined;
  context: PanelExtensionContext;
}): React.JSX.Element | null {
  if (robotName != undefined) {
    return null;
  }

  const setRobotName = (name: Robot) => {
    context.setVariable("ROBOT_NAME", name);
  };

  return (
    <Box mb={1}>
      <Alert variant="filled" severity="error">
        The ROBOT_NAME Foxglove variable is not defined. Valid options:{" "}
        {ROBOTS.map((name, index) => (
          <React.Fragment key={name}>
            <Link
              onClick={() => {
                setRobotName(name);
              }}
            >
              {name}
            </Link>
            {index < ROBOTS.length - 1 ? ", " : "."}
          </React.Fragment>
        ))}
      </Alert>
    </Box>
  );
}
