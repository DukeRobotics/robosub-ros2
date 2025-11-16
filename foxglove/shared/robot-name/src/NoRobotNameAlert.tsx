import { Box } from "@mui/material";
import Alert from "@mui/material/Alert";
import React from "react";

import { Robot, ROBOTS } from "./robot-name";

export function NoRobotNameAlert({ robotName }: { robotName: Robot | undefined }): React.JSX.Element | null {
  if (robotName != undefined) {
    return null;
  }

  return (
    <Box mb={1}>
      <Alert variant="filled" severity="error">
        {`The ROBOT_NAME Foxglove variable is not defined. Valid options: {${ROBOTS.map((name) => `"${name}"`).join(", ")}}.`}
      </Alert>
    </Box>
  );
}
