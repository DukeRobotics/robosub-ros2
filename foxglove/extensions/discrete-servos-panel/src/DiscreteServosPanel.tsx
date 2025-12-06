import { CustomMsgs } from "@duke-robotics/defs/types";
import { NoRobotNameAlert, Robot, useRobotName } from "@duke-robotics/robot-name";
import useTheme from "@duke-robotics/theme";
import { ThemeProvider } from "@emotion/react";
import { PanelExtensionContext } from "@foxglove/extension";
import { ButtonGroup, Stack, Typography } from "@mui/material";
import Alert from "@mui/material/Alert";
import Box from "@mui/material/Box";
import Button from "@mui/material/Button/Button";
import React, { useLayoutEffect, useMemo, useState, useEffect } from "react";
import { createRoot } from "react-dom/client";

interface DiscreteServo {
  name: string;
  service: string;
  states: string[];
}

const ROBOT_SERVOS: Record<Robot, DiscreteServo[]> = {
  [Robot.Crush]: [{ name: "Marker Dropper", service: "/servos/marker_dropper", states: ["left", "right"] }],
  [Robot.Oogway]: [{ name: "Torpedo", service: "/servos/torpedo", states: ["left", "right"] }],
};

type DiscreteServosPanel = {
  message?: string; // Response from service call
  success?: boolean; // Whether the service call was successful
};

function DiscreteServosPanel({ context }: { context: PanelExtensionContext }): React.JSX.Element {
  const [panelState, setPanelState] = useState<DiscreteServosPanel>();
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const { robotName, withRobotName } = useRobotName(context);

  useLayoutEffect(() => {
    context.onRender = withRobotName((_, done) => {
      setRenderDone(() => done);
    });
  }, [context, withRobotName]);

  // Send a SetDiscreteServoRequest to the specified service
  const callService = (service: string, state: string) => {
    if (!context.callService) {
      console.error("Calling services is not supported by this connection");
      return;
    }

    // Send the service request
    const request: CustomMsgs.SetDiscreteServoRequest = { state };
    (context.callService(service, request) as Promise<CustomMsgs.SetDiscreteServoResponse | undefined>).then(
      (response) => {
        if (response) {
          setPanelState((oldState) => ({
            ...oldState,
            message: response.message,
            success: response.success,
          }));
        } else {
          setPanelState((oldState) => ({
            ...oldState,
            message: `No response from service ${service}.`,
            success: false,
          }));
        }
      },
      (error: unknown) => {
        // Handle service call errors (e.g., service is not advertised)
        setPanelState((oldState) => ({
          ...oldState,
          message: (error as Error).message,
          success: false,
        }));
      },
    );
  };

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const robotServos = useMemo(() => (robotName != undefined ? ROBOT_SERVOS[robotName] : []), [robotName]);

  const theme = useTheme();
  return (
    <ThemeProvider theme={theme}>
      <Box m={1}>
        <NoRobotNameAlert robotName={robotName} context={context} />
        {context.callService == undefined && (
          <Box mb={1}>
            <Alert variant="filled" severity="error">
              Calling services is not supported by this connection.
            </Alert>
          </Box>
        )}

        <Stack spacing={1}>
          {robotServos.map((servo) => (
            <Box key={servo.name}>
              <Typography>{servo.name}</Typography>
              <ButtonGroup variant="outlined" disabled={context.callService == undefined}>
                {servo.states.map((state) => (
                  <Button
                    key={state}
                    onClick={() => {
                      callService(servo.service, state);
                    }}
                  >
                    {state}
                  </Button>
                ))}
              </ButtonGroup>
            </Box>
          ))}
          {panelState?.message && (
            <Alert variant="filled" severity={(panelState.success ?? false) ? "success" : "error"}>
              {panelState.message}
            </Alert>
          )}
        </Stack>
      </Box>
    </ThemeProvider>
  );
}

export function initDiscreteServosPanel(context: PanelExtensionContext): () => void {
  context.panelElement.style.overflow = "auto"; // Enable scrolling

  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<DiscreteServosPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
