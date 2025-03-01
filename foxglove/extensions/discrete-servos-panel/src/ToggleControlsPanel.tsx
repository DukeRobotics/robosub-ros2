import { CustomMsgs } from "@duke-robotics/defs/types";
import useTheme from "@duke-robotics/theme";
import { ThemeProvider } from "@emotion/react";
import { PanelExtensionContext } from "@foxglove/extension";
import { Typography } from "@mui/material";
import Alert from "@mui/material/Alert";
import Box from "@mui/material/Box";
import Button from "@mui/material/Button/Button";
import React, { useState } from "react";
import { createRoot } from "react-dom/client";

interface DiscreteServo {
  name: string;
  service: string;
  states: string[];
}

// Array of discrete servo panel objects
const DISCRETE_SERVOS: DiscreteServo[] = [
  { name: "Marker Dropper", service: "/servos/marker_dropper", states: ["left", "right"] },
  { name: "Torpedoes", service: "/servos/torpedoes", states: ["left", "right"] },
];

type ToggleControlsPanel = {
  error?: Error | undefined; // Error object if service call fails
  response?: string | undefined; // Response from service call
  lastServiceCall?: { response: string; state: boolean } | undefined; // Last service call
};

function ToggleControlsPanel({ context }: { context: PanelExtensionContext }): React.JSX.Element {
  const [panelState, setState] = useState<ToggleControlsPanel>({});

  // Call the /enable_controls service to toggle controls
  const callService = (service: string, state: string) => {
    console.log(service, state);
    // Check if service calling is supported by the context
    if (!context.callService) {
      console.error("Calling services is not supported by this connection");
      return;
    }

    // Request payload to toggle controls
    const request: CustomMsgs.SetDiscreteServoRequest = { state };
    // Make the service call
    (context.callService(service, request) as Promise<CustomMsgs.SetDiscreteServoResponse>).then(
      (response) => {
        console.log(JSON.stringify(response)); // Attempt serializing the response, to throw an error on failure

        setState((oldState) => ({ ...oldState, lastServiceCall: { response: response.message, state: response.success } }));
      },
      (error: unknown) => {
        // Handle service call errors (e.g., service is not advertised)
        setState((oldState) => ({
          ...oldState,
          error: error as Error,
          lastServiceCall: { response: error as string, state: false },
        }));
      },
    );
  };

  const theme = useTheme();
  return (
    <ThemeProvider theme={theme}>
      <Box m={1}>
        {/* Error messages */}
        {(context.callService == undefined || panelState.error != undefined) && (
          <Box mb={1}>
            {context.callService == undefined && (
              <Alert variant="filled" severity="error">
                Calling services is not supported by this connection.
              </Alert>
            )}
            {panelState.error != undefined && (
              <Alert variant="filled" severity="error">
                {panelState.error.message}
              </Alert>
            )}
          </Box>
        )}

        {/* Toggle button */}
        <>
          {DISCRETE_SERVOS.map((servo) => (
            <>
              <Typography key={servo.name}>{servo.name}</Typography>
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
            </>
          ))}
          <>
            <Alert variant="filled" severity={(panelState.lastServiceCall?.state ?? false) ? "success" : "error"}>
              {panelState.lastServiceCall?.response}
            </Alert>
          </>
        </>
        {/*
        <Button
          fullWidth
          variant="contained"
          color={state.controlsEnabled ? "error" : "success"}
          onClick={toggleControls}
          disabled={context.callService == undefined}
        >
          {state.controlsEnabled ? "Disable Controls" : "Enable Controls"}
        </Button> */}
      </Box>
    </ThemeProvider>
  );
}

export function initToggleControlsPanel(context: PanelExtensionContext): () => void {
  context.panelElement.style.overflow = "auto"; // Enable scrolling

  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<ToggleControlsPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
