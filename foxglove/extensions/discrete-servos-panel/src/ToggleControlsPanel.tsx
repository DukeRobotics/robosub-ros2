import { StdSrvs, StdMsgs } from "@duke-robotics/defs/types";
import { CustomMsgs } from "@duke-robotics/defs/types";
import useTheme from "@duke-robotics/theme";
import { ThemeProvider } from "@emotion/react";
import { PanelExtensionContext, RenderState, Immutable, MessageEvent } from "@foxglove/extension";
import { Typography } from "@mui/material";
import Alert from "@mui/material/Alert";
import Box from "@mui/material/Box";
import Button from "@mui/material/Button/Button";
import React, { useState, useEffect } from "react";
import { createRoot } from "react-dom/client";

// Array of discrete servo panel objects
const DISCRETE_SERVOS = [
  { name: "Marker Dropper", serviceName: "/servos/marker_dropper", states: ["left", "right"] },
  { name: "Torpedoes", serviceName: "/servos/torpedoes", states: ["left", "right"] },
];

type ToggleControlsPanel = {
  error?: Error | undefined; // Error object if service call fails
  response?: string | undefined; // Response from service call
};

function ToggleControlsPanel({ context }: { context: PanelExtensionContext }): React.JSX.Element {
  const [panelState, setState] = useState<ToggleControlsPanel>({});
  const [renderDone, setRenderDone] = useState<() => void | undefined>();

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // Render and watch renderState
  // Save the value from the most recent message
  useEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);
    };

    context.watch("currentFrame");
  }, [context]);

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
    context.callService("/servos/torpedoes", request).then(
      (response) => {
        console.log(JSON.stringify(response)); // Attempt serializing the response, to throw an error on failure
      },
      (error: unknown) => {
        // Handle service call errors (e.g., service is not advertised)
        setState((oldState) => ({ ...oldState, error: error as Error }));
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
                    callService(servo.serviceName, state);
                  }}
                >
                  {state}
                </Button>
              ))}
            </>
          ))}
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
