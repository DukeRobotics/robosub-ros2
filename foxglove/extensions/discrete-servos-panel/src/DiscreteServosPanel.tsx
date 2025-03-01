import { CustomMsgs } from "@duke-robotics/defs/types";
import useTheme from "@duke-robotics/theme";
import { ThemeProvider } from "@emotion/react";
import { PanelExtensionContext } from "@foxglove/extension";
import { ButtonGroup, Stack, Typography } from "@mui/material";
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

type DiscreteServosPanel = {
  error?: Error | undefined; // Error object if service call fails
  response?: string | undefined; // Response from service call
  lastServiceCall?: { message: string; success: boolean } | undefined; // Last service call
};

function DiscreteServosPanel({ context }: { context: PanelExtensionContext }): React.JSX.Element {
  const [panelState, setState] = useState<DiscreteServosPanel>({});

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
        JSON.stringify(response); // Attempt serializing the response, to throw an error on failure

        setState((oldState) => ({
          ...oldState,
          lastServiceCall: { message: response.message, success: response.success },
        }));
      },
      (error: unknown) => {
        // Handle service call errors (e.g., service is not advertised)
        setState((oldState) => ({
          ...oldState,
          error: error as Error,
        }));
      },
    );
  };

  console.log("DiscreteServosPanel:", panelState.error);

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

        <Stack spacing={2}>
          {DISCRETE_SERVOS.map((servo) => (
            <Box key={servo.name}>
              <Typography>{servo.name}</Typography>
              <ButtonGroup variant="outlined" color="primary">
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
          <>
            {panelState.lastServiceCall?.message && (
              <Alert variant="filled" severity={panelState.lastServiceCall.success ? "success" : "error"}>
                {panelState.lastServiceCall.message}
              </Alert>
            )}
          </>
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
