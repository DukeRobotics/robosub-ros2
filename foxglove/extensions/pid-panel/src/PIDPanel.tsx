import { CustomMsgs } from "@duke-robotics/defs/types";
import useTheme from "@duke-robotics/theme";
import { Immutable, PanelExtensionContext, RenderState, MessageEvent } from "@foxglove/extension";
import {
  Alert,
  Tab,
  Tabs,
  TableBody,
  TableContainer,
  TableCell,
  TableRow,
  TableHead,
  Table,
  Box,
  Button,
  TextField,
} from "@mui/material";
import Grid from "@mui/material/Grid2";
import { ThemeProvider } from "@mui/material/styles";
import React, { useEffect, useState } from "react";
import { JSX } from "react/jsx-runtime";
import { createRoot } from "react-dom/client";

const PID_TOPIC = "/controls/pid_gains";
const PID_SERVICE = "/controls/set_pid_gains";

type PIDPanelState = {
  error?: Error;
  loop: CustomMsgs.PidGainLoop;
  editedGains: Record<number, Record<number, number>>; // Indexed by axis and gain type
  pid: Record<number, Record<number, Record<number, number>>>; // Indexed by loop, axis, and gain type
};

// Triple nested object to store current PID gains
const initPid = () => {
  // Initialize gains to -1 to signify that the true values have not been received yet
  const gains: Record<number, number> = {
    [CustomMsgs.PidGainGain.KP]: -1,
    [CustomMsgs.PidGainGain.KI]: -1,
    [CustomMsgs.PidGainGain.KD]: -1,
    [CustomMsgs.PidGainGain.FF]: -1,
  };
  const axes: Record<number, Record<number, number>> = {
    [CustomMsgs.PidGainAxis.X]: { ...gains },
    [CustomMsgs.PidGainAxis.Y]: { ...gains },
    [CustomMsgs.PidGainAxis.Z]: { ...gains },
    [CustomMsgs.PidGainAxis.ROLL]: { ...gains },
    [CustomMsgs.PidGainAxis.PITCH]: { ...gains },
    [CustomMsgs.PidGainAxis.YAW]: { ...gains },
  };
  const loops: Record<number, Record<number, Record<number, number>>> = {
    [CustomMsgs.PidGainLoop.POSITION]: { ...axes },
    [CustomMsgs.PidGainLoop.VELOCITY]: { ...axes },
  };

  return loops;
};

function PIDPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  // Initialize state
  const [state, setState] = useState<PIDPanelState>(() => {
    const initialState = context.initialState as PIDPanelState | undefined;

    return {
      loop: initialState?.loop ?? CustomMsgs.PidGainLoop.POSITION,
      editedGains: {},
      error: undefined,
      pid: initPid(),
    };
  });

  // Save state upon change
  useEffect(() => {
    context.saveState(state);
  }, [state, context]);

  context.subscribe([{ topic: PID_TOPIC }]);

  // Update state when a new PID message is received
  useEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);
      if (renderState.currentFrame && renderState.currentFrame.length > 0) {
        const lastFrame = renderState.currentFrame.at(-1) as MessageEvent<CustomMsgs.PidGains>;

        // Loop through received pid_gains and update our PID values
        const pidGains = lastFrame.message.pid_gains;

        const updatedPid = { ...state.pid }; // Create a copy of the current pid state
        for (const pidGain of pidGains) {
          updatedPid[pidGain.loop]![pidGain.axis] = {
            ...updatedPid[pidGain.loop]![pidGain.axis],
            [pidGain.gain]: pidGain.value,
          };
        }
        setState((prevState) => ({
          ...prevState,
          pid: updatedPid,
        }));
      }
    };
  }, [context, state.pid]);
  context.watch("currentFrame");

  // Call our done function at the end of each render
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // Call a CustomMsgsSetPidGains service with a given request
  const callService = (serviceName: string, request: CustomMsgs.SetPidGainsRequest) => {
    if (!context.callService) {
      return;
    }

    (context.callService(serviceName, request) as Promise<CustomMsgs.SetPidGainsResponse>).then(
      (response) => {
        // Update the state based on the service response
        // If the service responds with failure, display the response message as an error
        const error = response.success ? undefined : Error(response.message);
        setState((prevState) => ({
          ...prevState,
          error,
        }));
      },
      (error: unknown) => {
        // Handle service call errors (e.g., service is not advertised)
        setState((prevState) => ({
          ...prevState,
          error: error as Error,
        }));
      },
    );
  };

  /**
   * Switch between position and velocity PID loops.
   * @param {CustomMsgsPidGainLoop} desiredLoop
   */
  const handleLoopChange = (_: React.ChangeEvent<unknown>, desiredLoop: CustomMsgs.PidGainLoop) => {
    setState((prevState) => ({ ...prevState, loop: desiredLoop }));
    handleReset(); // Reset the edited gains when switching loops
  };

  /**
   * Convert an axis enum value to its name. See {@link CustomMsgsPidGainConst} for possible axis enum values.
   * @param {number} enumValue
   * @return {string} Axis enum name
   */
  function getAxisEnumName(enumValue: number): string {
    // Get enum entries that start with "AXIS" and have the given value
    const filteredEnumEntries = Object.entries(CustomMsgs.PidGainAxis).filter(([_, value]) => {
      return Number(value) === enumValue;
    });

    if (filteredEnumEntries.length === 1 && filteredEnumEntries[0] != null) {
      return filteredEnumEntries[0][0];
    } else if (filteredEnumEntries.length > 1) {
      console.error(`Multiple axis enum names map to value ${enumValue}`);
      return "Unknown";
    } else {
      console.error(`Could not find axis enum name for value ${enumValue}`);
      return "Unknown";
    }
  }

  /**
   * Update a given axis and gain type of editedGains with a new value.
   * @param {number} value - New gain value
   * @param {number} axis - Axis enum value
   * @param {number} gainType - Gain enum value
   */
  const updateEditedGains = (value: number, axis: number, gainType: number) => {
    setState((prevState) => ({
      ...prevState,
      editedGains: {
        ...prevState.editedGains,
        [axis]: {
          ...prevState.editedGains[axis],
          [gainType]: value,
        },
      },
    }));
  };

  /**
   * Reset the edited gains to an empty object.
   */
  const handleReset = () => {
    setState((prevState) => ({
      ...prevState,
      editedGains: {},
    }));
  };

  /**
   * Submit {@link state.editedGains} to the {@link PID_SERVICE} and reset.
   */
  const handleSubmit = () => {
    const request: CustomMsgs.SetPidGainsRequest = {
      // eslint-disable-next-line camelcase
      pid_gains: [],
    };

    Object.entries(state.editedGains).forEach(([axis, gains]) => {
      Object.entries(gains).forEach(([gain, value]) => {
        request.pid_gains.push({
          loop: state.loop,
          axis: Number(axis),
          gain: Number(gain),
          value: Number(value),
        });
      });
    });

    callService(PID_SERVICE, request);
    handleReset();
  };

  const theme = useTheme();
  return (
    <ThemeProvider theme={theme}>
      <Box m={1}>
        {/* Error messages */}
        {(context.callService == undefined || state.error != undefined) && (
          <Box mb={1}>
            {context.callService == undefined && (
              <Alert variant="filled" severity="error">
                Calling services is not supported by this connection.
              </Alert>
            )}
            {state.error != undefined && (
              <Alert variant="filled" severity="error">
                {state.error.message}
              </Alert>
            )}
          </Box>
        )}

        {/* PID Loop Tabs */}
        <Tabs value={state.loop} onChange={handleLoopChange} variant="fullWidth">
          <Tab label="Position" value={CustomMsgs.PidGainLoop.POSITION} />
          <Tab label="Velocity" value={CustomMsgs.PidGainLoop.VELOCITY} />
        </Tabs>

        {/* PID Gains Table */}
        <TableContainer>
          <Table size="small">
            <TableHead>
              <TableRow>
                <TableCell />
                <TableCell style={{ textAlign: "center" }}>KP</TableCell>
                <TableCell style={{ textAlign: "center" }}>KI</TableCell>
                <TableCell style={{ textAlign: "center" }}>KD</TableCell>
                <TableCell style={{ textAlign: "center" }}>FF</TableCell>
              </TableRow>
            </TableHead>
            <TableBody>
              {/* Loop through all axes/gain types */}
              {Object.entries(state.pid[state.loop] ?? {}).map(([axis, gains]) => (
                <TableRow key={axis}>
                  {/* Axis Label */}
                  <TableCell>{getAxisEnumName(Number(axis))}</TableCell>

                  {/* Axis Gains */}
                  {Object.values(gains).map((gain, gainType) => {
                    const inEditMode = gainType in (state.editedGains[Number(axis)] ?? {});
                    return (
                      <TableCell key={gainType} sx={{ padding: "0" }}>
                        {/* TODO: TextField doesn't take up full width of TableCell when panel is big */}
                        <TextField
                          type="number"
                          sx={{
                            "& input": {
                              boxSizing: "border-box",
                            },
                            "& .MuiOutlinedInput-root": {
                              "& fieldset": {
                                // Border color is red if in edit mode, otherwise divider color
                                borderColor: inEditMode ? theme.palette.error.main : theme.palette.divider,
                              },
                            },
                          }}
                          // Display the true gain value if not in edit mode
                          value={inEditMode ? undefined : gain}
                          onFocus={() => {
                            // Add the current gain to editedGains when entering edit mode
                            updateEditedGains(state.pid[state.loop]![Number(axis)]![gainType]!, Number(axis), gainType);
                          }}
                          onChange={(event: React.ChangeEvent<HTMLInputElement>) => {
                            // Update editedGains when the input value changes
                            updateEditedGains(Number(event.target.value), Number(axis), gainType);
                          }}
                          slotProps={{
                            htmlInput: { step: 0.1 },
                          }}
                        />
                      </TableCell>
                    );
                  })}
                </TableRow>
              ))}
            </TableBody>
          </Table>
        </TableContainer>

        {/* Buttons */}
        <Box my={1}>
          <Grid container spacing={1}>
            <Grid size={{ xs: 6 }}>
              <Button variant="contained" fullWidth onClick={handleReset} color="error">
                Reset
              </Button>
            </Grid>
            <Grid size={{ xs: 6 }}>
              <Button
                variant="contained"
                fullWidth
                onClick={handleSubmit}
                color="success"
                disabled={context.callService == undefined}
              >
                Submit
              </Button>
            </Grid>
          </Grid>
        </Box>
      </Box>
    </ThemeProvider>
  );
}

export function initPIDPanel(context: PanelExtensionContext): () => void {
  context.panelElement.style.overflow = "auto"; // Enable scrolling

  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<PIDPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
