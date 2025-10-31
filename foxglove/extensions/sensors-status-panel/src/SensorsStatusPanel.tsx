import useTheme from "@duke-robotics/theme";
import { Immutable, PanelExtensionContext, RenderState, MessageEvent, Subscription } from "@foxglove/extension";
import { Password } from "@mui/icons-material";
import { Box, Paper, Table, TableBody, TableCell, TableContainer, TableRow, Typography, Tooltip } from "@mui/material";
import Alert from "@mui/material/Alert";
import { ThemeProvider } from "@mui/material/styles";
import React, { useEffect, useState, useLayoutEffect } from "react";
import { createRoot } from "react-dom/client";

const robotSensorMap = new Map<string, Array<string>>([
  ["oogway", ["DVL", "IMU", "Pressure", "Gyro"]],
  ["crush", ["Front Mono", "Bottom Mono", "Sonar", "IVC Modem", "IMU"]],
]);

const ALL_TOPICS_MAP = {
  DVL: "/sensors/dvl/raw",
  IMU: "/vectornav/imu",
  Pressure: "/sensors/depth",
  Gyro: "/sensors/gyro/status",
  "Front DAI": "/camera/front/rgb/preview/compressed",
  "Front Mono": "/camera/usb/front/compressed",
  "Bottom Mono": "/camera/usb/bottom/compressed",
  Sonar: "/sonar/status",
  "IVC Modem": "/sensors/modem/status",
};
let robotSpecificTopics: Record<string, string> = {}; // Robot-specific topics map
let varDict: Record<string, string> = {}; // Foxglove environment vars
const robotNames = new Set<string>([...robotSensorMap.keys()]); // Set of acceptable robot var names
type topicsMapKeys = keyof typeof ALL_TOPICS_MAP;
// Seconds until sensor is considered disconnected
const SENSOR_DOWN_THRESHOLD = 1;
const TOPICS_MAP_REVERSED: Record<string, topicsMapKeys> = {};
for (const [key, value] of Object.entries(ALL_TOPICS_MAP)) {
  TOPICS_MAP_REVERSED[value] = key as topicsMapKeys;
}
let visibleSensors: string[] | undefined = [];
let visibleSubscriptions: string[] | undefined = [];
// Array of all topics: [{topic: topic1}, {topic: topic2}, ... ]
// const TOPICS_LIST: Subscription[] = [];
// for (const value of Object.values(TOPICS_MAP)) {
//   TOPICS_LIST.push({ topic: value });
// }
// Time of last message received from sensor
type SensorsTime = Record<topicsMapKeys, number>;
// True if SensorsTime is within SENSOR_DOWN_THRESHOLD seconds
type ConnectStatus = Record<topicsMapKeys, boolean>;
type SensorsStatusPanelState = {
  sensorsTime: SensorsTime;
  connectStatus: ConnectStatus;
  currentTime: number;
};
const initState = () => {
  const state: Partial<SensorsStatusPanelState> = {};
  // Initialize sensorsTime with 0's
  const sensorsTime: Partial<SensorsTime> = {};
  for (const key in ALL_TOPICS_MAP) {
    sensorsTime[key as keyof SensorsTime] = 0;
  }
  state.sensorsTime = sensorsTime as SensorsTime;
  // Initialize connectStatus with false's
  const connectStatus: Partial<ConnectStatus> = {};
  for (const key in ALL_TOPICS_MAP) {
    connectStatus[key as keyof ConnectStatus] = false;
  }
  state.connectStatus = connectStatus as ConnectStatus;
  // Initialize currentTime with Infinity
  // This ensures that (currentTime - sensorsTime > SENSOR_DOWN_THRESHOLD) so that the sensor is initially considered disconnected
  state.currentTime = Infinity;
  return state as SensorsStatusPanelState;
};
function SensorsStatusPanel({ context }: { context: PanelExtensionContext }): React.JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [state, setState] = useState<SensorsStatusPanelState>(initState());
  const [envVars, setEnvVars] = useState<Record<string, string>>({});
  // Add new state for robotSpecificTopics

  // Update robotSpecificTopics when envVars changes
  useEffect(() => {
    const robotName = envVars["ROBOT_NAME"]?.toString() ?? "";
    const newTopics: Record<string, string> = {};

    if (robotNames.has(robotName) && robotSensorMap.get(robotName)) {
      for (const sensorName of robotSensorMap.get(robotName) ?? []) {
        if (sensorName in ALL_TOPICS_MAP) {
          console.log(sensorName);
          newTopics[sensorName] = ALL_TOPICS_MAP[sensorName as topicsMapKeys];
        }
      }
      robotSpecificTopics = newTopics;
    } else {
      robotSpecificTopics = {};
    }
  console.log(robotSpecificTopics);
  }, [envVars]);

  // ... rest of the component remains the same
  // Watch currentFrame for messages from each sensor
  useLayoutEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done: unknown) => {
      // parse variables into plain object and update envVars state
      if (renderState.variables instanceof Map) {
        // Cast to unknown first, then to Map with known types
        const variablesMap = renderState.variables as unknown as Map<string, unknown>;

        // Filter and ensure all values are strings
        const parsed = Object.fromEntries(
          Array.from(variablesMap.entries()).map(([key, value]) => [
            key,
            typeof value === "string" ? value : String(value),
          ]),
        );
        varDict = parsed; // optional: maintain global for legacy uses
        setEnvVars(parsed);
      } else if (renderState.variables && typeof renderState.variables === "object") {
        // Cast through unknown first for type safety
        const variables = renderState.variables as unknown as Record<string, string>;
        varDict = variables;
        setEnvVars(variables);
        console.log("Variables updated:", variables);
      }

      // Reset state when the user seeks the video
      if (renderState.didSeek ?? false) {
        setState(initState());
      }
      // Updates currentTime
      if (renderState.currentTime != undefined) {
        setState((prevState) => ({
          ...prevState,
          currentTime: renderState.currentTime!.sec,
        }));
      }
      if (renderState.currentFrame && renderState.currentFrame.length !== 0) {
        const lastFrame = renderState.currentFrame.at(-1) as MessageEvent;
        const sensorName = TOPICS_MAP_REVERSED[lastFrame.topic] as string;
        // Update sensorsTime to the current time and set connectStatus to true
        setState((prevState) => ({
          ...prevState,
          sensorsTime: {
            ...prevState.sensorsTime,
            [sensorName]: prevState.currentTime,
          },
          connectStatus: {
            ...prevState.connectStatus,
            [sensorName]: true,
          },
        }));
      }
      // Compare current time to each sensorsTime and set connectStatus to false if the sensor is down
      for (const key in ALL_TOPICS_MAP) {
        setState((prevState) => {
          if (prevState.currentTime - prevState.sensorsTime[key as topicsMapKeys] > SENSOR_DOWN_THRESHOLD) {
            return {
              ...prevState,
              connectStatus: {
                ...prevState.connectStatus,
                [key as topicsMapKeys]: false,
              },
            };
          }
          return prevState;
        });
      }
    };

    context.watch("currentTime");
    context.watch("currentFrame");
    context.watch("didSeek");
    context.watch("variables");
  }, [context]);
  // Call our done function at the end of each render.
  // useEffect(() => {
  //   context.subscribe(TOPICS_LIST);
  //   renderDone?.();
  // }, [renderDone]);
  // Create a table of all the sensors and their status
  const theme = useTheme();
  return (
    <ThemeProvider theme={theme}>
      {!("ROBOT_NAME" in varDict) && (
        <Box mb={1}>
          <Alert variant="filled" severity="warning">
            ROBOT_NAME not defined in Env vars.
          </Alert>
        </Box>
      )}
      {"ROBOT_NAME" in varDict && !robotNames.has(varDict["ROBOT_NAME"]) && (
        <Box mb={1}>
          <Alert variant="filled" severity="warning">
            {`Robot name, "${varDict["ROBOT_NAME"]}" is not an acceptable name: {${Array.from(robotNames)
              .map((name) => `"${name}"`)
              .join(", ")}}.`}
          </Alert>
        </Box>
      )}

      <Box m={1}>
        <TableContainer component={Paper}>
          <Table size="small">
            <TableBody>
              {Object.entries(robotSpecificTopics).map(([sensor, topic]) => (
                <TableRow
                  key={sensor}
                  style={{
                    backgroundColor: state.connectStatus[sensor as topicsMapKeys]
                      ? theme.palette.success.dark
                      : theme.palette.error.dark,
                  }}
                >
                  <TableCell>
                    <Tooltip title={topic} arrow placement="left">
                      <Typography variant="subtitle2" color={theme.palette.common.white}>
                        {sensor}
                      </Typography>
                    </Tooltip>
                  </TableCell>
                </TableRow>
              ))}
            </TableBody>
          </Table>
        </TableContainer>
      </Box>
    </ThemeProvider>
  );
}
export function initSensorsStatusPanel(context: PanelExtensionContext): () => void {
  context.panelElement.style.overflow = "auto"; // Enable scrolling
  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<SensorsStatusPanel context={context} />);
  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
