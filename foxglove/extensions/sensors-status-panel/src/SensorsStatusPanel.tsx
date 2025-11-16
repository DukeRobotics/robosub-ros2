import { Robot, getRobotName, NoRobotNameAlert } from "@duke-robotics/robot-name";
import useTheme from "@duke-robotics/theme";
import { timeToNsec } from "@duke-robotics/utils";
import { Immutable, PanelExtensionContext, RenderState } from "@foxglove/extension";
import { Box, Paper, Table, TableBody, TableCell, TableContainer, TableRow, Typography, Tooltip } from "@mui/material";
import { ThemeProvider } from "@mui/material/styles";
import React, { useEffect, useState, useLayoutEffect } from "react";
import { createRoot } from "react-dom/client";

import {
  SensorConfig,
  Sensor,
  SENSORS,
  ROBOT_CONFIG,
  SENSOR_CONFIG,
  TOPIC_TO_SENSOR,
  SENSOR_DOWN_THRESHOLD_NSEC,
} from "./config";

type SensorNsecs = Record<Sensor, number | undefined>; // Time of last message received from sensor
const initSensorNsecs = (): SensorNsecs => Object.fromEntries(SENSORS.map((s) => [s, undefined])) as SensorNsecs;

function SensorsStatusPanel({ context }: { context: PanelExtensionContext }): React.JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const [robotName, setRobotName] = useState<Robot | undefined>(undefined);
  const [sensorNsecs, setSensorNsecs] = useState<SensorNsecs>(initSensorNsecs);
  const [currentNsecState, setCurrentNsecState] = useState<number | undefined>(undefined);

  const sensorsToMonitor = React.useMemo(() => {
    const map: Partial<Record<Sensor, SensorConfig>> = {};
    if (robotName != undefined) {
      for (const sensor of ROBOT_CONFIG[robotName]) {
        map[sensor] = SENSOR_CONFIG[sensor];
      }
    }
    return map;
  }, [robotName]);

  useLayoutEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done: () => void) => {
      setRenderDone(() => done);

      setRobotName(getRobotName(renderState));

      // Reset state when the user seeks the video
      if (renderState.didSeek ?? false) {
        setSensorNsecs(initSensorNsecs());
        setCurrentNsecState(undefined);
        return;
      }

      if (renderState.currentTime != undefined) {
        const currentNsec = timeToNsec(renderState.currentTime);
        setCurrentNsecState(currentNsec);

        if (renderState.currentFrame != undefined) {
          // Find all sensors that have published in the current frame
          const seenSensors = new Set<Sensor>();
          for (const event of renderState.currentFrame) {
            const sensor = TOPIC_TO_SENSOR[event.topic];
            if (sensor != undefined) {
              seenSensors.add(sensor);
            }
          }

          // Update the last seen time for all sensors that have published in this frame
          setSensorNsecs((prev) => ({
            ...prev,
            ...Object.fromEntries([...seenSensors].map((sensor) => [sensor, currentNsec])),
          }));
        }
      }
    };

    context.subscribe(Object.values(sensorsToMonitor).map((config) => ({ topic: config.topic })));
    context.watch("currentTime");
    context.watch("currentFrame");
    context.watch("didSeek");
    context.watch("variables");
  }, [context, sensorsToMonitor]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const theme = useTheme();
  return (
    <ThemeProvider theme={theme}>
      <Box m={1}>
        <NoRobotNameAlert robotName={robotName} />
        <TableContainer component={Paper}>
          <Table size="small">
            <TableBody>
              {Object.entries(sensorsToMonitor).map(([sensor, config]) => {
                const sensorNsec = sensorNsecs[sensor as Sensor];
                const sensorPublishing =
                  currentNsecState != undefined &&
                  sensorNsec != undefined &&
                  0 <= currentNsecState - sensorNsec &&
                  currentNsecState - sensorNsec <= SENSOR_DOWN_THRESHOLD_NSEC;
                return (
                  <TableRow
                    key={sensor}
                    style={{
                      backgroundColor: sensorPublishing ? theme.palette.success.dark : theme.palette.error.dark,
                    }}
                  >
                    <TableCell>
                      <Tooltip title={config.topic} arrow placement="left">
                        <Typography
                          variant="subtitle2"
                          color={theme.palette.common.white}
                          onClick={() => {
                            void navigator.clipboard.writeText(config.topic);
                          }}
                        >
                          {config.displayName}
                        </Typography>
                      </Tooltip>
                    </TableCell>
                  </TableRow>
                );
              })}
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
