import { CustomMsgs, StdMsgs } from "@duke-robotics/defs/types";
import useTheme from "@duke-robotics/theme";
import { PanelExtensionContext, RenderState, MessageEvent, Immutable } from "@foxglove/extension";
import {
  Box,
  Typography,
  Paper,
  Table,
  TableBody,
  TableCell,
  TableContainer,
  TableRow,
  ThemeProvider,
} from "@mui/material";
import React, { useEffect, useState } from "react";
import { createRoot } from "react-dom/client";

interface SensorConfig {
  name: string;
  topic: string;
  suffix: string;
  // Extract a numeric value from an incoming ROS message event.
  parse: (event: MessageEvent) => number;
  // Return whether the sensor reading should trigger a "warning" styling.
  warn: (value: number | undefined) => boolean;
}

// Array of all sensors we want to listen to and display
const sensorConfigs: SensorConfig[] = [
  {
    name: "CPU",
    topic: "/system/usage",
    suffix: "%",
    parse: (event) => {
      const msgEvent = event as MessageEvent<CustomMsgs.SystemUsage>;
      return msgEvent.message.cpu_percent;
    },
    warn: (value) => value != undefined && value >= 90,
  },
  {
    name: "RAM",
    topic: "/system/usage",
    suffix: "%",
    parse: (event) => {
      const msgEvent = event as MessageEvent<CustomMsgs.SystemUsage>;
      return msgEvent.message.ram.percentage;
    },
    warn: (value) => value != undefined && value >= 90,
  },
  {
    name: "Voltage",
    topic: "/sensors/voltage",
    suffix: "V",
    parse: (event) => {
      const msgEvent = event as MessageEvent<StdMsgs.Float64>;
      return msgEvent.message.data;
    },
    warn: (value) => value != undefined && value <= 15,
  },
  {
    name: "Humidity",
    topic: "/sensors/humidity",
    suffix: "%",
    parse: (event) => {
      const msgEvent = event as MessageEvent<StdMsgs.Float64>;
      return msgEvent.message.data;
    },
    warn: (value) => value != undefined && value >= 80,
  },
  {
    name: "Temperature",
    topic: "/sensors/temperature",
    suffix: "F",
    parse: (event) => {
      const msgEvent = event as MessageEvent<StdMsgs.Float64>;
      return msgEvent.message.data;
    },
    warn: (value) => value != undefined && value >= 100,
  },
];

// A mapping from the sensor’s name to its last known value.
type SensorValues = Record<string, number | undefined>;

function SystemStatusPanel({ context }: { context: PanelExtensionContext }): React.JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [sensorValues, setSensorValues] = useState<SensorValues>({});

  // Subscribe to all topics used by the sensor configs
  const uniqueTopics = Array.from(new Set(sensorConfigs.map((sc) => sc.topic)));
  context.subscribe(uniqueTopics.map((topic) => ({ topic })));

  // Watch system usage topic and update state
  useEffect(() => {
    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);

      // Reset state when the user seeks the video
      if (renderState.didSeek ?? false) {
        setSensorValues({});
      }

      // Check for new messages
      if (renderState.currentFrame && renderState.currentFrame.length > 0) {
        // Get last message
        const latestFrame = renderState.currentFrame.at(-1);
        if (latestFrame) {
          // Find all configs that match the latest frame’s topic
          const matchingConfigs = sensorConfigs.filter((sc) => sc.topic === latestFrame.topic);

          if (matchingConfigs.length > 0) {
            // For each matching config, parse the value and store it
            matchingConfigs.forEach((config) => {
              const parsedValue = config.parse(latestFrame);
              setSensorValues((prevValues) => ({
                ...prevValues,
                [config.name]: parsedValue,
              }));
            });
          }
        }
      }
    };
    context.watch("currentFrame");
    context.watch("didSeek");
  }, [context]);

  // Call the done function after each render
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // Construct table rows based on the config array and sensorValues state
  const rows = sensorConfigs.map((config) => {
    const value = sensorValues[config.name];
    return {
      name: config.name,
      value,
      suffix: config.suffix,
      warn: config.warn(value),
    };
  });

  const theme = useTheme();
  return (
    <ThemeProvider theme={theme}>
      <Box m={1}>
        <TableContainer component={Paper}>
          <Table size="small">
            <TableBody>
              {rows.map((row) => (
                <TableRow
                  key={row.name}
                  style={{
                    backgroundColor: (() => {
                      // If no value has ever been set for this sensor, show error color
                      if (row.value == undefined) {
                        return theme.palette.error.dark;
                      } else if (row.warn) {
                        return theme.palette.warning.main;
                      }
                      return theme.palette.success.dark;
                    })(),
                  }}
                >
                  <TableCell>
                    <Typography variant="subtitle2" color={theme.palette.common.white}>
                      {row.name}
                    </Typography>
                  </TableCell>
                  <TableCell align="right">
                    <Typography variant="subtitle2" color={theme.palette.common.white}>
                      {row.value?.toFixed(1)}
                      {row.suffix}
                    </Typography>
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

export function initSystemStatusPanel(context: PanelExtensionContext): () => void {
  context.panelElement.style.overflow = "auto"; // Enable scrolling

  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<SystemStatusPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
