import { CustomMsgs, StdMsgs } from "@duke-robotics/defs/types";
import { NoRobotNameAlert, Robot, useRobotName } from "@duke-robotics/robot-name";
import useTheme from "@duke-robotics/theme";
import { secToNsec, timeToNsec } from "@duke-robotics/utils";
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
  Tooltip,
} from "@mui/material";
import React, { useEffect, useLayoutEffect, useMemo, useState } from "react";
import { createRoot } from "react-dom/client";

const TOPIC_DOWN_THRESHOLD_NSEC = secToNsec(2);

enum Status {
  CPU = "CPU",
  RAM = "RAM",
  Voltage = "Voltage",
  HumiditySignal = "HumiditySignal",
  TempSignal = "TempSignal",
  HumidityBattery = "HumidityBattery",
  TempBattery = "TempBattery",
  TempGyro = "TempGyro",
}
const STATUSES = Object.values(Status);

interface StatusConfig {
  displayName: string;
  topic: string;
  suffix: string;
  // Extract a numeric value from a ROS message event.
  parse: (event: MessageEvent) => number;
  // Return whether the sensor reading should trigger a "warning" styling.
  warn: (value: number | undefined) => boolean;
}

const STATUS_CONFIG: Record<Status, StatusConfig> = {
  [Status.CPU]: {
    displayName: "CPU",
    topic: "/system/usage",
    suffix: "%",
    parse: (event) => {
      const msgEvent = event as MessageEvent<CustomMsgs.SystemUsage>;
      return msgEvent.message.cpu_percent;
    },
    warn: (value) => value != undefined && value >= 90,
  },
  [Status.RAM]: {
    displayName: "RAM",
    topic: "/system/usage",
    suffix: "%",
    parse: (event) => {
      const msgEvent = event as MessageEvent<CustomMsgs.SystemUsage>;
      return msgEvent.message.ram.percentage;
    },
    warn: (value) => value != undefined && value >= 90,
  },
  [Status.Voltage]: {
    displayName: "Voltage",
    topic: "/sensors/voltage",
    suffix: "V",
    parse: (event) => {
      const msgEvent = event as MessageEvent<StdMsgs.Float64>;
      return msgEvent.message.data;
    },
    warn: (value) => value != undefined && value <= 15,
  },
  [Status.HumiditySignal]: {
    displayName: "Humidity (Signal)",
    topic: "/sensors/humidity/signal",
    suffix: "%",
    parse: (event) => {
      const msgEvent = event as MessageEvent<StdMsgs.Float64>;
      return msgEvent.message.data;
    },
    warn: (value) => value != undefined && value >= 80,
  },
  [Status.TempSignal]: {
    displayName: "Temp (Signal)",
    topic: "/sensors/temperature/signal",
    suffix: "F",
    parse: (event) => {
      const msgEvent = event as MessageEvent<StdMsgs.Float64>;
      return msgEvent.message.data;
    },
    warn: (value) => value != undefined && value >= 100,
  },
  [Status.HumidityBattery]: {
    displayName: "Humidity (Battery)",
    topic: "/sensors/humidity/battery",
    suffix: "%",
    parse: (event) => {
      const msgEvent = event as MessageEvent<StdMsgs.Float64>;
      return msgEvent.message.data;
    },
    warn: (value) => value != undefined && value >= 80,
  },
  [Status.TempBattery]: {
    displayName: "Temp (Battery)",
    topic: "/sensors/temperature/battery",
    suffix: "F",
    parse: (event) => {
      const msgEvent = event as MessageEvent<StdMsgs.Float64>;
      return msgEvent.message.data;
    },
    warn: (value) => value != undefined && value >= 100,
  },
  [Status.TempGyro]: {
    displayName: "Temp (Gyro)",
    topic: "/sensors/gyro/temperature/slow",
    suffix: "F",
    parse: (event) => {
      const msgEvent = event as MessageEvent<StdMsgs.Float64>;
      return msgEvent.message.data;
    },
    warn: (value) => value != undefined && value >= 150,
  },
};

const TOPIC_TO_STATUSES = STATUSES.reduce<Record<string, Status[]>>((acc, status) => {
  const topic = STATUS_CONFIG[status].topic;
  acc[topic] = acc[topic] ?? [];
  acc[topic].push(status);
  return acc;
}, {});

const ROBOT_CONFIG: Record<Robot, Status[]> = {
  [Robot.Crush]: STATUSES,
  [Robot.Oogway]: [Status.CPU, Status.RAM, Status.Voltage, Status.HumiditySignal, Status.TempSignal, Status.TempGyro],
};

type StatusValues = Partial<Record<Status, number>>;
type TopicNsecs = Record<string, number | undefined>;

function SystemStatusPanel({ context }: { context: PanelExtensionContext }): React.JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [statusValues, setStatusValues] = useState<StatusValues>({});
  const [topicNsecs, setTopicNsecs] = useState<TopicNsecs>({});
  const [currentNsecState, setCurrentNsecState] = useState<number | undefined>(undefined);

  const { robotName, withRobotName } = useRobotName(context);

  const statusesToMonitor = useMemo<Status[]>(() => {
    if (robotName == undefined) {
      return [];
    }
    return ROBOT_CONFIG[robotName];
  }, [robotName]);

  const topicsToMonitor = useMemo(() => {
    const topics = new Set<string>();
    statusesToMonitor.forEach((status) => {
      topics.add(STATUS_CONFIG[status].topic);
    });
    return [...topics];
  }, [statusesToMonitor]);

  useEffect(() => {
    context.subscribe(topicsToMonitor.map((topic) => ({ topic })));
  }, [context, topicsToMonitor]);

  // Update status values
  useLayoutEffect(() => {
    context.onRender = withRobotName((renderState: Immutable<RenderState>, done) => {
      setRenderDone(() => done);

      // Reset state when the user seeks the video
      if (renderState.didSeek ?? false) {
        setStatusValues({});
        setTopicNsecs({});
        setCurrentNsecState(undefined);
        return;
      }

      const currentNsec = renderState.currentTime != undefined ? timeToNsec(renderState.currentTime) : undefined;
      if (currentNsec != undefined) {
        setCurrentNsecState(currentNsec);
      }

      // Check for new messages
      if (renderState.currentFrame) {
        const updates: StatusValues = {};
        const topicsSeen = new Set<string>();
        renderState.currentFrame.forEach((event) => {
          const statusesForTopic = TOPIC_TO_STATUSES[event.topic];
          if (statusesForTopic == undefined) {
            console.warn(`Received message for unmonitored topic: ${event.topic}`);
            return;
          }

          topicsSeen.add(event.topic);

          statusesForTopic.forEach((status) => {
            const config = STATUS_CONFIG[status];
            updates[status] = config.parse(event);
          });
        });

        if (currentNsec != undefined) {
          setTopicNsecs((prev) => ({
            ...prev,
            ...Object.fromEntries([...topicsSeen].map((topic) => [topic, currentNsec])),
          }));
        }

        setStatusValues((prev) => ({
          ...prev,
          ...updates,
        }));
      }
    });
    context.watch("currentTime");
    context.watch("currentFrame");
    context.watch("didSeek");
  }, [context, withRobotName]);

  // Call the done function after each render
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const rows = statusesToMonitor.map((status) => {
    const config = STATUS_CONFIG[status];
    const value = statusValues[status];
    const topicNsec = topicNsecs[config.topic];
    const topicPublishing =
      currentNsecState != undefined &&
      topicNsec != undefined &&
      0 <= currentNsecState - topicNsec &&
      currentNsecState - topicNsec <= TOPIC_DOWN_THRESHOLD_NSEC;
    return {
      topic: config.topic,
      name: config.displayName,
      value,
      suffix: config.suffix,
      warn: config.warn(value),
      publishing: topicPublishing,
    };
  });

  const theme = useTheme();
  return (
    <ThemeProvider theme={theme}>
      <Box m={1}>
        <NoRobotNameAlert robotName={robotName} />
        <TableContainer component={Paper}>
          <Table size="small">
            <TableBody>
              {rows.map((row) => (
                <Tooltip key={row.name} title={row.topic} followCursor>
                  <TableRow
                    style={{
                      backgroundColor: (() => {
                        if (!row.publishing || row.value == undefined) {
                          return theme.palette.error.dark;
                        } else if (row.warn) {
                          return theme.palette.warning.main;
                        }
                        return theme.palette.success.dark;
                      })(),
                    }}
                    onClick={() => {
                      void navigator.clipboard.writeText(row.topic);
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
                </Tooltip>
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

  const root = createRoot(context.panelElement);
  root.render(<SystemStatusPanel context={context} />);

  return () => {
    root.unmount();
  };
}
