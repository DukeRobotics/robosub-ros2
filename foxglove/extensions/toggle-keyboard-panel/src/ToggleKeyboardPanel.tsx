import { allDatatypeMaps } from "@duke-robotics/defs/datatype_maps";
import { GeometryMsgs, CustomMsgs } from "@duke-robotics/defs/types";
import useTheme from "@duke-robotics/theme";
import { Immutable, PanelExtensionContext, RenderState } from "@foxglove/extension";
import { Button, Box, Alert, ThemeProvider } from "@mui/material";
import React, { useCallback, useEffect, useState } from "react";
import { createRoot } from "react-dom/client";

const PUBLISH_RATE = 20; // Hz

const SET_CONTROL_TYPES_SERVICE = "/controls/set_control_types";

const DESIRED_POWER_TOPIC = "/controls/desired_power";
const DESIRED_POWER_SCHEMA = "geometry_msgs/Twist";

const KEYBOARD_KEY_MAP = {
  linear: {
    forward: "w",
    backward: "s",
    left: "a",
    right: "d",
    up: "arrowup",
    down: "arrowdown",
  },
  orientation: {
    rollClockwise: "l",
    rollAnticlockwise: "j",
    pitchUp: "k",
    pitchDown: "i",
    yawClockwise: "arrowright",
    yawAnticlockwise: "arrowleft",
  },
} as const;

const VALID_KEYS = new Set<string>([
  "w",
  "a",
  "s",
  "d",
  "arrowup",
  "arrowdown",
  "arrowleft",
  "arrowright",
  "l",
  "j",
  "i",
  "k",
]);

// Helper function for determining if a key is pressed
function isKeyPressed(pressedKeys: Set<string>, keys: string[]): boolean {
  return keys.some((key) => pressedKeys.has(key));
}

type TransformedKeyboardInputs = {
  xAxis: number;
  yAxis: number;
  zAxis: number;
  rollAxis: number;
  pitchAxis: number;
  yawAxis: number;
};

type ToggleKeyboardPanelState = {
  error?: Error;
  colorScheme?: RenderState["colorScheme"];
  keyboardEnabled: boolean;
  transformedKeyboardInputs: TransformedKeyboardInputs;
  keyboardConnected: boolean;
};

function ToggleKeyboardPanel({ context }: { context: PanelExtensionContext }): React.JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [state, setState] = useState<ToggleKeyboardPanelState>({
    keyboardEnabled: false,
    keyboardConnected: false,
    transformedKeyboardInputs: {
      xAxis: 0,
      yAxis: 0,
      yawAxis: 0,
      zAxis: 0,
      pitchAxis: 0,
      rollAxis: 0,
    },
  });

  const [pressedKeys, setPressedKeys] = useState<Set<string>>(new Set());

  // Update color scheme and register render callback (cleaned up on unmount)
  useEffect(() => {
    // Preserve any existing handler so we can restore it on cleanup
    const prevOnRender = context.onRender;

    context.onRender = (renderState: Immutable<RenderState>, done) => {
      setState((prevState) => ({ ...prevState, colorScheme: renderState.colorScheme }));
      // Store the done callback so the panel lifecycle can call it later
      setRenderDone(() => done);
    };

    context.watch("colorScheme");

    return () => {
      // Restore previous handler if any
      context.onRender = prevOnRender;
    };
  }, [context]);

  const handleKeyDown = useCallback(
    (e: KeyboardEvent) => {
      // Normalize key and ignore repeats
      if (e.repeat) {
        return;
      }
      const key = e.key.toLowerCase();

      // Only handle keys we consider valid
      if (!VALID_KEYS.has(key)) {
        return;
      }

      // Prevent default browser behavior only for our handled keys
      e.preventDefault();

      setPressedKeys((prev) => {
        const next = new Set(prev);
        if (!next.has(key)) {
          next.add(key);
          console.log("Key down:", key);
          console.log(pressedKeys);
        }
        return next;
      });
    },
    [pressedKeys],
  );

  const handleKeyUp = useCallback((e: KeyboardEvent) => {
    if (e.repeat) {
      return;
    }
    const key = e.key.toLowerCase();

    if (!VALID_KEYS.has(key)) {
      return;
    }

    e.preventDefault();

    setPressedKeys((prev) => {
      const next = new Set(prev);
      if (next.has(key)) {
        next.delete(key);
        console.log("Key up:", key);
      }
      return next;
    });
  }, []);

  // Keyboard handlers: add/remove listeners and log keys
  useEffect(() => {
    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);

    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
    };
  }, [handleKeyDown, handleKeyUp]); // No deps so we install once

  // Call our done function at the end of each render
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const toggleKeyboard = useCallback(() => {
    // Check if service calling is supported by the context

    if (!context.callService) {
      console.error("Calling services is not supported by this connection");
      return;
    }

    // Request payload to toggle control types
    const desiredControl: CustomMsgs.ControlTypesConst = state.keyboardEnabled
      ? CustomMsgs.ControlTypesConst.DESIRED_POSITION
      : CustomMsgs.ControlTypesConst.DESIRED_POWER;
    const request: CustomMsgs.SetControlTypesRequest = {
      // eslint-disable-next-line camelcase
      control_types: {
        x: desiredControl,
        y: desiredControl,
        z: desiredControl,
        roll: desiredControl,
        pitch: desiredControl,
        yaw: desiredControl,
      },
    };

    // Make the service call
    (context.callService(SET_CONTROL_TYPES_SERVICE, request) as Promise<CustomMsgs.SetControlTypesResponse>).then(
      (response) => {
        // Update the state based on the service response
        // If the service responds with failure, display the response message as an error
        if (response.success) {
          setState((prevState) => ({ ...prevState, error: undefined, keyboardEnabled: !prevState.keyboardEnabled }));
        } else {
          setState((prevState) => ({ ...prevState, error: Error(response.message) }));
        }
      },
      (error: unknown) => {
        // Handle service call errors (e.g., service is not advertised)
        setState((prevState) => ({ ...prevState, error: error as Error }));
      },
    );
  }, [context, state.keyboardEnabled]);

  /**
   * Publish transformed joystick inputs as a desired power message
   * @param transformedKeyboardInputs A TransformedKeyboardInputs object used to create the desired power message
   */
  const publishPower = useCallback(
    (transformedKeyboardInputs: TransformedKeyboardInputs) => {
      if (!context.advertise || !context.publish) {
        return;
      }

      context.advertise(DESIRED_POWER_TOPIC, DESIRED_POWER_SCHEMA, {
        datatypes: allDatatypeMaps.ros2jazzy[DESIRED_POWER_SCHEMA],
      });

      // Create and publish desired power message
      const request: GeometryMsgs.Twist = {
        linear: {
          x: transformedKeyboardInputs.xAxis,
          y: transformedKeyboardInputs.yAxis,
          z: transformedKeyboardInputs.zAxis,
        },
        angular: {
          x: transformedKeyboardInputs.rollAxis,
          y: transformedKeyboardInputs.pitchAxis,
          z: transformedKeyboardInputs.yawAxis,
        },
      };
      context.publish(DESIRED_POWER_TOPIC, request);
    },
    [context],
  );

  useEffect(() => {
    /**
     * Query keyboard to read and transform inputs
     */
    function queryKeyboard() {
      const transformedKeyboardInputs = {
        xAxis: isKeyPressed(pressedKeys, [KEYBOARD_KEY_MAP.linear.forward])
          ? 0.5
          : isKeyPressed(pressedKeys, [KEYBOARD_KEY_MAP.linear.backward])
            ? -0.5
            : 0,
        yAxis: isKeyPressed(pressedKeys, [KEYBOARD_KEY_MAP.linear.left])
          ? 0.5
          : isKeyPressed(pressedKeys, [KEYBOARD_KEY_MAP.linear.right])
            ? -0.5
            : 0,
        zAxis: isKeyPressed(pressedKeys, [KEYBOARD_KEY_MAP.linear.up])
          ? 0.5
          : isKeyPressed(pressedKeys, [KEYBOARD_KEY_MAP.linear.down])
            ? -0.5
            : 0,
        yawAxis: isKeyPressed(pressedKeys, [KEYBOARD_KEY_MAP.orientation.yawAnticlockwise])
          ? 0.5
          : isKeyPressed(pressedKeys, [KEYBOARD_KEY_MAP.orientation.yawClockwise])
            ? -0.5
            : 0,
        pitchAxis: isKeyPressed(pressedKeys, [KEYBOARD_KEY_MAP.orientation.pitchUp])
          ? 0.5
          : isKeyPressed(pressedKeys, [KEYBOARD_KEY_MAP.orientation.pitchDown])
            ? -0.5
            : 0,
        rollAxis: isKeyPressed(pressedKeys, [KEYBOARD_KEY_MAP.orientation.rollAnticlockwise])
          ? 0.5
          : isKeyPressed(pressedKeys, [KEYBOARD_KEY_MAP.orientation.rollClockwise])
            ? -0.5
            : 0,
      };
      console.log("Transformed Keyboard Inputs:", transformedKeyboardInputs);

      // Update state
      setState((prevState) => ({ ...prevState, transformedKeyboardInputs }));

      // Publish
      if (state.keyboardEnabled) {
        publishPower(transformedKeyboardInputs);
      }
    }

    const intervalDelay = 1000 / PUBLISH_RATE; // Convert Hz to milliseconds
    const intervalId = setInterval(() => {
      queryKeyboard();
    }, intervalDelay);

    return () => {
      clearInterval(intervalId); // Clear the interval on component unmount
    };
  }, [pressedKeys, publishPower, state.keyboardEnabled]);

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
        {/* Toggle button */}
        <Box my={1}>
          <Button
            fullWidth
            variant="contained"
            color={state.keyboardEnabled ? "error" : "success"}
            onClick={toggleKeyboard}
            disabled={false}
          >
            {state.keyboardEnabled ? "Disable Keyboard" : "Enable Keybord"}
          </Button>
        </Box>
      </Box>
    </ThemeProvider>
  );
}

export function initToggleKeyboardPanel(context: PanelExtensionContext): () => void {
  context.panelElement.style.overflow = "auto"; // Enable scrolling

  const root = createRoot(context.panelElement as HTMLElement);
  root.render(<ToggleKeyboardPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
