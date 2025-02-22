import { ExtensionContext } from "@foxglove/extension";

import { initToggleControlsPanel } from "./ToggleControlsPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Discrete Servos (Jazzy)", initPanel: initToggleControlsPanel });
}
