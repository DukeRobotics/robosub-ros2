import { ExtensionContext } from "@foxglove/extension";

import { initSensorsStatusPanel } from "./SensorsStatusPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "Sensors Status (Jazzy)",
    initPanel: initSensorsStatusPanel,
  });
}
