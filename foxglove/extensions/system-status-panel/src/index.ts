import { ExtensionContext } from "@foxglove/extension";

import { initSystemStatusPanel } from "./SystemStatusPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "System Status (Jazzy)",
    initPanel: initSystemStatusPanel,
  });
}
