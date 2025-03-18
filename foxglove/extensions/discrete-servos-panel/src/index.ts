import { ExtensionContext } from "@foxglove/extension";

import { initDiscreteServosPanel } from "./DiscreteServosPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Discrete Servos (Jazzy)", initPanel: initDiscreteServosPanel });
}
