import { ExtensionContext } from "@foxglove/extension";

import { initToggleKeyboardPanel } from "./ToggleKeyboardPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Toggle Keyboard (Jazzy)", initPanel: initToggleKeyboardPanel });
}
