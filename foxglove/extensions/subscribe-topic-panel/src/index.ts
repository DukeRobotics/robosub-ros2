import { ExtensionContext } from "@foxglove/extension";

import { initSubscribeTopicPanel } from "./SubscribeTopicPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Subscribe Topic (Jazzy)", initPanel: initSubscribeTopicPanel });
}
