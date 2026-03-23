import { ExtensionContext } from "@foxglove/extension";

import { initAgriMissionPanel } from "./AgriMissionPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "agri-mission",
    initPanel: initAgriMissionPanel,
  });
}
