/** What the page and the arm worker say to each other. */

import type { ArmStatus } from "@makermods/arm-core";

export type Command =
  | { cmd: "connect"; product: "metal" | "maker"; port: string }
  | { cmd: "enable" }
  | { cmd: "move"; joint: string; delta: number }
  | { cmd: "rest" }
  | { cmd: "estop" }
  | { cmd: "disconnect" };

export type Reply =
  | { type: "status"; status: ArmStatus | null; jointNames: string[] }
  | { type: "done"; cmd: Command["cmd"]; ok: boolean; message: string; target?: number[] }
  | { type: "log"; line: string };
