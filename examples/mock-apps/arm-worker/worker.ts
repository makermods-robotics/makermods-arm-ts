/** The arm and its control loop, in a Web Worker: the adapter comes from grantedAdapters(), commands by postMessage. */

import { type Arm, GsUsbBackend, grantedAdapters, selectAdapter } from "@makermods/arm-core";
import { createArm as createMaker } from "@makermods/maker-arm";
import { createArm as createMetal } from "@makermods/metal-arm";
import type { Command, Reply } from "./protocol.ts";

const post = (reply: Reply) => self.postMessage(reply);
const line = (text: string) => post({ type: "log", line: text });
const logger = { info: line, warn: (m: string) => line(`warning: ${m}`), error: (m: string) => line(`error: ${m}`) };
let arm: Arm | null = null;

setInterval(() => post({ type: "status", status: arm?.status() ?? null, jointNames: arm?.jointNames ?? [] }), 100);

async function run(c: Command): Promise<Omit<Extract<Reply, { type: "done" }>, "type" | "cmd" | "ok">> {
  switch (c.cmd) {
    case "connect": {
      // Granted to the page from a click; a worker sees it here.
      const device = selectAdapter(await grantedAdapters(), c.port);
      const create = c.product === "maker" ? createMaker : createMetal;
      const candidate = create(new GsUsbBackend(device, { logger }), { logger });
      await candidate.connect();
      arm = candidate;
      return { message: `Connected to ${device.serialNumber} in the worker: ${arm.jointNames.length} joints.` };
    }
    case "enable":
      await arm!.enable();
      return { message: "Holding the present pose." };
    case "move": {
      const q = arm!.getCommandedPositions();
      const i = arm!.jointNames.indexOf(c.joint);
      q[i] = q[i]! + c.delta;
      const arrived = await arm!.moveTo(q, { speed: 0.5 });
      if (!arrived) throw new Error("The move did not arrive.");
      return { message: `Arrived: ${c.joint}.`, target: q };
    }
    case "rest":
      if (!(await arm!.stop())) throw new Error("Return to rest not verified: the arm keeps holding.");
      return { message: "At rest, torque released." };
    case "estop":
      await arm!.estop();
      return { message: "E-stop: torque released on every motor." };
    case "disconnect": {
      const a = arm!;
      arm = null;
      await a.disconnect();
      return { message: "Disconnected." };
    }
  }
}

self.onmessage = async (event: MessageEvent<Command>) => {
  const c = event.data;
  try {
    post({ type: "done", cmd: c.cmd, ok: true, ...(await run(c)) });
  } catch (e) {
    post({ type: "done", cmd: c.cmd, ok: false, message: e instanceof Error ? e.message : String(e) });
  }
};

self.addEventListener("close", () => void arm?.estop());
