/**
 * What the page's mock and a worker's mock say to each other over a BroadcastChannel: the adapters the page granted
 * (a worker's `getDevices()` lists them), adapters plugged in and pulled out, and test-hook calls on the simulated
 * motors of whichever side opened the adapter.
 */

import { FakeGsUsbDevice } from "@makermods/arm-core/testing";
import { type Product, type SimulatedCanArm, simulatedCanArm } from "./can-arm.ts";

export const CHANNEL = "makermods-mock-hw";

export type HubMessage =
  | { kind: "granted?"; id: number }
  | { kind: "granted"; id: number; serials: string[] }
  | { kind: "plug"; product: Product; serial: string }
  | { kind: "unplug"; serial: string }
  | { kind: "call"; id: number; op: string; args: unknown[]; serial: string | null }
  | { kind: "result"; id: number; value?: unknown; error?: string };

/** The simulated arms one side (page or worker) built, by adapter serial. */
export class Bench {
  readonly arms = new Map<string, SimulatedCanArm & { device: FakeGsUsbDevice }>();

  add(product: Product, serial: string): SimulatedCanArm & { device: FakeGsUsbDevice } {
    const arm = simulatedCanArm(product, serial);
    this.arms.set(serial, arm);
    return arm;
  }

  /** The arm whose adapter this side opened most recently (or the one with this serial, once opened), or null. */
  active(serial: string | null = null): (SimulatedCanArm & { device: FakeGsUsbDevice }) | null {
    if (serial !== null) {
      const arm = this.arms.get(serial);
      return arm && arm.device.opens > 0 ? arm : null;
    }
    let best: (SimulatedCanArm & { device: FakeGsUsbDevice }) | null = null;
    for (const arm of this.arms.values()) if (arm.device.opens > 0) best = arm;
    return best;
  }
}

/** The test hooks on one simulated arm. */
export function callOp(arm: SimulatedCanArm, op: string, args: unknown[]): unknown {
  const { sim, device } = arm;
  const joint = args[0] as string | number;
  switch (op) {
    case "product":
      return arm.product;
    case "jointNames":
      return [...sim.motors.keys()];
    case "positions":
      return sim.positions;
    case "enabled":
      return [...sim.motors.values()].map((m) => m.enabled);
    case "setPositions": {
      // A hand moving the (limp or floated) arm: each joint's motor put at this joint position, rad.
      const q = args[0] as number[];
      sim.config.joints.forEach((j, i) => {
        if (Number.isFinite(q[i])) sim.motor(i).position = q[i]! * j.direction + j.offset;
      });
      return null;
    }
    case "fault":
      sim.fault(joint, (args[1] as boolean | undefined) ?? true);
      return null;
    case "offline":
      sim.offline(joint, (args[1] as boolean | undefined) ?? true);
      return null;
    case "setTemperature":
      sim.setTemperature(joint, args[1] as number);
      return null;
    case "protocolErrors":
      return [...device.protocolErrors];
    case "frames":
      return device instanceof FakeGsUsbDevice ? device.writes.length : device.frames.length;
    case "isOpen":
      return device.opened;
    default:
      throw new Error(`unknown mock op ${op}`);
  }
}
