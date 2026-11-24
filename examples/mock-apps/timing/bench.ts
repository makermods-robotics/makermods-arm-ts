/**
 * The control loop's tick rate with the whole stack in place: a real Arm with the shipped profile (its control rate,
 * its frame spacing, RealClock) on GsUsbBackend over a fake candleLight adapter with simulated motors behind it.
 * One tick is marked each time the first joint's motor gets its command frame.
 */

import { type Arm, GsUsbBackend } from "@makermods/arm-core";
import { createArm as createMaker } from "@makermods/maker-arm";
import { createArm as createMetal } from "@makermods/metal-arm";
import { type Product, simulatedCanArm } from "../mock/can-arm.ts";

export interface Timing {
  product: Product;
  where: "page" | "worker";
  /** "single": one arm alone; "left" / "right": one of two arms whose loops run together on this thread. */
  arm: "single" | "left" | "right";
  targetHz: number;
  achievedHz: number;
  /** Tick-to-tick period, ms. */
  medianMs: number;
  p99Ms: number;
  maxMs: number;
  ticks: number;
  seconds: number;
}

const quiet = { info() {}, warn() {}, error: (m: string) => console.error(m) };

/** One arm on its own fake adapter, its ticks marked. */
async function rigArm(product: Product, serial: string) {
  const rig = simulatedCanArm(product, serial);
  const backend = new GsUsbBackend(rig.device, { logger: quiet });
  const arm: Arm = (product === "maker" ? createMaker : createMetal)(backend, { logger: quiet });
  const first = arm.config.joints[0]!.motorId;
  const marks: number[] = [];
  const state = { armed: false };
  rig.observe = (kind, id) => {
    if (state.armed && kind === "command" && id === first) marks.push(performance.now());
  };
  await arm.connect();
  return { arm, marks, state };
}

function stats(product: Product, where: "page" | "worker", label: Timing["arm"], arm: Arm, marks: number[], seconds: number): Timing {
  const periods = marks.slice(1).map((m, i) => m - marks[i]!).sort((a, b) => a - b);
  const at = (p: number) => periods[Math.min(periods.length - 1, Math.floor(p * periods.length))] ?? NaN;
  return {
    product,
    where,
    arm: label,
    targetHz: arm.config.controlRateHz,
    achievedHz: (marks.length - 1) / ((marks[marks.length - 1]! - marks[0]!) / 1000),
    medianMs: at(0.5),
    p99Ms: at(0.99),
    maxMs: periods[periods.length - 1] ?? NaN,
    ticks: marks.length,
    seconds,
  };
}

/**
 * The control loop's ticks for one arm, or for two arms whose loops run together on this thread (`arms` 2), each
 * streaming a slow wave to its wrist.
 */
export async function measureLoop(product: Product, where: "page" | "worker", seconds = 3, arms: 1 | 2 = 1): Promise<Timing[]> {
  const rigs = [];
  for (let i = 0; i < arms; i++) rigs.push(await rigArm(product, `BENCH-${product}-${i}`));
  try {
    await Promise.all(rigs.map((r) => r.arm.enable()));
    const starts = rigs.map((r) => r.arm.getCommandedPositions());
    const roll = rigs[0]!.arm.jointNames.indexOf("wrist_roll");
    for (const r of rigs) r.state.armed = true;
    const clock = rigs[0]!.arm.clock;
    const t0 = clock.now();
    while (clock.now() - t0 < seconds) {
      const t = clock.now() - t0;
      rigs.forEach((r, k) => r.arm.setJointTargets(starts[k]!.map((q, i) => (i === roll ? q + 0.17 * Math.sin(Math.PI * t) : q))));
      await clock.sleep(0.02);
    }
    for (const r of rigs) r.state.armed = false;
  } finally {
    await Promise.allSettled(rigs.map(async (r) => (await r.arm.estop(), await r.arm.disconnect())));
  }
  const labels: Timing["arm"][] = arms === 1 ? ["single"] : ["left", "right"];
  return rigs.map((r, i) => stats(product, where, labels[i]!, r.arm, r.marks, seconds));
}
