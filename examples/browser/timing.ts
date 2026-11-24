/** Run a real Arm at its shipped control rate on simulated motors and measure how evenly the loop ticks. */

import { Arm, RealClock } from "@makermods/arm-core";
import { defaultConfig as makerConfig } from "@makermods/maker-arm";
import * as makerSim from "@makermods/maker-arm/testing";
import { defaultConfig as metalConfig } from "@makermods/metal-arm";
import * as metalSim from "@makermods/metal-arm/testing";

export interface Timing {
  where: string;
  product: string;
  targetHz: number;
  achievedHz: number;
  /** Tick-to-tick period, ms. */
  medianMs: number;
  p99Ms: number;
  maxMs: number;
  ticks: number;
}

export async function measureLoop(product: "maker" | "metal", where: string, seconds = 3): Promise<Timing> {
  const config = product === "maker" ? makerConfig() : metalConfig();
  const sim = product === "maker" ? makerSim : metalSim;
  const motors = config.joints.map((j) => sim.forJoint(j, config.hostId, Math.min(j.hi, Math.max(j.lo, 0)) * j.direction + j.offset));
  const bus = new sim.SimBus(motors as never);
  const clock = new RealClock();
  // One mark per tick: the first frame to the first joint after a frame to another joint (a status
  // query to the first joint can precede its command).
  const first = config.joints[0]!.motorId;
  const marks: number[] = [];
  let previous = -1;
  const send = bus.send.bind(bus);
  bus.send = (canId: number, data: Uint8Array, extended?: boolean) => {
    if (canId === first && previous !== first && armed) marks.push(performance.now());
    previous = canId;
    return send(canId, data, extended);
  };
  let armed = false;
  const arm = new Arm(config, bus, { clock });
  await arm.connect();
  await arm.enable();
  armed = true;
  const start = arm.getJointPositions();
  const t0 = clock.now();
  while (clock.now() - t0 < seconds) {
    const t = clock.now() - t0;
    arm.setJointTargets(start.map((q, i) => (i === 5 ? q + 0.17 * Math.sin((2 * Math.PI * t) / 2) : q)));
    await clock.sleep(0.02);
  }
  armed = false;
  await arm.disable();
  await arm.disconnect();
  const periods = marks.slice(1).map((m, i) => m - marks[i]!).sort((a, b) => a - b);
  const at = (p: number) => periods[Math.min(periods.length - 1, Math.floor(p * periods.length))] ?? NaN;
  return {
    where,
    product,
    targetHz: config.controlRateHz,
    achievedHz: (marks.length - 1) / ((marks[marks.length - 1]! - marks[0]!) / 1000),
    medianMs: at(0.5),
    p99Ms: at(0.99),
    maxMs: periods[periods.length - 1] ?? NaN,
    ticks: marks.length,
  };
}
