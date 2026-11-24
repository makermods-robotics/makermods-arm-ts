// The SO-101 follower (the console's So101Follower, dreamlabs FeetechChain) on the simulated Feetech bus: start,
// command, switch-off, faults and missed replies. Ported from the console's test/so101.test.ts.
import { VirtualClock } from "@makermods/arm-core/testing";
import { describe, expect, it } from "vitest";
import { openSo101Link } from "../src/bus.js";
import { So101Follower } from "../src/follower.js";
import { armLimits } from "../src/limits.js";
import { armConfigOf, so101Profile } from "../src/profile.js";
import { GRIP_MARGIN, MissedReadError, SO101_IDS, So101FaultError, ticksToDeg, ticksToVector, type So101Calibration } from "../src/so101.js";
import { simulatedSo101, type SimFeetechBus } from "../src/sim.js";
import fixtures from "./fixtures/calibrations.json" with { type: "json" };

const LEFT_FILE = fixtures.follower_left as So101Calibration;
const LIMITS = armLimits(armConfigOf(so101Profile({ side: "left" })));
const present = (sim: SimFeetechBus, id: number) => sim.reg(id, "Present_Position");

async function follower(clock: VirtualClock, file = LEFT_FILE, limits = LIMITS) {
  const arm = simulatedSo101(clock, { calibration: file });
  const link = await openSo101Link(arm.port, clock);
  return { ...arm, link, f: new So101Follower({ name: "left", link, calibration: file, armLimits: limits }) };
}

describe("the SO-101 follower (dreamlabs FeetechChain)", () => {
  it("configures with the torque off, then holds where it is; commands are goal ticks; switches off confirmed", async () => {
    const clock = new VirtualClock();
    const { sim, f, port } = await follower(clock);
    const before = SO101_IDS.map((id) => present(sim, id));
    const q = await f.connect();
    expect(q).toEqual(ticksToVector(before, LEFT_FILE));
    for (const id of SO101_IDS) {
      expect(sim.reg(id, "Torque_Enable")).toBe(1);
      expect(sim.reg(id, "P_Coefficient")).toBe(16);
      expect(sim.reg(id, "D_Coefficient")).toBe(32);
      expect(sim.writesTo(id, "Goal_Position")[0]).toBe(before[id - 1]);
    }
    expect(sim.reg(6, "Max_Torque_Limit")).toBe(500);
    // Nothing moved on enable.
    await clock.sleep(0.2);
    SO101_IDS.forEach((id, i) => expect(Math.abs(present(sim, id) - before[i]!)).toBeLessThanOrEqual(2));
    // A command: two joints a few degrees on, the pad ignored.
    const target = [...q];
    target[0] = target[0]! + 5;
    target[3] = target[3]! - 5;
    target[5] = 3;
    const applied = await f.sendAction(target);
    expect(applied[5]).toBe(0);
    expect(applied[0]).toBeCloseTo(target[0]!, 0);
    await clock.sleep(0.5);
    const read = await f.read();
    expect(read[0]).toBeCloseTo(target[0]!, 0);
    expect(read[3]).toBeCloseTo(target[3]!, 0);
    await f.release();
    for (const id of SO101_IDS) expect(sim.reg(id, "Torque_Enable")).toBe(0);
    expect(f.isOpen).toBe(false);
    expect(port.isOpen).toBe(false);
  });

  it("goalFor is the goal sendAction applies once reached: the tick inside a limit between ticks, the gripper's margin", async () => {
    const clock = new VirtualClock();
    const { f } = await follower(clock);
    const q = await f.connect();
    // A rest pose clipped to the limits: the elbow on its 89.0° limit (between two ticks), the gripper fully open.
    const rest = [...q];
    rest[2] = LIMITS[2]![1];
    rest[6] = 100;
    const goal = f.goalFor(rest);
    expect(rest[2]).toBe(89.0);
    expect(goal[2]).toBeLessThanOrEqual(89.0);
    expect(89.0 - goal[2]!).toBeGreaterThan(0.05);
    expect(89.0 - goal[2]!).toBeLessThan(360 / 4096);
    expect(goal[5]).toBe(0);
    expect(goal[6]).toBeLessThanOrEqual(100 - GRIP_MARGIN + 0.1);
    let applied = await f.sendAction(rest);
    for (let i = 0; i < 100 && applied.some((v, j) => Math.abs(v - goal[j]!) > 1e-9); i++) {
      await clock.sleep(0.02);
      applied = await f.sendAction(rest);
    }
    applied.forEach((v, j) => expect(v, `slot ${j}`).toBeCloseTo(goal[j]!, 9));
    await f.release();
  });

  it("refuses an arm whose servos hold another calibration than its file", async () => {
    const clock = new VirtualClock();
    const { sim, f } = await follower(clock);
    sim.servo(4).setReg("Homing_Offset", 1700);
    await expect(f.connect()).rejects.toThrow(/wrist_flex homing_offset 1700 \(file 1823\)/);
    for (const id of SO101_IDS) expect(sim.writesTo(id, "Torque_Enable")).toEqual([]);
  });

  it("a missed read holds the tick (MissedReadError) until 300 ms of them fail the arm", async () => {
    const clock = new VirtualClock();
    const { sim, f } = await follower(clock);
    await f.connect();
    sim.setOnline(2, false);
    await expect(f.read()).rejects.toBeInstanceOf(MissedReadError);
    sim.setOnline(2, true);
    expect((await f.read())[1]).toBeCloseTo(ticksToDeg(present(sim, 2)), 6);
    sim.setOnline(2, false);
    await expect(f.read()).rejects.toBeInstanceOf(MissedReadError);
    await clock.sleep(0.35);
    await expect(f.read()).rejects.toThrow(/left: no position for/);
  });

  it("a servo that drops its torque or reports a fault fails the arm for good: every later read and command throws", async () => {
    const clock = new VirtualClock();
    for (const cause of ["torque", "overheat"] as const) {
      const { sim, f } = await follower(clock);
      const q = await f.connect();
      if (cause === "torque") sim.servo(3).setReg("Torque_Enable", 0);
      else sim.servo(3).forcedError = 0x04;
      const pattern = cause === "torque" ? /elbow_flex: torque switched off/ : /elbow_flex \(overheat\)/;
      await expect(f.read()).rejects.toThrow(pattern);
      sim.servo(3).forcedError = 0;
      sim.servo(3).setReg("Torque_Enable", 1);
      await expect(f.read()).rejects.toBeInstanceOf(So101FaultError);
      const goals = sim.writesTo(1, "Goal_Position").length;
      await expect(f.sendAction(q)).rejects.toBeInstanceOf(So101FaultError);
      expect(sim.writesTo(1, "Goal_Position").length).toBe(goals);
    }
  });

  it("a stop during the enable switches on no further servo", async () => {
    const clock = new VirtualClock();
    const { sim, port, f } = await follower(clock);
    const abort = new AbortController();
    port.beforeWrite = () => {
      if (sim.writesTo(1, "Torque_Enable").includes(1)) abort.abort();
    };
    await expect(f.connect(abort.signal)).rejects.toThrow(/start cancelled/);
    expect(sim.writesTo(1, "Torque_Enable")).toContain(1);
    for (const id of [3, 4, 5, 6]) expect(sim.writesTo(id, "Torque_Enable")).not.toContain(1);
  });

  it("holdFrame: a joint below its limit (the ramp still bringing it in) is held where it is, not snapped to the limit", async () => {
    const clock = new VirtualClock();
    const arm = simulatedSo101(clock, { calibration: LEFT_FILE });
    const link = await openSo101Link(arm.port, clock);
    const at = ticksToVector(SO101_IDS.map((id) => present(arm.sim, id)), LEFT_FILE);
    const limits = LIMITS.map((r, i) => (i === 2 ? [at[2]! + 6, r[1]] : r) as [number, number]);
    const f = new So101Follower({ name: "left", link, calibration: LEFT_FILE, armLimits: limits });
    const q = await f.connect();
    const elbow = present(arm.sim, 3);
    await f.sendAction(q);
    await f.holdFrame(q);
    expect(arm.sim.writesTo(3, "Goal_Position").at(-1)).toBe(elbow);
  });

  it("a hold queued behind a read that finds a fault never writes (latched inside the read's transaction)", async () => {
    const clock = new VirtualClock();
    const { sim, f } = await follower(clock);
    const q = await f.connect();
    sim.servo(2).forcedError = 0x04;
    const goals = SO101_IDS.map((id) => sim.writesTo(id, "Goal_Position").length);
    const [reading, holding] = await Promise.allSettled([f.read(), f.holdFrame(q)]);
    expect(reading.status === "rejected" && String(reading.reason)).toMatch(/shoulder_lift \(overheat\)/);
    expect(holding.status === "rejected" && holding.reason).toBeInstanceOf(So101FaultError);
    SO101_IDS.forEach((id, i) => expect(sim.writesTo(id, "Goal_Position").length).toBe(goals[i]));
  });

  it("a fault seen in an incomplete batch is latched, not lost to the missing reply", async () => {
    const clock = new VirtualClock();
    const { sim, f } = await follower(clock);
    await f.connect();
    sim.servo(2).forcedError = 0x04;
    sim.setOnline(6, false);
    await expect(f.read()).rejects.toBeInstanceOf(So101FaultError);
    sim.servo(2).forcedError = 0;
    sim.setOnline(6, true);
    await expect(f.read()).rejects.toBeInstanceOf(So101FaultError);
  });

  it("a goal far from the last one is approached at 5 rad/s, never jumped to", async () => {
    const clock = new VirtualClock();
    const { f } = await follower(clock);
    const q = await f.connect();
    const far = q.map((v, i) => (i === 1 ? v + 60 : v));
    await clock.sleep(0.25); // a long gap: the bound still takes at most 50 ms of it
    const applied = await f.sendAction(far);
    expect(applied[1]! - q[1]!).toBeGreaterThan(10);
    expect(applied[1]! - q[1]!).toBeLessThanOrEqual(((5 * 180) / Math.PI) * 0.05 + 0.1);
  });

  it("a fault in the start's pose read (a later batch than the first check) refuses the start", async () => {
    const clock = new VirtualClock();
    const { sim, f } = await follower(clock);
    const original = f.bus.syncReadState.bind(f.bus);
    let calls = 0;
    f.bus.syncReadState = async (ids, timeoutMs) => {
      sim.servo(5).forcedError = ++calls === 2 ? 0x04 : 0;
      try {
        return await original(ids, timeoutMs);
      } finally {
        sim.servo(5).forcedError = 0;
      }
    };
    await expect(f.connect()).rejects.toBeInstanceOf(So101FaultError);
    for (const id of SO101_IDS) expect(sim.writesTo(id, "Torque_Enable")).not.toContain(1);
  });

  it("refuses to start an arm with a servo reporting a fault, nothing switched", async () => {
    const clock = new VirtualClock();
    const { sim, f } = await follower(clock);
    sim.servo(5).forcedError = 0x20;
    await expect(f.connect()).rejects.toThrow(/wrist_roll \(overload\)/);
    for (const id of SO101_IDS) expect(sim.writesTo(id, "Torque_Enable")).toEqual([]);
  });
});
