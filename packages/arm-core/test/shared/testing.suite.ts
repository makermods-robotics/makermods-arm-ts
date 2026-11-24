// Port of the Python SDK's tests/unit/test_testing.py: simulated arms an application can drive without
// hardware.
import { readFileSync } from "node:fs";
import { describe, expect, it } from "vitest";
import { type Arm, type ArmConfig, ArmState, ValueError } from "@makermods/arm-core";
import type { ArmGroup } from "../../src/rig.js";
import {
  SIM_CONTROL_RATE_HZ, SIM_FEEDBACK_TIMEOUT, SIM_MAX_TICK_PERIODS, type SimulatedMotors, VirtualClock,
} from "@makermods/arm-core/testing";
import { type ArmFakes, type FakeMotorLike, type FakeMotorOptions, RecordingLogger, waitFor } from "./fakes.js";

/** The package's simulatedGroup (from its "/testing" entry). */
export type SimulatedGroupFn<M extends FakeMotorLike> = (rig: unknown, options?: { clock?: VirtualClock; logger?: RecordingLogger }) =>
  Promise<[ArmGroup, Map<string, SimulatedMotors<M>>]>;

/** Registers the shared tests on `fk`'s arm. */
export function testingSuite<M extends FakeMotorLike>(fk: ArmFakes<M>, simulatedGroup: SimulatedGroupFn<M>): void {
  const { defaultConfig } = fk;
  type Sim = SimulatedMotors<M>;

  function close(actual: readonly number[], expected: readonly number[], abs: number): boolean {
    return actual.length === expected.length && actual.every((a, i) => Math.abs(a - expected[i]!) <= abs);
  }

  /** simulated_arm(...) with Arm.PARAM_TIMEOUT = 0.005 (the Python autouse fixture) on `clock`. */
  function simArm(clock: VirtualClock, options: { config?: ArmConfig; positions?: number[]; motor?: FakeMotorOptions } = {}):
      [Arm, Sim] {
    const [arm, sim] = fk.simulatedArm({ ...options, clock, logger: new RecordingLogger() });
    arm.PARAM_TIMEOUT = 0.005;
    return [arm, sim];
  }

  describe("testing", () => {
    it("testing does not need the test framework", () => {
      for (const file of fk.testingSources) {
        expect(readFileSync(file, "utf8")).not.toContain("vitest");
      }
    });

    it("simulated arm on the shipped profile moves and stops", async () => {
      const clock = new VirtualClock();
      const [arm, sim] = simArm(clock);
      const joints = arm.config.joints;
      expect(close(sim.positions, joints.map((j) => Math.min(j.hi, Math.max(j.lo, 0.0))), 1e-3)).toBe(true);
      await arm.connect();
      await arm.enable();
      try {
        const goal = joints.map((j) => (j.lo + j.hi) / 2);
        expect(await arm.moveTo(goal, { timeout: 10.0 })).toBe(true);
        expect(close(sim.positions, goal, 0.05)).toBe(true); // moveTo arrives within its tolerance
        expect(close(arm.getJointPositions(), sim.positions, 1e-3)).toBe(true);
      } finally {
        expect(await arm.stop()).toBe(true);
        await arm.disconnect();
      }
      expect([...sim.motors.values()].some((m) => m.enabled)).toBe(false);
    });

    it("simulated arm starts where asked and reports temperatures", async () => {
      const clock = new VirtualClock();
      const config = defaultConfig();
      const start = config.joints.map((j) => Math.min(j.hi, Math.max(j.lo, 0.1)));
      const [arm] = simArm(clock, { config, positions: start, motor: { temperature: 40.0 } });
      await arm.connect();
      try {
        expect(close(arm.getJointPositions(), start, 1e-3)).toBe(true);
        expect(close(arm.getTemperatures(), start.map(() => 40.0), 1.0)).toBe(true);
        expect(() => simArm(clock, { config, positions: [0.0] })).toThrow(ValueError);
        expect(() => simArm(clock, { config, positions: [0.0] })).toThrow(/one value per joint/);
      } finally {
        await arm.disconnect();
      }
    });

    it("injected fault overheating and silence stop the arm", async () => {
      const injections: Array<(sim: Sim) => void> = [
        (sim) => sim.fault(0), (sim) => sim.setTemperature("gripper", 150.0), (sim) => sim.offline("gripper"),
      ];
      for (const inject of injections) {
        const clock = new VirtualClock();
        const [arm, sim] = simArm(clock);
        await arm.connect();
        await arm.enable();
        try {
          inject(sim);
          expect(await waitFor(() => arm.state === ArmState.FAULT, clock, 5.0), JSON.stringify(arm.status())).toBe(true);
          expect(arm.faultReason).toBeTruthy();
        } finally {
          await arm.disable();
          await arm.disconnect();
        }
      }
    });

    it("simulated arms relax only the loop timing", () => {
      const shipped = defaultConfig();
      const [arm] = simArm(new VirtualClock());
      expect(arm.config.feedbackTimeout).toBe(Math.max(shipped.feedbackTimeout, SIM_FEEDBACK_TIMEOUT));
      expect(arm.config.controlRateHz).toBe(Math.min(shipped.controlRateHz, SIM_CONTROL_RATE_HZ));
      const speed = shipped.startupStep * shipped.controlRateHz; // the same startup speed, rad/s
      expect(Math.abs(arm.config.startupStep * arm.config.controlRateHz - speed)).toBeLessThanOrEqual(1e-6 * Math.abs(speed));
      expect(arm.config.joints).toEqual(shipped.joints);
      expect(arm.config.maxVelocity).toBe(shipped.maxVelocity);
    });

    it("a late tick of a simulated arm catches up with the clock", async () => {
      const clock = new VirtualClock();
      const [arm] = simArm(clock);
      await arm.connect();
      await arm.enable({ startLoop: false });
      try {
        arm["synced"] = new Array<boolean>(arm.config.nJoints).fill(true);
        const q = arm.getJointPositions();
        const start = q[0]!;
        q[0] = q[0]! + 1.0;
        expect(arm.setJointTargets(q)).toBe(true);
        const dt = 1.0 / arm.config.controlRateHz;
        const step = arm.config.maxVelocity * dt;
        await arm["tick"](dt); // the first tick: one period
        await clock.sleep(2 * SIM_MAX_TICK_PERIODS * dt);
        await arm["tick"](dt); // very late: it catches up as far as the cap
        const moved = arm.getCommandedPositions()[0]! - start;
        const expected = (1 + SIM_MAX_TICK_PERIODS) * step;
        expect(Math.abs(moved - expected)).toBeLessThanOrEqual(Math.max(1e-6 * Math.abs(expected), 1e-12));
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("unknown joint is named", () => {
      const [, sim] = simArm(new VirtualClock());
      // Python raises KeyError; the TS port throws RangeError.
      expect(() => sim.fault("tail")).toThrow(RangeError);
      expect(() => sim.fault("tail")).toThrow(/no joint/);
    });

    it("simulated group from a rig description", async () => {
      const clock = new VirtualClock();
      const rig = { arms: { left: { backend: "gs_usb" }, right: { backend: "socketcan", channel: "can1" } } };
      const [group, sims] = await simulatedGroup(rig, { clock, logger: new RecordingLogger() });
      for (const [, arm] of group) arm.PARAM_TIMEOUT = 0.005;
      expect(new Set(sims.keys())).toEqual(new Set(["left", "right"]));
      expect(group.parallel).toBe(true);
      await group.open();
      try {
        await group.enable();
        const left = group.get("left");
        const target = left.gripperTarget(1.0);
        const q = left.getJointPositions();
        q[q.length - 1] = target;
        expect(await left.moveTo(q, { timeout: 10.0 })).toBe(true);
        expect(Math.abs(sims.get("left")!.positions.at(-1)! - target)).toBeLessThanOrEqual(0.05);
        expect(Math.abs(left.gripperOpening() - 1.0)).toBeLessThanOrEqual(0.05);
      } finally {
        await group.close();
      }
      expect([...sims.values()].some((sim) => [...sim.motors.values()].some((m) => m.enabled))).toBe(false);
      expect(sims.get("right")!.positions.every((v) => Number.isFinite(v))).toBe(true);
    });
  });
}
