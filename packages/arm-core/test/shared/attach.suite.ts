// Port of the Python SDK's tests/unit/test_attach.py: Arm.attach(), taking over an arm left holding with
// torque on, on simulated motors.
import { describe, expect, it } from "vitest";
import { Arm, type ArmConfig, ArmState, type CanBackend, ConnectError, radians, StateError } from "@makermods/arm-core";
import { type SimulatedMotors, VirtualClock } from "@makermods/arm-core/testing";
import { type ArmFakes, type FakeMotorLike, type FakeMotorOptions, RecordingLogger, waitFor } from "./fakes.js";

/** Registers the shared tests on `fk`'s arm. */
export function attachSuite<M extends FakeMotorLike>(fk: ArmFakes<M>): void {
  const HOME: Record<string, number> = { shoulder_lift: -1.0, elbow_flex: 1.2, wrist_flex: 0.3 };
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

  function homePose(config: ArmConfig): number[] {
    const q = config.joints.map((j) => Math.min(j.hi, Math.max(j.lo, 0.0)));
    config.joints.forEach((j, i) => {
      if (j.name in HOME) q[i] = Math.min(j.hi - 0.05, Math.max(j.lo + 0.05, HOME[j.name]!));
    });
    return q;
  }

  /**
   * [sim, backend]: a previous session enabled the arm, moved it home and detached, then the
   * gripper's watchdog released it.
   */
  async function leftEnergized(clock: VirtualClock): Promise<[Sim, CanBackend]> {
    const [first, sim] = simArm(clock);
    await first.connect();
    await first.enable();
    expect(await first.moveTo(homePose(first.config), { speed: 20.0, timeout: 10.0 })).toBe(true);
    await first.detach();
    sim.watchdogTimeout("gripper");
    return [sim, first.backend];
  }

  function freshArm(sim: Sim, backend: CanBackend, clock: VirtualClock): Arm {
    const arm = new Arm(sim.config, backend, { clock, frameSpacing: 0, logger: new RecordingLogger() });
    arm.PARAM_TIMEOUT = 0.005;
    return arm;
  }

  function clearLogs(sim: Sim): void {
    for (const m of sim.motors.values()) {
      m.log.length = 0;
      m.frames.length = 0;
    }
  }

  function stiff(sim: Sim): M[] {
    const motors = [...sim.motors.values()];
    return sim.config.joints.flatMap((j, i) => (j.releaseOnFault ? [] : [motors[i]!]));
  }

  async function refused(sim: Sim, arm: Arm, match: RegExp): Promise<void> {
    clearLogs(sim);
    const done = arm.attach();
    await expect(done).rejects.toThrow(ConnectError);
    await expect(done).rejects.toThrow(match);
    expect(arm.state).toBe(ArmState.CONNECTED);
    for (const m of stiff(sim)) {
      expect(m.enabled && !m.frames.length && !m.log.includes("disable") && !m.log.includes("enable")).toBe(true);
    }
  }

  describe("attach", () => {
    it("attach takes over without a sag and rests only on an explicit target", async () => {
      const clock = new VirtualClock();
      const [sim, backend] = await leftEnergized(clock);
      let arm = freshArm(sim, backend, clock);
      if (sim.motor("gripper").faulted) { // the protocol reports the watchdog release as a status
        await expect(arm.connect()).rejects.toThrow(ConnectError);
        arm = freshArm(sim, backend, clock);
      }
      await arm.connect(false, true);
      const held = [...sim.motors.values()].map((m) => m.position);
      const before = sim.positions;
      clearLogs(sim);
      await arm.attach();
      try {
        expect(arm.state).toBe(ArmState.ENABLED);
        expect(arm.restPose).toBeNull();
        expect(await waitFor(() => [...sim.motors.values()].every((m) => m.frames.length >= 5), clock)).toBe(true);
        // zip(stiff(sim), held): Python zip stops at the shorter list.
        stiff(sim).forEach((m, k) => {
          const position = held[k]!;
          expect(m.log.includes("enable") || m.log.includes("disable")).toBe(false);
          expect(m.frames.every(([, kp]) => kp > 0)).toBe(true); // no zero-gain frame
          expect(Math.abs(m.frames[0]![0] - position)).toBeLessThanOrEqual(1e-3); // first command: the measured pose
        });
        expect(sim.motor("gripper").log).toContain("enable"); // re-enabled where it is
        expect(close(sim.positions, before, 1e-3)).toBe(true); // nothing moved
        expect(await arm.stop()).toBe(false); // no rest pose to verify: keeps holding
        expect(arm.state).toBe(ArmState.ENABLED);
        expect([...sim.motors.values()].every((m) => m.enabled)).toBe(true);
        const rest = arm.config.joints.map((j) => Math.min(j.hi, Math.max(j.lo, 0.0)));
        expect(await arm.returnToRest(rest)).toBe(true);
      } finally {
        await arm.disable();
      }
      expect([...sim.motors.values()].some((m) => m.enabled)).toBe(false);
    });

    it("attach refuses an arm with torque off", async () => {
      const clock = new VirtualClock();
      const [arm, sim] = simArm(clock);
      await arm.connect();
      clearLogs(sim);
      const done = arm.attach();
      await expect(done).rejects.toThrow(ConnectError);
      await expect(done).rejects.toThrow(/use enable/);
      expect(arm.state).toBe(ArmState.CONNECTED);
      const motors = [...sim.motors.values()];
      expect(motors.some((m) => m.log.includes("enable") || m.log.includes("disable"))).toBe(false);
      expect(motors.some((m) => m.frames.length > 0)).toBe(false);
    });

    it("attach refuses a fault a silent motor and a joint outside its limits", async () => {
      const clock = new VirtualClock();
      let [sim, backend] = await leftEnergized(clock);
      let arm = freshArm(sim, backend, clock);
      await arm.connect(false, true);
      sim.fault("elbow_flex");
      await refused(sim, arm, /fault/);
      sim.fault("elbow_flex", false);

      arm = freshArm(sim, backend, clock);
      await arm.connect(false, true);
      sim.offline("wrist_roll");
      await refused(sim, arm, /no reply/);
      sim.offline("wrist_roll", false);

      const config = simArm(clock)[0].config;
      const q = homePose(config);
      const i = config.joints.map((j) => j.name).indexOf("elbow_flex");
      q[i] = config.joints[i]!.lo - radians(40);
      [arm, sim] = simArm(clock, { config, positions: q, motor: { enabled: true } });
      await arm.connect();
      await refused(sim, arm, /outside its limits/);
    });

    it("attach cancelled sends nothing", async () => {
      const clock = new VirtualClock();
      const [sim, backend] = await leftEnergized(clock);
      const arm = freshArm(sim, backend, clock);
      await arm.connect(false, true);
      clearLogs(sim);
      const cancel = new AbortController();
      cancel.abort();
      await expect(arm.attach(cancel.signal)).rejects.toThrow(StateError);
      expect(stiff(sim).every((m) => m.enabled && !m.frames.length)).toBe(true);
    });

    it.for([true, false])("unknown torque gets a holding frame before its enable (was_on=%s)", async (wasOn, ctx) => {
      const clock = new VirtualClock();
      const config = simArm(clock)[0].config;
      const [arm, sim] = simArm(clock, { config, positions: homePose(config), motor: { enabled: wasOn, paramsSupported: false } });
      await arm.connect();
      if (arm.motors.some((m) => m.feedback!.enabled !== null)) ctx.skip(); // this protocol always reports torque state
      const before = sim.positions;
      clearLogs(sim);
      await arm.attach();
      try {
        const motors = [...sim.motors.values()];
        config.joints.forEach((j, k) => {
          const m = motors[k]!;
          const first = m.log.indexOf("enable");
          expect(first).toBeGreaterThan(0);
          const entry = m.log[first - 1]!; // holding frame, profile gains, then enable
          expect(Array.isArray(entry)).toBe(true);
          const [kind, kp] = entry as [string, number];
          expect(kind).toBe("cmd");
          expect(Math.abs(kp - j.kp)).toBeLessThanOrEqual(0.01 * Math.abs(j.kp));
          expect(m.frames.every(([, k2]) => k2 > 0)).toBe(true);
        });
        expect(motors.every((m) => m.enabled)).toBe(true);
        expect(close(sim.positions, before, 1e-3)).toBe(true);
      } finally {
        await arm.disable();
      }
    });
  });
}
