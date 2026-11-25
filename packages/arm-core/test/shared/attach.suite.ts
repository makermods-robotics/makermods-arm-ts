// Port of the Python SDK's tests/unit/test_attach.py: Arm.attach(), taking over an arm left holding with
// torque on, on simulated motors.
import { describe, expect, it } from "vitest";
import { Arm, type ArmConfig, ArmState, type CanBackend, ConnectError, gravity, kinematics, radians, StateError } from "@makermods/arm-core";
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
  async function leftEnergized(clock: VirtualClock, config?: ArmConfig): Promise<[Sim, CanBackend]> {
    const [first, sim] = simArm(clock, config ? { config } : {});
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

  describe("attach gravity at takeover (TypeScript only)", () => {
    interface Takeover {
      mode?: "ramp" | "measured";
      /** The chain motors report the capped command at this gravity level: clip(model * carried, ±cap). */
      carried: number;
      prepare?: (sim: Sim, arm: Arm, clock: VirtualClock) => void | Promise<void>;
      config?: ArmConfig;
    }

    /** Attach to the arm a previous session left holding; the first frame's torque to shoulder_lift and the full model value. */
    async function firstLiftTorque({ mode, carried, prepare, config }: Takeover):
        Promise<{ tau: number; full: number; reported: number; arm: Arm }> {
      const clock = new VirtualClock();
      const [sim, backend] = await leftEnergized(clock, config);
      const make = () => {
        const a = new Arm(sim.config, backend, { clock, frameSpacing: 0, logger: new RecordingLogger(), attachGravity: mode });
        a.PARAM_TIMEOUT = 0.005;
        return a;
      };
      let arm = make();
      if (sim.motor("gripper").faulted) {
        await expect(arm.connect()).rejects.toThrow(ConnectError);
        arm = make();
      }
      const model = gravity.fromConfig(sim.config, arm.chain)!;
      const q = sim.positions.slice(0, model.n);
      const capped = (level: number): number[] =>
        model.model(q).map((t, i) => Math.max(-model.limitsNm[i]!, Math.min(model.limitsNm[i]!, t * level)));
      const holding = capped(1.0), reports = capped(carried);
      const motors = [...sim.motors.values()] as unknown as Array<{ torque: number }>;
      reports.forEach((t, i) => (motors[i]!.torque = t * sim.config.joints[i]!.direction));
      await arm.connect(false, true);
      await prepare?.(sim, arm, clock);
      const lift = sim.config.joints.findIndex((j) => j.name === "shoulder_lift");
      clearLogs(sim);
      await arm.attach();
      expect(await waitFor(() => sim.motor("shoulder_lift").frames.length >= 1, clock)).toBe(true);
      const full = holding[lift]! * sim.config.joints[lift]!.direction;
      const reported = reports[lift]! * sim.config.joints[lift]!.direction;
      return { tau: sim.motor("shoulder_lift").frames[0]![2], full, reported, arm };
    }

    it("eases gravity in from zero by default, as the Python SDK does", async () => {
      const { tau, full, arm } = await firstLiftTorque({ carried: 1.0 });
      try {
        expect(Math.abs(full)).toBeGreaterThan(0.5);
        expect(Math.abs(tau)).toBeLessThan(0.1);
      } finally {
        await arm.disable();
      }
    });

    it("measured: starts near full level when the motors carry the model's holding torque", async () => {
      const { tau, full, arm } = await firstLiftTorque({ mode: "measured", carried: 1.0 });
      try {
        expect(Math.abs(full)).toBeGreaterThan(0.5); // a loaded joint
        expect(Math.abs(tau - full)).toBeLessThan(0.05 * Math.abs(full) + 0.05);
      } finally {
        await arm.disable();
      }
    });

    it("measured: starts in between when the motors carry part of it", async () => {
      const { tau, full, arm } = await firstLiftTorque({ mode: "measured", carried: 0.5 });
      try {
        expect(Math.abs(tau - 0.5 * full)).toBeLessThan(0.1 * Math.abs(full));
      } finally {
        await arm.disable();
      }
    });

    for (const [what, carried] of [["an energized arm lying on its support with no torque", 0.0], ["a wrong-sign torque", -1.0]] as const) {
      it(`measured: starts from zero for ${what}`, async () => {
        const { tau, full, arm } = await firstLiftTorque({ mode: "measured", carried });
        try {
          expect(Math.abs(full)).toBeGreaterThan(0.5);
          expect(Math.abs(tau)).toBeLessThan(0.1); // level 0 (one torque step of rounding)
        } finally {
          await arm.disable();
        }
      });
    }

    it("measured: a torque sample older than feedbackTimeout keeps the ease-in from zero", async () => {
      const { tau, full, arm } = await firstLiftTorque({
        mode: "measured", carried: 1.0,
        prepare: (_sim, a, clock) => {
          // a slow gripper enable: by the first holding frame the chain's last reading is older than feedbackTimeout
          const g = a.config.joints.findIndex((j) => j.name === "gripper");
          const enable = a.motors[g]!.enableConfirmed.bind(a.motors[g]!);
          a.motors[g]!.enableConfirmed = async (timeout?: number) => {
            await clock.sleep(a.config.feedbackTimeout + 0.2);
            return enable(timeout);
          };
        },
      });
      try {
        expect(Math.abs(full)).toBeGreaterThan(0.5);
        expect(Math.abs(tau)).toBeLessThan(0.1);
      } finally {
        await arm.disable();
      }
    });

    it("measured: the level fits the capped command the sender computes", async () => {
      const base = fk.defaultConfig();
      const lift = base.joints.findIndex((j) => j.name === "shoulder_lift");
      const model0 = gravity.fromConfig(base, kinematics.chainFromConfig(base)!)!;
      const t = Math.abs(model0.model(homePose(base).slice(0, model0.n))[lift]!);
      // The shoulder carries the load (the other chain joints are capped near zero) and its model
      // torque is twice its cap. The previous owner held it at gravity level 0.3, under the cap: a
      // fit against the capped model would put the first command at the cap instead.
      const config = base.replace({
        joints: base.joints.map((j, i) => (i === lift ? j.replace({ gravityLimitNm: 0.5 * t })
          : j.gravityLimitNm !== null ? j.replace({ gravityLimitNm: 0.01 }) : j)),
      });
      const { tau, reported, full, arm } = await firstLiftTorque({ mode: "measured", carried: 0.3, config });
      try {
        expect(Math.abs(reported)).toBeLessThan(0.8 * Math.abs(full)); // held under the cap
        expect(Math.abs(tau - reported)).toBeLessThan(0.05 * t + 0.02);
      } finally {
        await arm.disable();
      }
    });

    it("measured: a chain joint whose fresh reading reports torque off by the fit starts from zero", async () => {
      const { tau, full, arm } = await firstLiftTorque({
        mode: "measured", carried: 1.0,
        prepare: (sim, a) => {
          // while the gripper is being enabled, the elbow reports torque off in a fresh reading
          const g = a.config.joints.findIndex((j) => j.name === "gripper");
          const e = a.config.joints.findIndex((j) => j.name === "elbow_flex");
          const enable = a.motors[g]!.enableConfirmed.bind(a.motors[g]!);
          a.motors[g]!.enableConfirmed = async (timeout?: number) => {
            sim.motor("elbow_flex").enabled = false;
            await a.motors[e]!.readState(0.05);
            return enable(timeout);
          };
        },
      });
      try {
        expect(Math.abs(full)).toBeGreaterThan(0.5);
        expect(Math.abs(tau)).toBeLessThan(0.2 * Math.abs(full));
      } finally {
        await arm.disable();
      }
    });

    it("measured: starts from zero when a chain joint reports torque off or no torque state", async () => {
      for (const state of ["off", "unknown"] as const) {
        let applies = true;
        const { tau, full, arm } = await firstLiftTorque({
          mode: "measured", carried: 1.0,
          prepare: (sim, a) => {
            const i = sim.config.joints.findIndex((j) => j.name === "elbow_flex");
            if (state === "off") {
              sim.motor("elbow_flex").enabled = false; // lying on its support, say
            } else if ("paramsSupported" in a.motors[i]!) {
              (a.motors[i] as unknown as { paramsSupported: boolean }).paramsSupported = false; // firmware without torque state
            } else {
              applies = false; // this product's motors always report torque state
            }
          },
        });
        try {
          if (!applies) continue;
          expect(Math.abs(full)).toBeGreaterThan(0.5);
          // still easing in: the first tick comes a few tens of ms after attach() starts the ramp
          expect(Math.abs(tau), `torque state ${state}`).toBeLessThan(0.2 * Math.abs(full));
        } finally {
          await arm.disable();
        }
      }
    });
  });

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
