// Ports of the Python SDK's motion tests on the shipped profile (tests/unit/test_arm_model.py: move_ee
// planning and ownership; tests/unit/test_arm_robstride.py: the return onto the support), plus the
// control loop's timing (loop rate, late ticks, frame spacing).
import { describe, expect, it } from "vitest";
import {
  Arm, type ArmConfig, ArmState, FRAME_SPACING_S, MAX_TICK_PERIODS, REST_PRESS, REST_PRESS_NM, trajectoryIk,
} from "@makermods/arm-core";
import { type SimulatedMotors, VirtualClock } from "@makermods/arm-core/testing";
import * as planner from "../../src/planner.js";
import { type ArmFakes, type FakeMotorLike, type FakeMotorOptions, RecordingLogger } from "./fakes.js";

const radians = (deg: number): number => deg * (Math.PI / 180);
const degrees = (rad: number): number => rad * (180 / Math.PI);
const HOVER_DEG = [0.0, -15.0, 22.0, 0.0, 0.0, 0.0];
// Near the supported rest pose (zero clipped into the limits) of both shipped profiles.
const NEAR_REST_DEG = [0.0, -4.5, 10.0, -1.2, 0.0, 0.0];

/** Registers the shared tests on `fk`'s arm. */
export function motionSuite<M extends FakeMotorLike>(fk: ArmFakes<M>): void {
  type Sim = SimulatedMotors<M>;

  function close(actual: readonly number[], expected: readonly number[], abs: number): boolean {
    return actual.length === expected.length && actual.every((a, i) => Math.abs(a - expected[i]!) <= abs);
  }

  /** The shipped profile's joints at `deg` (the gripper at its closed-side limit nearest zero). */
  function pose(config: ArmConfig, deg: number[]): number[] {
    return config.joints.map((j, i) => (i < deg.length ? radians(deg[i]!) : Math.min(j.hi, Math.max(j.lo, 0.0))));
  }

  function simArm(options: { config?: ArmConfig; positions?: number[]; motor?: FakeMotorOptions } = {}):
      { arm: Arm; sim: Sim; clock: VirtualClock; logger: RecordingLogger } {
    const clock = new VirtualClock();
    const logger = new RecordingLogger();
    const config = options.config ?? fk.defaultConfig();
    const [arm, sim] = fk.simulatedArm({ positions: pose(config, HOVER_DEG), ...options, config, clock, logger });
    arm.PARAM_TIMEOUT = 0.005;
    return { arm, sim, clock, logger };
  }

  async function enabled(options: Parameters<typeof simArm>[0] = {}) {
    const made = simArm(options);
    await made.arm.connect();
    await made.arm.enable();
    return made;
  }

  async function teardown(arm: Arm): Promise<void> {
    if (arm.state !== ArmState.DISCONNECTED) {
      await arm.disable();
      await arm.disconnect();
    }
  }

  /** The commanded tool point moved by `d` (m). */
  function goalFrom(arm: Arm, d: number[]): number[] {
    const chain = arm.chain;
    const p = chain.fk(arm.getCommandedPositions().slice(0, chain.n));
    return [p[0]![3]! + d[0]!, p[1]![3]! + d[1]!, p[2]![3]! + d[2]!];
  }

  /** Planning that takes no clock time (its pauses are microtasks). */
  function instantPlanning(arm: Arm): void {
    arm["planPause"] = () => Promise.resolve();
  }

  /** Planning that takes `seconds` of clock time per pause of the solve. */
  function slowPlanning(arm: Arm, clock: VirtualClock, seconds = 0.01): void {
    arm["planPause"] = () => clock.sleep(seconds);
  }

  const logged = (logger: RecordingLogger, text: string): boolean => logger.messages.some(([, m]) => m.includes(text));

  describe("moveEe", () => {
    it("moves the tool point to the goal", async () => {
      const { arm } = await enabled();
      try {
        instantPlanning(arm);
        const goal = goalFrom(arm, [0.0, 0.0, 0.02]);
        expect(await arm.moveEe(goal, { timeout: 5.0 })).toBe(true);
        expect(close(arm.eePose()[0], goal, 3e-3)).toBe(true);
      } finally {
        await teardown(arm);
      }
    });

    it("is refused unless enabled", async () => {
      const { arm } = simArm();
      await arm.connect();
      try {
        expect(await arm.moveEe(goalFrom(arm, [0.0, 0.0, 0.02]))).toBe(false);
      } finally {
        await arm.disconnect();
      }
    });

    for (const stop of ["timeout", "cancel"] as const) {
      it(`planning that does not finish returns false holding (${stop})`, async () => {
        const { arm, clock, logger } = await enabled();
        try {
          slowPlanning(arm, clock, 0.02);
          const goal = goalFrom(arm, [0.0, 0.0, 0.02]);
          const before = arm.getCommandedPositions(), ticks = arm["tickCount"];
          const cancel = new AbortController();
          if (stop === "cancel") clock.timer(0.3, () => cancel.abort());
          const t0 = clock.now();
          const ok = await arm.moveEe(goal, { timeout: stop === "timeout" ? 0.3 : 30.0, signal: cancel.signal });
          const took = clock.now() - t0;
          expect(ok).toBe(false);
          expect(took).toBeLessThan(0.5);
          expect(logged(logger, stop === "timeout" ? "timed out" : "cancelled")).toBe(true);
          expect(logged(logger, "nothing was sent")).toBe(true);
          expect(arm.state).toBe(ArmState.ENABLED);
          expect(close(arm.getCommandedPositions().slice(0, 6), before.slice(0, 6), 1e-9)).toBe(true); // the chain held still
          expect(arm["tickCount"] - ticks).toBeGreaterThan(0.5 * took * arm.config.controlRateHz); // the loop kept running
          arm["planPause"] = () => Promise.resolve();
          expect(await arm.moveTo(before, { timeout: 2.0 })).toBe(true); // and the arm still takes targets
        } finally {
          await teardown(arm);
        }
      });
    }

    it("a newer command during planning owns the arm", async () => {
      const { arm, clock } = await enabled();
      try {
        slowPlanning(arm, clock);
        const goal = goalFrom(arm, [0.0, 0.0, 0.02]);
        let at = 0;
        const moving = arm.moveEe(goal, { timeout: 10.0 }).then((ok) => {
          at = clock.now();
          return ok;
        });
        await clock.sleep(0.1);
        const newer = [0.3, ...arm.getCommandedPositions().slice(1)];
        expect(await arm.moveTo(newer, { wait: false })).toBe(true);
        const superseded = clock.now();
        expect(await moving).toBe(false);
        expect(at - superseded).toBeLessThan(0.5);
        await clock.sleep(1.2);
        expect(close(arm["userTargets"], newer, 1e-12)).toBe(true);
      } finally {
        await teardown(arm);
      }
    });

    it("an e-stop and a new enable during planning send nothing of the old move", async () => {
      const { arm, clock } = await enabled();
      try {
        slowPlanning(arm, clock);
        const moving = arm.moveEe(goalFrom(arm, [0.0, 0.0, 0.02]), { timeout: 10.0 });
        await clock.sleep(0.1);
        await arm.estop();
        await arm.enable();
        const held = arm.getCommandedPositions();
        expect(await moving).toBe(false);
        await clock.sleep(1.2);
        expect(close(arm["userTargets"], held, 1e-12)).toBe(true);
      } finally {
        await teardown(arm);
      }
    });

    for (const stop of ["timeout", "abort"] as const) {
      it(`a planner that never settles does not keep moveEe pending (${stop})`, async () => {
        const { arm, clock } = await enabled();
        try {
          arm["planFn"] = () => new Promise<never>(() => {});
          const cancel = new AbortController();
          if (stop === "abort") clock.timer(0.2, () => cancel.abort());
          const t0 = clock.now();
          const before = arm.getCommandedPositions();
          expect(await arm.moveEe(goalFrom(arm, [0.0, 0.0, 0.02]), { timeout: stop === "timeout" ? 0.3 : undefined, signal: cancel.signal }))
            .toBe(false);
          expect(clock.now() - t0).toBeLessThan(0.5);
          expect(close(arm.getCommandedPositions().slice(0, 6), before.slice(0, 6), 1e-9)).toBe(true);
        } finally {
          await teardown(arm);
        }
      });
    }

    it("a never-settling planner with a long timeout gives up at the planning stage's deadline", async () => {
      const { arm, clock } = await enabled();
      try {
        arm["planFn"] = () => new Promise<never>(() => {});
        const t0 = clock.now();
        expect(await arm.moveEe(goalFrom(arm, [0.0, 0.0, 0.02]), { timeout: 3600 })).toBe(false);
        expect(clock.now() - t0).toBeLessThan(planner.PLAN_START_S + 0.5);
      } finally {
        await teardown(arm);
      }
    });

    it("a never-settling planner gives up when a newer hold supersedes it", async () => {
      const { arm, clock } = await enabled();
      try {
        arm["planFn"] = () => new Promise<never>(() => {});
        clock.timer(0.3, () => arm.holdCurrentPosition());
        const t0 = clock.now();
        expect(await arm.moveEe(goalFrom(arm, [0.0, 0.0, 0.02]), { timeout: 3600 })).toBe(false);
        expect(clock.now() - t0).toBeLessThan(0.5);
      } finally {
        await teardown(arm);
      }
    });

    it("a hold during planning drops the motion; no row is committed", async () => {
      const { arm, clock } = await enabled();
      try {
        let pauses = 0;
        arm["planPause"] = async () => {
          pauses += 1;
          if (pauses === 3) arm.holdCurrentPosition(); // stop()'s hold takes over
          await clock.sleep(0);
        };
        const before = arm.getCommandedPositions();
        expect(await arm.moveEe(goalFrom(arm, [0.0, 0.0, 0.02]), { timeout: 5.0 })).toBe(false);
        await clock.sleep(0.2);
        expect(close(arm["userTargets"], before, 1e-3)).toBe(true);
      } finally {
        await teardown(arm);
      }
    });

    it("a cancelled moveEe never overwrites a newer command", async () => {
      const { arm, clock } = await enabled();
      try {
        instantPlanning(arm);
        const cancel = new AbortController();
        const moving = arm.moveEe(goalFrom(arm, [0.0, 0.0, 0.04]), { speed: 0.5, signal: cancel.signal });
        await clock.sleep(0.1); // streaming
        const newer = [0.3, ...arm.getCommandedPositions().slice(1)];
        expect(arm.setJointTargets(newer)).toBe(true);
        cancel.abort(); // the cancel path would hold the measured pose
        expect(await moving).toBe(false);
        await clock.sleep(0.1);
        expect(close(arm["userTargets"], newer, 1e-12)).toBe(true);
      } finally {
        await teardown(arm);
      }
    });

    it("a cancelled moveEe streaming in the background holds the measured pose", async () => {
      const { arm, clock } = await enabled();
      try {
        instantPlanning(arm);
        const cancel = new AbortController();
        expect(await arm.moveEe(goalFrom(arm, [0.0, 0.0, 0.04]), { speed: 0.5, wait: false, signal: cancel.signal })).toBe(true);
        await clock.sleep(0.1);
        cancel.abort();
        await clock.sleep(0.05);
        expect(close(arm["userTargets"], arm.getJointPositions(), 0.02)).toBe(true);
        const held = [...arm["userTargets"]];
        await clock.sleep(0.2);
        expect(arm["userTargets"]).toEqual(held); // nothing streams afterwards
      } finally {
        await teardown(arm);
      }
    });
  });

  describe("motion ownership (stricter than Python 28621d6 where noted)", () => {
    it("a motion superseded after its plan but before its first row commits no row", async () => {
      const { arm, clock } = await enabled();
      try {
        instantPlanning(arm);
        const plan = arm["planFn"];
        arm["planFn"] = async (...args: Parameters<typeof plan>) => {
          const result = await plan(...args);
          arm.holdCurrentPosition(); // stop()'s hold takes over at the last moment
          return result;
        };
        const before = arm.getCommandedPositions();
        expect(await arm.moveEe(goalFrom(arm, [0.0, 0.0, 0.02]), { timeout: 5.0 })).toBe(false);
        await clock.sleep(0.2);
        expect(close(arm["userTargets"].slice(0, 6), before.slice(0, 6), 1e-3)).toBe(true);
      } finally {
        await teardown(arm);
      }
    });

    it("an e-stop and a new enable during the planner setup leave the new session alone", async () => {
      const { arm, clock } = await enabled();
      const setup = trajectoryIk.TrajectoryIK.fromConfig;
      let overtaken: Promise<void> | null = null;
      trajectoryIk.TrajectoryIK.fromConfig = (...args: Parameters<typeof setup>) => {
        overtaken = (async () => {
          await arm.estop();
          await arm.enable();
        })();
        return setup(...args);
      };
      try {
        instantPlanning(arm);
        expect(await arm.moveEe(goalFrom(arm, [0.0, 0.0, 0.02]), { timeout: 5.0 })).toBe(false);
        await overtaken;
        const held = arm.getCommandedPositions();
        await clock.sleep(0.3);
        expect(close(arm["userTargets"], held, 1e-6)).toBe(true);
      } finally {
        trajectoryIk.TrajectoryIK.fromConfig = setup;
        await teardown(arm);
      }
    });

    it("a cancel during the last row's period holds (stricter)", async () => {
      const { arm, clock } = await enabled();
      try {
        const q0 = arm.getCommandedPositions().slice(0, 6);
        const last = [q0[0]! + 0.4, ...q0.slice(1)];
        arm["planFn"] = async () => ({ q: [q0, last], reason: "" });
        const period = 1.0 / arm.config.controlRateHz;
        const cancel = new AbortController();
        clock.timer(1.5 * period, () => cancel.abort());
        expect(await arm.moveEe(goalFrom(arm, [0.0, 0.0, 0.0]), { speed: 0.5, signal: cancel.signal })).toBe(false);
        expect(Math.abs(arm["userTargets"][0]! - last[0]!)).toBeGreaterThan(0.1); // the measured pose, not the last row
        expect(close(arm["userTargets"].slice(0, 6), arm.getJointPositions().slice(0, 6), 0.03)).toBe(true);
      } finally {
        await teardown(arm);
      }
    });

    it("a frame computed before a hold is not sent after it (stricter)", async () => {
      const { arm, sim, clock } = await enabled();
      try {
        const lift = 1;
        const far = [...arm.getCommandedPositions()];
        far[lift] = far[lift]! - 0.5;
        expect(arm.setJointTargets(far)).toBe(true);
        await clock.sleep(0.1); // moving
        // The next send to shoulder_pan is slow; meanwhile a hold is installed.
        const bus = arm.backend as unknown as { send: (id: number, d: Uint8Array, e?: boolean) => Promise<void> };
        const send = bus.send.bind(bus);
        let armed = true, held: number[] | null = null, mark = 0;
        bus.send = async (id: number, d: Uint8Array, e?: boolean) => {
          if (armed && id === arm.motors[0]!.motorId && d.length === 8 && !d.slice(0, 6).every((b) => b === 0xff)) {
            armed = false;
            await send(id, d, e);
            await clock.sleep(0.003);
            arm.holdCurrentPosition();
            held = arm.getCommandedPositions();
            mark = sim.motor("shoulder_lift").frames.length;
            return;
          }
          return send(id, d, e);
        };
        await clock.sleep(0.05);
        expect(held).not.toBeNull();
        const frames = sim.motor("shoulder_lift").frames;
        // every shoulder_lift frame sent after the hold carries the held position
        const after = frames.slice(mark);
        expect(after.length).toBeGreaterThan(0);
        expect(after.every(([p]) => Math.abs(p - held![lift]! * arm.config.joints[lift]!.direction) < 1e-3)).toBe(true);
      } finally {
        await teardown(arm);
      }
    });
  });

  describe("planning limits", () => {
    function setup() {
      const config = fk.defaultConfig();
      const { arm } = simArm({ config });
      const chain = arm.chain;
      const ik = trajectoryIk.TrajectoryIK.fromConfig(config, { chain });
      const q0 = pose(config, HOVER_DEG).slice(0, chain.n);
      const p = chain.fk(q0);
      return { chain, ik, q0, start: [p[0]![3]!, p[1]![3]!, p[2]![3]!], rate: config.controlRateHz };
    }

    it("a path longer than the tick budget is refused before it is built", async () => {
      const { chain, ik, q0, start, rate } = setup();
      const result = await planner.planEe(chain, ik, [start[0]!, start[1]!, start[2]! + 0.03], null, q0, rate, 1e-4);
      expect(result.q).toBe(null);
      expect(result.reason).toContain("split the move");
    });

    for (const stop of ["timeout", "cancel"] as const) {
      it(`planning honours the limit and cancel between pauses (${stop})`, async () => {
        const { chain, ik, q0, start, rate } = setup();
        const clock = new VirtualClock();
        let cancelled = false;
        if (stop === "cancel") clock.timer(0.3, () => (cancelled = true));
        const result = await planner.plan(chain, ik, [start[0]!, start[1]!, start[2]! + 0.03], null, q0, rate, 0.05, {
          clock, limit: stop === "timeout" ? 0.3 : null, cancel: () => cancelled, pause: () => clock.sleep(0.02),
        });
        expect(result.q).toBe(null);
        expect(result.reason).toContain(stop === "timeout" ? "timed out" : "cancelled");
        expect(clock.now()).toBeLessThan(0.3 + 0.02 + 1e-9); // checked at every pause
      });
    }

    it("a plan that ends past its limit is refused", async () => {
      const { chain, ik, q0, start, rate } = setup();
      const clock = new VirtualClock();
      const fast = await planner.plan(chain, ik, [start[0]!, start[1]!, start[2]! + 0.01], null, q0, rate, null,
        { clock, pause: () => Promise.resolve() });
      expect(fast.q).not.toBe(null);
      // every pause takes 1 ms: the plan crosses a limit set just short of its length
      const timed = new VirtualClock();
      await planner.plan(chain, ik, [start[0]!, start[1]!, start[2]! + 0.01], null, q0, rate, null,
        { clock: timed, pause: () => timed.sleep(0.001) });
      const length = timed.now();
      const late = new VirtualClock();
      const result = await planner.plan(chain, ik, [start[0]!, start[1]!, start[2]! + 0.01], null, q0, rate, null,
        { clock: late, limit: length - 0.0005, pause: () => late.sleep(0.001) });
      expect(result.q).toBe(null);
      expect(result.reason).toContain("timed out");
    });

    it("an expired allowance is checked before the path's own replaces it (stricter)", () => {
      const clock = new VirtualClock({ start: 10 });
      const budget = new planner.Budget(clock, null, null);
      void clock.advance(planner.PLAN_START_S + 0.1);
      return clock.advance(0).then(() => {
        expect(() => budget.ticks(100)).toThrow(planner.PlanStopped);
      });
    });

    it("straightLineAsync gives the event loop a turn every few hundred rows and builds the same rows", async () => {
      const { chain, q0 } = setup();
      const start = chain.fk(q0), goal = chain.fk(q0.map((q) => q + 0.2));
      let pauses = 0;
      const rows = await trajectoryIk.straightLineAsync(start, goal, 10000, { pause: async () => void (pauses += 1) });
      expect(pauses).toBe(Math.floor(9999 / trajectoryIk.STRAIGHT_LINE_ROWS_PER_PAUSE));
      expect(rows).toEqual(trajectoryIk.straightLine(start, goal, 10000));
    });

    it("the budget: start allowance, then margin plus per tick, never past the ceiling or the caller's limit", () => {
      const clock = new VirtualClock({ start: 10 });
      const budget = new planner.Budget(clock, null, null);
      expect(budget.end()).toBe(10 + planner.PLAN_START_S);
      budget.ticks(100);
      expect(budget.end()).toBe(10 + planner.PLAN_MARGIN_S + 100 * planner.PLAN_S_PER_TICK);
      budget.ticks(planner.MAX_PATH_TICKS);
      expect(budget.end()).toBe(10 + planner.PLAN_CEILING_S);
      expect(new planner.Budget(clock, 0.4, null).end()).toBe(10.4);
    });

    it("solvePath stops when its between hook throws", () => {
      const { chain, ik, q0 } = setup();
      const poses = trajectoryIk.straightLine(chain.fk(q0), chain.fk(q0.map((q) => q + 0.01)), 200);
      let chunks = 0;
      expect(() => trajectoryIk.solvePath(ik, poses, q0, 200, null, 64, () => {
        chunks += 1;
        if (chunks === 2) throw new planner.PlanStopped("stop");
      })).toThrow(planner.PlanStopped);
      expect(chunks).toBe(2);
    });
  });

  describe("the return onto the support", () => {
    const window = (config: ArmConfig): ArmConfig => config.replace({ restWindow: radians(12.0) });

    it("the press toward the support is bounded by effort and stays inside the limits", async () => {
      const config = window(fk.defaultConfig());
      const { arm } = await enabled({ config, positions: pose(config, NEAR_REST_DEG) });
      try {
        const cfg = arm.config, q = arm.getJointPositions(), rest = arm.restPose!;
        for (const i of [1, 2, 3]) { // the joints that carry the arm's weight
          const press = Math.abs(rest[i]! - q[i]!);
          expect(press).toBeGreaterThan(0);
          expect(press).toBeLessThanOrEqual(Math.min(REST_PRESS, REST_PRESS_NM / cfg.joints[i]!.kp) + 1e-9);
          expect(press * cfg.joints[i]!.kp).toBeLessThanOrEqual(REST_PRESS_NM + 1e-9); // N*m of proportional effort at most
          expect(cfg.joints[i]!.lo <= rest[i]! && rest[i]! <= cfg.joints[i]!.hi).toBe(true);
        }
        expect(Math.abs(rest[0]! - q[0]!)).toBeLessThan(1e-9);
        expect(close(rest.slice(4), q.slice(4), 1e-9)).toBe(true);
      } finally {
        await teardown(arm);
      }
    });

    it("an arm enabled away from rest returns where it started", async () => {
      const config = window(fk.defaultConfig());
      const { arm } = await enabled({ config });
      try {
        expect(close(arm.restPose!.slice(0, 6), HOVER_DEG.map(radians), 1e-3)).toBe(true);
      } finally {
        await teardown(arm);
      }
    });

    it("without rest_window the return goes where the enable found the arm", async () => {
      const config = fk.defaultConfig().replace({ restWindow: null });
      const { arm } = await enabled({ config, positions: pose(config, NEAR_REST_DEG) });
      try {
        expect(arm.restPose).toEqual(arm.getJointPositions());
      } finally {
        await teardown(arm);
      }
    });

    it("enable and rest cycles keep the arm on its support", async () => {
      // The simulated shoulder_lift as the real one behaves at rest: it cannot go past its support
      // (-4.5 deg, short of the clipped rest pose); coming back under power it stops where the
      // proportional term balances a standing 1 N*m (on the lifted side); released, it stays.
      const config = window(fk.defaultConfig()).replace({ restSpeed: radians(120), restRateHz: 200 });
      const { arm: first, sim, clock } = simArm({ config, positions: pose(config, NEAR_REST_DEG) });
      const motor = sim.motor("shoulder_lift");
      motor.track = false;
      const support = radians(-4.5), force = 1.0;
      let q = motor.position;
      const handle = motor.handle.bind(motor);
      motor.handle = (canId: number, data: Uint8Array) => {
        const before = motor.frames.length;
        const replies = handle(canId, data);
        if (motor.frames.length > before) {
          const [cmd, kp] = motor.frames[motor.frames.length - 1]!;
          if (motor.enabled && kp >= 1.0) {
            q = cmd >= q ? Math.max(q, cmd - force / kp) : cmd;
            q = Math.min(q, support);
          }
          motor.position = q;
        }
        return replies;
      };
      const lifts: number[] = [];
      for (let cycle = 0; cycle < 8; cycle++) {
        const arm = new Arm(sim.config, first.backend, { clock, logger: new RecordingLogger(), frameSpacing: 0 });
        arm.PARAM_TIMEOUT = 0.005;
        await arm.connect();
        await arm.enable();
        try {
          const now = arm.getJointPositions();
          expect(await arm.moveTo([...HOVER_DEG.map(radians), ...now.slice(6)], { timeout: 5.0 })).toBe(true);
          expect(await arm.stop()).toBe(true);
        } finally {
          await teardown(arm);
        }
        await arm.connect();
        lifts.push(degrees(arm.getJointPositions()[1]!));
        await arm.disconnect();
      }
      expect(lifts.every((v) => Math.abs(v - -4.5) < 0.5), JSON.stringify(lifts)).toBe(true); // every session ends on the support
    });

    it("stop does not release a weight-carrying joint left out of the return", async () => {
      const base = fk.defaultConfig();
      const config = base.replace({ joints: base.joints.map((j, i) => (i === 1 ? j.replace({ returnToRest: false }) : j)) });
      const { arm, sim } = await enabled({ config });
      try {
        expect(await arm.stop()).toBe(false); // shoulder_lift is raised (hover) and nothing brings it back
        expect(arm.state).toBe(ArmState.ENABLED);
        expect(sim.motor("shoulder_lift").enabled).toBe(true);
      } finally {
        await teardown(arm);
      }
    });
  });

  describe("stop() rechecks (stricter than Python 28621d6)", () => {
    it("enabled outside a soft limit near the support, the rest target is not clamped past the press bound", async () => {
      const base = fk.defaultConfig();
      const config = base.replace({ restWindow: radians(12.0) });
      const lift = config.joints.findIndex((j) => j.name === "shoulder_lift");
      const positions = pose(config, NEAR_REST_DEG);
      positions[lift] = config.joints[lift]!.hi + radians(4.0); // past the soft limit, inside fullTurnGrace
      const { arm } = await enabled({ config, positions });
      try {
        const q = arm.getJointPositions(), j = arm.config.joints[lift]!;
        expect(Math.abs(arm.restPose![lift]! - q[lift]!)).toBeLessThanOrEqual(Math.min(REST_PRESS, REST_PRESS_NM / j.kp) + 1e-9);
      } finally {
        await teardown(arm);
      }
    });

    it("a weight-carrying joint left out of the return is rechecked right before the release", async () => {
      const base = fk.defaultConfig();
      const lift = base.joints.findIndex((j) => j.name === "shoulder_lift");
      const config = base.replace({
        joints: base.joints.map((j, i) => (i === lift ? j.replace({ returnToRest: false }) : j)), restWindow: null,
      });
      const positions = pose(config, NEAR_REST_DEG);
      positions[lift] = Math.min(config.joints[lift]!.hi, Math.max(config.joints[lift]!.lo, 0.0)); // at its support
      const { arm, sim } = await enabled({ config, positions });
      const tick = arm["tick"].bind(arm);
      let moved = false;
      arm["tick"] = (dt: number, session: number | null = null, poll = true) => {
        if (!arm["running"] && !moved) {
          moved = true;
          const m = sim.motor("shoulder_lift");
          m.track = false;
          m.position -= 0.4; // raised during the final recheck
        }
        return tick(dt, session, poll);
      };
      try {
        expect(await arm.stop()).toBe(false);
        expect(moved).toBe(true);
        expect(arm.state).toBe(ArmState.ENABLED);
        expect(sim.motor("shoulder_lift").enabled).toBe(true);
      } finally {
        arm["tick"] = tick;
        await teardown(arm);
      }
    });
  });

  describe("control loop timing", () => {
    it("measures the loop rate and reports it in status()", async () => {
      const { arm, clock } = fk.makeArm(null, {}, new VirtualClock({ start: 1000 }));
      await arm.connect();
      await arm.enable();
      try {
        expect(arm.loopRateHz).toBe(null);
        expect(arm.status().loopRateHz).toBe(null);
        await clock.sleep(2.5);
        expect(Math.abs(arm.loopRateHz! - arm.config.controlRateHz)).toBeLessThan(2.0);
        expect(arm.status().loopRateHz).toBe(Math.round(arm.loopRateHz! * 10) / 10);
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("warns once per session when the loop runs below 90 % of the rate", async () => {
      const { arm, clock, logger } = fk.makeArm(null, {}, new VirtualClock({ start: 1000 }));
      const tick = arm["tick"].bind(arm);
      arm["tick"] = async (dt: number, session: number | null = null, poll = true) => {
        await tick(dt, session, poll);
        await clock.sleep(0.009); // each tick takes 9 ms of a 5 ms period
      };
      await arm.connect();
      await arm.enable();
      try {
        await clock.sleep(5.5);
        expect(arm.loopRateHz!).toBeLessThan(0.9 * arm.config.controlRateHz);
        const warnings = logger.messages.filter(([level, m]) => level === "warn" && m.includes("cannot keep up"));
        expect(warnings.length).toBe(1);
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("the slow-loop warning goes out once per session, also after a recovery", async () => {
      const { arm, clock, logger } = fk.makeArm(null, {}, new VirtualClock({ start: 1000 }));
      let slow = true;
      const tick = arm["tick"].bind(arm);
      arm["tick"] = async (dt: number, session: number | null = null, poll = true) => {
        await tick(dt, session, poll);
        if (slow) await clock.sleep(0.009);
      };
      await arm.connect();
      await arm.enable();
      try {
        await clock.sleep(3.5);
        slow = false;
        await clock.sleep(2.5); // recovered
        slow = true;
        await clock.sleep(3.5);
        const warnings = logger.messages.filter(([level, m]) => level === "warn" && m.includes("cannot keep up"));
        expect(warnings.length).toBe(1);
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("the loop rate after stop() restarted the loop does not count the downtime", async () => {
      const { arm, clock, fakes } = fk.makeArm(null, { restSpeed: radians(600), restRateHz: 200, restTimeout: 2.0 },
        new VirtualClock({ start: 1000 }));
      await arm.connect();
      await arm.enable();
      const tick = arm["tick"].bind(arm);
      let moved = false;
      arm["tick"] = async (dt: number, session: number | null = null, poll = true) => {
        if (!arm["running"] && !moved) {
          moved = true;
          await clock.sleep(0.4); // a slow recheck
          fakes[1]!.track = false;
          fakes[1]!.position += 0.1; // and the joint moved: stop() holds and restarts the loop
        }
        return tick(dt, session, poll);
      };
      try {
        await clock.sleep(0.7);
        expect(await arm.stop()).toBe(false);
        const restarted = clock.now();
        const rates: number[] = [];
        while (clock.now() - restarted < 1.2) {
          await clock.sleep(0.01);
          if (arm.loopRateHz !== null && rates[rates.length - 1] !== arm.loopRateHz) rates.push(arm.loopRateHz);
        }
        expect(rates.length).toBeGreaterThan(0);
        expect(Math.abs(rates[rates.length - 1]! - arm.config.controlRateHz)).toBeLessThan(2.0);
        expect(rates.every((r) => Math.abs(r - arm.config.controlRateHz) < 2.0), JSON.stringify(rates)).toBe(true);
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("each tick is credited the time that really passed, at most four periods", async () => {
      const { arm, clock } = fk.makeArm(null, {}, new VirtualClock({ start: 1000 }));
      const period = 1.0 / arm.config.controlRateHz;
      const dts: number[] = [];
      let stall = 0;
      const tick = arm["tick"].bind(arm);
      arm["tick"] = async (dt: number, session: number | null = null, poll = true) => {
        if (session !== null) dts.push(dt);
        await tick(dt, session, poll);
        if (stall > 0) {
          const s = stall;
          stall = 0;
          await clock.sleep(s);
        }
      };
      await arm.connect();
      await arm.enable();
      try {
        await clock.sleep(0.1);
        expect(dts[0]).toBe(0.0); // the first tick: no time has passed
        expect(dts.slice(1).every((dt) => Math.abs(dt - period) < 1e-9)).toBe(true);
        let mark = dts.length;
        stall = 0.0123; // one late tick
        await clock.sleep(0.05);
        expect(Math.abs(dts[mark + 1]! - 0.0123)).toBeLessThan(1e-9);
        expect(Math.abs(dts[mark + 2]! - period)).toBeLessThan(1e-9); // the next one only its own interval
        mark = dts.length;
        stall = 0.2; // a long stall: credited four periods
        await clock.sleep(0.3);
        expect(dts[mark + 1]).toBe(MAX_TICK_PERIODS * period);
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("frames of a tick are spaced start to start", async () => {
      const clock = new VirtualClock({ start: 1000 });
      const motors = [new fk.FakeMotor(1, undefined, { position: 0.3 }), new fk.FakeMotor(2, undefined, { position: -0.5 })];
      const bus = new fk.FakeBus(motors);
      const arm = new Arm(fk.twoJointConfig(), bus, { clock, logger: new RecordingLogger() });
      arm.PARAM_TIMEOUT = 0.005;
      await arm.connect();
      await arm.enable({ startLoop: false });
      const sends: number[] = [];
      let slow = 0;
      const send = bus.send.bind(bus);
      bus.send = async (canId: number, data: Uint8Array, extended = true) => {
        sends.push(clock.now());
        if (slow > 0) await clock.sleep(slow);
        return send(canId, data, extended);
      };
      try {
        await arm["tick"](0.005);
        expect(sends.length).toBe(2);
        expect(Math.abs(sends[1]! - sends[0]! - FRAME_SPACING_S)).toBeLessThan(1e-12);
        sends.length = 0;
        slow = 0.0003; // a send slower than the spacing needs no extra wait
        await arm["tick"](0.005);
        expect(Math.abs(sends[1]! - sends[0]! - 0.0003)).toBeLessThan(1e-12);
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });
  });
}
