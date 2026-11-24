// Port of tests/unit/test_teleop.py (identical in both Python SDKs), on VirtualClock.
import { describe, expect, it } from "vitest";
import { ArmState, ValueError } from "@makermods/arm-core";
import { VirtualClock } from "@makermods/arm-core/testing";
import { LeaderError } from "../../src/leaders/base.js";
import { MAX_PENDING_HOOKS, MAX_STEP_S, runMany, SpeedCap, Teleop, type TeleopOptions, type TeleopStep } from "../../src/teleop.js";
import type { ArmFakes } from "./fakes.js";
import { RecordingLogger } from "./fakes.js";
import { FakeArm, OffsetClock, ScriptedLeader } from "./teleop-fakes.js";
import type { Leader } from "../../src/leaders/base.js";

/** Reads `pose` `n` times, then never settles (a wedged serial port) or, with `lateBy`, answers after that many seconds. */
class HangingLeader implements Leader {
  readonly name = "hanging";
  reads = 0;
  constructor(readonly n: number, readonly pose: number[], readonly clock?: VirtualClock, readonly lateBy?: number,
              readonly lostAfter?: number) {}
  async connect(): Promise<void> {}
  async close(): Promise<void> {}
  read(): Promise<number[] | null> {
    this.reads += 1;
    if (this.lostAfter !== undefined && this.reads > this.lostAfter) return Promise.reject(new LeaderError("unplugged"));
    if (this.reads <= this.n) return Promise.resolve([...this.pose]);
    if (this.lateBy !== undefined) return this.clock!.sleep(this.lateBy).then(() => this.pose.map((v) => v + 1));
    return new Promise(() => {});
  }
}

/** Records the clock time of every hold. */
function holdTimes(arm: FakeArm, clock: VirtualClock): number[] {
  const times: number[] = [];
  const hold = arm.holdCurrentPosition.bind(arm);
  arm.holdCurrentPosition = () => {
    times.push(clock.now());
    return hold();
  };
  return times;
}

const ZERO = [0, 0, 0, 0, 0, 0, 0];
const pose = (...head: number[]): number[] => [...head, ...new Array<number>(7 - head.length).fill(0.0)];
const close = (a: number, b: number, rel = 1e-6): boolean => Math.abs(a - b) <= Math.max(rel * Math.abs(b), 1e-12);

function expectApprox(actual: readonly number[], expected: readonly number[]): void {
  expect(actual.length).toBe(expected.length);
  actual.forEach((a, i) => expect(close(a, expected[i]!), `[${i}] ${a} ≈ ${expected[i]}`).toBe(true));
}

/** onStep hook that aborts after n cycles and keeps every step. */
function stopAfter(controller: AbortController, n: number): { steps: TeleopStep[]; hook: (s: TeleopStep) => void } {
  const steps: TeleopStep[] = [];
  return {
    steps,
    hook: (step) => {
      steps.push(step);
      if (steps.length >= n) controller.abort();
    },
  };
}

async function run(leader: ScriptedLeader, follower: FakeArm, cycles = 20, options: TeleopOptions = {}):
  Promise<[string, TeleopStep[], Teleop]> {
  const clock = new VirtualClock({ start: 100 });
  const stop = new AbortController();
  const { steps, hook } = stopAfter(stop, cycles);
  const loop = new Teleop(leader, follower, { onStep: hook, clock, logger: new RecordingLogger(), ...options });
  return [await loop.run(stop.signal), steps, loop];
}

const enabled = (options: ConstructorParameters<typeof FakeArm>[0] = {}): FakeArm => new FakeArm({ state: ArmState.ENABLED, ...options });

/** Registers the tests (product-neutral; run once per arm package). */
export function teleopSuite(fk: Pick<ArmFakes, "name">): void {
  describe(`${fk.name}: teleop following`, () => {
    it("targets pass straight through without a cap", async () => {
      const follower = enabled();
      const [reason] = await run(new ScriptedLeader([pose(0.1), pose(0.2), pose(0.3)]), follower, 3);
      expect(reason).toBe("stopped");
      expectApprox(follower.targets.map((t) => t[0]!), [0.2, 0.3, 0.3]); // first read was the startup read
      expect(follower.calls.at(-1)).toBe("hold");
    });

    it("follower must be enabled", async () => {
      const [reason] = await run(new ScriptedLeader([ZERO]), new FakeArm({ state: ArmState.CONNECTED }));
      expect(reason.startsWith("follower_not_enabled")).toBe(true);
    });

    it("max velocity caps each joint but not the gripper", async () => {
      const follower = enabled();
      const goal = [...pose(1.0, -1.0).slice(0, 6), 1.0];
      await run(new ScriptedLeader([goal]), follower, 10, { rateHz: 100, maxVelocity: 2.0, syncThreshold: 2.0 });
      const first = follower.targets[0]!;
      expect(close(first[0]!, 0.02) && close(first[1]!, -0.02)).toBe(true); // 2 rad/s * 10 ms
      expect(first[6]).toBe(1.0); // exempt
      expect(close(follower.targets.at(-1)![0]!, 0.2)).toBe(true);
    });

    it("acceleration cap ramps the speed up", () => {
      const cap = new SpeedCap([0.0]);
      const moves = Array.from({ length: 5 }, () => cap.step([10.0], 0.01, [5.0], [100.0])[0]!);
      const speeds = moves.map((b, i) => b - ([0.0, ...moves][i]!));
      expectApprox(speeds, [0.01, 0.02, 0.03, 0.04, 0.05]);
    });

    it("acceleration cap brakes onto the goal without overshoot", () => {
      const cap = new SpeedCap([0.0]);
      const path = Array.from({ length: 200 }, () => cap.step([0.3], 0.01, [5.0], [20.0])[0]!);
      expect(Math.max(...path)).toBeLessThanOrEqual(0.3 + 1e-12);
      expect(close(path.at(-1)!, 0.3)).toBe(true);
    });

    it("a paused cycle cannot bank motion", async () => {
      const follower = enabled();
      const clock = new OffsetClock(new VirtualClock({ start: 100 }));
      const stop = new AbortController();
      const hook = (): void => {
        clock.offset += 10.0; // the machine froze for 10 s after this cycle
        if (follower.targets.length >= 2) stop.abort();
      };
      await new Teleop(new ScriptedLeader([pose(5.0)]), follower, {
        maxVelocity: 1.0, syncThreshold: 10, onStep: hook, clock, logger: new RecordingLogger(),
      }).run(stop.signal);
      expect(close(follower.targets[1]![0]! - follower.targets[0]![0]!, MAX_STEP_S * 1.0)).toBe(true);
    });
  });

  describe(`${fk.name}: teleop startup alignment`, () => {
    it("a large startup gap is closed slowly, then following resumes", async () => {
      const follower = enabled();
      const [, steps, loop] = await run(new ScriptedLeader([pose(1.0)]), follower, 400,
        { rateHz: 100, syncThreshold: 0.5, syncVelocity: 0.5 });
      expect(close(loop.startupGap!, 1.0)).toBe(true);
      expect(close(follower.targets[0]![0]!, 0.005)).toBe(true);
      const synced = steps.findIndex((s) => !s.syncing);
      expect(synced).toBeGreaterThanOrEqual(195); // 1 rad at 0.5 rad/s, 100 Hz
      expect(synced).toBeLessThanOrEqual(205);
      follower.targets.slice(1).forEach((b, i) => expect(Math.abs(b[0]! - follower.targets[i]![0]!)).toBeLessThanOrEqual(0.005 + 1e-9));
    });

    it("a small startup gap follows at once", async () => {
      const follower = enabled({ positions: pose(0.1) });
      const [, steps] = await run(new ScriptedLeader([pose(0.3)]), follower, 2, { syncThreshold: 0.5 });
      expect(steps[0]!.syncing).toBe(false);
      expect(close(follower.targets[0]![0]!, 0.3)).toBe(true);
    });

    it("no leader reading at startup never moves the follower", async () => {
      const follower = enabled();
      const [reason] = await run(new ScriptedLeader([null]), follower, 20, { startupTimeout: 0.5 });
      expect(reason.startsWith("leader_lost")).toBe(true);
      expect(follower.targets).toEqual([]);
    });
  });

  describe(`${fk.name}: teleop failsafes`, () => {
    it("consecutive misses stop following and hold", async () => {
      const follower = enabled();
      const script = [pose(0.1), pose(0.1), ...new Array<null>(10).fill(null)];
      const [reason] = await run(new ScriptedLeader(script), follower, 100, { missLimit: 5 });
      expect(reason).toBe("leader_lost: 5 reads missed in a row");
      expect(follower.targets.length).toBe(1 + 4); // the good cycle, then the last goal held through 4 misses
      expect(follower.targets.every((t) => close(t[0]!, 0.1))).toBe(true);
      expect(follower.calls.at(-1)).toBe("hold");
    });

    it("intermittent misses and read errors are tolerated", async () => {
      const follower = enabled();
      const script = [ZERO, null, pose(0.1), new Error("glitch"), pose(0.2), null, pose(0.3)];
      const [reason] = await run(new ScriptedLeader(script), follower, 6, { missLimit: 2 });
      expect(reason).toBe("stopped");
      expect(close(follower.targets.at(-1)![0]!, 0.3)).toBe(true);
    });

    it("a leader error stops at once", async () => {
      const [reason] = await run(new ScriptedLeader([ZERO, new LeaderError("servo 3 lost")]), enabled());
      expect(reason).toBe("leader_lost: servo 3 lost");
    });

    it("a follower fault stops the loop", async () => {
      const follower = enabled();
      const hook = (): void => {
        if (follower.targets.length === 3) follower.fault("motor 2 overcurrent");
      };
      const reason = await new Teleop(new ScriptedLeader([ZERO]), follower, {
        onStep: hook, clock: new VirtualClock({ start: 100 }), logger: new RecordingLogger(),
      }).run();
      expect(reason).toBe("follower_fault: motor 2 overcurrent");
      expect(follower.calls.at(-1)).toBe("hold");
    });

    it("a stall holds and names the motor", async () => {
      const follower = enabled();
      const records: Array<[number, number[]]> = [];
      const stall = {
        record(elapsed: number, targets: number[]) {
          records.push([elapsed, targets]);
          return records.length === 4 ? [{ motorId: 3 }] : [];
        },
      };
      const [reason] = await run(new ScriptedLeader([pose(0.1)]), follower, 20, { stall });
      expect(reason).toBe("stall: motor 3");
      expect(records.length).toBe(4);
      expect(follower.calls.at(-1)).toBe("hold");
    });

    it("a crash inside the loop still holds", async () => {
      const follower = enabled();
      follower.setJointTargets = () => {
        throw new Error("division by zero");
      };
      const [reason] = await run(new ScriptedLeader([ZERO]), follower);
      expect(reason.startsWith("error")).toBe(true);
      expect(follower.calls.at(-1)).toBe("hold");
    });

    it.each([{ rateHz: 0 }, { maxVelocity: 0 }, { maxAcceleration: -1 }, { missLimit: 0 }, { syncVelocity: NaN }])(
      "bad settings are refused (%o)", (options) => {
        expect(() => new Teleop(new ScriptedLeader([ZERO]), new FakeArm(), options)).toThrow(ValueError);
      });
  });

  describe(`${fk.name}: teleop bimanual`, () => {
    it("runMany stops both pairs when one stops", async () => {
      const clock = new VirtualClock({ start: 100 });
      const left = enabled();
      const right = enabled();
      const bad = new ScriptedLeader([ZERO, new LeaderError("unplugged")]);
      const good = new ScriptedLeader([pose(0.1)]);
      const logger = new RecordingLogger();
      const reasons = await runMany([
        new Teleop(bad, left, { rateHz: 200, clock, logger }), new Teleop(good, right, { rateHz: 200, clock, logger }),
      ]);
      expect(reasons).toEqual(["leader_lost: unplugged", "stopped"]);
      expect(left.calls.at(-1)).toBe("hold");
      expect(right.calls.at(-1)).toBe("hold");
    });

    it("runMany honours the caller's signal", async () => {
      const clock = new VirtualClock({ start: 100 });
      const arms = [enabled(), enabled()];
      const stop = new AbortController();
      clock.timer(0.1, () => stop.abort());
      const logger = new RecordingLogger();
      const reasons = await runMany(arms.map((a) => new Teleop(new ScriptedLeader([ZERO]), a, { rateHz: 200, clock, logger })),
        stop.signal);
      expect(reasons).toEqual(["stopped", "stopped"]);
      expect(arms.every((a) => a.targets.length > 0)).toBe(true);
    });
  });
  describe(`${fk.name}: teleop abort and hooks (the promise-based loop)`, () => {
    it("an abort while a leader read never settles holds at once and ends the run", async () => {
      const clock = new VirtualClock({ start: 100 });
      const follower = enabled();
      const holds = holdTimes(follower, clock);
      const stop = new AbortController();
      clock.timer(0.5, () => stop.abort());
      const reason = await new Teleop(new HangingLeader(3, pose(0.1)), follower, { clock, logger: new RecordingLogger() })
        .run(stop.signal);
      expect(reason).toBe("stopped");
      expect(holds[0]).toBe(100.5);
      expect(follower.targets.length).toBe(2); // the startup read, then two cycles before the read that hangs
    });

    it("a leader reading that arrives after the abort is discarded", async () => {
      const clock = new VirtualClock({ start: 100 });
      const follower = enabled();
      const stop = new AbortController();
      clock.timer(0.3, () => stop.abort());
      const reason = await new Teleop(new HangingLeader(3, pose(0.1), clock, 1.0), follower, { clock, logger: new RecordingLogger() })
        .run(stop.signal);
      expect(reason).toBe("stopped");
      const sent = follower.targets.length;
      await clock.advance(2.0);
      expect(follower.targets.length).toBe(sent);
      expect(follower.targets.every((t) => t[0] === 0.1)).toBe(true);
    });

    it("an abort while a stall hook never settles holds at once and ends the run", async () => {
      const clock = new VirtualClock({ start: 100 });
      const follower = enabled();
      const holds = holdTimes(follower, clock);
      let calls = 0;
      const stall = { record: () => (++calls >= 3 ? new Promise<never>(() => {}) : []) };
      const stop = new AbortController();
      clock.timer(0.25, () => stop.abort());
      const reason = await new Teleop(new ScriptedLeader([pose(0.1)]), follower, { clock, stall, logger: new RecordingLogger() })
        .run(stop.signal);
      expect(reason).toBe("stopped");
      expect(holds[0]).toBe(100.25);
      expect(follower.targets.length).toBe(3);
    });

    it("runMany holds the other follower the moment one pair ends, though its leader hangs", async () => {
      const clock = new VirtualClock({ start: 100 });
      const left = enabled();
      const right = enabled();
      const leftHolds = holdTimes(left, clock);
      const rightHolds = holdTimes(right, clock);
      const logger = new RecordingLogger();
      const reasons = await runMany([
        new Teleop(new HangingLeader(2, pose(0.1)), left, { clock, logger }),
        new Teleop(new HangingLeader(1000, pose(0.2), undefined, undefined, 30), right, { clock, logger }),
      ]);
      expect(reasons).toEqual(["stopped", "leader_lost: unplugged"]);
      expect(leftHolds[0]).toBe(rightHolds[0]);
      expect(left.targets.length).toBe(1);
    });

    it("runMany holds every follower the moment the caller aborts", async () => {
      const clock = new VirtualClock({ start: 100 });
      const arms = [enabled(), enabled()];
      const holds = arms.map((a) => holdTimes(a, clock));
      const stop = new AbortController();
      clock.timer(0.2, () => stop.abort());
      const logger = new RecordingLogger();
      const reasons = await runMany([
        new Teleop(new HangingLeader(5, pose(0.1)), arms[0]!, { clock, logger }),
        new Teleop(new ScriptedLeader([pose(0.2)]), arms[1]!, { clock, logger }),
      ], stop.signal);
      expect(reasons).toEqual(["stopped", "stopped"]);
      expect(holds.map((h) => h[0])).toEqual([100.2, 100.2]);
    });

    it("a rejected async onStep is logged and never escapes", async () => {
      const logger = new RecordingLogger();
      const stop = new AbortController();
      let n = 0;
      const onStep = async (): Promise<void> => {
        if (++n >= 5) stop.abort();
        throw new Error("boom");
      };
      const clock = new VirtualClock({ start: 100 });
      const reason = await new Teleop(new ScriptedLeader([pose(0.1)]), enabled(), { clock, logger, onStep }).run(stop.signal);
      await clock.advance(0.1);
      expect(reason).toBe("stopped");
      expect(logger.messages.filter(([level, m]) => level === "error" && m === "onStep failed: boom").length).toBe(5);
    });

    it("onStep promises that never settle are bounded: later calls are skipped, targets keep flowing", async () => {
      const logger = new RecordingLogger();
      const clock = new VirtualClock({ start: 100 });
      const follower = enabled();
      const stop = new AbortController();
      clock.timer(0.3, () => stop.abort());
      let calls = 0;
      const loop = new Teleop(new ScriptedLeader([pose(0.1)]), follower, {
        clock, logger, onStep: () => {
          calls += 1;
          return new Promise(() => {});
        },
      });
      expect(await loop.run(stop.signal)).toBe("stopped");
      expect(calls).toBe(MAX_PENDING_HOOKS);
      expect(follower.targets.length).toBeGreaterThan(20);
      expect(loop.droppedSteps).toBe(follower.targets.length - MAX_PENDING_HOOKS);
      expect(logger.messages.some(([level, m]) => level === "warn" && m.startsWith("onStep skipped"))).toBe(true);
    });
  });
}
