// Port of the Python SDK's tests/unit/test_arm.py.
import { describe, expect, it } from "vitest";
import { type ArmConfigInit, ArmState, ConnectError, HeldByAnotherProgram, StateError, TimeoutError, ValueError } from "@makermods/arm-core";
import { VirtualClock } from "@makermods/arm-core/testing";
import { type ArmFakes, commands, type FakeMotorLike, type FakeMotorOptions, type Made, waitFor } from "./fakes.js";

/** Registers the shared tests on `fk`'s arm. */
export function armSuite<M extends FakeMotorLike>(fk: ArmFakes<M>): void {
  const MODEL = fk.product.defaultModel;
  const { classify, FAULT_TEXT, FakeMotor, FULL_TURN_SUPPORTED, makeArm, twoJointConfig, WARNING_SUPPORTED } = fk;
  const Base = FakeMotor as unknown as new (motorId: number, model?: string, options?: FakeMotorOptions) => FakeMotorLike;
  const radians = (deg: number): number => deg * (Math.PI / 180);

  /** pytest.approx(expected, abs=abs) (default: rel 1e-6, abs 1e-12). */
  function approx(actual: number, expected: number, abs?: number): void {
    const tol = abs ?? Math.max(1e-6 * Math.abs(expected), 1e-12);
    expect(Math.abs(actual - expected), `${actual} ≈ ${expected} ± ${tol}`).toBeLessThanOrEqual(tol);
  }

  function approxAll(actual: number[], expected: number[], abs?: number): void {
    expect(actual.length).toBe(expected.length);
    actual.forEach((v, i) => approx(v, expected[i]!, abs));
  }

  // Python's monotonic clock is far from zero; the tests' 0.0 "sent long ago" sentinels rely on it.
  function make(motors?: M[] | null, overrides: Partial<ArmConfigInit> = {}) {
    return makeArm(motors, overrides, new VirtualClock({ start: 1000 }));
  }

  async function enabledArm(motors?: M[] | null, overrides: Partial<ArmConfigInit> = {}) {
    const made = make(motors, overrides);
    await made.arm.connect();
    await made.arm.enable({ startLoop: false });
    return made;
  }

  type Entry = FakeMotorLike["log"][number];
  const isCmd = (e: Entry): boolean => Array.isArray(e) && e[0] === "cmd";

  function sentTorqueFrames(fake: M, since: number): Entry[] {
    return fake.log.slice(since).filter((e) => e === "enable" || e === "disable" || isCmd(e));
  }

  function fastRest(overrides: Partial<ArmConfigInit> = {}): Partial<ArmConfigInit> {
    return { restSpeed: radians(600), restRateHz: 200, restTimeout: 2.0, ...overrides };
  }

  describe("connect", () => {
    it("reads positions without touching torque", async () => {
      const { arm, fakes } = make([new FakeMotor(1, MODEL, { position: 0.3, enabled: true }), new FakeMotor(2, MODEL, { position: -0.5 })]);
      await arm.connect();
      expect(arm.state).toBe(ArmState.CONNECTED);
      approxAll(arm.getJointPositions(), [0.3, -0.5], 1e-3);
      expect(fakes[1]!.enabled).toBe(true); // an energized arm is not dropped by connect
      for (const fake of Object.values(fakes)) {
        expect(fake.log[0]).toBe("status_query");
        expect(fake.log).not.toContain("disable");
        expect(fake.log).not.toContain("enable");
      }
    });

    it("names missing motors", async () => {
      const { arm, bus } = make([new FakeMotor(1), new FakeMotor(2, MODEL, { online: false })]);
      arm.REPLY_TIMEOUT = 0.005;
      const p = arm.connect();
      await expect(p).rejects.toThrow(ConnectError);
      await expect(p).rejects.toThrow(/no reply from motors \[2\]/);
      expect(bus.isOpen).toBe(false);
      expect(arm.state).toBe(ArmState.DISCONNECTED);
    });

    it("keeps fault evidence by default", async () => {
      const { arm, fakes } = make([new FakeMotor(1), new FakeMotor(2, MODEL, { faulted: true })]);
      const p = arm.connect();
      await expect(p).rejects.toThrow(ConnectError);
      await expect(p).rejects.toThrow(new RegExp(FAULT_TEXT));
      expect(fakes[2]!.faulted).toBe(true);
      expect(fakes[2]!.log).not.toContain("clear");
    });

    it("can clear faults explicitly", async () => {
      const { arm, fakes } = make([new FakeMotor(1), new FakeMotor(2, MODEL, { faulted: true, position: -0.5 })]);
      await arm.connect(true);
      expect(fakes[2]!.faulted).toBe(false);
      expect(arm.state).toBe(ArmState.CONNECTED);
    });

    it.skipIf(!FULL_TURN_SUPPORTED)("applies the full-turn correction per session", async () => {
      const { arm } = make([new FakeMotor(1, MODEL, { position: 0.3 + 2 * Math.PI }), new FakeMotor(2, MODEL, { position: -0.5 })]);
      await arm.connect();
      approx(arm.getJointPositions()[0]!, 0.3, 1e-3);
    });
  });

  describe("enable", () => {
    it("preloads a neutral command then enables", async () => {
      const { arm, fakes } = make();
      await arm.connect();
      await arm.enable({ startLoop: false });
      for (const fake of Object.values(fakes)) {
        const i = fake.log.indexOf("enable");
        expect(fake.log[i - 1]).toEqual(["cmd", 0.0]);
      }
      expect(arm.state).toBe(ArmState.ENABLED);
      approxAll(arm.restPose!, [0.3, -0.5], 1e-3);
    });

    it("refuses a stale zero", async () => {
      const { arm, fakes } = make([new FakeMotor(1, MODEL, { position: 3.0 }), new FakeMotor(2, MODEL, { position: -0.5 })]);
      await arm.connect();
      const p = arm.enable({ startLoop: false });
      await expect(p).rejects.toThrow(ConnectError);
      await expect(p).rejects.toThrow(/stale or missing zero/);
      expect(fakes[1]!.log).not.toContain("enable");
    });

    it("refuses a hot gripper", async () => {
      const { arm, fakes } = make([new FakeMotor(1), new FakeMotor(2, MODEL, { position: -0.5, temperature: 46.0 })]);
      await arm.connect();
      const p = arm.enable({ startLoop: false });
      await expect(p).rejects.toThrow(ConnectError);
      await expect(p).rejects.toThrow(/below 45/);
      expect(Object.values(fakes).every((f) => !f.log.includes("enable"))).toBe(true);
    });

    it("refuses motors that already hold without sending them anything", async () => {
      const { arm, fakes } = make([new FakeMotor(1, MODEL, { position: 0.3, enabled: true, paramsSupported: true }),
        new FakeMotor(2, MODEL, { position: -0.5, paramsSupported: true })]);
      await arm.connect();
      const marks = Object.fromEntries(Object.entries(fakes).map(([k, f]) => [k, f.log.length]));
      const p = arm.enable({ startLoop: false });
      await expect(p).rejects.toThrow(HeldByAnotherProgram);
      await expect(p).rejects.toThrow(ConnectError);
      await expect(p).rejects.toThrow(/motors 1 \(j1\) already have torque on.*release/);
      expect(fakes[1]!.enabled).toBe(true);
      expect(arm.state).toBe(ArmState.CONNECTED);
      for (const [k, f] of Object.entries(fakes)) expect(sentTorqueFrames(f, marks[k]!)).toEqual([]); // no preload anywhere
    });

    it("getFaults is zero for healthy enabled motors", async () => {
      const { arm } = make();
      expect(arm.getFaults()).toEqual([null, null]);
      await arm.connect();
      await arm.enable({ startLoop: false });
      expect(arm.getFaults()).toEqual([0, 0]);
      await arm.disable();
    });
  });

  describe("control", () => {
    it("startup sync caps the command near the measured pose", async () => {
      const { arm } = await enabledArm();
      expect(arm.setJointTargets([1.5, -0.5])).toBe(true);
      await arm["tick"](0.005);
      const step = radians(1.0);
      approx(arm.getCommandedPositions()[0]!, 0.3 + step, 1e-3);
    });

    it("a joint is synced once within three degrees", async () => {
      const { arm } = await enabledArm();
      arm.setJointTargets([0.3 + radians(2.5), -0.5]);
      await arm["tick"](0.005);
      expect(arm["synced"][0]).toBe(true);
    });

    it("a stuck joint is released after one second", async () => {
      const { arm, fakes } = await enabledArm();
      fakes[1]!.track = false;
      arm.setJointTargets([1.5, -0.5]);
      await arm["tick"](0.005);
      expect(arm["synced"][0]).toBe(false);
      const [anchor, since] = arm["stuckAnchor"][0]!;
      arm["stuckAnchor"][0] = [anchor, since - 1.1];
      await arm["tick"](0.005);
      expect(arm["synced"][0]).toBe(true);
    });

    it("targets are clipped to limits", async () => {
      const { arm } = await enabledArm();
      arm["synced"] = [true, true];
      arm.setJointTargets([5.0, 0.5]);
      await arm["tick"](1.0);
      approxAll(arm.getCommandedPositions(), [2.0, -0.04]);
    });

    it("enable speed caps the pull inside a limit until the first target", async () => {
      const { arm } = make([new FakeMotor(1, MODEL, { position: 2.2 }), new FakeMotor(2, MODEL, { position: -0.5 })]);
      await arm.connect();
      const p = arm.enable({ startLoop: false, speed: 0.0 });
      await expect(p).rejects.toThrow(ValueError);
      await expect(p).rejects.toThrow(/positive/);
      await arm.enable({ startLoop: false, speed: 0.5 });
      arm["synced"] = [true, true];
      await arm["tick"](0.1);
      approx(arm.getCommandedPositions()[0]!, 2.2 - 0.05, 1e-3);
      arm.setJointTargets(arm.getCommandedPositions());
      await arm["tick"](0.1);
      approx(arm.getCommandedPositions()[0]!, 2.0, 1e-3);
    });

    it("non-finite targets are rejected", async () => {
      const { arm } = await enabledArm();
      expect(arm.setJointTargets([NaN, 0.0])).toBe(false);
    });
  });

  describe("protection", () => {
    it("gripper overtemperature releases the gripper and holds the arm", async () => {
      const { arm, fakes } = await enabledArm();
      fakes[2]!.temperature = 55.0;
      await arm["tick"](0.005);
      expect(arm.state).toBe(ArmState.FAULT);
      expect(arm.faultReason).toContain("overtemperature");
      expect(fakes[2]!.enabled).toBe(false);
      expect(fakes[1]!.enabled).toBe(true);
      const sent = commands(fakes[2]!).length;
      await arm["tick"](0.005);
      expect(commands(fakes[2]!).length).toBe(sent); // released gripper gets no more commands
    });

    it("a motor fault latches and holds the pose", async () => {
      const { arm, fakes } = await enabledArm();
      fakes[1]!.faulted = true;
      arm["pollSent"] = new Map([[1, 0.0], [2, 1e9]]);
      await arm["tick"](0.005); // a status poll or the command reply reports it; the fault latches
      await arm["tick"](0.005);
      expect(arm.state).toBe(ArmState.FAULT);
      expect(arm.faultReason).toContain(FAULT_TEXT);
      expect(fakes[1]!.enabled).toBe(true); // joints hold instead of dropping
      expect(fakes[1]!.faulted).toBe(true); // evidence kept
    });

    it("feedback timeout faults", async () => {
      const { arm, fakes, clock } = await enabledArm(null, { feedbackTimeout: 0.01 });
      fakes[1]!.online = false;
      await clock.sleep(0.02);
      await arm["tick"](0.005);
      expect(arm.state).toBe(ArmState.FAULT);
      expect(arm.faultReason).toContain("feedback timeout");
    });

    it("clearFaults releases and clears", async () => {
      const { arm, fakes } = await enabledArm();
      fakes[1]!.faulted = true;
      arm["pollSent"] = new Map([[1, 0.0], [2, 1e9]]);
      await arm["tick"](0.005);
      await arm["tick"](0.005);
      await arm.clearFaults();
      expect(arm.state).toBe(ArmState.CONNECTED);
      expect(fakes[1]!.faulted).toBe(false);
      expect(arm.motors.every((m) => m.faultLatched === null)).toBe(true);
    });

    it("clearFaults requires a connection", async () => {
      const { arm } = make();
      await expect(arm.clearFaults()).rejects.toThrow(StateError);
    });
  });

  describe("stop path", () => {
    it("returns to rest then releases", async () => {
      const { arm, fakes, clock } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      try {
        arm["synced"] = [true, true];
        arm.setJointTargets([1.0, -1.0]);
        await clock.sleep(0.15);
        approx(arm.getJointPositions()[0]!, 1.0, 0.01);
        expect(await arm.stop()).toBe(true);
        approx(arm.getJointPositions()[0]!, 0.3, radians(2));
        approx(arm.getJointPositions()[1]!, -1.0, 0.01); // gripper excluded
        expect(arm.state).toBe(ArmState.CONNECTED);
        expect(fakes[1]!.enabled).toBe(false);
      } finally {
        await arm.disconnect();
      }
    });

    it("an unverified return keeps holding", async () => {
      const { arm, fakes, clock } = make(null, fastRest({ restTimeout: 0.3 }));
      await arm.connect();
      await arm.enable();
      try {
        arm["synced"] = [true, true];
        arm.setJointTargets([1.0, -0.5]);
        await clock.sleep(0.15);
        fakes[1]!.track = false; // the joint is blocked
        expect(await arm.stop()).toBe(false);
        expect(arm.state).toBe(ArmState.ENABLED);
        expect(fakes[1]!.enabled).toBe(true);
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("already at rest releases immediately", async () => {
      const { arm, fakes } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      try {
        expect(await arm.stop()).toBe(true);
        expect(fakes[1]!.enabled).toBe(false);
      } finally {
        await arm.disconnect();
      }
    });

    it("a joint that went silent is not judged at rest on its last reading", async () => {
      const { arm, fakes } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      try {
        fakes[1]!.online = false; // at rest, but its last reading is still within feedbackTimeout
        expect(await arm.stop()).toBe(false);
        expect([ArmState.ENABLED, ArmState.FAULT]).toContain(arm.state);
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("a stalled loop does not release on a reading from before the stall", async () => {
      // The motors answered after the return began, then the loop stalled: nothing is unanswered,
      // but the readings are old. Release needs a reading younger than feedbackTimeout.
      const { arm, fakes, clock } = make(null, fastRest({ restTimeout: 0.6 }));
      await arm.connect();
      await arm.enable();
      let stalled = false;
      const tick = arm["tick"].bind(arm), wait = arm["waitTicks"].bind(arm);
      arm["tick"] = async (dt: number, session: number | null = null, poll = true) => {
        if (stalled) {
          await clock.sleep(0.01); // the loop runs late and sends nothing
          return;
        }
        return tick(dt, session, poll);
      };
      arm["waitTicks"] = async (n: number, timeout = 1.0) => {
        const ok = await wait(n, timeout);
        stalled = true;
        await clock.sleep(0.4);
        return ok;
      };
      try {
        expect(await arm.stop()).toBe(false);
        expect([ArmState.ENABLED, ArmState.FAULT]).toContain(arm.state);
        expect(fakes[1]!.enabled).toBe(true);
      } finally {
        stalled = false;
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("a return to rest overtaken by an e-stop and an enable writes nothing into the new session", async () => {
      const { arm, clock } = make(null, { restSpeed: radians(20), restRateHz: 200, restTimeout: 3.0 });
      await arm.connect();
      await arm.enable();
      try {
        arm["synced"] = [true, true];
        expect(await arm.moveTo([1.2, -0.5], { timeout: 3.0 })).toBe(true);
        const returning = arm.returnToRest();
        await clock.sleep(0.05); // the return is ramping
        const overtaken = (async () => {
          await arm.estop();
          await arm.enable();
          return arm.getCommandedPositions();
        })();
        expect(await returning).toBe(false);
        const held = await overtaken;
        await clock.sleep(0.1);
        approxAll(arm["userTargets"], held, 1e-6);
        expect(arm["returning"]).toBe(false);
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });

    /** The profile's kp of the returned joint (twoJointConfig), whatever setGains()/floatMode() did since. */
    const PROFILE_KP = twoJointConfig().joints[0]!.kp;

    /** The loop runs and the last frame to the returned joint carries the profile's kp. */
    async function holdingAtFullGains(arm: Made["arm"], fake: FakeMotorLike, clock: VirtualClock): Promise<boolean> {
      await clock.sleep(0.05);
      const last = fake.frames[fake.frames.length - 1];
      return arm["running"] && arm["loop"] !== null && arm["loop"].alive && last !== undefined
        && Math.abs(last[1] - PROFILE_KP) <= 0.2;
    }

    for (const change of ["floatMode", "setGains"] as const) {
      it(`an interrupted stop holds at the profile's gains after ${change}()`, async () => {
        const { arm, fakes, clock } = make(null, fastRest({ restRateHz: 1 }));
        await arm.connect();
        await arm.enable();
        try {
          arm["synced"] = [true, true];
          expect(await arm.moveTo([1.0, -0.5], { timeout: 3.0 })).toBe(true);
          if (change === "floatMode") arm.floatMode(0.2, () => [0.1, 0.0]);
          else arm.setGains([5.0, 20.0], [0.5, 0.5]);
          await clock.sleep(0.05);
          const interrupt = new AbortController();
          clock.timer(0.02, () => interrupt.abort());
          expect(await arm.stop({ signal: interrupt.signal })).toBe(false);
          expect(arm.state).toBe(ArmState.ENABLED);
          expect(await holdingAtFullGains(arm, fakes[1]!, clock)).toBe(true);
          expect(arm.config.joints[0]!.kp).toBe(PROFILE_KP);
          expect(arm["extraTorque"]).toBe(null);
          expect(arm["floating"]).toBe(false);
        } finally {
          await arm.disable();
          await arm.disconnect();
        }
      });
    }

    it("an abort wakes stop() at once, even at a 1 Hz rest rate", async () => {
      const { arm, clock } = make(null, fastRest({ restRateHz: 1, restSpeed: radians(5) }));
      await arm.connect();
      await arm.enable();
      try {
        arm["synced"] = [true, true];
        expect(await arm.moveTo([1.0, -0.5], { timeout: 3.0 })).toBe(true);
        const interrupt = new AbortController();
        let aborted = 0;
        clock.timer(0.3, () => {
          aborted = clock.now();
          interrupt.abort();
        });
        expect(await arm.stop({ signal: interrupt.signal })).toBe(false);
        expect(clock.now() - aborted).toBeLessThan(0.02);
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("an abort after the first torque-off frame finishes the release and reports it", async () => {
      const { arm, fakes } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      const interrupt = new AbortController();
      // abort when the first neutral (kp 0) command goes out
      const send = fakes[1]!.handle.bind(fakes[1]!);
      fakes[1]!.handle = (canId: number, data: Uint8Array) => {
        const replies = send(canId, data);
        const last = fakes[1]!.frames[fakes[1]!.frames.length - 1];
        if (classify(canId, data)[0] === "command" && last !== undefined && last[1] === 0 && !arm["running"]) interrupt.abort();
        return replies;
      };
      try {
        expect(await arm.stop({ signal: interrupt.signal })).toBe(true);
        expect(interrupt.signal.aborted).toBe(true);
        expect(arm.state).toBe(ArmState.CONNECTED);
        expect(fakes[1]!.enabled).toBe(false);
      } finally {
        await arm.disconnect();
      }
    });

    it("stop() does not report a release it could not confirm", async () => {
      const { arm, fakes } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      const handle = fakes[1]!.handle.bind(fakes[1]!);
      let dropping = false;
      fakes[1]!.handle = (canId: number, data: Uint8Array) => {
        if (dropping && classify(canId, data)[0] === "disable") return []; // every disable frame to it is lost
        return handle(canId, data);
      };
      const tick = arm["tick"].bind(arm);
      arm["tick"] = (dt: number, session: number | null = null, poll = true) => {
        if (!arm["running"]) dropping = true;
        return tick(dt, session, poll);
      };
      try {
        expect(await arm.stop()).toBe(false);
        expect(arm.state).toBe(ArmState.CONNECTED);
        expect(arm.unconfirmedDisable).toEqual([1]);
        expect(arm.status().unconfirmedDisable).toEqual([1]);
        expect(arm["running"]).toBe(false);
      } finally {
        dropping = false;
        await arm.disable();
        expect(arm.unconfirmedDisable).toEqual([]); // disable() tries again
        await arm.disconnect();
      }
    });

    it("a buffered reply from before the recheck does not count as the recheck's", async () => {
      // The last loop command's reply is held back (an adapter buffer) and comes out when the next
      // frame goes; the recheck's own reply is held back the same way. Meanwhile the joint moved.
      const { arm, fakes, bus } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      let buffering = false, held: Array<[number, Uint8Array]> = [];
      const handle = fakes[1]!.handle.bind(fakes[1]!);
      fakes[1]!.handle = (canId: number, data: Uint8Array) => {
        const replies = handle(canId, data);
        if (!buffering) return replies;
        const out = held;
        held = replies;
        return out;
      };
      const tick = arm["tick"].bind(arm);
      arm["tick"] = async (dt: number, session: number | null = null, poll = true) => {
        if (!arm["running"]) {
          fakes[1]!.track = false;
          fakes[1]!.position += 0.45; // moved after the loop's last frame
        } else if (arm["returning"]) {
          buffering = true; // from the return's ticks on, each reply comes one frame late
        }
        return tick(dt, session, poll);
      };
      try {
        expect(await arm.stop()).toBe(false);
        expect(fakes[1]!.enabled).toBe(true);
      } finally {
        buffering = false;
        arm["tick"] = tick;
        await arm.disable();
        await arm.disconnect();
      }
      void bus;
    });

    it("a fault latched after the recheck tick's own health check refuses the release", async () => {
      const { arm, fakes } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      let latched: string | null = null;
      Object.defineProperty(arm.motors[0]!, "faultLatched", { get: () => latched, configurable: true });
      const tick = arm["tick"].bind(arm);
      arm["tick"] = async (dt: number, session: number | null = null, poll = true) => {
        await tick(dt, session, poll);
        if (!arm["running"]) latched = "a reply latched a fault"; // after this tick's health check
      };
      try {
        expect(await arm.stop()).toBe(false);
        expect(fakes[1]!.enabled).toBe(true);
      } finally {
        latched = null;
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("the recheck needs a reading younger than feedbackTimeout when that is below REST_FRESH_S", async () => {
      const { arm, fakes, clock } = make(null, fastRest({ feedbackTimeout: 0.015 }));
      await arm.connect();
      await arm.enable();
      const tick = arm["tick"].bind(arm);
      arm["tick"] = async (dt: number, session: number | null = null, poll = true) => {
        await tick(dt, session, poll);
        if (!arm["running"]) await clock.sleep(0.03); // the recheck reading is 30 ms old when judged
      };
      try {
        expect(await arm.stop()).toBe(false);
        expect(fakes[1]!.enabled).toBe(true);
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("an e-stop during the recheck is followed by no holding frame", async () => {
      const { arm, fakes } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      let stopping: Promise<void> | null = null;
      const tick = arm["tick"].bind(arm);
      arm["tick"] = (dt: number, session: number | null = null, poll = true) => {
        if (!arm["running"] && stopping === null) stopping = arm.estop();
        return tick(dt, session, poll);
      };
      try {
        expect(await arm.stop()).toBe(false);
        await stopping;
        const log = fakes[1]!.log;
        const first = log.indexOf("disable");
        expect(first).toBeGreaterThanOrEqual(0);
        expect(log.slice(first).some((e) => Array.isArray(e) && e[0] === "cmd" && e[1] > 0)).toBe(false);
        expect(arm.state).toBe(ArmState.CONNECTED);
        expect(fakes[1]!.enabled).toBe(false);
      } finally {
        await arm.disconnect();
      }
    });

    // ── second review: the release recheck, admission, aborts, superseded frames, shutdown ownership ──
    /** Reply frames of `fake` withheld one frame (an adapter buffer) once `on()` holds. */
    function oneBehind(fake: M, on: () => boolean): void {
      let held: Array<[number, Uint8Array]> = [];
      const handle = fake.handle.bind(fake);
      fake.handle = (canId: number, data: Uint8Array) => {
        const replies = handle(canId, data);
        if (!on()) return replies;
        const out = held;
        held = replies;
        return out;
      };
    }

    it("recheck: a surplus reply earlier in the session does not hide a buffered reply later", async () => {
      const { arm, fakes } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      // one surplus reply (a duplicate) early in the session
      const handle = fakes[1]!.handle.bind(fakes[1]!);
      let duplicate = true;
      fakes[1]!.handle = (canId: number, data: Uint8Array) => {
        const replies = handle(canId, data);
        if (duplicate && replies.length) {
          duplicate = false;
          return [...replies, ...replies];
        }
        return replies;
      };
      await arm["tick"](0.005);
      let buffering = false;
      oneBehind(fakes[1]!, () => buffering);
      const tick = arm["tick"].bind(arm);
      arm["tick"] = async (dt: number, session: number | null = null, poll = true) => {
        if (!arm["running"]) {
          fakes[1]!.track = false;
          if (fakes[1]!.position < 0.6) fakes[1]!.position += 0.45; // the shoulder moved after the loop's last frame
        } else if (arm["returning"]) {
          buffering = true;
        }
        return tick(dt, session, poll);
      };
      try {
        expect(await arm.stop()).toBe(false);
        expect(fakes[1]!.enabled).toBe(true);
      } finally {
        buffering = false;
        arm["tick"] = tick;
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("recheck: movement between the two reads keeps the arm holding", async () => {
      const { arm, fakes } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      let reads = 0;
      const tick = arm["tick"].bind(arm);
      arm["tick"] = async (dt: number, session: number | null = null, poll = true) => {
        if (!arm["running"] && ++reads === 2) {
          fakes[1]!.track = false;
          fakes[1]!.position += 0.45;
        }
        return tick(dt, session, poll);
      };
      try {
        expect(await arm.stop()).toBe(false);
        expect(reads).toBe(2);
        expect(fakes[1]!.enabled).toBe(true);
      } finally {
        arm["tick"] = tick;
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("recheck: a stale reply delivered just after the read, with the read's own reply lost, keeps the arm holding", async () => {
      const { arm, fakes, bus, clock } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      let last: Array<[number, Uint8Array]> = [];
      let reads = 0;
      const handle = fakes[1]!.handle.bind(fakes[1]!);
      fakes[1]!.handle = (canId: number, data: Uint8Array) => {
        const replies = handle(canId, data);
        if (arm["running"]) {
          if (replies.length) last = replies; // a reply from before the loop stopped
          return replies;
        }
        if (classify(canId, data)[0] === "command" && ++reads === 1) {
          const stale = last;
          fakes[1]!.track = false;
          fakes[1]!.position += 0.45; // the joint moved
          clock.timer(0.003, () => stale.forEach(([id, d]) => bus.inject(id, d))); // the old reply comes out late
        }
        return classify(canId, data)[0] === "command" ? [] : replies; // the reads' own replies are lost
      };
      try {
        expect(await arm.stop()).toBe(false);
        expect(fakes[1]!.enabled).toBe(true);
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("stop() keeps resolving false while a release is unconfirmed; enable() and setMotorId() refuse", async () => {
      const { arm, fakes } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      let dropping = false;
      const handle = fakes[1]!.handle.bind(fakes[1]!);
      fakes[1]!.handle = (canId: number, data: Uint8Array) =>
        (dropping && classify(canId, data)[0] === "disable" ? [] : handle(canId, data));
      const tick = arm["tick"].bind(arm);
      arm["tick"] = (dt: number, session: number | null = null, poll = true) => {
        if (!arm["running"]) dropping = true;
        return tick(dt, session, poll);
      };
      try {
        expect(await arm.stop()).toBe(false);
        expect(arm.unconfirmedDisable).toEqual([1]);
        expect(await arm.stop()).toBe(false); // a second stop() does not report it released
        await expect(arm.enable()).rejects.toThrow(/not confirmed/);
        await expect(arm.setMotorId(1, 9, { torqueOffConfirmed: true })).rejects.toThrow(/not confirmed/);
        dropping = false;
        await arm.disable();
        expect(arm.unconfirmedDisable).toEqual([]);
        expect(await arm.stop()).toBe(true);
      } finally {
        dropping = false;
        await arm.disconnect();
      }
    });

    it("disable() then enable() issued during an e-stop never energize", async () => {
      const { arm, fakes, clock } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      const stopping = arm.estop();
      await clock.sleep(0.002); // the e-stop is confirming its release
      const disabling = arm.disable();
      const enabling = arm.enable().then(() => null, (e: unknown) => e);
      await Promise.all([stopping, disabling]);
      expect(await enabling).toBeInstanceOf(StateError);
      expect(fakes[1]!.enabled).toBe(false);
      expect(arm.state).toBe(ArmState.CONNECTED);
      await arm.disconnect();
    });

    it("an abort during a paced recheck with a missing reply ends stop() at once", async () => {
      const { arm, fakes, bus, clock } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      const interrupt = new AbortController();
      let aborted = 0;
      const handle = fakes[1]!.handle.bind(fakes[1]!);
      fakes[1]!.handle = (canId: number, data: Uint8Array) => {
        if (!arm["running"] && arm.state === ArmState.ENABLED && classify(canId, data)[0] === "command") {
          if (!aborted) clock.timer(0.002, () => {
            aborted = clock.now();
            interrupt.abort();
          });
          return []; // the recheck's reply is missing
        }
        return handle(canId, data);
      };
      bus.feedbackPacedControl = true;
      bus.feedbackWaitTimeout = 0.3; // a slow adapter's reply wait
      try {
        expect(await arm.stop({ signal: interrupt.signal })).toBe(false);
        expect(aborted).toBeGreaterThan(0);
        expect(clock.now() - aborted).toBeLessThan(0.02);
        expect(fakes[1]!.enabled).toBe(true);
      } finally {
        bus.feedbackPacedControl = false;
        bus.feedbackWaitTimeout = 0.015;
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("an abort when the first torque-off write failed before it was sent keeps the arm holding", async () => {
      const { arm, fakes } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      const interrupt = new AbortController();
      const motor = arm.motors[0]!;
      const send = motor.sendCommand.bind(motor);
      let failed = false;
      motor.sendCommand = (pos: number, vel: number, kp: number, kd: number, tau: number) => {
        if (!failed && !arm["running"] && kp === 0 && kd === 0) {
          failed = true;
          interrupt.abort();
          return Promise.reject(new Error("write refused")); // definitely not sent
        }
        return send(pos, vel, kp, kd, tau);
      };
      try {
        expect(await arm.stop({ signal: interrupt.signal })).toBe(false);
        expect(failed).toBe(true);
        expect(arm.state).toBe(ArmState.ENABLED);
        expect(fakes[1]!.enabled).toBe(true);
        expect(fakes[1]!.log.filter((e) => e === "disable").length).toBe(0);
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("after a failed stop() restored the gains, no frame of a tick in flight carries the float-mode command", async () => {
      const { arm, fakes, bus, clock } = make(null, fastRest({ restRateHz: 1 }));
      await arm.connect();
      await arm.enable();
      arm["synced"] = [true, true];
      expect(await arm.moveTo([1.0, -0.5], { timeout: 3.0 })).toBe(true);
      arm.floatMode(0.2, () => [1.08, 0.0]);
      await clock.sleep(0.05);
      const interrupt = new AbortController();
      const send = bus.send.bind(bus);
      bus.send = async (canId: number, data: Uint8Array, extended = true) => {
        // During the return, a loop tick's first frame is slow; the stop() fails meanwhile.
        if (arm["returning"] && arm["running"] && !interrupt.signal.aborted && classify(canId, data)[0] === "command" && canId === 1) {
          await send(canId, data, extended);
          interrupt.abort();
          await clock.sleep(0.002);
          return;
        }
        return send(canId, data, extended);
      };
      try {
        expect(await arm.stop({ signal: interrupt.signal })).toBe(false);
        const marks = [fakes[1]!.frames.length, fakes[2]!.frames.length];
        await clock.sleep(0.1);
        const profile = twoJointConfig().joints;
        for (const [k, id] of [[0, 1], [1, 2]] as const) {
          const after = fakes[id]!.frames.slice(marks[k]);
          expect(after.length).toBeGreaterThan(0);
          expect(after.every(([, kp, tau]) => Math.abs(kp - profile[k]!.kp) <= 0.2 && Math.abs(tau) < 0.05), `motor ${id}`).toBe(true);
        }
      } finally {
        bus.send = send;
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("a hold drops the regulated gripper frame a tick computed before it", async () => {
      const joints = twoJointConfig().joints.map((j, i) => (i === 1 ? j.replace({ holdTorqueNm: 0.5 }) : j));
      const { arm, fakes, bus, clock } = make(null, { joints });
      await arm.connect();
      await arm.enable({ startLoop: false });
      const send = bus.send.bind(bus);
      let slow = true;
      bus.send = async (canId: number, data: Uint8Array, extended = true) => {
        if (slow && classify(canId, data)[0] === "command" && canId === 1) {
          slow = false;
          await send(canId, data, extended);
          await clock.sleep(0.002);
          arm.holdCurrentPosition(); // installed while the gripper's frame is still unsent
          return;
        }
        return send(canId, data, extended);
      };
      const before = fakes[2]!.frames.length;
      await arm["tick"](0.005);
      expect(fakes[2]!.frames.length).toBe(before); // dropped, not sent stale
      await arm["tick"](0.005);
      expect(fakes[2]!.frames.length).toBe(before + 1); // the next tick computes it afresh
      bus.send = send;
      await arm.disable();
      await arm.disconnect();
    });

    it("user gains survive returnToRest(), a successful stop() and a new enable", async () => {
      const { arm } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      arm.setGains([30.0, 10.0], [1.5, 0.4]);
      expect(await arm.returnToRest(arm.restPose)).toBe(true);
      expect(arm.config.joints.map((j) => j.kp)).toEqual([30.0, 10.0]);
      expect(await arm.stop()).toBe(true);
      expect(arm.config.joints.map((j) => j.kp)).toEqual([30.0, 10.0]);
      await arm.enable();
      expect(arm.config.joints.map((j) => j.kp)).toEqual([30.0, 10.0]);
      await arm.disable();
      await arm.disconnect();
    });

    it("the Python-frame replay switch is not on the public object", () => {
      const { arm } = make();
      expect("pythonFrames" in arm).toBe(false);
      expect(Object.keys(arm).some((k) => /python|compat|replay/i.test(k))).toBe(false);
    });

    it("detach() racing estop() reports a motor whose release was not confirmed", async () => {
      const { arm, fakes } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      const handle = fakes[1]!.handle.bind(fakes[1]!);
      fakes[1]!.handle = (canId: number, data: Uint8Array) => (classify(canId, data)[0] === "disable" ? [] : handle(canId, data));
      const detaching = arm.detach().then(() => null, (e: unknown) => e);
      await arm.estop();
      await detaching;
      expect(fakes[1]!.enabled).toBe(true); // its disables were lost
      expect(arm.unconfirmedDisable).toContain(1);
    });

    it("holdOnFault off: a lost fault-path disable is reported", async () => {
      const { arm, fakes } = make(null, { holdOnFault: false });
      await arm.connect();
      await arm.enable({ startLoop: false });
      const handle = fakes[1]!.handle.bind(fakes[1]!);
      fakes[1]!.handle = (canId: number, data: Uint8Array) => (classify(canId, data)[0] === "disable" ? [] : handle(canId, data));
      fakes[1]!.faulted = true;
      arm["pollSent"] = new Map([[1, 0.0], [2, 1e9]]);
      await arm["tick"](0.005);
      await arm["tick"](0.005);
      expect(arm.state).toBe(ArmState.FAULT);
      expect(arm.unconfirmedDisable).toContain(1);
      await arm.disconnect();
    });

    it("a failed bus close during detach() leaves the arm serviced", async () => {
      const { arm, bus, clock } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      const close = bus.close.bind(bus);
      bus.close = () => Promise.reject(new Error("the adapter went away"));
      await expect(arm.detach()).rejects.toThrow(/went away/);
      expect(arm.state).toBe(ArmState.ENABLED);
      expect(arm.loopRunning).toBe(true);
      const ticks = arm["tickCount"];
      await clock.sleep(0.1);
      expect(arm["tickCount"] - ticks).toBeGreaterThan(10);
      bus.close = close;
      await arm.disable();
      await arm.disconnect();
    });

    it("a recheck write that never settles does not leave stop() pending", async () => {
      const { arm, bus } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      const send = bus.send.bind(bus);
      bus.send = (canId: number, data: Uint8Array, extended = true) =>
        (!arm["running"] && arm.state === ArmState.ENABLED && classify(canId, data)[0] === "command" && canId === 1
          ? new Promise<void>(() => {}) // never settles
          : send(canId, data, extended));
      try {
        expect(await arm.stop()).toBe(false);
        expect([ArmState.ENABLED, ArmState.FAULT]).toContain(arm.state);
        expect(arm.loopRunning).toBe(true);
      } finally {
        bus.send = send;
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("estop() does not wait behind a stop() that is stuck", async () => {
      const { arm, fakes, clock } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      arm["waitTicks"] = () => new Promise<boolean>(() => {}); // stop() never gets past its two-tick wait
      void arm.stop();
      await clock.sleep(0.05);
      await arm.estop();
      expect(arm.state).toBe(ArmState.CONNECTED);
      expect(fakes[1]!.enabled).toBe(false);
    });

    // ── third review: per-joint recheck snapshots, no quiet window ──
    it("recheck: a duplicate of a joint's earlier reading arriving during another joint's command does not count", async () => {
      // Both joints are checked; in read 2 a copy of the second joint's read-1 reply arrives while the
      // first joint is commanded, and the second joint's own reply is lost after it moved.
      const joints = twoJointConfig().joints.map((j, i) => (i === 1 ? j.replace({ returnToRest: true }) : j));
      const { arm, fakes, bus } = make(null, fastRest({ joints }));
      await arm.connect();
      await arm.enable();
      let reads = 0, first: Array<[number, Uint8Array]> = [];
      const h1 = fakes[1]!.handle.bind(fakes[1]!), h2 = fakes[2]!.handle.bind(fakes[2]!);
      fakes[1]!.handle = (canId: number, data: Uint8Array) => {
        const replies = h1(canId, data);
        if (!arm["running"] && arm.state === ArmState.ENABLED && classify(canId, data)[0] === "command" && ++reads === 2) {
          first.forEach(([id, d]) => bus.inject(id, d)); // the duplicate, during joint 1's command
        }
        return replies;
      };
      fakes[2]!.handle = (canId: number, data: Uint8Array) => {
        const replies = h2(canId, data);
        if (arm["running"] || arm.state !== ArmState.ENABLED || classify(canId, data)[0] !== "command") return replies;
        if (reads === 1) {
          first = replies;
          return replies;
        }
        fakes[2]!.track = false;
        fakes[2]!.position += 0.45; // moved, and its own reply is lost
        return [];
      };
      try {
        expect(await arm.stop()).toBe(false);
        expect(fakes[2]!.enabled).toBe(true);
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("recheck: a 30 ms motor watchdog on a stiff joint never trips and the arm releases", async () => {
      const joints = twoJointConfig().joints.map((j, i) => (i === 0 ? j.replace({ canTimeoutMs: 30 }) : j));
      const { arm, fakes, clock } = make([new FakeMotor(1, MODEL, { position: 0.3, paramsSupported: true }),
        new FakeMotor(2, MODEL, { position: -0.5 })], fastRest({ joints }));
      await arm.connect();
      await arm.enable();
      let last = clock.now(), tripped = false;
      const handle = fakes[1]!.handle.bind(fakes[1]!);
      fakes[1]!.handle = (canId: number, data: Uint8Array) => {
        if (classify(canId, data)[0] === "command" && fakes[1]!.enabled) {
          if (clock.now() - last > 0.030) tripped = true; // the motor's own watchdog would have released it
          last = clock.now();
        }
        return handle(canId, data);
      };
      try {
        expect(await arm.stop()).toBe(true);
        expect(tripped).toBe(false);
      } finally {
        await arm.disconnect();
      }
    });

    it("recheck: unsolicited feedback every 20 ms from a motor that is not checked does not block the release", async () => {
      const { arm, fakes, bus, clock } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      let reply: Array<[number, Uint8Array]> = [];
      const handle = fakes[2]!.handle.bind(fakes[2]!);
      fakes[2]!.handle = (canId: number, data: Uint8Array) => {
        const replies = handle(canId, data);
        if (classify(canId, data)[0] === "command" && replies.length) reply = replies;
        return replies;
      };
      let chatter = true;
      const tick = (): void => {
        if (!chatter) return;
        reply.forEach(([id, d]) => bus.inject(id, d)); // the gripper (not checked) reports on its own
        clock.timer(0.02, tick);
      };
      clock.timer(0.02, tick);
      try {
        expect(await arm.stop()).toBe(true);
      } finally {
        chatter = false;
        await arm.disconnect();
      }
    });

    it("an abort during the first neutral write that the transport refused before submitting it keeps the arm holding", async () => {
      const { arm, fakes } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      const interrupt = new AbortController();
      const motor = arm.motors[0]!;
      const send = motor.sendCommand.bind(motor);
      let refused = false;
      motor.sendCommand = (pos: number, vel: number, kp: number, kd: number, tau: number) => {
        if (!refused && !arm["running"] && kp === 0 && kd === 0) {
          refused = true;
          interrupt.abort();
          // gs_usb's in-flight cap: no transmit slot came free, nothing was handed to the device
          const e = new TimeoutError("frames were never acknowledged on the CAN bus");
          Object.defineProperty(e, "submitted", { value: false }); // as notSubmitted() marks it
          return Promise.reject(e);
        }
        return send(pos, vel, kp, kd, tau);
      };
      try {
        expect(await arm.stop({ signal: interrupt.signal })).toBe(false);
        expect(refused).toBe(true);
        expect(arm.state).toBe(ArmState.ENABLED);
        expect(fakes[1]!.enabled).toBe(true);
        expect(fakes[1]!.log.filter((e) => e === "disable").length).toBe(0);
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("a dropped gripper frame is recomputed after the hold, not re-sent from the regulator's cache", async () => {
      const joints = twoJointConfig().joints.map((j, i) => (i === 1 ? j.replace({ holdTorqueNm: 0.5 }) : j));
      const { arm, fakes, bus, clock } = make(null, { joints });
      await arm.connect();
      await arm.enable({ startLoop: false });
      arm["synced"] = [true, true];
      fakes[2]!.track = false; // the gripper stays where it is
      const held = arm.getJointPositions()[1]!;
      expect(arm.setJointTargets([0.3, -1.4])).toBe(true);
      const send = bus.send.bind(bus);
      let slow = true;
      bus.send = async (canId: number, data: Uint8Array, extended = true) => {
        if (slow && classify(canId, data)[0] === "command" && canId === 1) {
          slow = false;
          await send(canId, data, extended);
          await clock.sleep(0.002);
          arm.holdCurrentPosition(); // the gripper's frame of this tick is dropped
          return;
        }
        return send(canId, data, extended);
      };
      await arm["tick"](0.005);
      const before = fakes[2]!.frames.length;
      await arm["tick"](0.005); // same gripper feedback sample as the dropped frame was computed from
      expect(fakes[2]!.frames.length).toBe(before + 1);
      expect(Math.abs(fakes[2]!.frames.at(-1)![0] - held)).toBeLessThan(0.05);
      bus.send = send;
      await arm.disable();
      await arm.disconnect();
    });

    it("ending float mode and holding across the feed-forward drops the tick: no old extra torque goes out", async () => {
      const { arm, fakes } = make(null, fastRest());
      await arm.connect();
      await arm.enable({ startLoop: false });
      arm.floatMode(0.2, () => {
        queueMicrotask(() => {
          arm.setGains(); // float mode ends (profile stiffness) ...
          arm.holdCurrentPosition(); // ... and a hold is installed while this tick awaits its feed-forward
        });
        return [1.077, 0.0];
      });
      const mark = fakes[1]!.frames.length;
      await arm["tick"](0.005);
      const sent = fakes[1]!.frames.slice(mark);
      expect(sent.some(([, , tau]) => Math.abs(tau - 1.077) < 0.05)).toBe(false);
      await arm["tick"](0.005);
      expect(fakes[1]!.frames.at(-1)![1]).toBeCloseTo(twoJointConfig().joints[0]!.kp, 0);
      await arm.disable();
      await arm.disconnect();
    });

    it("a failed bus close during detach() holds where the arm is, not the old target", async () => {
      const { arm, bus, clock } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      arm["synced"] = [true, true];
      expect(arm.setJointTargets([1.8, -0.5])).toBe(true);
      await clock.sleep(0.01); // moving toward 1.8
      const close = bus.close.bind(bus);
      bus.close = () => Promise.reject(new Error("the adapter went away"));
      await expect(arm.detach()).rejects.toThrow(/went away/);
      const where = arm.getJointPositions()[0]!;
      await clock.sleep(0.3);
      expect(Math.abs(arm.getJointPositions()[0]! - where)).toBeLessThan(0.02);
      expect(arm.loopRunning).toBe(true);
      bus.close = close;
      await arm.disable();
      await arm.disconnect();
    });

    it("a close the transport records as failed is reported by detach(): energized and not serviced", async () => {
      const { arm, bus } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      const close = bus.close.bind(bus);
      bus.close = async () => {
        await close();
        bus.closeError = "MODE reset failed"; // gs_usb and SLCAN record it instead of rejecting
      };
      await expect(arm.detach()).rejects.toThrow(/did not close cleanly.*energized/);
      expect(arm.state).toBe(ArmState.ENABLED);
      expect(arm.loopRunning).toBe(false);
    });

    it("enable() right after estop() is refused until the e-stop has finished", async () => {
      const { arm, fakes } = make(null, fastRest());
      await arm.connect();
      const stopping = arm.estop();
      const enabling = arm.enable().then(() => null, (e: unknown) => e);
      await stopping;
      const refused = await enabling;
      expect(refused).toBeInstanceOf(StateError);
      expect(String(refused)).toMatch(/e-stop is in progress/);
      expect(fakes[1]!.log).not.toContain("enable");
      expect(fakes[1]!.enabled).toBe(false);
      await arm.disconnect();
    });

    it("an interrupted return (a second Ctrl-C) leaves the arm holding at full gains", async () => {
      const { arm, fakes, clock } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      try {
        arm["synced"] = [true, true];
        expect(await arm.moveTo([1.0, -0.5], { timeout: 3.0 })).toBe(true);
        const interrupt = new AbortController();
        clock.timer(0.02, () => interrupt.abort());
        expect(await arm.stop({ signal: interrupt.signal })).toBe(false);
        expect(arm.state).toBe(ArmState.ENABLED);
        expect(arm["returning"]).toBe(false);
        expect(fakes[1]!.enabled).toBe(true);
        expect(await holdingAtFullGains(arm, fakes[1]!, clock)).toBe(true);
        expect(arm.setJointTargets([0.9, -0.5])).toBe(true); // it takes targets again
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("an interrupt just before the release leaves the arm holding at full gains", async () => {
      const { arm, fakes, clock } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      const interrupt = new AbortController();
      const tick = arm["tick"].bind(arm);
      arm["tick"] = (dt: number, session: number | null = null, poll = true) => {
        if (!arm["running"]) interrupt.abort(); // stop()'s own reading after the loop stopped
        return tick(dt, session, poll);
      };
      try {
        expect(await arm.stop({ signal: interrupt.signal })).toBe(false);
        expect(arm.state).toBe(ArmState.ENABLED);
        expect(arm["returning"]).toBe(false);
        expect(fakes[1]!.enabled).toBe(true);
        expect(await holdingAtFullGains(arm, fakes[1]!, clock)).toBe(true);
      } finally {
        arm["tick"] = tick;
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("stop rechecks the pose after the loop stops and holds if it moved", async () => {
      const { arm, fakes, clock } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      const tick = arm["tick"].bind(arm);
      let moved = false;
      arm["tick"] = (dt: number, session: number | null = null, poll = true) => {
        if (!arm["running"] && !moved && arm.state === ArmState.ENABLED) {
          moved = true;
          fakes[1]!.track = false;
          fakes[1]!.position += 0.1; // nudged while the loop was stopping
        }
        return tick(dt, session, poll);
      };
      try {
        expect(await arm.stop()).toBe(false);
        expect(moved).toBe(true);
        expect(arm.state).toBe(ArmState.ENABLED);
        expect(fakes[1]!.enabled).toBe(true);
        expect(arm["returning"]).toBe(false);
        expect(await holdingAtFullGains(arm, fakes[1]!, clock)).toBe(true);
      } finally {
        arm["tick"] = tick;
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("stop does not release when no joint returns to rest", async () => {
      const joints = twoJointConfig().joints.map((j) => j.replace({ returnToRest: false }));
      const { arm, fakes } = make(null, fastRest({ joints }));
      await arm.connect();
      await arm.enable();
      try {
        arm["synced"] = [true, true];
        expect(await arm.moveTo([1.0, -0.5], { timeout: 3.0 })).toBe(true); // raised, and nothing will bring it back
        expect(await arm.stop()).toBe(false);
        expect(arm.state).toBe(ArmState.ENABLED);
        expect(fakes[1]!.enabled).toBe(true);
        expect(arm["returning"]).toBe(false);
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });
  });

  describe("feedback silence counts from the request", () => {
    it("a stalled loop is not taken for a silent motor", async () => {
      const { arm, clock } = await enabledArm(null, { feedbackTimeout: 0.05 });
      await arm["tick"](0.005);
      await clock.sleep(0.15); // the loop could not run, so nothing asked the motors
      await arm["tick"](0.005);
      expect(arm.state).toBe(ArmState.ENABLED);
    });

    it("a silent motor still faults after a stall", async () => {
      const { arm, clock, fakes } = await enabledArm(null, { feedbackTimeout: 0.05 });
      await arm["tick"](0.005);
      await clock.sleep(0.15);
      fakes[1]!.online = false;
      await arm["tick"](0.005); // asked now; no answer
      expect(arm.state).toBe(ArmState.ENABLED);
      await clock.sleep(0.06);
      await arm["tick"](0.005);
      expect(arm.state).toBe(ArmState.FAULT);
      expect(arm.faultReason).toContain("feedback timeout");
      await arm.disable();
    });
  });

  describe("regressions from the independent review", () => {
    it("a second fault while holding is still handled", async () => {
      const { arm, fakes } = await enabledArm();
      fakes[1]!.faulted = true;
      arm["pollSent"] = new Map([[1, 0.0], [2, 1e9]]);
      await arm["tick"](0.005);
      await arm["tick"](0.005);
      expect(arm.state).toBe(ArmState.FAULT);
      expect(fakes[2]!.enabled).toBe(true);
      fakes[2]!.temperature = 70.0;
      await arm["tick"](0.005);
      expect(fakes[2]!.enabled).toBe(false);
      expect(arm["released"].has(1)).toBe(true); // gripper is joint index 1
      const sent = commands(fakes[2]!).length;
      const held = fakes[1]!.position;
      fakes[1]!.log.length = 0;
      await arm["tick"](0.005);
      expect(commands(fakes[2]!).length).toBe(sent); // released gripper gets no commands
      expect(fakes[2]!.log.includes("disable") || arm["releaseSent"].size > 0).toBe(true); // torque-off keeps being re-sent
      expect(commands(fakes[1]!).length).toBeGreaterThan(0);
      approx(fakes[1]!.position, held, 1e-3); // j1 holds
    });

    it("startup sync waits for the first real target", async () => {
      const { arm } = await enabledArm();
      await arm["tick"](0.005);
      await arm["tick"](0.005);
      expect(arm["synced"]).toEqual([false, false]);
      arm.setJointTargets([1.5, -0.5]);
      await arm["tick"](0.005);
      approx(arm.getCommandedPositions()[0]!, 0.3 + radians(1.0), 1e-3);
    });

    it("targets are refused while returning to rest", async () => {
      const { arm } = await enabledArm();
      arm["returning"] = true;
      expect(arm.setJointTargets([0.3, -0.5])).toBe(false);
    });

    it("stop with a pending target still ends at rest", async () => {
      const { arm, fakes } = make(null, fastRest());
      await arm.connect();
      await arm.enable();
      try {
        arm["synced"] = [true, true];
        arm.setJointTargets([1.5, -0.5]);
        expect(await arm.stop()).toBe(true);
        approx(fakes[1]!.position, 0.3, radians(2));
        expect(fakes[1]!.enabled).toBe(false);
      } finally {
        await arm.disconnect();
      }
    });

    it("an e-stop during enable cancels it", async () => {
      let estopped: Promise<void> | null = null;

      class EstopOnEnable extends Base {
        arm: { estop(): Promise<void> } | null = null;

        override handle(canId: number, data: Uint8Array): Array<[number, Uint8Array]> {
          if (classify(canId, data)[0] === "enable") estopped = this.arm!.estop();
          return super.handle(canId, data);
        }
      }

      const motor2 = new EstopOnEnable(2, MODEL, { position: -0.5 });
      const { arm, fakes, clock } = make([new FakeMotor(1, MODEL, { position: 0.3 }), motor2 as unknown as M]);
      motor2.arm = arm;
      await arm.connect();
      const p = arm.enable({ startLoop: false });
      await expect(p).rejects.toThrow(StateError);
      await expect(p).rejects.toThrow(/e-stop/);
      await clock.sleep(0.05);
      await estopped;
      expect(arm.state).toBe(ArmState.CONNECTED);
      expect(fakes[1]!.enabled).toBe(false);
      expect(fakes[2]!.enabled).toBe(false);
    });

    it("a send error keeps the loop holding the other joints", async () => {
      const { arm, bus, fakes, clock } = make();
      await arm.connect();
      await arm.enable();
      try {
        const original = bus.send.bind(bus);
        const failed: boolean[] = [];
        bus.send = (canId: number, data: Uint8Array, extended = true): Promise<void> => {
          const [kind, id] = classify(canId, data);
          if (kind === "command" && id === 1 && !failed.length) {
            failed.push(true);
            throw new Error("USB hiccup");
          }
          return original(canId, data, extended);
        };
        expect(await waitFor(() => arm.state === ArmState.FAULT, clock)).toBe(true);
        const before = commands(fakes[2]!).length;
        await clock.sleep(0.05);
        expect(arm["running"]).toBe(true);
        expect(commands(fakes[2]!).length).toBeGreaterThan(before);
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("enable is refused when the neutral preload is not acknowledged", async () => {
      class IgnoresCommandsWhileDisabled extends Base {
        override handle(canId: number, data: Uint8Array): Array<[number, Uint8Array]> {
          if (classify(canId, data)[0] === "command" && !this.enabled) return [];
          return super.handle(canId, data);
        }
      }

      const { arm, fakes } = make([new IgnoresCommandsWhileDisabled(1, MODEL, { position: 0.3 }) as unknown as M, new FakeMotor(2, MODEL, { position: -0.5 })]);
      await arm.connect();
      const p = arm.enable({ startLoop: false });
      await expect(p).rejects.toThrow(ConnectError);
      await expect(p).rejects.toThrow(/neutral preload/);
      expect(fakes[1]!.log).not.toContain("enable");
    });

    it("refresh never erases a new fault", async () => {
      const { arm, fakes } = make();
      await arm.connect();
      fakes[1]!.faulted = true;
      fakes[1]!.log.length = 0;
      expect(await arm.refresh()).toEqual([false, true]);
      expect(fakes[1]!.faulted).toBe(true);
      expect(fakes[1]!.log).not.toContain("clear");
    });

    it.skipIf(!WARNING_SUPPORTED)("the gripper warning flag is a fault", async () => {
      const { arm, fakes } = await enabledArm(null, {
        joints: [twoJointConfig().joints[0]!, twoJointConfig().joints[1]!.replace({ warningIsFault: true })],
      });
      fakes[2]!.warning = true;
      await arm["tick"](0.005);
      expect(arm.state).toBe(ArmState.FAULT);
      expect(arm.faultReason).toContain("warning");
      expect(fakes[2]!.enabled).toBe(false);
    });

    it("the gripper closing effort is capped", async () => {
      const holdJoint = twoJointConfig().joints[1]!.replace({ holdTorqueNm: 0.5 });
      const { arm, fakes } = await enabledArm(null, { joints: [twoJointConfig().joints[0]!, holdJoint] });
      fakes[2]!.track = false; // jaws blocked by an object
      arm["synced"] = [true, true];
      arm.setJointTargets([0.3, -0.04]); // close fully: 0.46 rad of error
      await arm["tick"](1.0);
      expect(fakes[2]!.lastKp * 0.46).toBeLessThanOrEqual(1.0 + 1e-6); // Kp x error <= 1 Nm closing cap
    });

    it("a released gripper's disable is retried after a lost frame", async () => {
      const { arm, bus, fakes } = await enabledArm();
      const original = bus.send.bind(bus);
      const dropped: boolean[] = [];
      bus.send = (canId: number, data: Uint8Array, extended = true): Promise<void> => {
        const [kind, id] = classify(canId, data);
        if (kind === "disable" && id === 2 && !dropped.length) {
          dropped.push(true);
          return Promise.resolve();
        }
        return original(canId, data, extended);
      };
      fakes[2]!.temperature = 60.0;
      await arm["tick"](0.005);
      expect(dropped.length).toBeGreaterThan(0);
      expect(fakes[2]!.enabled).toBe(true); // the first torque-off was lost
      arm["releaseSent"].set(1, 0.0);
      await arm["tick"](0.005);
      expect(fakes[2]!.enabled).toBe(false); // re-sent on a later tick
    });

    it("disable sends torque-off even if the neutral command fails", async () => {
      const { arm, bus, fakes } = await enabledArm();
      const original = bus.send.bind(bus);
      bus.send = (canId: number, data: Uint8Array, extended = true): Promise<void> => {
        if (classify(canId, data)[0] === "command") throw new Error("command path broken");
        return original(canId, data, extended);
      };
      await arm.disable();
      expect(fakes[1]!.enabled).toBe(false);
      expect(fakes[2]!.enabled).toBe(false);
    });

    it("an offline motor does not delay torque-off to the others", async () => {
      const { arm, bus, fakes, clock } = await enabledArm();
      fakes[1]!.online = false;
      const order: Array<[number, number]> = [];
      const original = bus.send.bind(bus);
      bus.send = (canId: number, data: Uint8Array, extended = true): Promise<void> => {
        if (classify(canId, data)[0] === "disable") order.push([canId, clock.now()]);
        return original(canId, data, extended);
      };
      const start = clock.now();
      await arm.disable();
      const firstGripperOff = order.find(([cid]) => cid === 2)![1];
      expect(firstGripperOff - start).toBeLessThan(0.02);
      expect(arm.unconfirmedDisable).toEqual([1]);
    });

    it("a stale loop session cannot send", async () => {
      const { arm, fakes } = await enabledArm();
      const stale = arm["session"];
      arm["session"] += 1;
      arm["running"] = true;
      fakes[1]!.log.length = 0;
      await arm["tick"](0.005, stale);
      expect(commands(fakes[1]!)).toEqual([]);
      arm["running"] = false;
    });

    it("enable is refused while the previous loop lives", async () => {
      const { arm } = make();
      await arm.connect();
      let open!: () => void;
      const gate = new Promise<void>((resolve) => (open = resolve));
      const handle = { promise: gate, alive: true };
      void gate.then(() => {
        handle.alive = false;
      });
      arm["loop"] = handle;
      try {
        const p = arm.enable({ startLoop: false });
        await expect(p).rejects.toThrow(StateError);
        await expect(p).rejects.toThrow(/previous control loop/);
      } finally {
        open();
        await handle.promise;
      }
    });
  });

  describe("float mode", () => {
    it("a fault while floating restores the profile gains at the measured pose", async () => {
      const { arm, fakes } = await enabledArm();
      arm.floatMode(null, () => [0.5, 0.5]);
      expect(arm["profileGains"]).toEqual([[60, 2.0], [20, 0.5]]); // floatMode leaves the profile untouched
      fakes[1]!.position = 0.4; // moved by hand while floating
      await arm["tick"](0.005);
      await arm["enterFault"]("j1: overcurrent");
      expect(arm.config.joints.map((j) => [j.kp, j.kd])).toEqual([[60, 2.0], [20, 0.5]]);
      expect(arm["extraTorque"]).toBeNull();
      expect(arm.state).toBe(ArmState.FAULT);
      approx(arm.getCommandedPositions()[0]!, 0.4, 1e-3);
      await arm["tick"](0.005);
      const [position, kp, tau] = fakes[1]!.frames.at(-1)!;
      approx(kp, 60, 0.1);
      approx(position, 0.4, 1e-3);
      approx(tau, 0, 0.05);
      await arm.disable();
    });

    it("losing gravity while floating is a fault that holds", async () => {
      class Broken {
        n = 1;

        torques(..._args: unknown[]): number[] {
          throw new Error("model blew up");
        }
      }

      const { arm, fakes } = await enabledArm();
      arm.floatMode();
      arm["gravity"] = new Broken() as never;
      await arm["tick"](0.005);
      expect(arm.state).toBe(ArmState.FAULT);
      expect(arm.faultReason).toContain("gravity");
      expect(arm.config.joints.map((j) => j.kp)).toEqual([60, 20]);
      approx(fakes[1]!.frames.at(-1)![1], 60, 0.1); // this tick's frame already holds
      await arm.disable();
    });
  });
}
