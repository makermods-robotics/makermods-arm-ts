// Port of the Python SDK's tests/unit/test_arm.py.
import { describe, expect, it } from "vitest";
import { type ArmConfigInit, ArmState, ConnectError, StateError, ValueError } from "@makermods/arm-core";
import { VirtualClock } from "@makermods/arm-core/testing";
import { type ArmFakes, commands, type FakeMotorLike, type FakeMotorOptions, waitFor } from "./fakes.js";

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
