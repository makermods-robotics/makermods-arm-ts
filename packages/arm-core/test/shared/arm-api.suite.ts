// Port of the Python SDK's tests/unit/test_arm_api.py.
import { describe, expect, it } from "vitest";
import { Arm, ArmError, type ArmConfigInit, ArmState, ConnectError, RealClock, StateError, ValueError } from "@makermods/arm-core";
import { VirtualClock } from "@makermods/arm-core/testing";
import { type ArmFakes, type FakeMotorLike, type FakeMotorOptions, RecordingLogger } from "./fakes.js";

/** Registers the shared tests on `fk`'s arm. */
export function armApiSuite<M extends FakeMotorLike>(fk: ArmFakes<M>): void {
  const MODEL = fk.product.defaultModel;
  const { classify, FAULT_TEXT, FakeBus, FakeMotor, FULL_TURN_SUPPORTED, makeArm, twoJointConfig } = fk;
  const ID_RANGE = fk.product.idRange;
  const KD_RANGE = fk.product.kdRange;
  const Base = FakeMotor as unknown as new (motorId: number, model?: string, options?: FakeMotorOptions) => FakeMotorLike;
  /** pytest.approx(expected, abs=abs) (default: rel 1e-6, abs 1e-12). */
  function approx(actual: number, expected: number, abs?: number): void {
    const tol = abs ?? Math.max(1e-6 * Math.abs(expected), 1e-12);
    expect(Math.abs(actual - expected), `${actual} ≈ ${expected} ± ${tol}`).toBeLessThanOrEqual(tol);
  }

  /** json.dumps(value, allow_nan=False): throws on a non-finite number. */
  function strictJson(value: unknown): string {
    return JSON.stringify(value, (_key, v: unknown) => {
      if (typeof v === "number" && !Number.isFinite(v)) throw new Error(`out of range float value: ${v}`);
      return v;
    });
  }

  // Python's monotonic clock is far from zero; the tests' 0.0 "sent long ago" sentinels rely on it.
  function make(motors?: M[] | null, overrides: Partial<ArmConfigInit> = {}) {
    return makeArm(motors, overrides, new VirtualClock({ start: 1000 }));
  }

  async function enabledArm(motors?: M[] | null, startLoop = false, overrides: Partial<ArmConfigInit> = {}) {
    const made = make(motors, overrides);
    await made.arm.connect();
    await made.arm.enable({ startLoop });
    return made;
  }

  describe("introspection", () => {
    it("joint names", () => {
      const { arm } = make();
      expect(arm.jointNames).toEqual(["j1", "gripper"]);
    });

    it("detach leaves torque on and closes the bus", async () => {
      const { arm, bus, fakes } = await enabledArm(null, true);
      await arm.detach();
      expect(arm.state).toBe(ArmState.DISCONNECTED);
      expect(arm.backend).toBe(bus);
      expect(bus.isOpen).toBe(false);
      expect(Object.values(fakes).every((m) => m.enabled)).toBe(true);
    });

    it("detach names the joints whose watchdog releases them", async () => {
      let { arm } = await enabledArm([new FakeMotor(1, MODEL, { position: 0.3, paramsSupported: true }),
        new FakeMotor(2, MODEL, { position: -0.5, paramsSupported: true })], true);
      expect(arm.watchdogJoints()).toEqual({ gripper: 0.5 });
      expect(arm.detachUnsafe()).toEqual([]);
      expect(await arm.detach()).toEqual({ gripper: 0.5 });
      const joints = twoJointConfig().joints.map((j) => j.replace({ canTimeoutMs: 200 }));
      ({ arm } = await enabledArm([new FakeMotor(1, MODEL, { position: 0.3, paramsSupported: true }),
        new FakeMotor(2, MODEL, { position: -0.5, paramsSupported: true })], false, { joints }));
      expect(arm.detachUnsafe()).toEqual(["j1"]); // a stiff joint would drop
      await arm.disable();
    });

    it("moveTo stops where it is when cancelled", async () => {
      const { arm, clock } = await enabledArm(null, true, { maxVelocity: 0.5 });
      try {
        arm["synced"] = [true, true];
        const cancel = new AbortController();
        clock.timer(0.2, () => cancel.abort());
        const started = clock.now();
        expect(await arm.moveTo([1.5, -0.5], { signal: cancel.signal })).toBe(false);
        expect(clock.now() - started).toBeLessThan(1.0);
        const here = arm.getJointPositions()[0]!;
        expect(here).toBeGreaterThan(0.05);
        expect(here).toBeLessThan(1.0);
        await clock.sleep(0.1);
        approx(arm.getJointPositions()[0]!, here, 0.01); // held, not still moving
        expect(await arm.moveTo([1.5, -0.5], { signal: cancel.signal })).toBe(false); // already cancelled: nothing sent
        approx(arm.getCommandedPositions()[0]!, here, 0.01);
      } finally {
        await arm.disable();
      }
    });

    it("status is JSON before and after connect", async () => {
      const { arm } = make();
      const before = arm.status();
      expect(before.state).toBe("disconnected");
      expect(before.joints.j1!.position).toBeNull();
      expect(before.joints.j1!.feedbackAge).toBeNull();
      strictJson(before);
      await arm.connect();
      const after = arm.status();
      strictJson(after);
      const j1 = after.joints.j1!;
      expect(after.state).toBe("connected");
      expect(after.faultReason).toBeNull();
      expect(j1.motorId).toBe(1);
      approx(j1.position!, 0.3, 1e-3);
      approx(j1.temperature!, 30.0);
      expect(Object.keys(j1.temperatures).length).toBeGreaterThan(0);
      expect(j1.fault).toBeNull();
      expect(j1.feedbackAge!).toBeGreaterThanOrEqual(0);
    });

    it("status reports a motor fault", async () => {
      const { arm } = make([new FakeMotor(1, MODEL, { position: 0.3 }), new FakeMotor(2, MODEL, { position: -0.5, faulted: true })]);
      await expect(arm.connect()).rejects.toThrow(ConnectError);
      expect(arm.status().joints.gripper!.fault).toContain(FAULT_TEXT);
    });
  });

  describe("enable(signal)", () => {
    it("cancelled before it starts sends nothing", async () => {
      const { arm, fakes } = make();
      await arm.connect();
      const cancel = new AbortController();
      cancel.abort();
      const p = arm.enable({ startLoop: false, signal: cancel.signal });
      await expect(p).rejects.toThrow(StateError);
      await expect(p).rejects.toThrow(/cancelled/);
      expect(arm.state).toBe(ArmState.CONNECTED);
      expect(Object.values(fakes).every((f) => !f.log.includes("enable"))).toBe(true);
    });

    it("cancelled midway releases torque", async () => {
      const cancel = new AbortController();

      class CancelOnEnable extends Base {
        override handle(canId: number, data: Uint8Array): Array<[number, Uint8Array]> {
          const replies = super.handle(canId, data);
          if (classify(canId, data)[0] === "enable") cancel.abort();
          return replies;
        }
      }

      const { arm, fakes } = make([new CancelOnEnable(1, MODEL, { position: 0.3 }) as unknown as M, new FakeMotor(2, MODEL, { position: -0.5 })]);
      await arm.connect();
      const p = arm.enable({ startLoop: false, signal: cancel.signal });
      await expect(p).rejects.toThrow(StateError);
      await expect(p).rejects.toThrow(/cancelled/);
      expect(arm.state).toBe(ArmState.CONNECTED);
      expect(fakes[1]!.enabled).toBe(false);
      expect(fakes[2]!.log).not.toContain("enable");
    });
  });

  describe("gains", () => {
    it("setGains reaches the next frames and restores", async () => {
      const { arm, fakes } = await enabledArm();
      arm.setGains([30.0, 10.0], [1.0, 0.2]);
      await arm["tick"](0.005);
      approx(fakes[1]!.lastKp, 30.0, 0.2);
      expect(arm.config.joints.map((j) => j.kp)).toEqual([30.0, 10.0]);
      arm.setGains(null, [0.5, 0.5]);
      expect(arm.config.joints.map((j) => j.kp)).toEqual([30.0, 10.0]);
      arm.setGains();
      expect(arm.config.joints.map((j) => [j.kp, j.kd])).toEqual([[60, 2.0], [20, 0.5]]);
    });

    it.each([
      [[1.0], null],
      [[NaN, 1.0], null],
      [[-1.0, 1.0], null],
      [null, [KD_RANGE[1] + 1, 0.0]],
    ] as Array<[number[] | null, number[] | null]>)("setGains rejects bad values (kp=%j, kd=%j)", (kp, kd) => {
      const { arm } = make();
      expect(() => arm.setGains(kp, kd)).toThrow(ValueError);
      expect(arm.config.joints.map((j) => j.kp)).toEqual([60, 20]);
    });

    it("floatMode follows the hand and restoring gains holds there", async () => {
      const hold = twoJointConfig().joints[1]!.replace({ holdTorqueNm: 0.5 });
      const { arm, fakes } = await enabledArm(null, false, { joints: [twoJointConfig().joints[0]!, hold] });
      arm.floatMode();
      await arm["tick"](0.005);
      expect(Object.values(fakes).map((f) => f.lastKp)).toEqual([0.0, 0.0]);
      expect(arm.config.joints.every((j) => j.kd === Arm.FLOAT_KD)).toBe(true);
      fakes[1]!.position = 0.9; // moved by hand
      await arm["tick"](0.005);
      await arm["tick"](0.005);
      expect(arm.state).toBe(ArmState.ENABLED);
      approx(arm.getCommandedPositions()[0]!, 0.9, 1e-3);
      arm.setGains();
      await arm["tick"](0.005);
      expect(fakes[1]!.frames.at(-1)![1]).toBeGreaterThan(0);
      approx(fakes[1]!.frames.at(-1)![0], 0.9, 1e-3);
    });
  });

  describe("moveTo", () => {
    it("waits for arrival (real clock: the control loop's own pacing)", async () => {
      const motors = [new FakeMotor(1, MODEL, { position: 0.3 }), new FakeMotor(2, MODEL, { position: -0.5 })];
      const bus = new FakeBus(motors);
      const arm = new Arm(twoJointConfig(), bus, { clock: new RealClock(), frameSpacing: 0, logger: new RecordingLogger() });
      arm.PARAM_TIMEOUT = 0.005;
      await arm.connect();
      await arm.enable();
      try {
        arm["synced"] = [true, true];
        expect(await arm.moveTo([1.0, -0.5])).toBe(true);
        approx(arm.getJointPositions()[0]!, 1.0, 0.04);
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("speed caps the command rate", async () => {
      const { arm, clock } = await enabledArm(null, true);
      try {
        arm["synced"] = [true, true];
        const start = clock.now();
        expect(await arm.moveTo([0.8, -0.5], { speed: 2.0 })).toBe(true);
        expect(clock.now() - start).toBeGreaterThanOrEqual(0.2); // 0.5 rad at 2 rad/s
        expect(arm["speed"]).toBe(2.0);
        expect(arm.setJointTargets([0.8, -0.5])).toBe(true);
        expect(arm["speed"]).toBeNull();
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("times out when blocked and refuses when not enabled", async () => {
      const { arm, fakes } = make();
      expect(await arm.moveTo([0.5, -0.5], { wait: false })).toBe(false);
      await arm.connect();
      await arm.enable();
      try {
        arm["synced"] = [true, true];
        fakes[1]!.track = false;
        expect(await arm.moveTo([1.0, -0.5], { timeout: 0.2 })).toBe(false);
        expect(arm.state).toBe(ArmState.ENABLED);
        expect(await arm.moveTo([0.3, -0.5], { wait: false })).toBe(true);
        await expect(arm.moveTo([0.3, -0.5], { speed: 0.0 })).rejects.toThrow(ValueError);
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });

    it("returns false when targets are taken over", async () => {
      const { arm, fakes, clock } = await enabledArm(null, true);
      try {
        fakes[1]!.track = false;
        clock.timer(0.05, () => arm.setJointTargets([0.3, -0.5]));
        expect(await arm.moveTo([1.5, -0.5], { timeout: 1.0 })).toBe(false);
      } finally {
        await arm.disable();
        await arm.disconnect();
      }
    });
  });

  describe("operator actions", () => {
    it("setZero only with torque off", async () => {
      const turned = FULL_TURN_SUPPORTED;
      const { arm, fakes: fake } = make([new FakeMotor(1, MODEL, { position: 0.3 + (turned ? 2 * Math.PI : 0.0) }),
        new FakeMotor(2, MODEL, { position: -0.5 })]);
      await expect(arm.setZero()).rejects.toThrow(StateError);
      await arm.connect();
      expect(arm["wrap"][0] !== 0.0).toBe(turned);
      await arm.setZero([0]);
      expect(fake[1]!.log).toContain("zero");
      expect(fake[2]!.log).not.toContain("zero");
      expect(arm["wrap"][0]).toBe(0.0);
      await arm.refresh();
      approx(arm.getJointPositions()[0]!, 0.0, 1e-3);
      await expect(arm.setZero([2])).rejects.toThrow(RangeError);
      await arm.enable({ startLoop: false });
      await expect(arm.setZero()).rejects.toThrow(StateError);
    });

    it("setMotorId validation", async () => {
      const { arm } = make();
      const hi = ID_RANGE[1];
      await expect(arm.setMotorId(1, 1)).rejects.toThrow(ValueError);
      await expect(arm.setMotorId(1, hi + 1)).rejects.toThrow(ValueError);
      await arm.connect();
      await arm.enable({ startLoop: false });
      await expect(arm.setMotorId(1, 9)).rejects.toThrow(StateError);
    });

    it("setMotorId refuses an id already on the bus and cleans up", async () => {
      const { arm, bus } = make();
      let p = arm.setMotorId(1, 2);
      await expect(p).rejects.toThrow(ArmError);
      await expect(p).rejects.toThrow(/already answers/);
      expect(bus.isOpen).toBe(false);
      expect(arm.state).toBe(ArmState.DISCONNECTED);
      await arm.connect();
      p = arm.setMotorId(1, 2);
      await expect(p).rejects.toThrow(ArmError);
      await expect(p).rejects.toThrow(/already answers/);
      expect(bus.isOpen).toBe(true);
      expect(await arm.refresh()).toEqual([true, true]); // the arm's receive path is back
    });

    it("gripper opening and target from the gripper joint", async () => {
      const { arm, fakes: motors } = make(); // gripper limits [-1.6, -0.04], closing toward +: closed -0.04, open -1.6
      await arm.connect();
      try {
        approx(arm.gripperTarget(0.0), -0.04);
        approx(arm.gripperTarget(1.0), -1.6);
        approx(arm.gripperTarget(2.0), -1.6); // clipped
        motors[2]!.position = arm.gripperTarget(0.25);
        await arm.refresh();
        approx(arm.gripperOpening(), 0.25, 1e-3);
        expect(() => arm.gripperTarget(NaN)).toThrow(ValueError);
      } finally {
        await arm.disconnect();
      }
      const joints = twoJointConfig().joints;
      const opened = make(null, { joints: [joints[0]!, joints[1]!.replace({ openPosition: -1.0 })] }).arm;
      approx(opened.gripperTarget(1.0), -1.0);
      const hand = make(null, { joints: [joints[0]!, joints[1]!.replace({ name: "hand" })] }).arm;
      expect(() => hand.gripperOpening()).toThrow(ArmError);
      expect(() => hand.gripperOpening()).toThrow(/gripper/);
    });
  });
}
