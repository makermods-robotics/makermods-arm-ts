// So101Arm: the Arm-shaped API (lifecycle, limits, refusals, stop / return to rest, estop, lost replies) on the
// simulated SO-101 and virtual time.
import { ArmState, ConnectError, StateError, type ArmStatus, type JointStatus } from "@makermods/arm-core";
import { VirtualClock } from "@makermods/arm-core/testing";
import { afterEach, describe, expect, it } from "vitest";
import { So101Arm } from "../src/arm.js";
import { So101Follower } from "../src/follower.js";
import { MemoryCalibrationStore } from "../src/calibration.js";
import { LIMIT_ENTRY_SPEED_DEG_S } from "../src/limits.js";
import { SO101_IDS, SO101_MOTORS, type So101Calibration } from "../src/so101.js";
import { SAMPLE_CALIBRATION, simulatedArm } from "../src/testing.js";

const D2R = Math.PI / 180;
const arms: So101Arm[] = [];

function setup(options: Parameters<typeof simulatedArm>[1] = {}) {
  const clock = new VirtualClock();
  const made = simulatedArm(clock, options);
  arms.push(made.arm);
  return { clock, ...made };
}

afterEach(async () => {
  // Every test leaves its arm disconnected, so no control loop keeps the virtual clock running.
  for (const arm of arms.splice(0)) await arm.disconnect().catch(() => undefined);
});

const torque = (sim: ReturnType<typeof setup>["sim"]) => SO101_IDS.map((id) => sim.reg(id, "Torque_Enable"));

describe("So101Arm lifecycle", () => {
  it("connects without touching torque, enables holding where it is, stops back at rest with torque off", async () => {
    const { arm, sim, port } = setup();
    expect(arm.state).toBe(ArmState.DISCONNECTED);
    expect(arm.jointNames).toEqual([...SO101_MOTORS]);
    await arm.connect();
    expect(arm.state).toBe(ArmState.CONNECTED);
    expect(port.baudRate).toBe(1_000_000);
    for (const id of SO101_IDS) expect(sim.writesTo(id, "Torque_Enable")).toEqual([]);
    const start = arm.getJointPositions();
    expect(start.every(Number.isFinite)).toBe(true);
    expect(arm.gripperOpening()).toBeGreaterThan(0.4);
    expect(arm.getTemperatures()).toEqual(new Array(6).fill(30));
    expect(arm.calibrationProblem).toBeNull();

    await arm.enable();
    expect(arm.state).toBe(ArmState.ENABLED);
    expect(torque(sim)).toEqual([1, 1, 1, 1, 1, 1]);
    // Holding: nothing moves without a target.
    const held = arm.getJointPositions();
    held.forEach((v, i) => expect(Math.abs(v - start[i]!)).toBeLessThan(0.01));

    // A target: the shoulder lift 10° on. The loop sends it through the limiter at 15°/s.
    const target = [...held];
    target[1] = target[1]! + 10 * D2R;
    expect(arm.setJointTargets(target)).toBe(true);
    await arm.clock.sleep(0.25);
    const commanded = arm.getCommandedPositions();
    expect(commanded[1]! - held[1]!).toBeGreaterThan(2 * D2R);
    expect(commanded[1]! - held[1]!).toBeLessThan(5 * D2R); // 15°/s × 0.25 s, plus a tick
    await arm.clock.sleep(1.0);
    expect(arm.getJointPositions()[1]).toBeCloseTo(target[1]!, 2);

    // stop(): back to the enable pose, then torque off.
    expect(await arm.stop()).toBe(true);
    expect(arm.state).toBe(ArmState.CONNECTED);
    expect(torque(sim)).toEqual([0, 0, 0, 0, 0, 0]);
    arm.getJointPositions().forEach((v, i) => expect(Math.abs(v - start[i]!), SO101_MOTORS[i]).toBeLessThan(3 * D2R));

    await arm.disconnect();
    expect(arm.state).toBe(ArmState.DISCONNECTED);
    expect(port.isOpen).toBe(false);
  });

  it("refuses calls in the wrong state", async () => {
    const { arm } = setup();
    await expect(arm.enable()).rejects.toBeInstanceOf(StateError);
    expect(arm.setJointTargets([0, 0, 0, 0, 0, 0])).toBe(false);
    expect(await arm.stop()).toBe(true);
    await arm.connect();
    await expect(arm.connect()).rejects.toBeInstanceOf(StateError);
    expect(arm.setJointTargets(arm.getJointPositions())).toBe(false);
    expect(arm.holdCurrentPosition()).toBe(false);
    expect(await arm.returnToRest()).toBe(false);
    await arm.enable();
    expect(await arm.refresh()).toEqual(new Array(6).fill(false));
    expect(arm.setJointTargets([0, 0, 0])).toBe(false);
    expect(arm.setJointTargets([0, 0, 0, 0, 0, NaN])).toBe(false);
    await expect(arm.enable()).rejects.toBeInstanceOf(StateError);
    await arm.disable();
    expect(arm.state).toBe(ArmState.CONNECTED);
    expect(await arm.refresh()).toEqual(new Array(6).fill(true));
  });

  it("disconnect while enabled switches torque off first", async () => {
    const { arm, sim, port } = setup();
    await arm.connect();
    await arm.enable();
    await arm.disconnect();
    expect(torque(sim)).toEqual([0, 0, 0, 0, 0, 0]);
    expect(port.isOpen).toBe(false);
  });

  it("refuses to connect when a servo is missing or reports a fault, and closes the port", async () => {
    const { arm, sim, port } = setup();
    sim.setOnline(3, false);
    await expect(arm.connect()).rejects.toBeInstanceOf(ConnectError);
    expect(port.isOpen).toBe(false);
    sim.setOnline(3, true);
    sim.servo(4).forcedError = 0x04;
    await expect(arm.connect()).rejects.toThrow(/wrist_flex \(overheat\)/);
    expect(arm.state).toBe(ArmState.DISCONNECTED);
    sim.servo(4).forcedError = 0x01; // input voltage: reported, not a fault
    await arm.connect();
    expect(arm.state).toBe(ArmState.CONNECTED);
  });
});

describe("So101Arm refusals before torque", () => {
  it("refuses to enable without a calibration, nothing switched", async () => {
    const { arm, sim } = setup({ arm: { calibrationStore: new MemoryCalibrationStore() } });
    await arm.connect();
    expect(arm.calibrationProblem).toMatch(/no calibration 'so101_left'/);
    expect(arm.gripperOpening()).toBeNaN();
    expect(() => arm.gripperTarget(0.5)).toThrow(StateError);
    await expect(arm.enable()).rejects.toThrow(/calibrate it first/);
    for (const id of SO101_IDS) expect(sim.writesTo(id, "Torque_Enable")).toEqual([]);
  });

  it("refuses to enable when the servos hold another calibration (a swapped cable)", async () => {
    const other: So101Calibration = structuredClone(SAMPLE_CALIBRATION);
    other.elbow_flex.homing_offset += 5;
    const { arm, sim } = setup({ arm: { calibrationStore: new MemoryCalibrationStore({ so101_left: other }) } });
    await arm.connect();
    expect(arm.calibrationProblem).toMatch(/elbow_flex homing_offset -1365 \(file -1360\)/);
    await expect(arm.enable()).rejects.toThrow(/holds another calibration/);
    for (const id of SO101_IDS) expect(sim.writesTo(id, "Torque_Enable")).toEqual([]);
  });

  it("refuses to enable with an incomplete calibration marker", async () => {
    const store = new MemoryCalibrationStore({ so101_left: SAMPLE_CALIBRATION });
    await store.markIncomplete("so101_left");
    const { arm, sim } = setup({ arm: { calibrationStore: store } });
    await arm.connect();
    await expect(arm.enable()).rejects.toThrow(/left incomplete/);
    for (const id of SO101_IDS) expect(sim.writesTo(id, "Torque_Enable")).toEqual([]);
  });

  it("refuses a servo that already has torque on (nothing sent to it)", async () => {
    const { arm, sim } = setup();
    await arm.connect();
    sim.servo(2).setReg("Torque_Enable", 1);
    await expect(arm.enable()).rejects.toThrow(/servos 2 \(shoulder_lift\) already have torque on/);
    for (const id of SO101_IDS) expect(sim.writesTo(id, "Torque_Enable")).toEqual([]);
    expect(sim.reg(2, "Torque_Enable")).toBe(1);
  });

  it("refuses a joint further past its limit than the tolerance", async () => {
    // The base placed 15° past the left arm's -84.1° limit (more than the 10° tolerance; inside its stops).
    const { arm, sim } = setup();
    const c = SAMPLE_CALIBRATION.shoulder_pan;
    sim.place(1, (c.homing_offset + 2048 + Math.round(((-84.1 - 15) * 4096) / 360) + 4096) % 4096);
    await arm.connect();
    await expect(arm.enable()).rejects.toThrow(/shoulder_pan -99.1° vs -84.1–118.5°/);
    for (const id of SO101_IDS) expect(sim.writesTo(id, "Torque_Enable")).toEqual([]);
  });
});

describe("So101Arm limits", () => {
  it("clamps targets to the soft limits and the gripper short of fully open", async () => {
    const { arm, sim } = setup();
    await arm.connect();
    await arm.enable();
    const q = arm.getJointPositions();
    const far = [...q];
    far[2] = 170 * D2R; // past the elbow's 89° limit
    far[5] = arm.gripperTarget(1);
    expect(arm.setJointTargets(far)).toBe(true);
    await arm.clock.sleep(12);
    const commanded = arm.getCommandedPositions();
    expect(commanded[2]! / D2R).toBeLessThanOrEqual(89.0);
    expect(commanded[2]! / D2R).toBeGreaterThan(88.9);
    // 2 (the limiter's open margin) short of 100, then the driver's 3.
    expect(arm.gripperOpening()).toBeLessThanOrEqual(0.97 + 1e-3);
    const c = SAMPLE_CALIBRATION.elbow_flex;
    expect(sim.reg(3, "Present_Position")).toBeLessThanOrEqual(c.range_max);
  });

  it("brings a joint resting past its limit in at the entry speed, never further out", async () => {
    // The elbow rests 6° past its 89° limit (within the tolerance).
    const { arm, sim } = setup();
    const c = SAMPLE_CALIBRATION.elbow_flex;
    sim.place(3, (c.homing_offset + 2048 + Math.round(((89 + 6) * 4096) / 360) + 4096) % 4096);
    await arm.connect();
    await arm.enable();
    const start = arm.getJointPositions()[2]! / D2R;
    expect(start).toBeGreaterThan(94);
    // A target further out and the rest of the arm where it is: the elbow is only brought in.
    const target = arm.getJointPositions();
    target[2] = 120 * D2R;
    arm.setJointTargets(target);
    await arm.clock.sleep(0.5);
    const after = arm.getCommandedPositions()[2]! / D2R;
    expect(after).toBeLessThanOrEqual(start + 0.1);
    expect(start - after).toBeLessThanOrEqual(LIMIT_ENTRY_SPEED_DEG_S * 0.5 + 0.3);
    expect(start - after).toBeGreaterThan(1.5);
    await arm.clock.sleep(2);
    expect(arm.getCommandedPositions()[2]! / D2R).toBeLessThanOrEqual(89.0);
  });

  it("moveTo waits for arrival and honours its speed cap", async () => {
    const { arm } = setup();
    await arm.connect();
    await arm.enable();
    const q = arm.getJointPositions();
    const goal = [...q];
    goal[0] = goal[0]! - 20 * D2R;
    const t0 = arm.clock.now();
    expect(await arm.moveTo(goal, { speed: 10 * D2R })).toBe(true);
    const took = arm.clock.now() - t0;
    expect(took).toBeGreaterThan(1.6); // 20° at 10°/s, less the 3° arrival band
    expect(Math.abs(arm.getJointPositions()[0]! - goal[0]!)).toBeLessThan(3 * D2R);
    // Cancelled: holds where it is.
    const abort = new AbortController();
    const back = arm.moveTo(q, { signal: abort.signal });
    void arm.clock.sleep(0.3).then(() => abort.abort());
    expect(await back).toBe(false);
    expect(await arm.stop()).toBe(true);
  });
});

describe("So101Arm stopping", () => {
  it("returnToRest keeps torque on; stop afterwards switches it off", async () => {
    const { arm, sim } = setup();
    await arm.connect();
    await arm.enable();
    const start = arm.getJointPositions();
    const target = [...start];
    target[3] = target[3]! + 15 * D2R;
    expect(await arm.moveTo(target)).toBe(true);
    // Targets are refused while the return runs.
    const returning = arm.returnToRest();
    await arm.clock.sleep(0.1);
    expect(arm.setJointTargets(target)).toBe(false);
    expect(await returning).toBe(true);
    expect(torque(sim)).toEqual([1, 1, 1, 1, 1, 1]);
    expect(Math.abs(arm.getJointPositions()[3]! - start[3]!)).toBeLessThan(3 * D2R);
    expect(arm.setJointTargets(target)).toBe(true);
    expect(await arm.stop()).toBe(true);
    expect(torque(sim)).toEqual([0, 0, 0, 0, 0, 0]);
  });

  it("returns at the rest speed (15°/s, eased)", async () => {
    const { arm } = setup();
    await arm.connect();
    await arm.enable();
    const start = arm.getJointPositions();
    const target = [...start];
    target[0] = target[0]! + 30 * D2R;
    expect(await arm.moveTo(target, { timeout: 10 })).toBe(true);
    let fastest = 0;
    let prev = arm.getCommandedPositions()[0]!;
    const stopping = arm.stop();
    for (let i = 0; i < 40; i++) {
      await arm.clock.sleep(0.05);
      const now = arm.getCommandedPositions()[0];
      if (now === undefined || !Number.isFinite(now)) break;
      fastest = Math.max(fastest, Math.abs(now - prev) / 0.05);
      prev = now;
    }
    expect(await stopping).toBe(true);
    expect(fastest / D2R).toBeLessThanOrEqual(15 * 1.2 + 2); // a 25 ms tick quantizes the 50 ms window
  });

  it("estop from another async context switches torque off at once, even mid-move", async () => {
    const { arm, sim } = setup();
    await arm.connect();
    await arm.enable();
    const target = arm.getJointPositions();
    target[0] = target[0]! + 40 * D2R;
    const moving = arm.moveTo(target, { timeout: 10 });
    const stopped = (async () => {
      await arm.clock.sleep(0.4);
      await arm.estop();
      return arm.clock.now();
    })();
    expect(await moving).toBe(false);
    const at = await stopped;
    expect(torque(sim)).toEqual([0, 0, 0, 0, 0, 0]);
    expect(arm.state).toBe(ArmState.CONNECTED);
    // No goal written after the estop's switch-off.
    const lastGoal = Math.max(...sim.writes.filter((w) => w.address === 42).map((w) => w.t));
    const lastOff = Math.max(...sim.writes.filter((w) => w.address === 40 && w.data[0] === 0).map((w) => w.t));
    expect(lastGoal).toBeLessThan(lastOff);
    expect(lastOff).toBeLessThanOrEqual(at);
    // Enable works again afterwards.
    await arm.enable();
    expect(arm.state).toBe(ArmState.ENABLED);
  });

  it("estop during enable cancels it: what started is switched off", async () => {
    const { arm, sim, port } = setup();
    await arm.connect();
    let estop: Promise<void> | null = null;
    port.beforeWrite = () => {
      if (!estop && sim.writesTo(2, "Torque_Enable").includes(1)) estop = arm.estop();
    };
    await expect(arm.enable()).rejects.toThrow(/e-stop during enable/);
    port.beforeWrite = null;
    await estop;
    expect(torque(sim)).toEqual([0, 0, 0, 0, 0, 0]);
    for (const id of [4, 5, 6]) expect(sim.writesTo(id, "Torque_Enable")).not.toContain(1);
    expect(arm.state).toBe(ArmState.CONNECTED);
  });
});

describe("So101Arm faults", () => {
  it("a servo that stops answering: missed ticks hold, then the arm faults holding (no switch-off)", async () => {
    const { arm, sim } = setup();
    await arm.connect();
    await arm.enable();
    const goals = sim.writesTo(1, "Goal_Position").length;
    sim.setOnline(4, false);
    await arm.clock.sleep(0.2);
    expect(arm.state).toBe(ArmState.ENABLED); // within MAX_MISSED_MS: the ticks hold
    const held = sim.writesTo(1, "Goal_Position").length;
    expect(held - goals).toBeLessThanOrEqual(2); // nothing commanded on stale feedback
    await arm.clock.sleep(0.5);
    expect(arm.state).toBe(ArmState.FAULT);
    expect(arm.faultReason).toMatch(/no position for/);
    expect(arm.faultReason).toMatch(/stopped answering and keeps holding/);
    // Holding: torque stays on, nothing more is written.
    expect(torque(sim).filter((t, i) => i !== 3)).toEqual([1, 1, 1, 1, 1]);
    const after = sim.writes.length;
    await arm.clock.sleep(0.5);
    expect(sim.writes.length).toBe(after);
    expect(await arm.stop()).toBe(false);
    expect(arm.setJointTargets(arm.getJointPositions())).toBe(false);
    // The caller decides: disable once supported. The silent servo can't confirm it.
    await expect(arm.disable()).rejects.toThrow(/wrist_flex \(id 4\)/);
    expect(arm.state).toBe(ArmState.FAULT);
    sim.setOnline(4, true);
    await arm.clearFaults();
    expect(arm.state).toBe(ArmState.CONNECTED);
    expect(arm.faultReason).toBeNull();
    expect(torque(sim)).toEqual([0, 0, 0, 0, 0, 0]);
  });

  it("a servo fault latches: the arm holds, every later command is refused", async () => {
    const { arm, sim } = setup();
    await arm.connect();
    await arm.enable();
    sim.servo(2).forcedError = 0x04;
    await arm.clock.sleep(0.1);
    expect(arm.state).toBe(ArmState.FAULT);
    expect(arm.faultReason).toMatch(/shoulder_lift \(overheat\)/);
    expect(arm.status().joints.shoulder_lift!.fault).toBe("overheat");
    expect(await arm.stop()).toBe(false);
    await arm.estop();
    expect(torque(sim)).toEqual([0, 0, 0, 0, 0, 0]);
    expect(arm.faultReason).toMatch(/overheat/);
  });

  it("a measured joint past its limit + tolerance mid-run: the arm can't return from there, so it holds", async () => {
    const { arm, sim } = setup();
    await arm.connect();
    await arm.enable();
    // The base forced far past its limit (modelled as a teleport while holding).
    const c = SAMPLE_CALIBRATION.shoulder_pan;
    const s = sim.servo(1);
    let actual = (c.homing_offset + 2048 + Math.round(((-84.1 - 13) * 4096) / 360) + 4096) % 4096;
    if (actual < s.stops!.min) actual += 4096;
    s.pos = actual;
    await arm.clock.sleep(0.05);
    expect(arm.state).toBe(ArmState.FAULT);
    expect(arm.faultReason).toMatch(/Can't return to rest \(left: measured position outside configured limits \(shoulder_pan/);
    expect(torque(sim)).toEqual([1, 1, 1, 1, 1, 1]);
    expect(await arm.stop()).toBe(false);
    await arm.disable();
    expect(torque(sim)).toEqual([0, 0, 0, 0, 0, 0]);
  });

  it("a failed tick on an arm that still answers: returns to rest, then torque off, the fault kept", async () => {
    const { arm, sim } = setup();
    await arm.connect();
    await arm.enable();
    const start = arm.getJointPositions();
    const target = [...start];
    target[0] = target[0]! + 10 * D2R;
    expect(await arm.moveTo(target)).toBe(true);
    // One tick's read fails with something other than a missed reply.
    const read = So101Follower.prototype.read;
    let thrown = false;
    So101Follower.prototype.read = function (this: So101Follower) {
      if (!thrown) {
        thrown = true;
        return Promise.reject(new RangeError("a glitch"));
      }
      return read.call(this);
    };
    try {
      await arm.clock.sleep(0.05);
    } finally {
      So101Follower.prototype.read = read;
    }
    expect(arm.state).toBe(ArmState.FAULT);
    expect(arm.faultReason).toMatch(/Control fault: RangeError: a glitch. Returning to rest, then torque off/);
    expect(arm.setJointTargets(target)).toBe(false);
    expect(await arm.stop()).toBe(true);
    expect(torque(sim)).toEqual([0, 0, 0, 0, 0, 0]);
    expect(Math.abs(arm.getJointPositions()[0]! - start[0]!)).toBeLessThan(3 * D2R);
    expect(arm.state).toBe(ArmState.FAULT);
    await expect(arm.enable()).rejects.toBeInstanceOf(StateError);
    await arm.clearFaults();
    expect(arm.state).toBe(ArmState.CONNECTED);
  });

  it("a return to rest that cannot arrive times out holding", async () => {
    const { arm, sim } = setup();
    await arm.connect();
    await arm.enable();
    const target = arm.getJointPositions();
    target[0] = target[0]! + 20 * D2R;
    expect(await arm.moveTo(target)).toBe(true);
    sim.servo(1).requiredTorque = 5000; // jammed: no torque moves it, but it keeps its torque on
    sim.servo(1).overloadMs = Infinity;
    expect(await arm.stop()).toBe(false);
    expect(arm.state).toBe(ArmState.FAULT);
    expect(arm.faultReason).toMatch(/tracking error exceeded|timed out/);
    expect(torque(sim)).toEqual([1, 1, 1, 1, 1, 1]);
    await arm.disable();
    expect(torque(sim)).toEqual([0, 0, 0, 0, 0, 0]);
  });
});

describe("So101Arm status", () => {
  it("has the core Arm.status() keys and nesting, plus the SO-101's own", async () => {
    const { arm, sim } = setup();
    const before = arm.status();
    expect(before.joints.gripper).toMatchObject({ position: null, temperature: null, temperatures: {}, enabled: null, feedbackAge: null });
    await arm.connect();
    const s = arm.status();
    // Usable wherever the core's ArmStatus is.
    const core: ArmStatus = s;
    expect(Object.keys(core)).toEqual(["state", "faultReason", "joints", "calibration"]);
    expect(s.state).toBe("connected");
    expect(s.faultReason).toBeNull();
    expect(s.calibration).toEqual({ name: "so101_left", loaded: true, problem: null });
    expect(Object.keys(s.joints)).toEqual([...SO101_MOTORS]);
    const elbow: JointStatus = s.joints.elbow_flex!;
    expect(Object.keys(elbow)).toEqual(["motorId", "position", "velocity", "torque", "temperature", "temperatures", "enabled", "fault", "feedbackAge", "voltage", "load", "errorBits"]);
    expect(s.joints.elbow_flex).toMatchObject({ motorId: 3, torque: null, temperature: 30, temperatures: { servo: 30 }, voltage: 12, load: 0, enabled: false, fault: null, errorBits: null, velocity: 0 });
    expect(s.joints.elbow_flex!.position).toBeCloseTo(arm.getJointPositions()[2]!, 12);
    expect(s.joints.elbow_flex!.feedbackAge).toBeLessThan(0.05);
    // The input-voltage bit: a warning in errorBits, not a fault.
    sim.servo(3).forcedError = 0x01;
    await arm.refresh();
    expect(arm.status().joints.elbow_flex).toMatchObject({ fault: null, errorBits: "input voltage" });
    expect(JSON.parse(JSON.stringify(arm.status()))).toEqual(arm.status());
  });

  it("gripperTarget maps an opening onto the calibrated range", async () => {
    const { arm } = setup();
    await arm.connect();
    expect(arm.gripperTarget(0)).toBeCloseTo(((SAMPLE_CALIBRATION.gripper.range_min - 2048) * 2 * Math.PI) / 4096, 12);
    expect(arm.gripperTarget(2)).toBe(arm.gripperTarget(1));
    expect(() => arm.gripperTarget(NaN)).toThrow();
  });
});

describe("So101Arm.wiggle", () => {
  it("wiggles only the gripper on the open port, returns it, switches it off, and keeps the port", async () => {
    const { arm, sim, port } = setup();
    await arm.connect();
    const start = sim.reg(6, "Present_Position");
    const result = await arm.wiggle();
    expect(result).toEqual({ success: true, message: "Wiggled the gripper on usb:1A86:55D3." });
    const goals = sim.writesTo(6, "Goal_Position");
    expect(goals[0]).toBe(start);
    expect(goals).toContain(Math.min(start + 200, SAMPLE_CALIBRATION.gripper.range_max));
    expect(goals).toContain(Math.max(start - 200, SAMPLE_CALIBRATION.gripper.range_min));
    expect(goals.at(-1)).toBe(start);
    expect(sim.reg(6, "Torque_Enable")).toBe(0);
    for (const id of [1, 2, 3, 4, 5]) expect(sim.writesTo(id, "Torque_Enable")).toEqual([]);
    expect(port.isOpen).toBe(true);
    expect(port.opens).toBe(1);
    expect(arm.state).toBe(ArmState.CONNECTED);
    await arm.enable();
    expect(await arm.stop()).toBe(true);
  });

  it("refuses where it would fight the control loop or a holding arm, nothing switched", async () => {
    const { arm, sim } = setup();
    await expect(arm.wiggle()).rejects.toThrow(/requires CONNECTED state \(torque off\), currently disconnected/);
    await arm.connect();
    sim.servo(6).setReg("Torque_Enable", 1);
    await expect(arm.wiggle()).rejects.toThrow(/gripper already has torque on/);
    sim.servo(6).setReg("Torque_Enable", 0);
    sim.servo(6).forcedError = 0x04;
    await expect(arm.wiggle()).rejects.toThrow(/gripper reports a fault \(overheat\)/);
    sim.servo(6).forcedError = 0;
    expect(sim.writesTo(6, "Torque_Enable")).toEqual([]);
    await arm.enable();
    const writes = sim.writesTo(6, "Torque_Enable").length;
    await expect(arm.wiggle()).rejects.toThrow(/currently enabled/);
    expect(sim.writesTo(6, "Torque_Enable").length).toBe(writes);
    await arm.disable();
    // While a calibration runs.
    const session = arm.calibrate({ method: "manual" });
    await expect(arm.wiggle()).rejects.toThrow(/a calibration is running/);
    await session.stop();
  });

  it("estop during a wiggle switches the gripper off at once, without the return", async () => {
    const { arm, sim } = setup();
    await arm.connect();
    const wiggling = arm.wiggle();
    await arm.clock.sleep(0.4);
    expect(sim.reg(6, "Torque_Enable")).toBe(1);
    const at = arm.clock.now();
    await arm.estop();
    const result = await wiggling;
    expect(result.success).toBe(false);
    expect(result.message).toMatch(/E-stop/);
    expect(sim.reg(6, "Torque_Enable")).toBe(0);
    expect(arm.clock.now() - at).toBeLessThan(0.2);
    expect(sim.writes.filter((w) => w.id === 6 && w.address === 42 && w.t > at)).toEqual([]);
    expect(arm.state).toBe(ArmState.CONNECTED);
  });

  it("a signal ends the jog early; the gripper still returns before it is switched off", async () => {
    const { arm, sim } = setup();
    await arm.connect();
    const start = sim.reg(6, "Present_Position");
    const abort = new AbortController();
    void arm.clock.sleep(0.4).then(() => abort.abort());
    expect((await arm.wiggle({ signal: abort.signal })).success).toBe(true);
    expect(sim.writesTo(6, "Goal_Position").at(-1)).toBe(start);
    expect(sim.writesTo(6, "Goal_Position").length).toBeLessThan(8);
    expect(sim.reg(6, "Torque_Enable")).toBe(0);
  });
});
