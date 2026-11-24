// SO-101 calibration by hand (MakerLab calibrate.py, dreamlabs so101_calibration) and its refusals, ported from the
// console's test/so101.test.ts; the stores; and calibrating through So101Arm.
import { ArmState } from "@makermods/arm-core";
import { VirtualClock } from "@makermods/arm-core/testing";
import { describe, expect, it } from "vitest";
import { LocalStorageCalibrationStore, MemoryCalibrationStore, calibrateSo101, type CalibrationStore, type StorageLike } from "../src/calibration.js";
import { simulatedSo101 } from "../src/sim.js";
import { SO101_IDS, SO101_MOTORS, referenceRanges, type So101Calibration } from "../src/so101.js";
import { simulatedArm } from "../src/testing.js";
import fixtures from "./fixtures/calibrations.json" with { type: "json" };

const LEFT_FILE = fixtures.follower_left as So101Calibration;

/** A store that records every call. */
function recordingStore() {
  const inner = new MemoryCalibrationStore();
  const calls: string[] = [];
  const s: CalibrationStore = {
    load: (n) => inner.load(n),
    save: async (n, c) => (calls.push(`save ${n}`), inner.save(n, c)),
    markIncomplete: async (n) => (calls.push(`mark ${n}`), inner.markIncomplete(n)),
    clearIncomplete: async (n) => (calls.push(`clear ${n}`), inner.clearIncomplete(n)),
    isIncomplete: (n) => inner.isIncomplete(n),
  };
  return { s, inner, calls };
}

describe("SO-101 calibration safety", () => {
  it("refuses without switching anything when a servo's torque state doesn't come back", async () => {
    const clock = new VirtualClock();
    const { sim, port } = simulatedSo101(clock, { calibration: LEFT_FILE });
    sim.servo(2).setReg("Torque_Enable", 1); // holding the arm up
    // The handshake is 12 exchanges (ping and model per servo); the 13th is the torque-state read, which servo 2 misses.
    port.beforeWrite = () => sim.setOnline(2, port.written.length !== 13);
    const session = await calibrateSo101(port, clock, { name: "lab_left", store: new MemoryCalibrationStore(), side: "left", method: "manual" });
    await session.done;
    expect(session.status().error).toMatch(/shoulder_lift did not report its torque state; nothing was switched/);
    expect(sim.reg(2, "Torque_Enable")).toBe(1);
    for (const id of SO101_IDS) expect(sim.writesTo(id, "Torque_Enable")).toEqual([]);
    expect(port.isOpen).toBe(false);
  });

  it("a stop before the recording starts ends the session promptly (no hang)", async () => {
    const clock = new VirtualClock();
    const { sim, port } = simulatedSo101(clock, { calibration: LEFT_FILE });
    const session = await calibrateSo101(port, clock, { name: "lab_left", store: new MemoryCalibrationStore(), side: "left", method: "manual" });
    const start = clock.now();
    await session.stop();
    await session.done;
    expect(clock.now() - start).toBeLessThan(2);
    expect(session.status().status).toBe("stopped");
    for (const id of SO101_IDS) expect(sim.reg(id, "Torque_Enable")).toBe(0);
  });

  it("refuses a servo holding torque, leaving it holding", async () => {
    const clock = new VirtualClock();
    const busy = simulatedSo101(clock, { calibration: LEFT_FILE });
    busy.sim.servo(2).setReg("Torque_Enable", 1);
    const refused = await calibrateSo101(busy.port, clock, { name: "lab_left", store: new MemoryCalibrationStore(), side: "left", method: "manual" });
    await refused.done;
    expect(refused.status().error).toMatch(/shoulder_lift has torque on/);
    expect(busy.sim.reg(2, "Torque_Enable")).toBe(1);
  });
});

describe("SO-101 calibration by hand", () => {
  it("homes at the middle pose, records each joint end to end, aligns a follower to the reference and saves", async () => {
    const clock = new VirtualClock();
    const { sim, port } = simulatedSo101(clock, { calibration: LEFT_FILE, hold: false });
    const ref = referenceRanges("left");
    const st = recordingStore();
    const session = await calibrateSo101(port, clock, { name: "lab_left", store: st.s, side: "left", method: "manual" });
    await clock.sleep(0.2);
    expect(session.status().status).toBe("recording");
    // The middle pose read 2047 everywhere; now sweep each joint with stops through the reference span.
    const homing = Object.fromEntries(SO101_IDS.map((id) => [id, sim.reg(id, "Homing_Offset")]));
    for (const [i, m] of SO101_MOTORS.entries()) {
      const id = SO101_IDS[i]!;
      expect(sim.reg(id, "Present_Position"), m).toBe(2047);
      const r = ref[m];
      const [lo, hi] = r ? [Math.round(r.low) + 30, Math.round(r.high) + 30] : m === "gripper" ? [1700, 2400] : [2047, 2047];
      // By hand, continuously (the recording refuses a jump that looks like an encoder wrap).
      for (const target of [lo, hi, 2047]) {
        const from = sim.reg(id, "Present_Position");
        const steps = Math.ceil(Math.abs(target - from) / 150) || 1;
        for (let k = 1; k <= steps; k++) {
          sim.moveByHand(id, homing[id]! + Math.round(from + ((target - from) * k) / steps));
          await clock.sleep(0.06);
        }
      }
    }
    const recorded = session.status().recordedRanges!;
    expect(recorded.elbow_flex!.max - recorded.elbow_flex!.min).toBe(Math.round(ref.elbow_flex!.high) - Math.round(ref.elbow_flex!.low));
    session.complete();
    await session.done;
    const status = session.status();
    expect(status.status, status.error).toBe("completed");
    expect(status.name).toBe("lab_left");
    const data = (await st.inner.load("lab_left"))!;
    expect(status.calibration).toEqual(data);
    for (const m of ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"] as const) {
      expect(Math.abs((data[m].range_min + data[m].range_max) / 2 - ref[m]!.middle)).toBeLessThanOrEqual(1);
      const r = recorded[m]!;
      const shift = Math.round((r.min + r.max) / 2 - ref[m]!.middle);
      expect([data[m].homing_offset, data[m].range_min, data[m].range_max]).toEqual([homing[SO101_IDS[SO101_MOTORS.indexOf(m)]!]! + shift, r.min - shift, r.max - shift]);
    }
    expect([data.wrist_roll.range_min, data.wrist_roll.range_max]).toEqual([0, 4095]);
    // The servos hold exactly the calibration (persisted: Lock 0), torque off, marker set then cleared.
    sim.powerCycle();
    for (const m of SO101_MOTORS) {
      const id = data[m].id;
      expect([sim.eeprom(id, "Homing_Offset"), sim.eeprom(id, "Min_Position_Limit"), sim.eeprom(id, "Max_Position_Limit")]).toEqual([data[m].homing_offset, data[m].range_min, data[m].range_max]);
      expect(sim.reg(id, "Torque_Enable")).toBe(0);
    }
    expect(st.calls).toEqual(["mark lab_left", "save lab_left", "clear lab_left"]);
    expect(port.isOpen).toBe(false);
  });

  it("refuses to save a joint that wasn't moved far enough, and gives the servos their old calibration back", async () => {
    const clock = new VirtualClock();
    const { sim, port } = simulatedSo101(clock, { calibration: LEFT_FILE });
    const st = recordingStore();
    const session = await calibrateSo101(port, clock, { name: "dlabs_xlerobot_left", store: st.s, side: "left", method: "manual" });
    await clock.sleep(0.2);
    expect(session.status().status).toBe("recording");
    session.complete();
    await session.done;
    const failed = session.status();
    expect(failed.status).toBe("error");
    expect(failed.error).toMatch(/Move base turn, shoulder, elbow, wrist bend, gripper all the way/);
    for (const m of SO101_MOTORS) {
      const id = LEFT_FILE[m].id;
      expect([sim.reg(id, "Homing_Offset"), sim.reg(id, "Min_Position_Limit"), sim.reg(id, "Max_Position_Limit")]).toEqual([LEFT_FILE[m].homing_offset, LEFT_FILE[m].range_min, LEFT_FILE[m].range_max]);
    }
    expect(st.calls).toEqual(["mark dlabs_xlerobot_left", "clear dlabs_xlerobot_left"]);
  });
});

describe("calibrating through So101Arm", () => {
  it("an uncalibrated arm is refused, calibrated on its own port, then enables", async () => {
    const clock = new VirtualClock();
    const store = new MemoryCalibrationStore();
    const { arm, sim } = simulatedArm(clock, { hold: false, arm: { calibrationStore: store } });
    await arm.connect();
    await expect(arm.enable()).rejects.toThrow(/calibrate it first/);
    const session = arm.calibrate({ method: "manual" });
    await expect(arm.enable()).rejects.toThrow(/a calibration is running/);
    expect(() => arm.calibrate({ method: "manual" })).toThrow(/already running/);
    await clock.sleep(0.2);
    expect(session.status().status).toBe("recording");
    const ref = referenceRanges("left");
    const homing = Object.fromEntries(SO101_IDS.map((id) => [id, sim.reg(id, "Homing_Offset")]));
    for (const [i, m] of SO101_MOTORS.entries()) {
      const id = SO101_IDS[i]!;
      const r = ref[m];
      const [lo, hi] = r ? [Math.round(r.low) + 30, Math.round(r.high) + 30] : m === "gripper" ? [1700, 2400] : [2047, 2047];
      for (const target of [lo, hi, 2047]) {
        const from = sim.reg(id, "Present_Position");
        const steps = Math.ceil(Math.abs(target - from) / 150) || 1;
        for (let k = 1; k <= steps; k++) {
          sim.moveByHand(id, homing[id]! + Math.round(from + ((target - from) * k) / steps));
          await clock.sleep(0.06);
        }
      }
    }
    session.complete();
    await session.done;
    expect(session.status().status).toBe("completed");
    await clock.sleep(0.1);
    expect(arm.calibrationProblem).toBeNull();
    expect(await store.load("so101_left")).toEqual(arm.calibration);
    await arm.enable();
    expect(arm.state).toBe(ArmState.ENABLED);
    expect(await arm.stop()).toBe(true);
    await arm.disconnect();
  });
});

describe("estop during an automatic calibration", () => {
  it("switches the arm off at once (no return), ends the session, and leaves the arm CONNECTED", async () => {
    const clock = new VirtualClock();
    const left = fixtures.autocal_left as So101Calibration;
    const { arm, sim } = simulatedArm(clock, {
      calibration: left,
      hold: false,
      start: { shoulder_pan: 0.5, shoulder_lift: 0.96, elbow_flex: 0.95, wrist_flex: 0.2, wrist_roll: 0.5, gripper: 0.05 },
      arm: { calibrationStore: new MemoryCalibrationStore() },
    });
    await arm.connect();
    const session = arm.calibrate({ method: "auto" });
    await clock.sleep(12);
    expect(session.status().status).toBe("running");
    expect(SO101_IDS.some((id) => sim.reg(id, "Torque_Enable") === 1)).toBe(true);
    const at = clock.now();
    await arm.estop();
    expect(clock.now() - at).toBeLessThan(1);
    for (const id of SO101_IDS) expect(sim.reg(id, "Torque_Enable")).toBe(0);
    expect(sim.writes.some((w) => w.address === 40 && w.data[0] === 1 && w.t > at)).toBe(false);
    expect(session.active).toBe(false);
    expect(session.status().status).toBe("stopped");
    expect(arm.state).toBe(ArmState.CONNECTED);
    expect(await arm.calibrationStore.isIncomplete("so101_left")).toBe(true);
    await arm.disconnect();
  }, 60_000);
});

describe("calibration stores", () => {
  it("MemoryCalibrationStore keeps copies and the incomplete marker", async () => {
    const store = new MemoryCalibrationStore();
    expect(await store.load("x")).toBeNull();
    await store.save("x", LEFT_FILE);
    const got = (await store.load("x"))!;
    got.gripper.range_max = 1;
    expect((await store.load("x"))!.gripper.range_max).toBe(LEFT_FILE.gripper.range_max);
    await expect(store.save("bad", { ...LEFT_FILE, gripper: { ...LEFT_FILE.gripper, id: 9 } })).rejects.toThrow(/id 9/);
    await store.markIncomplete("x");
    expect(await store.isIncomplete("x")).toBe(true);
    await store.clearIncomplete("x");
    expect(await store.isIncomplete("x")).toBe(false);
  });

  it("LocalStorageCalibrationStore saves LeRobot JSON under its prefix", async () => {
    const data = new Map<string, string>();
    const storage: StorageLike = { getItem: (k) => data.get(k) ?? null, setItem: (k, v) => void data.set(k, v), removeItem: (k) => void data.delete(k) };
    const store = new LocalStorageCalibrationStore({ storage, prefix: "p." });
    await store.save("lab_left", LEFT_FILE);
    expect(JSON.parse(data.get("p.lab_left")!)).toEqual(LEFT_FILE);
    expect(await new LocalStorageCalibrationStore({ storage, prefix: "p." }).load("lab_left")).toEqual(LEFT_FILE);
    await store.markIncomplete("lab_left");
    expect(data.get("p.lab_left.incomplete")).toBe("1");
    expect(await new LocalStorageCalibrationStore({ storage, prefix: "p." }).isIncomplete("lab_left")).toBe(true);
    await store.clearIncomplete("lab_left");
    expect(data.has("p.lab_left.incomplete")).toBe(false);
    data.set("p.broken", "{not json");
    expect(await store.load("broken")).toBeNull();
    expect(store.persistent).toBe(true);
  });

  it("degrades to memory when storage is missing, throws on access, or is full", async () => {
    const missing = new LocalStorageCalibrationStore({ storage: null });
    expect(missing.persistent).toBe(false);
    await missing.save("a", LEFT_FILE);
    expect(await missing.load("a")).toEqual(LEFT_FILE);

    const blocked: StorageLike = {
      getItem: () => {
        throw new DOMException("The operation is insecure.", "SecurityError");
      },
      setItem: () => {
        throw new DOMException("The operation is insecure.", "SecurityError");
      },
      removeItem: () => undefined,
    };
    const b = new LocalStorageCalibrationStore({ storage: blocked });
    expect(await b.load("a")).toBeNull();
    expect(b.persistent).toBe(false);
    expect(b.lastError).toMatch(/insecure/);
    await b.save("a", LEFT_FILE);
    expect(await b.load("a")).toEqual(LEFT_FILE);

    const data = new Map<string, string>();
    const full: StorageLike = {
      getItem: (k) => data.get(k) ?? null,
      setItem: () => {
        throw new DOMException("Quota exceeded", "QuotaExceededError");
      },
      removeItem: (k) => void data.delete(k),
    };
    const f = new LocalStorageCalibrationStore({ storage: full });
    await f.save("a", LEFT_FILE);
    expect(f.persistent).toBe(false);
    expect(await f.load("a")).toEqual(LEFT_FILE);
    await f.markIncomplete("a");
    expect(await f.isIncomplete("a")).toBe(true);

    // No localStorage global at all (a Web Worker, or Node here).
    const none = new LocalStorageCalibrationStore();
    expect(none.persistent).toBe(typeof (globalThis as { localStorage?: unknown }).localStorage !== "undefined");
  });
});
