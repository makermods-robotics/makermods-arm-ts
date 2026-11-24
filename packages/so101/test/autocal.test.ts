/** SO-101 auto-calibration (src/autocal.ts, the port of MakerLab's feetech_autocal) on the simulated STS3215 bus: the
range math against the Python helpers, full runs against real arms' hard stops, Stop, and a servo dropping out.
Ported from the console's test/so101-autocal.test.ts. */

import { VirtualClock } from "@makermods/arm-core/testing";
import { describe, expect, it } from "vitest";
import { FeetechBus } from "../src/bus.js";
import { msClock } from "../src/clock-ms.js";
import {
  AutoCalStopped,
  RETURN_POS_SPEED,
  SO101_IDS,
  SO101_MOTORS,
  autoCalibrateSo101,
  clampTorqueLimit,
  nearestWrapTarget,
  rangeFromLimits,
  type AutoCalOptions,
  type MotorCalibration,
} from "../src/autocal.js";
import { SimFeetechBus } from "../src/sim.js";
import golden from "./fixtures/autocal-golden.json" with { type: "json" };
import calibrations from "./fixtures/calibrations.json" with { type: "json" };

/** Golden values from the Python helpers themselves (fixtures/autocal-golden.json): _compute_mid_and_range_from_limits
and _nearest_wrap_target. */
type CalFile = Record<(typeof SO101_MOTORS)[number], MotorCalibration>;

/** dlabs_xlerobot_left.json (written by this procedure on hardware): every joint's free arc crosses the encoder wrap. */
const LEFT = calibrations.autocal_left as CalFile;
/** dlabs_xlerobot_right.json: shoulder_lift and gripper do not cross the wrap, the others do. */
const RIGHT = calibrations.autocal_right as CalFile;

const mod = (a: number, n: number) => ((a % n) + n) % n;

/** Where each joint starts, as a fraction of its free arc from the low stop: the shoulder and elbow folded hard
against one stop (so their first lift attempt stalls, overloads and they lift the other way), the gripper closed. */
const START: Record<(typeof SO101_MOTORS)[number], number> = {
  shoulder_pan: 0.5,
  shoulder_lift: 0.96,
  elbow_flex: 0.95,
  wrist_flex: 0.2,
  wrist_roll: 0.5,
  gripper: 0.05,
};

/** A simulated arm whose hard stops are the ones `file` describes (actual = present + homing_offset). */
function arm(file: CalFile) {
  const clock = new VirtualClock();
  const sim = new SimFeetechBus(clock);
  const start = new Map<number, number>();
  for (const m of SO101_MOTORS) {
    const c = file[m];
    const lo = mod(c.homing_offset + c.range_min, 4096);
    const hi = mod(c.homing_offset + c.range_max, 4096);
    sim.setHardStops(c.id, lo, hi);
    const actual = mod(lo + Math.round(START[m] * (c.range_max - c.range_min)), 4096);
    sim.place(c.id, actual);
    start.set(c.id, actual);
  }
  const bus = new FeetechBus(sim.uart, msClock(clock));
  const progress: Array<[number, string]> = [];
  const logs: string[] = [];
  const options = (extra: Partial<AutoCalOptions> = {}): AutoCalOptions => ({
    clock,
    onProgress: (p, msg) => progress.push([p, msg]),
    log: (line) => logs.push(line),
    ...extra,
  });
  return { clock, sim, bus, start, progress, logs, options };
}

/** Shortest distance between two encoder readings. */
const wrapDist = (a: number, b: number) => Math.min(mod(a - b, 4096), mod(b - a, 4096));

function expectReleased(sim: SimFeetechBus) {
  for (const id of [1, 2, 3, 4, 5, 6]) {
    if (!sim.servo(id).online) continue;
    expect(sim.reg(id, "Torque_Enable"), `servo ${id} torque`).toBe(0);
    expect(sim.writesTo(id, "Torque_Enable").at(-1), `servo ${id} last torque write`).toBe(0);
  }
}

describe("so101 auto-calibration math (golden: the Python helpers)", () => {
  it("computes range and homing offset from the stall positions like _compute_mid_and_range_from_limits", () => {
    expect(golden.limits).toHaveLength(22);
    for (const { cw, ccw, ref, expected } of golden.limits) expect(rangeFromLimits(cw, ccw, ref), `cw=${cw} ccw=${ccw} ref=${ref}`).toEqual(expected);
  });

  it("picks the return target like _nearest_wrap_target", () => {
    expect(golden.wrap).toHaveLength(7);
    for (const { base, present, min, max, expected } of golden.wrap) expect(nearestWrapTarget(base, present, min, max), `${base} from ${present}`).toBe(expected);
  });

  it("clamps the working torque to 100..1000 (script main)", () => {
    expect(clampTorqueLimit()).toBe(380);
    expect(clampTorqueLimit(38)).toBe(100);
    expect(clampTorqueLimit(2000)).toBe(1000);
    expect(clampTorqueLimit(550)).toBe(550);
  });
});

describe("so101 auto-calibration on the simulated arm", () => {
  it("measures the left arm (every range across the encoder wrap) to its saved calibration", async () => {
    const { clock, sim, bus, progress, options } = arm(LEFT);
    const result = await autoCalibrateSo101(bus, options());

    expect(result).toEqual(LEFT);
    expect(Object.keys(result)).toEqual([...SO101_MOTORS]);
    // Persisted with write_calibration semantics (Lock 0), torque released, Lock left 0 by the disconnect.
    sim.powerCycle();
    for (const m of SO101_MOTORS) {
      const id = SO101_IDS[m];
      expect(sim.eeprom(id, "Homing_Offset"), m).toBe(LEFT[m].homing_offset);
      expect(sim.eeprom(id, "Min_Position_Limit"), m).toBe(LEFT[m].range_min);
      expect(sim.eeprom(id, "Max_Position_Limit"), m).toBe(LEFT[m].range_max);
      expect(sim.writesTo(id, "Lock").at(-1), m).toBe(0);
      expect(sim.writesTo(id, "Torque_Limit"), m).toEqual([380]);
      // Probed in velocity mode both ways at DEFAULT_VELOCITY_LIMIT.
      expect(sim.writesTo(id, "Goal_Velocity"), m).toEqual(expect.arrayContaining([1000, -1000]));
    }
    expectReleased(sim);
    // The folded rest pose: every joint inside its calibrated window.
    for (const m of SO101_MOTORS) {
      const present = sim.reg(SO101_IDS[m], "Present_Position");
      expect(present, m).toBeGreaterThanOrEqual(LEFT[m].range_min - 1);
      expect(present, m).toBeLessThanOrEqual(LEFT[m].range_max + 1);
    }
    const fractions = progress.map(([p]) => p);
    expect(fractions).toEqual([...fractions].sort((a, b) => a - b));
    expect(fractions.at(-1)).toBe(1);
  }, 60_000);

  it("measures the right arm (wrapping and non-wrapping ranges) at a requested torque", async () => {
    const { clock, sim, bus, options } = arm(RIGHT);
    const result = await autoCalibrateSo101(bus, options({ torqueLimit: 2000 }));
    expect(result).toEqual(RIGHT);
    for (const id of [1, 2, 3, 4, 5, 6]) expect(sim.writesTo(id, "Torque_Limit")).toEqual([1000]);
    expectReleased(sim);
  }, 60_000);

  it("on Stop mid-probe returns the arm to its start pose at speed 400, then releases torque", async () => {
    const { clock, sim, bus, start, logs, options } = arm(LEFT);
    const stop = new AbortController();
    let stoppedAt = 0;
    const run = autoCalibrateSo101(
      bus,
      options({
        signal: stop.signal,
        torqueLimit: 600,
        onProgress: (p) => {
          if (p === 0.25) void clock.sleep(1.5).then(() => ((stoppedAt = clock.now()), stop.abort()));
        },
      }),
    );
    await expect(run).rejects.toBeInstanceOf(AutoCalStopped);

    expect(logs).toContain("Arm returned to its starting position.");
    for (const [id, actual] of start) expect(wrapDist(sim.actual(id), actual), `servo ${id}`).toBeLessThanOrEqual(20);
    for (const id of [1, 2, 3, 4, 5, 6]) {
      // The freeze held at the working torque, the return ran at RETURN_POS_SPEED.
      expect(sim.writesTo(id, "Torque_Limit").at(-1)).toBe(600);
      const ret = sim.writes.filter((w) => w.id === id && w.t >= stoppedAt && w.address === 41 && w.data[5] === (RETURN_POS_SPEED & 0xff) && w.data[6] === RETURN_POS_SPEED >> 8);
      expect(ret.length, `servo ${id} return command`).toBe(1);
    }
    expectReleased(sim);
    // Nothing was persisted: the EEPROM still holds the power-on calibration.
    for (const id of [1, 2, 3, 4, 5, 6]) expect(sim.eeprom(id, "Max_Position_Limit")).toBe(4095);
  }, 60_000);

  it("on an e-stop mid-probe switches every servo off at once, with no freeze and no return", async () => {
    const { clock, sim, bus, logs, options } = arm(LEFT);
    const halt = new AbortController();
    let haltedAt = 0;
    const run = autoCalibrateSo101(
      bus,
      options({
        halt: halt.signal,
        onProgress: (p) => {
          if (p === 0.25) void clock.sleep(1.5).then(() => ((haltedAt = clock.now()), halt.abort()));
        },
      }),
    );
    await expect(run).rejects.toBeInstanceOf(AutoCalStopped);
    expect(logs).not.toContain("Stop requested: freezing the arm in place...");
    expectReleased(sim);
    const offs = sim.writes.filter((w) => w.address === 40 && w.data[0] === 0 && w.t >= haltedAt).map((w) => w.t);
    expect(Math.max(...offs) - haltedAt).toBeLessThan(0.5);
    expect(sim.writes.some((w) => w.address === 40 && w.data[0] === 1 && w.t > haltedAt)).toBe(false);
    for (const id of [1, 2, 3, 4, 5, 6]) expect(sim.eeprom(id, "Max_Position_Limit")).toBe(4095);
  }, 60_000);

  it("on Stop while a joint is overload-latched clears the latch so it returns too", async () => {
    const { clock, sim, bus, start, logs, options } = arm(LEFT);
    const stop = new AbortController();
    const latched: number[] = [];
    sim.onOverload = (id) => {
      latched.push(id);
      stop.abort();
    };
    await expect(autoCalibrateSo101(bus, options({ signal: stop.signal }))).rejects.toBeInstanceOf(AutoCalStopped);
    // shoulder_lift starts folded against a stop: its forward lift stalls and latches.
    expect(latched).toEqual([2]);
    expect(logs).toContain("Arm returned to its starting position.");
    for (const [id, actual] of start) expect(wrapDist(sim.actual(id), actual), `servo ${id}`).toBeLessThanOrEqual(20);
    expectReleased(sim);
  }, 60_000);

  it("on Stop with a joint that cannot move gives up on the return, holds 3 s, then releases", async () => {
    const { clock, sim, bus, logs, options } = arm(LEFT);
    const stop = new AbortController();
    let stoppedAt = 0;
    const run = autoCalibrateSo101(
      bus,
      options({
        signal: stop.signal,
        onProgress: (p) => {
          if (p === 0.5)
            void clock.sleep(1.5).then(() => {
              stoppedAt = clock.now();
              sim.servo(SO101_IDS.elbow_flex).requiredTorque = 2000; // jammed: no torque can move it
              stop.abort();
            });
        },
      }),
    );
    await expect(run).rejects.toBeInstanceOf(AutoCalStopped);
    expect(logs.some((l) => l.startsWith("Fallback: return stalled") || l.startsWith("Fallback: return settled short"))).toBe(true);
    expect(logs).toContain("Holding position for 3s before releasing...");
    const release = sim.writes.filter((w) => w.address === 40 && w.data[0] === 0 && w.t > stoppedAt).map((w) => w.t);
    // The freeze clears each latch with a Torque_Enable=0 first; the release comes after the 1.5 s stall window
    // and the 3 s hold.
    expect(Math.max(...release) - stoppedAt).toBeGreaterThan(1.5 + 3);
    expectReleased(sim);
  }, 60_000);

  it("rejects with a clear error and releases torque when a servo goes offline mid-run", async () => {
    const { clock, sim, bus, logs, options } = arm(LEFT);
    const run = autoCalibrateSo101(
      bus,
      options({
        onProgress: (p) => {
          if (p === 0.25) void clock.sleep(1.5).then(() => sim.setOnline(SO101_IDS.elbow_flex, false));
        },
      }),
    );
    await expect(run).rejects.toThrow(/elbow_flex/);
    expect(logs.some((l) => l.startsWith("Exception:"))).toBe(true);
    expectReleased(sim);
  }, 60_000);

  it("stops before any motion when a safety setting won't take (wrist_flex's Torque_Limit write is lost)", async () => {
    const { clock, sim, bus, options } = arm(LEFT);
    const request = bus.request.bind(bus);
    // WRITE (0x03) to servo 4 at Torque_Limit (48): never reaches the servo.
    bus.request = (packet, want, timeoutMs) => (packet[2] === 4 && packet[4] === 0x03 && packet[5] === 48 ? Promise.resolve([]) : request(packet, want, timeoutMs));
    await expect(autoCalibrateSo101(bus, options())).rejects.toThrow(/Could not set Torque_Limit=380 on wrist_flex/);
    for (const id of [1, 2, 3, 4, 5, 6]) expect(sim.writesTo(id, "Goal_Velocity").every((v) => v === 0), `servo ${id} moved`).toBe(true);
    expect(sim.writesTo(4, "Torque_Enable")).not.toContain(1);
    expectReleased(sim);
  });

  it("turns a wrist roll resting off its middle (camera on top) back to the middle before anything sweeps, and ends the run at the camera-on-top rest", async () => {
    const { clock, sim, bus, logs, options } = arm(LEFT);
    // The servo holds a calibrated wrist-roll range, and the roll rests 999 ticks below its middle (as a previous run
    // leaves it).
    const roll = sim.servo(5);
    roll.setReg("Min_Position_Limit", 117);
    roll.setReg("Max_Position_Limit", 3977);
    roll.setReg("Homing_Offset", 0);
    sim.place(5, 2047 - 999);
    roll.refresh();
    const result = await autoCalibrateSo101(bus, options());
    expect(result).toEqual(LEFT);
    const turn = logs.findIndex((l) => l.includes("Turning wrist_roll(5) 999 ticks"));
    const sweep = logs.findIndex((l) => l.startsWith("--- Unfolding wrist_flex"));
    expect(turn).toBeGreaterThanOrEqual(0);
    expect(turn).toBeLessThan(sweep);
    expect(logs.some((l) => /wrist_roll\(5\) current pos=\d+, turning to rest pos=1048/.test(l))).toBe(true);
    expectReleased(sim);
  }, 60_000);

  it("refuses an arm with a servo missing, before moving anything", async () => {
    const { clock, sim, bus, options } = arm(LEFT);
    sim.setOnline(4, false);
    await expect(autoCalibrateSo101(bus, options())).rejects.toThrow(/Missing motor IDs:\n {2}- 4 \(expected model: 777\)/);
    for (const id of [1, 2, 3, 5, 6]) expect(sim.writesTo(id, "Torque_Enable").every((v) => v === 0)).toBe(true);
  });

  it("does nothing when stopped before it starts", async () => {
    const { clock, sim, bus, options } = arm(LEFT);
    const stop = new AbortController();
    stop.abort();
    await expect(autoCalibrateSo101(bus, options({ signal: stop.signal }))).rejects.toBeInstanceOf(AutoCalStopped);
    expect(sim.requests).toEqual([]);
  });
});

describe("simulated STS3215 bus", () => {
  it("applies Homing_Offset, Torque_Enable=128, Lock and the overload latch", async () => {
    const clock = new VirtualClock();
    const sim = new SimFeetechBus(clock, [1]);
    const bus = new FeetechBus(sim.uart, msClock(clock));
    sim.setHardStops(1, 3500, 500); // across the wrap
    sim.place(1, 100);
    expect(await bus.read(1, "Present_Position")).toBe(100);
    await bus.write(1, "Homing_Offset", -200);
    expect(await bus.read(1, "Present_Position")).toBe(300);
    await bus.write(1, "Torque_Enable", 128);
    expect(await bus.read(1, "Present_Position")).toBe(2048);
    expect(await bus.read(1, "Homing_Offset")).toBe(100 - 2048);
    // Lock=1 (power-on): the offset written by hand is live but not persisted; 128's is.
    expect(sim.eeprom(1, "Homing_Offset")).toBe(100 - 2048);
    await bus.write(1, "Homing_Offset", 7);
    expect(sim.eeprom(1, "Homing_Offset")).toBe(100 - 2048);
    await bus.write(1, "Lock", 0);
    await bus.write(1, "Homing_Offset", 7);
    expect(sim.eeprom(1, "Homing_Offset")).toBe(7);
    // Velocity mode into the 500 stop: stalls there, then latches after overloadMs.
    await bus.write(1, "Operating_Mode", 1);
    await bus.write(1, "Torque_Enable", 1);
    await bus.write(1, "Goal_Velocity", 1000);
    await clock.sleep(0.6);
    expect(sim.actual(1)).toBe(500);
    expect(sim.reg(1, "Status")).toBe(0);
    await clock.sleep(1);
    expect(sim.reg(1, "Status")).toBe(0x20);
    expect(sim.servo(1).latched).toBe(true);
    sim.moveByHand(1, 450); // latched = limp
    expect(sim.actual(1)).toBe(450);
    await bus.write(1, "Torque_Enable", 0);
    expect(sim.servo(1).latched).toBe(false);
  });
});
