/** The SO-101 arm (The Robot Studio / LeRobot): five joints and a gripper on six Feetech STS3215 servos, ids 1-6, one
serial bus per arm at 1 Mbaud. Lifted from the console's hardware/so101.ts (dreamlabs 0.0.30's SO-101 rules on
LeRobot's bus):

- Units: a servo reads 0-4095 ticks a turn; LeRobot's calibration moves each servo's zero (its homing offset) so the
  arm's middle pose reads 2047, and that middle pose is dream's so101_faas URDF zero: a joint is
  `(ticks - 2048) * 360 / 4096` degrees (the 2048 against 2047 is dream's). The driver works in the console's seven
  slots: joint1..joint5 (pan, lift, elbow, wrist flex, wrist roll) in degrees, a constant-zero pad, then the gripper
  as LeRobot's 0 (range_min, closed) .. 100 (range_max, open). So101Arm turns these into radians.
- An arm's calibration must match what its servos hold (homing offset and range, servo by servo), or it does not
  run: the calibration of another arm (a swapped cable) would put every joint somewhere else. */

import { ArmError } from "@makermods/arm-core";
import type { FeetechBus } from "./bus.js";
import { FeetechError, errorText, isFault, type FeetechCalibration, type ServoState } from "./protocol.js";

/** The six servos, base to gripper (LeRobot's motor names). */
export const SO101_MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"] as const;
/** A servo's name. */
export type So101Motor = (typeof SO101_MOTORS)[number];
/** Each servo's bus id, in SO101_MOTORS order. */
export const SO101_IDS: readonly number[] = [1, 2, 3, 4, 5, 6];
/** Each servo's slot in the driver's seven (joint1..joint5, the pad at 5, the gripper at 6). */
export const SERVO_SLOTS: readonly number[] = [0, 1, 2, 3, 4, 6];
/** The constant-zero pad slot. */
export const PAD_SLOT = 5;
/** The gripper's slot. */
export const GRIPPER_SLOT = 6;

/** Encoder ticks a turn. */
export const TICKS_PER_TURN = 4096;
/** The tick that reads 0 degrees (dream's feetech.ticks_to_deg). */
export const CENTER_TICK = 2048;
/** LeRobot's half turn, where a calibration puts the middle pose (MotorsBus._get_half_turn_homings). */
export const HALF_TURN_TICK = 2047;
/** The highest tick. */
export const MAX_TICK = TICKS_PER_TURN - 1;
/** The largest homing offset a servo holds (sign-magnitude at bit 11). */
export const MAX_HOMING_OFFSET = 2047;
/** dream's grip_margin_mm (on the 0..100 scale): gripper goals stay this far inside the open end. */
export const GRIP_MARGIN = 3;
/** The gripper's torque caps, as LeRobot's SO-101 follower sets them (a gripper closed on an object stalls safely). */
export const GRIPPER_CAPS = [["Max_Torque_Limit", 500], ["Protection_Current", 250], ["Overload_Torque", 25]] as const;
/** P and D (LeRobot's SO-101 follower: its default P of 32 shakes). */
export const P_COEFFICIENT = 16;
/** D coefficient (LeRobot's SO-101 follower). */
export const D_COEFFICIENT = 32;
/** A run of failed reads this long fails the arm (feetech.MAX_MISSED_S). */
export const MAX_MISSED_MS = 300;
/** The fastest a goal moves from the last one (so101_faas max_speed, 5 rad/s), over at most MAX_COMMAND_DT_S: a goal
further than that is sent as far as that allows (the servos run Goal_Velocity 0, their top speed, toward a goal). */
export const MAX_COMMAND_DEG_S = (5 * 180) / Math.PI;
/** The longest gap the per-command step bound counts, seconds. */
export const MAX_COMMAND_DT_S = 0.05;
/** Tries per servo of a switch-off before it is reported as refused. */
export const DISABLE_RETRIES = 5;
/** Reads tried at start before the arm is refused: the first status packet after the port opens is often lost. */
export const FIRST_READ_ATTEMPTS = 5;
/** Wait between those reads. */
export const FIRST_READ_RETRY_MS = 200;

/** Ticks to degrees from the middle pose. */
export const ticksToDeg = (ticks: number) => ((ticks - CENTER_TICK) * 360) / TICKS_PER_TURN;
/** Degrees from the middle pose to the nearest tick. */
export const degToTicks = (deg: number) => Math.round((deg * TICKS_PER_TURN) / 360) + CENTER_TICK;

/** One LeRobot calibration entry per servo. */
export type So101Calibration = Record<So101Motor, FeetechCalibration>;

/** A refusal or failure of the SO-101 driver, worded for the operator. */
export class So101Error extends ArmError {}

/** A read that got no complete answer: the tick holds (nothing is commanded on stale feedback), as dream's MissedRead. */
export class MissedReadError extends FeetechError {}

/** A servo fault (its error byte or Status overload bit), or a servo that dropped its torque while energized. Latched by
the follower: that arm is never commanded again by that follower. */
export class So101FaultError extends So101Error {}

/** Torque off was not confirmed on every servo; `problems` names each one. */
export class ReleaseError extends ArmError {
  constructor(readonly problems: string[]) {
    super(problems.join("; "));
  }
}

/** A start (enable) stopped by its AbortSignal before it finished. */
export class StartCancelledError extends ArmError {}

// ---------- Calibration files ----------

/** feetech.read_calibration: every servo's entry, integers, its id, drive mode 0 and a range of ticks low to high. */
export function parseSo101Calibration(data: unknown, name = "the calibration"): So101Calibration {
  if (typeof data !== "object" || data === null || Array.isArray(data)) throw new So101Error(`${name} is not a LeRobot calibration: it is not a mapping`);
  const raw = data as Record<string, unknown>;
  const out = {} as So101Calibration;
  SO101_MOTORS.forEach((motor, i) => {
    const entry = raw[motor] as Record<string, unknown> | undefined;
    const fields = ["id", "drive_mode", "homing_offset", "range_min", "range_max"] as const;
    if (!entry || typeof entry !== "object" || !fields.every((f) => Number.isInteger(entry[f]))) {
      throw new So101Error(`${name} has no LeRobot entry for '${motor}' (integers ${fields.join(", ")}): is it an SO-101's?`);
    }
    const c = Object.fromEntries(fields.map((f) => [f, entry[f] as number])) as unknown as FeetechCalibration;
    if (c.id !== SO101_IDS[i] || c.drive_mode !== 0) {
      throw new So101Error(`${name} gives '${motor}' id ${c.id} and drive mode ${c.drive_mode}; an SO-101 has it at id ${SO101_IDS[i]}, drive mode 0`);
    }
    if (!(0 <= c.range_min && c.range_min < c.range_max && c.range_max < TICKS_PER_TURN)) {
      throw new So101Error(`${name} gives '${motor}' the range ${c.range_min} to ${c.range_max}, not a range of ticks (0 to ${MAX_TICK}, low to high)`);
    }
    out[motor] = c;
  });
  return out;
}

/** feetech.calibration_differences: `<servo> <field> <held> (file <value>)` where the file and the servos disagree. */
export function calibrationDifferences(file: So101Calibration, held: So101Calibration): string[] {
  return SO101_MOTORS.flatMap((m) =>
    (["homing_offset", "range_min", "range_max"] as const).flatMap((f) => (held[m][f] === file[m][f] ? [] : [`${m} ${f} ${held[m][f]} (file ${file[m][f]})`])),
  );
}

// ---------- Dream's reference arm (dreamlabs robots/so101_calibration.py) ----------

/** specs.so101_faas joint_units per side: rad = offset + scale * LeRobot unit (-100..100 over the reference arm's
calibrated range) for joint1..joint5. */
export const REFERENCE_UNITS: Readonly<Record<"left" | "right", { scale: readonly number[]; offset: readonly number[] }>> = {
  left: {
    scale: [0.0176867985, 0.0181546626, 0.0166973809, 0.0181086432, 0.0314082566],
    offset: [0.3006602344, 0.0268446638, -0.1158155495, 0.0283786446, 0.0479125096],
  },
  right: {
    scale: [0.0166283517, 0.0177481577, 0.0170195168, 0.0191440802, 0.0314082566],
    offset: [-0.2960582921, 0.0490873852, -0.2155243007, -0.0705631162, 0.0479125096],
  },
};
const RAD_PER_TICK = (2 * Math.PI) / TICKS_PER_TURN;
const DEG_PER_TICK = 360 / TICKS_PER_TURN;
/** so101_calibration: the joints with stops (the wrist roll turns all the way; the gripper's range is its own). */
export const REFERENCE_JOINTS: readonly So101Motor[] = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"];
/** A range this much shorter than the reference's is too short. */
export const SHORT_RANGE_DEG = 10;
/** A range this much longer than the reference's keeps its own zero. */
export const LONG_RANGE_DEG = 10;
/** The furthest a joint's zero is moved to the reference's. */
export const MAX_SHIFT_DEG = 25;
/** The shortest range of a joint with no reference span. */
export const MIN_RANGE_DEG = 30;

/** so101_calibration.reference: the reference arm's range per joint with stops, ticks of its own calibration. */
export function referenceRanges(side: "left" | "right"): Record<string, { low: number; high: number; middle: number; spanDeg: number }> {
  const { scale, offset } = REFERENCE_UNITS[side];
  return Object.fromEntries(
    REFERENCE_JOINTS.map((motor) => {
      const i = SO101_MOTORS.indexOf(motor);
      const middle = HALF_TURN_TICK + (offset[i] as number) / RAD_PER_TICK;
      const half = (100 * (scale[i] as number)) / RAD_PER_TICK;
      return [motor, { low: middle - half, high: middle + half, middle, spanDeg: 2 * half * DEG_PER_TICK }];
    }),
  );
}

/** What the operator calls each servo's joint (dreamlabs setup_wizard.js JOINT_WORDS). */
export const JOINT_WORDS: Record<string, string> = {
  shoulder_pan: "base turn",
  shoulder_lift: "shoulder",
  elbow_flex: "elbow",
  wrist_flex: "wrist bend",
  wrist_roll: "wrist roll",
  gripper: "gripper",
};

/** alignToReference's result. */
export interface AlignedCalibration {
  calibration: So101Calibration;
  /** Per joint whose zero was moved to the reference's: how far the recorded middle was from it, degrees. */
  movedDeg: Record<string, number>;
  notes: string[];
}

/** ArmCalibration.finish's alignment: each joint with stops has its zero moved by how far the middle of its recorded
range is from the reference arm's, when the two ranges are the same length and the move is at most MAX_SHIFT_DEG
(more throws: the middle pose was set too far off). Present = actual - homing, so raising the homing by the shift
moves every reading down by it. The wrist roll and gripper are kept as recorded. */
export function alignToReference(side: "left" | "right", entries: So101Calibration, words: Record<string, string> = JOINT_WORDS): AlignedCalibration {
  const ref = referenceRanges(side);
  const movedDeg: Record<string, number> = {};
  const notes: string[] = [];
  const calibration = {} as So101Calibration;
  for (const motor of SO101_MOTORS) {
    let { homing_offset: homing, range_min: low, range_max: high } = entries[motor];
    const r = ref[motor];
    if (r) {
      const span = (high - low) * DEG_PER_TICK;
      const shift = Math.round((low + high) / 2 - r.middle);
      if (span > r.spanDeg + LONG_RANGE_DEG) {
        notes.push(`${words[motor]} turns ${span.toFixed(0)} degrees end to end, more than an SO-101's ${r.spanDeg.toFixed(0)}: its zero is where you set it, not matched to the model's arm`);
      } else if (Math.abs(shift) * DEG_PER_TICK > MAX_SHIFT_DEG) {
        throw new So101Error(`${words[motor]} was ${(Math.abs(shift) * DEG_PER_TICK).toFixed(0)} degrees from its middle when the middle pose was set: put the arm in the middle pose shown and calibrate again`);
      } else if (shift) {
        homing += shift;
        low -= shift;
        high -= shift;
        movedDeg[motor] = Math.round(-shift * DEG_PER_TICK * 10) / 10;
      }
    }
    if (Math.abs(homing) > MAX_HOMING_OFFSET || !(0 <= low && low < high && high <= MAX_TICK)) {
      throw new So101Error(`${words[motor]} cannot hold this calibration (homing offset ${homing}, range ${low} to ${high}): put the arm in the middle pose shown and calibrate again`);
    }
    calibration[motor] = { ...entries[motor], homing_offset: homing, range_min: low, range_max: high };
  }
  return { calibration, movedDeg, notes };
}

/** The joints whose range is too short to be end to end: within SHORT_RANGE_DEG of a reference span, else
MIN_RANGE_DEG; the wrist roll is exempt. `"side"`: this side's reference arm (a range moved by hand into the stops).
`"either"`: the shorter of the two sides' (an automatic range ends where a torque-limited probe stalls, a little
before the stop). Words, base to gripper. */
export function shortJoints(calibration: So101Calibration, side: "left" | "right", reference: "side" | "either" | null): string[] {
  const own = referenceRanges(side);
  const other = referenceRanges(side === "left" ? "right" : "left");
  return SO101_MOTORS.flatMap((motor) => {
    if (motor === "wrist_roll") return [];
    const { range_min: low, range_max: high } = calibration[motor];
    const r = own[motor];
    const span = r && reference ? (reference === "either" ? Math.min(r.spanDeg, other[motor]!.spanDeg) : r.spanDeg) : null;
    const enough = span === null ? MIN_RANGE_DEG : span - SHORT_RANGE_DEG;
    return high <= low || (high - low) * DEG_PER_TICK < enough ? [JOINT_WORDS[motor]!] : [];
  });
}

// ---------- Bus helpers ----------

/** Every servo's held calibration (EEPROM Homing_Offset and position limits), three sync reads; a servo that did
not answer is named. drive_mode is 0 (LeRobot's SO-101 has no other). */
export async function readHeldCalibration(bus: FeetechBus): Promise<So101Calibration> {
  const homing = await bus.syncRead(SO101_IDS, "Homing_Offset");
  const lo = await bus.syncRead(SO101_IDS, "Min_Position_Limit");
  const hi = await bus.syncRead(SO101_IDS, "Max_Position_Limit");
  const out = {} as So101Calibration;
  const missing: string[] = [];
  SO101_MOTORS.forEach((motor, i) => {
    const id = SO101_IDS[i] as number;
    const [h, a, b] = [homing.get(id), lo.get(id), hi.get(id)];
    if (h === undefined || a === undefined || b === undefined) missing.push(`servo ${id} (${motor})`);
    else out[motor] = { id, drive_mode: 0, homing_offset: h, range_min: a, range_max: b };
  });
  if (missing.length) throw new So101Error(`${missing.join(" and ")} did not answer: reseat that servo's cable at both ends and check the arm's power supply`);
  return out;
}

/** Every servo's state in one sync read, in servo order; retried once. `inspect` sees every batch as it arrives,
complete or not (so a fault one servo reports is never lost to another's missed reply). MissedReadError names the
servos that did not answer. */
export async function readStates(bus: FeetechBus, inspect?: (states: ReadonlyMap<number, ServoState>) => void): Promise<ServoState[]> {
  for (let attempt = 0; ; attempt++) {
    const read = await bus.syncReadState(SO101_IDS);
    inspect?.(read);
    const missing = SO101_IDS.filter((id) => !read.has(id));
    if (!missing.length) return SO101_IDS.map((id) => read.get(id) as ServoState);
    if (attempt >= 1) throw new MissedReadError(`No reply from servo ${missing.join(", ")}`);
  }
}

/** `<servo> (<faults>)` for each servo reporting a fault, empty when none does. */
export function faultsOf(states: readonly ServoState[]): string[] {
  return states.flatMap((st, i) => (isFault(st) ? [`${SO101_MOTORS[i]} (${errorText(st.error || 0x20)})`] : []));
}

/** Raw ticks of the six servos to the driver's seven (degrees, the pad 0, the gripper 0..100). */
export function ticksToVector(ticks: readonly number[], calibration: So101Calibration): number[] {
  const q = new Array<number>(7).fill(0);
  SO101_MOTORS.forEach((motor, i) => {
    const t = ticks[i] as number;
    const c = calibration[motor];
    q[SERVO_SLOTS[i] as number] = motor === "gripper" ? ((t - c.range_min) / (c.range_max - c.range_min)) * 100 : ticksToDeg(t);
  });
  return q;
}

/** The driver's seven to goal ticks, each clamped to its servo's calibrated range (the firmware clamps silently to the
EEPROM limits anyway); the gripper stays GRIP_MARGIN inside its open end. With `bounds` (per slot, degrees) a joint
within its bound is rounded to a tick inside it rather than to the nearest: 89.0° is 1012.6 ticks, and the nearest,
1013, reads 89.03°, past a limit of 89.0°. */
export function vectorToTicks(q: readonly number[], calibration: So101Calibration, bounds?: ReadonlyArray<readonly [number, number]>): number[] {
  return SO101_MOTORS.map((motor, i) => {
    const c = calibration[motor];
    const slot = SERVO_SLOTS[i] as number;
    const v = q[slot] as number;
    if (motor === "gripper") {
      const open = Math.min(100 - GRIP_MARGIN, Math.max(0, v));
      return Math.round(c.range_min + (open / 100) * (c.range_max - c.range_min));
    }
    let t = degToTicks(v);
    const bound = bounds?.[slot];
    if (bound && v <= bound[1] && ticksToDeg(t) > bound[1] && ticksToDeg(t - 1) >= bound[0]) t -= 1;
    else if (bound && v >= bound[0] && ticksToDeg(t) < bound[0] && ticksToDeg(t + 1) <= bound[1]) t += 1;
    return Math.min(c.range_max, Math.max(c.range_min, t));
  });
}

/** LeRobot's disable_torque for one servo (Torque_Enable 0, then Lock 0), confirmed by reading Torque_Enable back.
The writes' error bytes don't stop it (a servo in overload flags every reply, yet takes the write); the read-back
says whether it took. True when the servo reads off. */
export async function switchOff(bus: FeetechBus, id: number): Promise<boolean> {
  let error: unknown = null;
  for (const [reg, value] of [["Torque_Enable", 0], ["Lock", 0]] as const) {
    try {
      await bus.write(id, reg, value);
    } catch (exc) {
      error ??= exc;
    }
  }
  const torque = (await bus.syncRead([id], "Torque_Enable")).get(id);
  if (torque === 0) return true;
  if (torque === undefined && error) throw error;
  return false;
}

/** LeRobot's disable_torque on every servo, each retried DISABLE_RETRIES times and confirmed by reading Torque_Enable
back; one line per servo that did not confirm it (empty when all are off). The caller holds the arbiter. */
export async function disableAllServos(bus: FeetechBus): Promise<string[]> {
  const failed: string[] = [];
  for (const [i, id] of SO101_IDS.entries()) {
    let ok = false;
    let last = "";
    for (let attempt = 0; attempt < DISABLE_RETRIES && !ok; attempt++) {
      ok = await switchOff(bus, id).catch((exc: unknown) => ((last = exc instanceof Error ? exc.message : String(exc)), false));
      if (!ok && !last) last = "it still reads torque on";
    }
    if (!ok) failed.push(`${SO101_MOTORS[i]} (id ${id}): it did not confirm the switch-off (${last})`);
  }
  return failed;
}

/** Lock 0 (the EEPROM area keeps what is written next), confirmed by reading Lock back; false when it won't take. */
export async function unlockEeprom(bus: FeetechBus, id: number, attempts = 3): Promise<boolean> {
  for (let attempt = 0; attempt < attempts; attempt++) {
    await bus.write(id, "Lock", 0).catch(() => undefined);
    if ((await bus.syncRead([id], "Lock")).get(id) === 0) return true;
  }
  return false;
}

/** A faulted servo's status code: FAULT_STATUS | its error byte (so101StatusLabel names the faults). */
export const FAULT_STATUS = 0x100;

/** A servo's status code: 1 torque on, 0 off, FAULT_STATUS | error bits a faulted one. */
export const statusCode = (st: ServoState) => (isFault(st) ? FAULT_STATUS | (st.error || 0x20) : st.torque ? 1 : 0);

/** A status code in words. */
export const so101StatusLabel = (code: number) =>
  code === 0 ? "torque off" : code === 1 ? "torque on" : code & FAULT_STATUS ? `fault: ${errorText(code & 0xff)}` : `status ${code}`;
