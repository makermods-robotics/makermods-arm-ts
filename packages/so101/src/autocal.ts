/** SO-101 motorized auto-calibration (Feetech STS3215, follower and leader alike): MakerLab's vendored
lerobot-MakerMods procedure, makermodslab/vendor/feetech_autocal/auto_calibrate_script.py run_full_calibration over
FeetechCalibrationMixin (auto_calibration.py), ported step for step: the same order, constants (calibration_defaults.py),
register writes and read-backs, stall rules, range/homing math and stop handling. Python names are kept on the
helpers so each can be read side by side with its original (file:line references below are to those two files,
"script" and "mixin").

The procedure, in script :814-933 order: clear residual state and handshake (_connect_and_clear), capture the start
pose, Stage 0 init (Lock=1, PID, acceleration, working Torque_Limit, Homing_Offset 0, limits 0..4095, torque on), lift
wrist_flex 80°, shoulder_lift 15° and elbow_flex 30° (recording 2/3's reference poses), centre 2/3, probe 2+3 together
(first direction opposite their lift), fold them, raise elbow 80°, probe wrist_roll+gripper+wrist_flex, fold elbow /
unfold wrist / close gripper, probe shoulder_pan alone, fold, Lock=0 + position mode, write_calibration, release.
A probe drives a joint in velocity mode against each hard stop; the homing offset centres the measured arc on 2047
(range = 2047 ± arc/2), the arc being the one that contains a reference pose, so a range across the encoder wrap
comes out right.

Stop (the AbortSignal) is the script's SIGTERM → KeyboardInterrupt path (_run_with_bus, _graceful_stop): freeze every
joint where it is (clearing an overload latch), drive back to the start pose at speed 400 for as long as the return
keeps progressing, hold 3 s if it could not, then release torque; the promise rejects with AutoCalStopped. Any other
error releases torque and rejects with it. Python checks for the interrupt between any two bytecodes; here it is
checked before every bus exchange and on every sleep (which the abort cuts short).

Deviations, all deliberate:
- Non-interactive (MakerLab runs the script without a TTY: main() :1012), so init warnings never wait for Enter;
  --save is always on, minus the file write (the caller owns persistence): the result is returned instead.
- The returned object is keyed in SO101_MOTORS order (lerobot's motor order); the script's file lists the motors in
  CALIBRATE_REST + CALIBRATE_FIRST order. The EEPROM writes keep the script's order.
- _wait_for_stall_multi polls in motor order (Python iterates a set: hash order).
- A second Stop during the stop sequence ("stop NOW", script :308) has no equivalent: an AbortSignal fires once.
- A Stop during the initial clear/handshake (outside the script's try) disables torque on every servo and rejects
  with AutoCalStopped; Python would die with the traceback (torque was already off).
- The overload handling depends on lerobot raising on a reply's error byte (a latched servo NAKs Torque_Enable=1,
  reads fail while latched): this module does its single-servo exchanges through FeetechBus.request (the raw
  exchange, error bytes unchecked) and raises ServoFaultError on a nonzero one, with lerobot's messages. */

import type { Clock } from "@makermods/arm-core";
import { ArmError } from "@makermods/arm-core";
import type { FeetechBus } from "./bus.js";
import { msClock, type MsClock } from "./clock-ms.js";
import {
  FeetechError,
  INST_REG_WRITE,
  REG,
  bytesOf,
  decodeValue,
  encodeValue,
  errorText,
  packet,
  pingPacket,
  readPacket,
  valueOf,
  writePacket,
  type RegName,
  type StatusPacket,
} from "./protocol.js";

// ---------- calibration_defaults.py ----------

export const SO101_MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"] as const;
export type So101Motor = (typeof SO101_MOTORS)[number];
/** SO_MOTOR_NUMBERS: ids 1..6 in SO101_MOTORS order. */
export const SO101_IDS: Record<So101Motor, number> = { shoulder_pan: 1, shoulder_lift: 2, elbow_flex: 3, wrist_flex: 4, wrist_roll: 5, gripper: 6 };
/** model_number_table["sts3215"]. */
const STS3215_MODEL_NUMBER = 777;

export const FULL_TURN = 4096;
export const MID_POS = 2047;
export const STS_HALF_TURN_RAW = 2047;
/** Homing_Offset is sign-magnitude at bit 11: ±2047. */
export const HOMING_OFFSET_MAX_MAG = 2047;
export const DEFAULT_VELOCITY_LIMIT = 1000;
export const DEFAULT_MAX_TORQUE = 1000;
export const DEFAULT_TORQUE_LIMIT = 380;
export const DEFAULT_ACCELERATION = 50;
export const DEFAULT_POS_SPEED = 1000;
export const DEFAULT_P_COEFFICIENT = 16;
export const DEFAULT_I_COEFFICIENT = 0;
export const DEFAULT_D_COEFFICIENT = 32;
/** Per-direction probe timeout, also go_to_mid's. */
export const DEFAULT_TIMEOUT_MS = 20_000;
/** Where the wrist roll rests at the end of the final fold (turnWristToRest), from the middle of its calibrated range:
the right arm's wrist turned by hand (2026-10-19) so the wrist camera sits on top of the gripper, not below it:
calibrated 1048, about 88° counterclockwise of the middle (the servo's negative). */
export const WRIST_REST_FROM_MIDDLE_TICKS = -999;
export const WRIST_REST_MARGIN_TICKS = 30;
const MAX_TICK = 4095;
export const WRIST_REST_SPEED = 400;
/** Tries per init setting before the run stops (the console's addition: see runInit). */
export const INIT_ATTEMPTS = 3;
export const POSITION_TOLERANCE = 20;
export const STALL_VELOCITY_THRESHOLD = 3;
export const STALL_POSITION_DELTA_THRESHOLD = 3;
export const OVERLOAD_SETTLE_MS = 200;
const SAFE_IO_RETRIES = 5;
const SAFE_IO_INTERVAL_MS = 200;
export const DEFAULT_UNFOLD_TIMEOUT_MS = 6000;
const UNFOLD_OVERLOAD_SETTLE_MS = 300;
const UNFOLD_TOLERANCE_DEG = 5.0;
const CALIBRATE_FIRST: So101Motor[] = ["shoulder_pan"];
const CALIBRATE_REST: So101Motor[] = ["wrist_roll", "gripper", "wrist_flex", "elbow_flex", "shoulder_lift"];

/** measure_ranges_of_motion(_multi) defaults (mixin :554-558, :618-622). */
const STALL_CONFIRM_SAMPLES = 2;
const SAMPLE_INTERVAL_MS = 50;
const INITIAL_MOVE_DELAY_MS = 500;
/** _compute_mid_and_range_from_limits move_timeout (mixin :287). */
const BACK_OFF_TIMEOUT_MS = 5000;

// ---------- script stop constants (:99-108) ----------

const RETURN_TO_REST_BUDGET_MS = 10_000;
export const RETURN_POS_SPEED = 400;
const RETURN_SETTLE_MS = 300;
const RETURN_POLL_MS = 50;
const RETURN_STALL_WINDOW_MS = 1500;
const RETURN_STALL_MIN_PROGRESS = 10;
export const STOP_HOLD_MS = 3000;

/** Write_pos_ex: Acceleration(41) + Goal_Position(42) + Goal_Time(44) + Goal_Velocity(46), 7 bytes (mixin :54). */
const POS_EX_START_ADDR = 41;
/** Min_Position_Limit(9) + Max_Position_Limit(11), 4 bytes (mixin :57). */
const POS_LIMITS_START_ADDR = 9;

// ---------- API ----------

export interface AutoCalOptions {
  /** The clock everything waits on (seconds; the procedure counts milliseconds on it). */
  clock: Clock;
  /** Abort = the operator pressed Stop: freeze, return to the start pose, release, then reject with AutoCalStopped. */
  signal?: AbortSignal;
  /** Abort = an e-stop: no freeze and no return, every servo's torque off at once (the arm drops), then reject with
  AutoCalStopped. It also cuts a Stop's return short. */
  halt?: AbortSignal;
  /** Torque_Limit while driving the arm, default 380, clamped 100..1000 (script main() :1006). */
  torqueLimit?: number;
  /** 0..1 and a short description, at each step. */
  onProgress?: (progress: number, message: string) => void;
  log?: (message: string) => void;
}

export interface MotorCalibration {
  id: number;
  drive_mode: number;
  homing_offset: number;
  range_min: number;
  range_max: number;
}

export class AutoCalStopped extends ArmError {
  constructor(message = "Auto-calibration stopped") {
    super(message);
  }
}

/** lerobot's RuntimeError: a status reply with a nonzero error byte (e.g. overload), or retries exhausted. Like
FeetechError (lerobot's ConnectionError, no reply) it is a COMM_ERR. */
export class ServoFaultError extends FeetechError {}

/** The interrupt (KeyboardInterrupt): not a COMM_ERR, not an Exception, so only the top level catches it. */
class StopRequested extends Error {
  override name = "StopRequested";
}

const isCommErr = (e: unknown): e is FeetechError => e instanceof FeetechError;

/** Torque_Limit for a working torque request: clamp(value, 100, DEFAULT_MAX_TORQUE) (script main() :1006); MakerLab's
motor_power percent maps to percent × 10 (motor_power.torque_limit_from_percent). */
export function clampTorqueLimit(value = DEFAULT_TORQUE_LIMIT): number {
  return Math.max(100, Math.min(DEFAULT_MAX_TORQUE, Math.trunc(value)));
}

const mod = (a: number, n: number) => ((a % n) + n) % n;
const label = (m: So101Motor) => `${m}(${SO101_IDS[m]})`;

export interface LimitsResult {
  rangeMin: number;
  rangeMax: number;
  mid: number;
  rawMin: number;
  rawMax: number;
  homingOffset: number;
}

/** The arc math of _compute_mid_and_range_from_limits (mixin :320-345), given the reference pose `startPos`
((Present_Position + Homing_Offset) % FULL_TURN somewhere inside the free arc). CW = the positive-velocity stall
position, CCW = the negative one; the free arc is the CCW→CW arc when it contains the reference, else CW→CCW. */
export function rangeFromLimits(posCw: number, posCcw: number, startPos: number): LimitsResult {
  const arcCcwToCw = mod(posCw - posCcw, FULL_TURN);
  const arcCwToCcw = mod(posCcw - posCw, FULL_TURN);
  const startInArcA = mod(startPos - posCcw, FULL_TURN) <= arcCcwToCw;
  let physicalRange: number;
  let mid: number;
  if (startInArcA) {
    physicalRange = arcCcwToCw;
    mid = mod(posCcw + Math.floor(physicalRange / 2), FULL_TURN);
  } else {
    physicalRange = arcCwToCcw;
    mid = mod(posCw + Math.floor(physicalRange / 2), FULL_TURN);
  }
  const homingOffset = Math.max(-HOMING_OFFSET_MAX_MAG, Math.min(HOMING_OFFSET_MAX_MAG, mid - MID_POS));
  const half = Math.floor(physicalRange / 2);
  return {
    rangeMin: Math.max(0, Math.min(FULL_TURN - 1, MID_POS - half)),
    rangeMax: Math.max(0, Math.min(FULL_TURN - 1, MID_POS + half)),
    mid,
    rawMin: Math.min(posCw, posCcw),
    rawMax: Math.max(posCw, posCcw),
    homingOffset,
  };
}

/** _nearest_wrap_target (script :185): the copy of `base` (mod FULL_TURN) nearest `present`, clamped into the limits. */
export function nearestWrapTarget(base: number, present: number, minLimit: number, maxLimit: number): number {
  let target = base - FULL_TURN;
  for (const c of [base, base + FULL_TURN]) if (Math.abs(c - present) < Math.abs(target - present)) target = c;
  return Math.max(minLimit, Math.min(maxLimit, target));
}

type Exchange = (request: Uint8Array, want: number) => Promise<StatusPacket[]>;

/** FeetechBus.request: one request and its replies (echo filtering and reply window included), error bytes as sent. */
const exchangeOf = (bus: FeetechBus): Exchange => bus.request.bind(bus);

type PosEx = [position: number, speed: number, acc: number];
type Direction = "forward" | "reverse";

/** One run: the bus, the clock and the interrupt, and the mixin + script helpers as methods. */
class AutoCal {
  /** Set once the stop sequence (or the error cleanup) runs: no more interrupt checks, sleeps run in full. */
  stopping = false;
  /** Set for the final release: nothing interrupts it. */
  releasing = false;
  private readonly exchange: Exchange;
  private readonly clock: MsClock;
  private readonly signal: AbortSignal | undefined;
  readonly halt: AbortSignal | undefined;
  readonly torqueLimit: number;
  readonly log: (message: string) => void;
  private readonly progressCb: ((progress: number, message: string) => void) | undefined;

  constructor(
    private readonly bus: FeetechBus,
    options: AutoCalOptions,
  ) {
    this.exchange = exchangeOf(bus);
    this.clock = msClock(options.clock);
    this.signal = options.signal;
    this.halt = options.halt;
    this.torqueLimit = clampTorqueLimit(options.torqueLimit);
    this.log = options.log ?? (() => undefined);
    this.progressCb = options.onProgress;
  }

  progress(fraction: number, message: string): void {
    this.check();
    this.log(message);
    this.progressCb?.(fraction, message);
  }

  check(): void {
    if (!this.releasing && this.halt?.aborted) throw new StopRequested("E-stop");
    if (!this.stopping && this.signal?.aborted) throw new StopRequested("Stop requested");
  }

  /** What cuts a sleep short: the halt until the release, and Stop until the stop sequence. */
  private sleepSignal(): AbortSignal | undefined {
    const signals = [this.releasing ? undefined : this.halt, this.stopping ? undefined : this.signal].filter((x): x is AbortSignal => x !== undefined);
    if (signals.length < 2) return signals[0];
    const both = new AbortController();
    for (const sig of signals) {
      if (sig.aborted) both.abort();
      else sig.addEventListener("abort", () => both.abort(), { once: true });
    }
    return both.signal;
  }

  async sleep(ms: number): Promise<void> {
    this.check();
    await this.clock.sleep(ms, this.sleepSignal());
    this.check();
  }

  now(): number {
    return this.clock.now();
  }

  // ---------- lerobot MotorsBus read/write (raise ConnectionError on no reply, RuntimeError on an error byte) ----------

  private async status(m: So101Motor, request: Uint8Array, what: string, numRetry = 0): Promise<StatusPacket> {
    const id = SO101_IDS[m];
    let reply: StatusPacket | undefined;
    for (let n = 0; n < 1 + numRetry && !reply; n++) {
      this.check();
      reply = (await this.exchange(request, 1)).find((p) => p.id === id);
    }
    if (!reply) throw new FeetechError(`No reply from ${label(m)} ${what}`);
    if (reply.error) throw new ServoFaultError(`${label(m)} ${what}: ${errorText(reply.error)}`);
    return reply;
  }

  async read(m: So101Motor, reg: RegName): Promise<number> {
    const [address, length] = REG[reg];
    const reply = await this.status(m, readPacket(SO101_IDS[m], address, length), `reading ${reg}`);
    if (reply.params.length < length) throw new FeetechError(`Short reply from ${label(m)} reading ${reg}`);
    return decodeValue(reg, valueOf(reply.params, 0, length));
  }

  async write(m: So101Motor, reg: RegName, value: number, numRetry = 0): Promise<void> {
    const [address, length] = REG[reg];
    await this.status(m, writePacket(SO101_IDS[m], address, bytesOf(encodeValue(reg, value), length)), `writing ${reg}=${value}`, numRetry);
  }

  /** _write_raw_bytes (mixin :710): writeTxRx of raw bytes, raising like write. */
  async writeRaw(m: So101Motor, address: number, data: number[], what: string): Promise<void> {
    await this.status(m, writePacket(SO101_IDS[m], address, data), what);
  }

  async syncWrite(reg: RegName, values: ReadonlyArray<readonly [So101Motor, number]>): Promise<void> {
    this.check();
    await this.bus.syncWrite(reg, values.map(([m, v]) => [SO101_IDS[m], v] as const));
  }

  /** lerobot ping: the model number, or null on no reply / an error byte (motors_bus.ping). */
  async ping(m: So101Motor): Promise<number | null> {
    try {
      await this.status(m, pingPacket(SO101_IDS[m]), "to ping");
      return await this.read(m, "Model_Number");
    } catch (e) {
      if (isCommErr(e)) return null;
      throw e;
    }
  }

  /** enable_torque / disable_torque (lerobot feetech.py :291-305): Torque_Enable then Lock. */
  async enableTorque(m: So101Motor): Promise<void> {
    await this.write(m, "Torque_Enable", 1);
    await this.write(m, "Lock", 1);
  }

  async disableTorque(m: So101Motor, numRetry = 0): Promise<void> {
    await this.write(m, "Torque_Enable", 0, numRetry);
    await this.write(m, "Lock", 0, numRetry);
  }

  // ---------- mixin I/O helpers ----------

  /** _read_with_retry (mixin :364). */
  async readWithRetry(m: So101Motor, reg: RegName, retries = 5, intervalMs = 200): Promise<number> {
    for (let i = 0; ; i++) {
      try {
        return await this.read(m, reg);
      } catch (e) {
        if (!isCommErr(e)) throw e;
        if (i < retries - 1) {
          await this.sleep(intervalMs);
          continue;
        }
        throw new ServoFaultError(`_read_with_retry: failed all ${retries} attempts ${reg} on ${m}: ${e.message}`);
      }
    }
  }

  /** safe_write (mixin :391). */
  async safeWrite(m: So101Motor, reg: RegName, value: number): Promise<void> {
    for (let i = 0; ; i++) {
      try {
        return await this.write(m, reg, value);
      } catch (e) {
        if (!isCommErr(e)) throw e;
        if (i < SAFE_IO_RETRIES - 1) {
          await this.sleep(SAFE_IO_INTERVAL_MS);
          continue;
        }
        throw new ServoFaultError(`safe_write: failed all ${SAFE_IO_RETRIES} attempts ${reg}=${value} on ${m}: ${e.message}`);
      }
    }
  }

  /** _write_torque_with_recovery (mixin :416): an error byte (overload) → disable to clear it, wait, retry; no
  reply → wait, retry; RuntimeError once the retries are spent. */
  async writeTorqueWithRecovery(m: So101Motor, value: number, retries = 3, intervalMs = 500): Promise<void> {
    for (let attempt = 0; attempt < retries; attempt++) {
      try {
        return await this.write(m, "Torque_Enable", value);
      } catch (e) {
        if (!isCommErr(e)) throw e;
        if (attempt === retries - 1) throw new ServoFaultError(`_write_torque_with_recovery: failed all ${retries} attempts Torque_Enable=${value} on ${m}: ${e.message}`);
        if (e instanceof ServoFaultError) {
          this.log(`  [${label(m)}] Torque_Enable=${value} Overload, clearing and retrying (${attempt + 1}/${retries}): ${e.message}`);
          try {
            await this.write(m, "Torque_Enable", 0);
          } catch (e2) {
            if (!isCommErr(e2)) throw e2;
          }
        } else {
          this.log(`  [${label(m)}] Torque_Enable=${value} no response, waiting and retrying (${attempt + 1}/${retries}): ${e.message}`);
        }
        await this.sleep(intervalMs);
      }
    }
  }

  /** Up to five tries of one write, failures swallowed (the `for _ in range(5): try … break except: sleep(0.1)` idiom). */
  private async tryFive(op: () => Promise<void>): Promise<void> {
    for (let i = 0; i < 5; i++) {
      try {
        return await op();
      } catch (e) {
        if (!isCommErr(e)) throw e;
        await this.sleep(100);
      }
    }
  }

  /** _clear_and_enable_torque (mixin :458). */
  async clearAndEnableTorque(m: So101Motor, settleMs = OVERLOAD_SETTLE_MS): Promise<void> {
    await this.tryFive(() => this.write(m, "Torque_Enable", 0));
    await this.sleep(settleMs);
    await this.writeTorqueWithRecovery(m, 1);
    await this.suppress(() => this.write(m, "Lock", 1));
  }

  /** contextlib.suppress(COMM_ERR). */
  async suppress(op: () => Promise<unknown>): Promise<void> {
    try {
      await op();
    } catch (e) {
      if (!isCommErr(e)) throw e;
    }
  }

  /** write_position_limits (mixin :738): one 4-byte write at 9. */
  async writePositionLimits(m: So101Motor, rmin: number, rmax: number): Promise<void> {
    await this.writeRaw(m, POS_LIMITS_START_ADDR, [...bytesOf(rmin, 2), ...bytesOf(rmax, 2)], `write_position_limits(id=${SO101_IDS[m]}, rmin=${rmin}, rmax=${rmax})`);
  }

  /** safe_write_position_limits (mixin :478). */
  async safeWritePositionLimits(m: So101Motor, rmin: number, rmax: number): Promise<void> {
    for (let i = 0; ; i++) {
      try {
        return await this.writePositionLimits(m, rmin, rmax);
      } catch (e) {
        if (!isCommErr(e)) throw e;
        if (i < SAFE_IO_RETRIES - 1) {
          await this.sleep(SAFE_IO_INTERVAL_MS);
          continue;
        }
        throw new ServoFaultError(`safe_write_position_limits: failed all ${SAFE_IO_RETRIES} attempts rmin=${rmin} rmax=${rmax} on ${m}: ${e.message}`);
      }
    }
  }

  async readPositionLimits(m: So101Motor): Promise<[number, number]> {
    return [await this.read(m, "Min_Position_Limit"), await this.read(m, "Max_Position_Limit")];
  }

  /** safe_disable_all (mixin :502): Torque_Enable=0 on every servo, three tries each, failures ignored. */
  async safeDisableAll(): Promise<void> {
    for (const m of SO101_MOTORS) {
      for (let i = 0; i < 3; i++) {
        try {
          await this.write(m, "Torque_Enable", 0);
          break;
        } catch (e) {
          if (!isCommErr(e)) throw e;
          await this.sleep(100);
        }
      }
    }
  }

  /** bus.disconnect() (motors_bus.py :547): disable_torque(num_retry=5) motor by motor; the first failure ends the
  loop and is reported, not raised (script _run_with_bus finally, :701-708). */
  async disconnect(): Promise<void> {
    try {
      for (const m of SO101_MOTORS) await this.disableTorque(m, 5);
    } catch (e) {
      if (e instanceof StopRequested) throw e;
      this.log(`Disconnect failed: ${(e as Error).message}`);
    }
  }

  /** _encode_sign + _split_into_byte_chunks of WritePosEx's 7 bytes. */
  private posExData(position: number, speed: number, acc: number): number[] {
    return [acc, ...bytesOf(encodeValue("Goal_Position", position), 2), 0, 0, ...bytesOf(encodeValue("Goal_Velocity", speed), 2)];
  }

  /** wait_until_stopped (mixin :768). */
  async waitUntilStopped(m: So101Motor, timeoutMs = 10_000, pollMs = 50): Promise<boolean> {
    const t0 = this.now();
    while (this.now() - t0 < timeoutMs) {
      let moving: number;
      try {
        moving = await this.read(m, "Moving");
      } catch (e) {
        if (!isCommErr(e)) throw e;
        await this.sleep(pollMs);
        continue;
      }
      if (moving === 0) return true;
      await this.sleep(pollMs);
    }
    return false;
  }

  /** write_pos_ex_and_wait (mixin :792): servo mode, one 7-byte write at 41, then poll Moving. */
  async writePosExAndWait(m: So101Motor, position: number, speed: number, acc: number, timeoutMs = 10_000, pollMs = 50): Promise<boolean> {
    try {
      await this.write(m, "Operating_Mode", 0);
      await this.sleep(50);
      await this.writeRaw(m, POS_EX_START_ADDR, this.posExData(position, speed, acc), `write_pos_ex_and_wait(id=${SO101_IDS[m]}, pos=${position}, speed=${speed}, acc=${acc})`);
      await this.sleep(300);
    } catch (e) {
      if (isCommErr(e)) return false;
      throw e;
    }
    const result = await this.waitUntilStopped(m, timeoutMs, pollMs);
    await this.sleep(100);
    return result;
  }

  /** sync_write_pos_ex (mixin :841): REG_WRITE each servo's 7 bytes, then one broadcast ACTION. A failed REG_WRITE
  is only logged, as in Python. */
  async syncWritePosEx(values: ReadonlyMap<So101Motor, PosEx>): Promise<void> {
    for (const [m, [position, speed, acc]] of values) {
      this.check();
      const id = SO101_IDS[m];
      const reply = (await this.exchange(packet(id, INST_REG_WRITE, [POS_EX_START_ADDR, ...this.posExData(position, speed, acc)]), 1)).find((p) => p.id === id);
      if (!reply) this.log(`sync_write_pos_ex RegWrite id=${id}: no status packet`);
      else if (reply.error) this.log(`sync_write_pos_ex RegWrite id=${id}: ${errorText(reply.error)}`);
    }
    this.check();
    await this.bus.action();
  }

  /** go_to_mid (mixin :519). */
  async goToMid(m: So101Motor, timeoutMs = DEFAULT_TIMEOUT_MS): Promise<boolean> {
    const ok = await this.writePosExAndWait(m, MID_POS, DEFAULT_POS_SPEED, DEFAULT_ACCELERATION, timeoutMs, 50);
    let cur = -1;
    await this.suppress(async () => (cur = await this.read(m, "Present_Position")));
    if (!ok) this.log(`${label(m)} centering timed out (${(timeoutMs / 1000).toFixed(1)}s), current position=${cur}`);
    return ok;
  }

  // ---------- unfold (mixin :885-1006) ----------

  /** _clear_overload_unfold (mixin :935). */
  async clearOverloadUnfold(m: So101Motor): Promise<void> {
    await this.suppress(async () => {
      await this.write(m, "Torque_Enable", 0);
      await this.sleep(UNFOLD_OVERLOAD_SETTLE_MS + 100);
      await this.write(m, "Torque_Enable", 1);
    });
  }

  /** _unfold_move_and_wait (mixin :885): arrived within 5°, or stalled / timed out / short. */
  async unfoldMoveAndWait(m: So101Motor, goalIn: number, timeoutMs: number): Promise<[boolean, number, string]> {
    const goal = Math.max(0, Math.min(goalIn, FULL_TURN - 1));
    const posNow = await this.readWithRetry(m, "Present_Position");
    this.log(`    [${label(m)}] current pos=${posNow}, target pos=${goal}`);
    const ok = await this.writePosExAndWait(m, goal, DEFAULT_POS_SPEED, DEFAULT_ACCELERATION, timeoutMs, 50);
    await this.sleep(300);
    let pos: number;
    try {
      pos = await this.read(m, "Present_Position");
    } catch (e) {
      if (!isCommErr(e)) throw e;
      await this.clearOverloadUnfold(m);
      pos = await this.readWithRetry(m, "Present_Position");
      this.log(`    [${label(m)}] stalled/abnormal stop: pos=${pos} (${Math.abs(pos - goal)} steps from target)`);
      return [false, pos, "stalled (comm exception)"];
    }
    if (!ok) {
      await this.clearOverloadUnfold(m);
      this.log(`    [${label(m)}] timeout: pos=${pos} (${Math.abs(pos - goal)} steps from target)`);
      return [false, pos, "timeout"];
    }
    const errorDeg = (Math.abs(pos - goal) * 360.0) / FULL_TURN;
    if (errorDeg <= UNFOLD_TOLERANCE_DEG) {
      this.log(`    [${label(m)}] arrived: pos=${pos} (error ${Math.abs(pos - goal)} steps)`);
      return [true, pos, "arrived"];
    }
    let status = 0;
    await this.suppress(async () => (status = await this.read(m, "Status")));
    if (status & 0x20) {
      await this.clearOverloadUnfold(m);
      this.log(`    [${label(m)}] stalled: pos=${pos} (Status BIT5 overload, ${Math.abs(pos - goal)} steps from target)`);
      return [false, pos, `stalled (Status=0x${status.toString(16)})`];
    }
    this.log(`    [${label(m)}] not arrived: pos=${pos} (${Math.abs(pos - goal)} steps from target)`);
    return [false, pos, "not arrived"];
  }

  /** unfold_single_joint (mixin :945): re-centre (Torque_Enable=128 makes the current position the midpoint), then
  move `angle` degrees forward (+), else back to the middle and reverse (−). */
  async unfoldSingleJoint(m: So101Motor, unfoldAngle: number, moveTimeoutMs: number): Promise<[Direction | null, number]> {
    const targetSteps = Math.trunc((unfoldAngle / 360.0) * FULL_TURN);
    this.log(`--- Unfolding ${label(m)} (${targetSteps} steps ≈ ${unfoldAngle.toFixed(1)}°) ---`);
    await this.writeTorqueWithRecovery(m, 128);
    await this.writeTorqueWithRecovery(m, 1);
    await this.sleep(100);
    this.log(`[${label(m)}] set current position as midpoint: Present_Position=${await this.readWithRetry(m, "Present_Position")}`);
    await this.sleep(100);
    // Restore servo mode (128 may have changed the mode state) and enable torque.
    await this.write(m, "Operating_Mode", 0);
    await this.writeTorqueWithRecovery(m, 1);
    await this.sleep(300);
    let [reached] = await this.unfoldMoveAndWait(m, MID_POS + targetSteps, moveTimeoutMs);
    if (reached) {
      this.log(`    [${label(m)}] forward succeeded`);
      return ["forward", targetSteps];
    }
    await this.writePosExAndWait(m, MID_POS, DEFAULT_POS_SPEED, DEFAULT_ACCELERATION, 5000, 50);
    [reached] = await this.unfoldMoveAndWait(m, MID_POS - targetSteps, moveTimeoutMs);
    if (reached) {
      this.log(`    [${label(m)}] reverse succeeded`);
      return ["reverse", targetSteps];
    }
    this.log(`    [${label(m)}] unfold failed, holding current position`);
    return [null, 0];
  }

  // ---------- range measurement (mixin :74-345, :550-708) ----------

  /** _wait_for_stall (mixin :74): the AND rule (|velocity| < 3, position steady, Moving 0) N times in a row, or the
  overload bit / a comm error N times in a row. */
  async waitForStall(m: So101Motor, confirm: number, timeoutMs: number, sampleMs: number): Promise<string> {
    let stallCount = 0;
    let stableCount = 0;
    let prev: number | null = null;
    const t0 = this.now();
    while (this.now() - t0 < timeoutMs) {
      let vel: number, pos: number, moving: number, status: number;
      try {
        vel = await this.read(m, "Present_Velocity");
        pos = await this.read(m, "Present_Position");
        moving = await this.read(m, "Moving");
        status = await this.read(m, "Status");
      } catch (e) {
        if (!isCommErr(e)) throw e;
        stableCount = 0;
        stallCount += 1;
        if (stallCount >= confirm) return `stall confirmed (${confirm}x): comm exception`;
        await this.sleep(sampleMs);
        continue;
      }
      if (Math.abs(vel) < STALL_VELOCITY_THRESHOLD && (prev === null || Math.abs(pos - prev) < STALL_POSITION_DELTA_THRESHOLD) && moving === 0) {
        stableCount += 1;
        if (stableCount >= confirm) return `limit confirmed (${confirm}x): velocity near zero + position stable + Moving=0`;
      } else stableCount = 0;
      prev = pos;
      if (status & 0x20) {
        stallCount += 1;
        if (stallCount >= confirm) return `stall confirmed (${confirm}x): Status=0x${status.toString(16)}(BIT5 overload)`;
      } else stallCount = 0;
      await this.sleep(sampleMs);
    }
    return `timeout (${timeoutMs / 1000}s)`;
  }

  /** _wait_for_stall_multi (mixin :137): the same rules per servo; whichever stalls gets Goal_Velocity=0. */
  async waitForStallMulti(motors: So101Motor[], confirm: number, timeoutMs: number, sampleMs: number): Promise<[Map<So101Motor, string>, Map<So101Motor, number>]> {
    const running = new Set(motors);
    const reasons = new Map<So101Motor, string>();
    const positions = new Map<So101Motor, number>();
    const prev = new Map<So101Motor, number | null>(motors.map((m) => [m, null]));
    const stable = new Map<So101Motor, number>(motors.map((m) => [m, 0]));
    const stall = new Map<So101Motor, number>(motors.map((m) => [m, 0]));
    const t0 = this.now();
    while (running.size && this.now() - t0 < timeoutMs) {
      for (const m of motors) {
        if (!running.has(m)) continue;
        let vel: number, pos: number, moving: number, status: number;
        try {
          vel = await this.read(m, "Present_Velocity");
          pos = await this.read(m, "Present_Position");
          moving = await this.read(m, "Moving");
          status = await this.read(m, "Status");
        } catch (e) {
          if (!isCommErr(e)) throw e;
          stall.set(m, (stall.get(m) ?? 0) + 1);
          if ((stall.get(m) ?? 0) >= confirm) {
            reasons.set(m, `stall confirmed (${confirm}x): comm exception`);
            positions.set(m, await this.readWithRetry(m, "Present_Position"));
            await this.write(m, "Goal_Velocity", 0);
            running.delete(m);
          }
          continue;
        }
        const p = prev.get(m) ?? null;
        if (Math.abs(vel) < STALL_VELOCITY_THRESHOLD && (p === null || Math.abs(pos - p) < STALL_POSITION_DELTA_THRESHOLD) && moving === 0) {
          stable.set(m, (stable.get(m) ?? 0) + 1);
          if ((stable.get(m) ?? 0) >= confirm) {
            reasons.set(m, `limit confirmed (${confirm}x): velocity near zero + position stable + Moving=0`);
            positions.set(m, pos);
            await this.write(m, "Goal_Velocity", 0);
            running.delete(m);
            continue;
          }
        } else stable.set(m, 0);
        prev.set(m, pos);
        if (status & 0x20) {
          stall.set(m, (stall.get(m) ?? 0) + 1);
          if ((stall.get(m) ?? 0) >= confirm) {
            reasons.set(m, `stall confirmed (${confirm}x): Status=0x${status.toString(16)}(BIT5 overload)`);
            positions.set(m, pos);
            await this.write(m, "Goal_Velocity", 0);
            running.delete(m);
          }
        } else stall.set(m, 0);
      }
      await this.sleep(sampleMs);
    }
    for (const m of motors) {
      if (!running.has(m)) continue;
      reasons.set(m, `timeout (${timeoutMs / 1000}s)`);
      try {
        positions.set(m, await this.read(m, "Present_Position"));
      } catch (e) {
        if (!isCommErr(e)) throw e;
        positions.set(m, 0);
      }
      await this.suppress(() => this.write(m, "Goal_Velocity", 0));
    }
    return [reasons, positions];
  }

  /** _safe_stop_and_clear_overload (mixin :347). */
  async safeStopAndClearOverload(m: So101Motor, settleMs = 500): Promise<void> {
    await this.tryFive(() => this.write(m, "Goal_Velocity", 0));
    await this.tryFive(() => this.disableTorque(m));
    await this.sleep(settleMs);
  }

  /** _prepare_motors_for_range_measure (mixin :222): stop and release, Phase bit 4 off (single-turn), Homing_Offset 0,
  velocity mode, torque (and Lock) on. */
  async prepareMotorsForRangeMeasure(motors: So101Motor[]): Promise<void> {
    for (const m of motors) await this.safeStopAndClearOverload(m);
    for (const m of motors) {
      const phase = await this.read(m, "Phase");
      if (phase & 0x10) await this.write(m, "Phase", phase & ~0x10);
      await this.write(m, "Homing_Offset", 0);
      await this.write(m, "Operating_Mode", 1);
      const mode = await this.read(m, "Operating_Mode");
      if (motors.length === 1) this.log(`  [${label(m)}] Operating_Mode=${mode} (expected 1, constant-speed mode)`);
      await this.enableTorque(m);
    }
    if (motors.length) await this.sleep(100);
  }

  /** _run_direction_until_stall (mixin :246). */
  async runDirectionUntilStall(motors: So101Motor[], velocity: ReadonlyMap<So101Motor, number>, timeoutMs: number): Promise<[Map<So101Motor, string>, Map<So101Motor, number>]> {
    await this.syncWrite("Goal_Velocity", motors.map((m) => [m, velocity.get(m) ?? 0] as const));
    await this.sleep(INITIAL_MOVE_DELAY_MS);
    if (motors.length === 1) {
      const m = motors[0] as So101Motor;
      const reason = await this.waitForStall(m, STALL_CONFIRM_SAMPLES, timeoutMs, SAMPLE_INTERVAL_MS);
      return [new Map([[m, reason]]), new Map([[m, await this.readWithRetry(m, "Present_Position")]])];
    }
    return this.waitForStallMulti(motors, STALL_CONFIRM_SAMPLES, timeoutMs, SAMPLE_INTERVAL_MS);
  }

  /** _compute_mid_and_range_from_limits (mixin :281): without a reference pose, back off a third of the short arc
  from the last stop (an unfold, which re-centres Homing_Offset) and use where that lands. */
  async computeMidAndRangeFromLimits(m: So101Motor, posCw: number, posCcw: number, referencePos: number | null): Promise<LimitsResult> {
    let startPos: number;
    if (referencePos !== null) {
      startPos = referencePos;
      this.log(`  [${label(m)}] using pre-sampled reference start_pos=${startPos}, skipping limit back-off`);
    } else {
      const shortest = Math.min(mod(posCw - posCcw, FULL_TURN), mod(posCcw - posCw, FULL_TURN));
      const stepsBack = Math.max(1, Math.floor(shortest / 3));
      const backDeg = (stepsBack * 360.0) / FULL_TURN;
      await this.unfoldSingleJoint(m, backDeg, BACK_OFF_TIMEOUT_MS);
      await this.sleep(100);
      const present = await this.readWithRetry(m, "Present_Position");
      const homing = await this.readWithRetry(m, "Homing_Offset");
      startPos = mod(present + homing, FULL_TURN);
      this.log(`  [${label(m)}] backed off ${stepsBack} steps from limit to reference: present=${present}, offset=${homing}, actual=${startPos}`);
    }
    const r = rangeFromLimits(posCw, posCcw, startPos);
    this.log(`  [${label(m)}] CW=${posCw}, CCW=${posCcw}, reference=${startPos}, range steps=${r.rangeMax - r.rangeMin}, physical mid=${r.mid}, crosses zero=${posCcw > posCw}`);
    return r;
  }

  /** measure_ranges_of_motion (mixin :550): one servo, CW (+) first, then CCW (−), then the back-off reference. */
  async measureRangesOfMotion(m: So101Motor, velocityLimit: number, timeoutMs: number): Promise<LimitsResult> {
    await this.prepareMotorsForRangeMeasure([m]);
    const [cwReasons, cwPos] = await this.runDirectionUntilStall([m], new Map([[m, velocityLimit]]), timeoutMs);
    this.log(`  [${label(m)}] CW stop reason: ${cwReasons.get(m)}, stall position: ${cwPos.get(m)}`);
    await this.clearAndEnableTorque(m);
    await this.sleep(50);
    const [ccwReasons, ccwPos] = await this.runDirectionUntilStall([m], new Map([[m, -velocityLimit]]), timeoutMs);
    this.log(`  [${label(m)}] CCW stop reason: ${ccwReasons.get(m)}, stall position: ${ccwPos.get(m)}`);
    return this.computeMidAndRangeFromLimits(m, cwPos.get(m) as number, ccwPos.get(m) as number, null);
  }

  /** measure_ranges_of_motion_multi (mixin :614): every servo at once, each first direction per `ccwFirst`. One servo
  goes to measure_ranges_of_motion (CW first, no reference: Python ignores both arguments there too). */
  async measureRangesOfMotionMulti(motors: So101Motor[], velocityLimit: number, timeoutMs: number, ccwFirst: Partial<Record<So101Motor, boolean>>, referencePositions: ReadonlyMap<So101Motor, number> | null): Promise<Map<So101Motor, LimitsResult>> {
    if (motors.length === 1) {
      const m = motors[0] as So101Motor;
      return new Map([[m, await this.measureRangesOfMotion(m, velocityLimit, timeoutMs)]]);
    }
    await this.prepareMotorsForRangeMeasure(motors);
    const first = new Map(motors.map((m) => [m, ccwFirst[m] ? -velocityLimit : velocityLimit]));
    const second = new Map(motors.map((m) => [m, ccwFirst[m] ? velocityLimit : -velocityLimit]));
    const [firstReasons, firstPos] = await this.runDirectionUntilStall(motors, first, timeoutMs);
    for (const m of motors) this.log(`  [${label(m)}] ${(first.get(m) as number) < 0 ? "CCW" : "CW"} stop reason: ${firstReasons.get(m)}, stall position: ${firstPos.get(m)}`);
    for (const m of motors) await this.clearAndEnableTorque(m);
    await this.sleep(50);
    const [secondReasons, secondPos] = await this.runDirectionUntilStall(motors, second, timeoutMs);
    for (const m of motors) this.log(`  [${label(m)}] ${(second.get(m) as number) < 0 ? "CCW" : "CW"} stop reason: ${secondReasons.get(m)}, stall position: ${secondPos.get(m)}`);
    await this.sleep(OVERLOAD_SETTLE_MS);
    const result = new Map<So101Motor, LimitsResult>();
    for (const m of motors) {
      const cwFirst = first.get(m) === velocityLimit;
      const posCw = (cwFirst ? firstPos.get(m) : secondPos.get(m)) as number;
      const posCcw = (cwFirst ? secondPos.get(m) : firstPos.get(m)) as number;
      result.set(m, await this.computeMidAndRangeFromLimits(m, posCw, posCcw, referencePositions?.get(m) ?? null));
    }
    return result;
  }

  // ---------- script (auto_calibrate_script.py) ----------

  /** _connect_and_clear (script :642): zero Goal_Velocity and torque three times, then the handshake connect() runs
  (lerobot _assert_motors_exist + _assert_same_firmware). */
  async connectAndClear(): Promise<void> {
    this.log("Clearing residual servo state...");
    for (let i = 0; i < 3; i++) {
      await this.suppress(() => this.syncWrite("Goal_Velocity", SO101_MOTORS.map((m) => [m, 0] as const)));
      await this.suppress(() => this.syncWrite("Torque_Enable", SO101_MOTORS.map((m) => [m, 0] as const)));
      await this.sleep(200);
    }
    await this.sleep(200); // disconnect(disable_torque=False); sleep(0.2); connect()
    const missing: string[] = [];
    const wrong: string[] = [];
    for (const m of SO101_MOTORS) {
      const model = await this.ping(m);
      if (model === null) missing.push(`  - ${SO101_IDS[m]} (expected model: ${STS3215_MODEL_NUMBER})`);
      else if (model !== STS3215_MODEL_NUMBER) wrong.push(`  - ${SO101_IDS[m]} (${m}): expected ${STS3215_MODEL_NUMBER}, found ${model}`);
    }
    if (missing.length || wrong.length) {
      const lines = ["FeetechMotorsBus motor check failed:"];
      if (missing.length) lines.push("Missing motor IDs:", ...missing);
      if (wrong.length) lines.push("Motors with incorrect model numbers:", ...wrong);
      throw new FeetechError(lines.join("\n"));
    }
    const versions = new Map<number, string>();
    for (const m of SO101_MOTORS) versions.set(SO101_IDS[m], `${await this.read(m, "Firmware_Major_Version")}.${await this.read(m, "Firmware_Minor_Version")}`);
    if (new Set(versions.values()).size !== 1) {
      throw new FeetechError(`Some Motors use different firmware versions: ${JSON.stringify(Object.fromEntries(versions))}. Update their firmware first using Feetech's software.`);
    }
    this.log("All servos ready.");
  }

  /** _capture_rest_pose (script :117): offset-independent pose (Present_Position + Homing_Offset) % FULL_TURN. */
  async captureRestPose(): Promise<Map<So101Motor, number>> {
    const pose = new Map<So101Motor, number>();
    for (const m of SO101_MOTORS) {
      try {
        const pr = await this.read(m, "Present_Position");
        const ho = await this.read(m, "Homing_Offset");
        pose.set(m, mod(pr + ho, FULL_TURN));
      } catch (e) {
        if (!isCommErr(e)) throw e;
      }
    }
    const missing = SO101_MOTORS.filter((m) => !pose.has(m));
    let line = `Start pose captured for: ${pose.size ? [...pose.keys()].join(", ") : "NO MOTORS"}`;
    if (missing.length) line += ` (MISSING: ${missing.join(", ")} — these will NOT be returned on a stop)`;
    this.log(line);
    return pose;
  }

  /** _init_checks + _run_init (script :715-777): per servo write → read back → compare; a mismatch or an error is
  a warning, never fatal (non-interactive). */
  async runInit(): Promise<void> {
    const checks: Array<[RegName, number]> = [
      ["Lock", 1],
      ["Return_Delay_Time", 0],
      ["Operating_Mode", 0],
      ["Max_Torque_Limit", DEFAULT_MAX_TORQUE],
      ["Torque_Limit", this.torqueLimit],
      ["Acceleration", DEFAULT_ACCELERATION],
      ["P_Coefficient", DEFAULT_P_COEFFICIENT],
      ["I_Coefficient", DEFAULT_I_COEFFICIENT],
      ["D_Coefficient", DEFAULT_D_COEFFICIENT],
      ["Homing_Offset", 0],
    ];
    // Stricter than the script (deliberately): its init logs a failed write or read-back and goes on to move the arm,
    // which can probe a joint at full torque (a dropped Torque_Limit=380 write leaves the servo at 1000). Here every
    // setting, the position limits and the enable must read back as written, within INIT_ATTEMPTS tries, or the run
    // stops (the caller's cleanup releases the arm) before anything moves.
    const settle = async <T>(what: string, m: So101Motor, apply: () => Promise<void>, readBack: () => Promise<T>, ok: (got: T) => boolean): Promise<void> => {
      let last = "";
      for (let attempt = 0; attempt < INIT_ATTEMPTS; attempt++) {
        try {
          await apply();
          await this.sleep(10);
          const got = await readBack();
          if (ok(got)) return;
          last = `read back ${JSON.stringify(got)}`;
        } catch (e) {
          if (e instanceof StopRequested) throw e;
          last = (e as Error).message;
        }
        this.log(`  [Retry] ${what} on ${m}: ${last}`);
      }
      throw new ServoFaultError(`Could not set ${what} on ${label(m)} (${last}); nothing was moved. Check the arm's cables and power, then calibrate again`);
    };
    for (const m of SO101_MOTORS) {
      this.log(`Configuring servo: ${label(m)}`);
      try {
        await this.write(m, "Torque_Enable", 0);
        await this.sleep(50);
      } catch (e) {
        if (!isCommErr(e)) throw e;
      }
      for (const [reg, expected] of checks) {
        await settle(`${reg}=${expected}`, m, () => this.write(m, reg, expected), () => this.read(m, reg), (got) => got === expected);
      }
      await settle("Position_Limits=(0, 4095)", m, async () => {
        await this.writePositionLimits(m, 0, 4095);
        await this.sleep(40);
      }, () => this.readPositionLimits(m), (got) => got[0] === 0 && got[1] === 4095);
      await this.sleep(200);
      await settle("Torque_Enable=1", m, async () => {
        await this.write(m, "Torque_Enable", 1);
        await this.sleep(40);
      }, () => this.read(m, "Torque_Enable"), (got) => got === 1);
      await this.sleep(100);
    }
    this.log(`Initialized and torque enabled (P=${DEFAULT_P_COEFFICIENT}, Acc=${DEFAULT_ACCELERATION}, Torque=${this.torqueLimit}).`);
  }

  /** _record_reference_position (script :558). */
  async recordReferencePosition(m: So101Motor, out: Map<So101Motor, number>): Promise<void> {
    try {
      const pr = await this.read(m, "Present_Position");
      const ho = await this.read(m, "Homing_Offset");
      out.set(m, mod(pr + ho, FULL_TURN));
    } catch (e) {
      if (!isCommErr(e)) throw e;
    }
  }

  /** _calibrate_motors (script :573): measure, then write Homing_Offset and the limits (with read-backs), then centre
  each servo except wrist_roll, and except shoulder_lift/elbow_flex when probed together (they keep torque). */
  async calibrateMotors(motors: So101Motor[], opts: { ccwFirst?: Partial<Record<So101Motor, boolean>>; unfoldDirections?: Map<So101Motor, Direction>; referencePositions?: Map<So101Motor, number> } = {}): Promise<Map<So101Motor, [number, number, number]>> {
    const raw = await this.measureRangesOfMotionMulti(motors, DEFAULT_VELOCITY_LIMIT, DEFAULT_TIMEOUT_MS, opts.ccwFirst ?? {}, opts.referencePositions ?? null);
    this.log("Preparing to write registers");
    const result = new Map<So101Motor, [number, number, number]>();
    for (const m of motors) {
      const r = raw.get(m) as LimitsResult;
      this.log(`  ${label(m)}: post-offset range_min=${r.rangeMin}, range_max=${r.rangeMax}, mid=${r.mid}, Homing_Offset register=${r.homingOffset}`);
      await this.sleep(50);
      await this.suppress(async () => {
        const ho = await this.read(m, "Homing_Offset");
        const mn = await this.read(m, "Min_Position_Limit");
        const mx = await this.read(m, "Max_Position_Limit");
        this.log(`  ${label(m)} pre-write: Min_Position_Limit=${mn}, Max_Position_Limit=${mx}, Homing_Offset=${ho}`);
      });
      await this.safeWrite(m, "Homing_Offset", r.homingOffset);
      await this.safeWritePositionLimits(m, r.rangeMin, r.rangeMax);
      await this.sleep(100);
      await this.suppress(async () => {
        const ho = await this.read(m, "Homing_Offset");
        const mn = await this.read(m, "Min_Position_Limit");
        const mx = await this.read(m, "Max_Position_Limit");
        this.log(`  ${label(m)} post-write: Min_Position_Limit=${mn}, Max_Position_Limit=${mx}, Homing_Offset=${ho}`);
      });
      await this.sleep(100);
      const together = opts.unfoldDirections !== undefined && motors.includes("shoulder_lift") && motors.includes("elbow_flex");
      if (m === "wrist_roll") {
        // no centring
      } else if (together && (m === "shoulder_lift" || m === "elbow_flex")) {
        // servos 2 and 3 keep torque
      } else await this.goToMid(m);
      result.set(m, [r.rangeMin, r.rangeMax, r.mid]);
    }
    return result;
  }

  /** The console's addition to the final fold (2026-10-19, Isaac): the wrist roll turned to WRIST_REST_FROM_MIDDLE_TICKS
  from the middle of its calibrated range, so the folded arm rests with its wrist camera on top of the gripper; kept
  WRIST_REST_MARGIN_TICKS inside the range. */
  async turnWristToRest(mins: Map<So101Motor, number>, maxes: Map<So101Motor, number>): Promise<void> {
    const m: So101Motor = "wrist_roll";
    const mn = mins.get(m);
    const mx = maxes.get(m);
    if (mn === undefined || mx === undefined) return;
    await this.write(m, "Operating_Mode", 0);
    const present = await this.readWithRetry(m, "Present_Position");
    const middle = Math.round((mn + mx) / 2);
    const target = Math.max(mn + WRIST_REST_MARGIN_TICKS, Math.min(mx - WRIST_REST_MARGIN_TICKS, middle + WRIST_REST_FROM_MIDDLE_TICKS));
    this.log(`  ${label(m)} current pos=${present}, turning to rest pos=${target} (${(((target - present) * 360) / FULL_TURN).toFixed(1)}°)`);
    const arrived = await this.writePosExAndWait(m, target, WRIST_REST_SPEED, DEFAULT_ACCELERATION);
    await this.suppress(async () => this.log(`  ${label(m)} end pos=${await this.read(m, "Present_Position")}${arrived ? "" : " (not arrived)"}`));
  }

  /** How far (ticks, signed) the wrist roll is from the middle of the calibrated range its servo holds, or null when
  it holds none (factory limits 0..4095) or can't be read: then it is left where the operator put it. */
  async heldWristRollTurn(): Promise<number | null> {
    const m: So101Motor = "wrist_roll";
    try {
      const mn = await this.read(m, "Min_Position_Limit");
      const mx = await this.read(m, "Max_Position_Limit");
      const present = await this.read(m, "Present_Position");
      if ((mn === 0 && mx === MAX_TICK) || !(mn < mx) || present < mn - WRIST_REST_MARGIN_TICKS || present > mx + WRIST_REST_MARGIN_TICKS) return null;
      const turn = Math.round((mn + mx) / 2) - present;
      this.log(`  ${label(m)} holds range ${mn}..${mx}, reads ${present}: ${turn} ticks from its middle`);
      return Math.abs(turn) > WRIST_REST_MARGIN_TICKS ? turn : null;
    } catch (e) {
      if (!isCommErr(e)) throw e;
      return null;
    }
  }

  /** Turns the wrist roll `ticks` from where it is: its current position made the midpoint (Torque_Enable 128, as
  unfold_single_joint does), then a position move, so the turn never crosses the encoder's seam. */
  async turnWristRoll(ticks: number): Promise<void> {
    const m: So101Motor = "wrist_roll";
    this.log(`--- Turning ${label(m)} ${ticks} ticks (${((ticks * 360) / FULL_TURN).toFixed(1)}°) back to its middle ---`);
    await this.writeTorqueWithRecovery(m, 128);
    await this.writeTorqueWithRecovery(m, 1);
    await this.sleep(100);
    await this.write(m, "Operating_Mode", 0);
    await this.writeTorqueWithRecovery(m, 1);
    const present = await this.readWithRetry(m, "Present_Position");
    const arrived = await this.writePosExAndWait(m, present + ticks, WRIST_REST_SPEED, DEFAULT_ACCELERATION);
    await this.suppress(async () => this.log(`  ${label(m)} end pos=${await this.read(m, "Present_Position")}${arrived ? "" : " (not arrived)"}`));
  }

  /** _fold_arm (script :424): every listed servo to its fold end (or unfold end) at once, then wait for Moving 0.
  The docstring (:436) says a forward-unfolding joint folds to range_max; the code (:461) folds it to range_min
  (forward unfold = increasing position, so folding = decreasing) and reverse ones to range_max. The code wins. */
  async foldArm(mins: Map<So101Motor, number>, maxes: Map<So101Motor, number>, dirs: Map<So101Motor, Direction>, motors: So101Motor[] | null, unfold = false, unfoldPerMotor: Partial<Record<So101Motor, boolean>> | null = null): Promise<void> {
    const order: So101Motor[] = motors?.length ? motors : ["shoulder_lift", "elbow_flex", "wrist_flex", "gripper"];
    const values = new Map<So101Motor, PosEx>();
    for (const m of order) {
      const mn = mins.get(m);
      const mx = maxes.get(m);
      if (mn === undefined || mx === undefined) continue;
      const perUnfold = unfoldPerMotor ? (unfoldPerMotor[m] ?? unfold) : unfold;
      const reverse = dirs.get(m) === "reverse";
      const foldEnd = m === "gripper" ? mn : reverse ? mx : mn;
      const unfoldEnd = m === "gripper" ? mx : reverse ? mn : mx;
      const target = perUnfold ? unfoldEnd : foldEnd;
      values.set(m, [target, DEFAULT_POS_SPEED, DEFAULT_ACCELERATION]);
      await this.write(m, "Operating_Mode", 0);
      let pos: number | string = "?";
      await this.suppress(async () => (pos = await this.read(m, "Present_Position")));
      this.log(`  ${label(m)} current pos=${pos}, ${perUnfold ? "Unfold" : "Fold"} to ${target === mx ? "range_max" : "range_min"}=${target}.`);
    }
    if (!values.size) return;
    await this.syncWritePosEx(values);
    await this.sleep(300);
    const t0 = this.now();
    while (this.now() - t0 < 10_000) {
      try {
        let all = true;
        // all(...) short-circuits: the first servo still moving ends the pass.
        for (const m of values.keys()) if ((await this.read(m, "Moving")) !== 0) {
          all = false;
          break;
        }
        if (all) break;
      } catch (e) {
        if (!isCommErr(e)) throw e;
      }
      await this.sleep(50);
    }
    for (const m of values.keys()) {
      await this.suppress(async () => this.log(`  ${label(m)} end pos=${await this.read(m, "Present_Position")}`));
    }
  }

  /** _move_arm_by_angle (script :502), unfold direction only (the one use: elbow_flex 80°, :863). */
  async moveArmByAngle(dirs: Map<So101Motor, Direction>, angleDeg: number, motors: So101Motor[], mins: Map<So101Motor, number>, maxes: Map<So101Motor, number>, fold = false): Promise<void> {
    const angleSteps = Math.trunc((angleDeg / 360.0) * FULL_TURN);
    for (const m of motors) {
      if (!mins.has(m) || !maxes.has(m)) continue;
      let present: number;
      try {
        present = await this.read(m, "Present_Position");
      } catch (e) {
        if (!isCommErr(e)) throw e;
        this.log(`  Warning: ${label(m)} failed to read current pos, skipping`);
        continue;
      }
      const forward = dirs.get(m) === "forward";
      let target = fold ? (forward ? present - angleSteps : present + angleSteps) : forward ? present + angleSteps : present - angleSteps;
      target = Math.max(mins.get(m) as number, Math.min(maxes.get(m) as number, target));
      this.log(`  ${label(m)} ${fold ? "fold" : "unfold"} ${angleDeg.toFixed(1)}°: pos ${present} -> ${target}`);
      const ok = await this.writePosExAndWait(m, target, DEFAULT_POS_SPEED, DEFAULT_ACCELERATION, DEFAULT_UNFOLD_TIMEOUT_MS, 50);
      this.log(ok ? `  ${label(m)} arrived` : `  Warning: ${label(m)} motion timeout, holding current position`);
    }
  }

  /** run_full_calibration's body (script :814-933). */
  async body(): Promise<Record<string, MotorCalibration>> {
    const mins = new Map<So101Motor, number>();
    const maxes = new Map<So101Motor, number>();
    const mids = new Map<So101Motor, number>();
    const dirs = new Map<So101Motor, Direction>();
    const refs = new Map<So101Motor, number>();
    const apply = (results: Map<So101Motor, [number, number, number]>) => {
      for (const [m, [mn, mx, mid]] of results) {
        mins.set(m, mn);
        maxes.set(m, mx);
        mids.set(m, mid);
      }
    };
    const unfold = async (m: So101Motor, angle: number) => {
      const [direction] = await this.unfoldSingleJoint(m, angle, DEFAULT_UNFOLD_TIMEOUT_MS);
      if (direction !== null) dirs.set(m, direction);
    };

    // The wrist roll's turn back to the middle of its range, read from the calibration the servo still holds (init
    // wipes it): an arm resting from a previous run has its wrist camera on top (turnWristToRest), where it hits the
    // arm while the shoulder and elbow sweep (2026-10-19).
    const rollBack = await this.heldWristRollTurn();
    this.progress(0.03, "Initializing servos");
    await this.runInit();
    if (rollBack !== null) await this.turnWristRoll(rollBack);
    this.progress(0.1, "Lifting the wrist");
    await unfold("wrist_flex", 80);
    await this.sleep(100);
    this.progress(0.14, "Lifting the shoulder and elbow");
    await unfold("shoulder_lift", 15);
    await this.recordReferencePosition("shoulder_lift", refs);
    await unfold("elbow_flex", 30);
    await this.recordReferencePosition("elbow_flex", refs);
    await this.sleep(100);
    this.progress(0.2, "Centering the shoulder and elbow");
    for (const m of ["shoulder_lift", "elbow_flex"] as const) {
      await this.goToMid(m);
      await this.sleep(100);
    }
    // First direction opposite each joint's lift: forward-lifted → CCW first, reverse → CW first (script :843).
    this.progress(0.25, "Measuring the shoulder and elbow range");
    apply(
      await this.calibrateMotors(["shoulder_lift", "elbow_flex"], {
        ccwFirst: { shoulder_lift: dirs.get("shoulder_lift") !== "reverse", elbow_flex: dirs.get("elbow_flex") !== "reverse" },
        unfoldDirections: dirs,
        referencePositions: refs,
      }),
    );
    this.progress(0.4, "Folding the shoulder and elbow");
    await this.foldArm(mins, maxes, dirs, ["shoulder_lift", "elbow_flex"]);
    await this.sleep(100);
    this.progress(0.45, "Raising the elbow");
    await this.moveArmByAngle(dirs, 80, ["elbow_flex"], mins, maxes);
    this.progress(0.5, "Measuring the wrist and gripper range");
    apply(await this.calibrateMotors(["wrist_roll", "gripper", "wrist_flex"], { referencePositions: refs }));
    await this.sleep(100);
    this.progress(0.75, "Folding the elbow, unfolding the wrist");
    await this.foldArm(mins, maxes, dirs, ["elbow_flex", "wrist_flex", "gripper"], false, { elbow_flex: false, wrist_flex: true, gripper: false });
    this.progress(0.8, "Measuring the shoulder pan range");
    apply(await this.calibrateMotors(["shoulder_pan"]));
    await this.sleep(100);
    for (const m of [...CALIBRATE_REST, ...CALIBRATE_FIRST]) {
      this.log(`  ${label(m)}: min=${mins.get(m)}, max=${maxes.get(m)}, mid=${mids.get(m)}, offset=${(mids.get(m) as number) - STS_HALF_TURN_RAW}`);
    }
    this.progress(0.92, "Folding the arm");
    await this.foldArm(mins, maxes, dirs, null);
    await this.turnWristToRest(mins, maxes);
    // Before persistence: unlock EEPROM (Lock=0) and servo mode everywhere (script :895).
    for (const m of SO101_MOTORS) {
      await this.write(m, "Lock", 0);
      await this.sleep(10);
      await this.write(m, "Operating_Mode", 0);
      await this.sleep(10);
    }
    await this.sleep(1000);

    this.progress(0.97, "Saving the calibration to the servos");
    await this.safeDisableAll();
    const cal = new Map<So101Motor, MotorCalibration>();
    for (const m of [...CALIBRATE_REST, ...CALIBRATE_FIRST]) {
      const offset = Math.max(-HOMING_OFFSET_MAX_MAG, Math.min(HOMING_OFFSET_MAX_MAG, (mids.get(m) as number) - STS_HALF_TURN_RAW));
      cal.set(m, { id: SO101_IDS[m], drive_mode: 0, homing_offset: offset, range_min: mins.get(m) as number, range_max: maxes.get(m) as number });
    }
    // write_calibration (lerobot feetech.py :268): Homing_Offset, Min_Position_Limit, Max_Position_Limit per motor.
    for (const [m, c] of cal) {
      await this.write(m, "Homing_Offset", c.homing_offset);
      await this.write(m, "Min_Position_Limit", c.range_min);
      await this.write(m, "Max_Position_Limit", c.range_max);
    }
    this.log("Wrote calibration to servo EEPROM.");
    this.log("Releasing all servos...");
    await this.safeDisableAll();
    const out: Record<string, MotorCalibration> = {};
    for (const m of SO101_MOTORS) out[m] = cal.get(m) as MotorCalibration;
    this.progress(1, "Calibration complete");
    return out;
  }

  // ---------- stop (script :145-339) ----------

  /** _freeze_arm (script :145): zero velocity everywhere, then per servo position mode, working Torque_Limit, torque
  cleared (drops an overload latch) and re-enabled, goal = where it is. */
  async freezeArm(): Promise<void> {
    await this.suppress(() => this.syncWrite("Goal_Velocity", SO101_MOTORS.map((m) => [m, 0] as const)));
    const values = new Map<So101Motor, PosEx>();
    for (const m of SO101_MOTORS) {
      try {
        await this.write(m, "Operating_Mode", 0);
        await this.write(m, "Torque_Limit", this.torqueLimit);
        await this.suppress(() => this.write(m, "Torque_Enable", 0));
        await this.sleep(50);
        const pos = await this.read(m, "Present_Position");
        await this.writeTorqueWithRecovery(m, 1, 2, 200);
        values.set(m, [pos, DEFAULT_POS_SPEED, DEFAULT_ACCELERATION]);
      } catch (e) {
        if (!isCommErr(e)) throw e;
        this.log(`  Freeze: ${label(m)} could not be re-energized; it will not hold or return.`);
      }
    }
    if (values.size) await this.syncWritePosEx(values);
  }

  /** _return_to_rest_pose (script :203): every captured joint back to its start pose at RETURN_POS_SPEED; true only
  once all are within POSITION_TOLERANCE; false when the summed distance stops shrinking by > 10 ticks for 1.5 s. */
  async returnToRestPose(restPose: ReadonlyMap<So101Motor, number>, ceilingMs: number): Promise<boolean> {
    const values = new Map<So101Motor, PosEx>();
    for (const [m, encoder] of restPose) {
      try {
        const ho = await this.read(m, "Homing_Offset");
        const present = await this.read(m, "Present_Position");
        let limits: [number, number];
        try {
          limits = await this.readPositionLimits(m);
        } catch (e) {
          if (!isCommErr(e)) throw e;
          limits = [0, FULL_TURN - 1];
        }
        const target = nearestWrapTarget(mod(encoder - ho, FULL_TURN), present, limits[0], limits[1]);
        values.set(m, [target, RETURN_POS_SPEED, DEFAULT_ACCELERATION]);
      } catch (e) {
        if (!isCommErr(e)) throw e;
      }
    }
    if (!values.size) {
      this.log("Fallback: could not compute any return target (comm errors on every motor).");
      return false;
    }
    await this.syncWritePosEx(values);
    await this.sleep(RETURN_SETTLE_MS);
    const t0 = this.now();
    let best: number | null = null;
    let lastProgress = t0;
    let remaining = -1;
    while (this.now() - t0 < ceilingMs) {
      remaining = 0;
      const distances = new Map<So101Motor, number>();
      let readable = false;
      let arrived = true;
      for (const [m, [target]] of values) {
        let pos: number;
        try {
          pos = await this.read(m, "Present_Position");
        } catch (e) {
          if (!isCommErr(e)) throw e;
          arrived = false;
          continue;
        }
        readable = true;
        const d = Math.abs(pos - target);
        distances.set(m, d);
        remaining += d;
        if (d > POSITION_TOLERANCE) arrived = false;
      }
      const now = this.now();
      if (readable && arrived) {
        this.log(`Return complete: max delta ${Math.max(...distances.values())} ticks.`);
        return true;
      }
      if (readable && (best === null || remaining < best - RETURN_STALL_MIN_PROGRESS)) {
        best = remaining;
        lastProgress = now;
      }
      if (now - lastProgress > RETURN_STALL_WINDOW_MS) {
        const detail = [...distances].filter(([, d]) => d > POSITION_TOLERANCE).map(([m, d]) => `${m}=${d}`).join(", ");
        let outcome = "stalled";
        await this.suppress(async () => {
          let settled = true;
          for (const m of values.keys()) if ((await this.read(m, "Moving")) !== 0) {
            settled = false;
            break;
          }
          if (settled) outcome = "settled short of target";
        });
        this.log(`Fallback: return ${outcome} after ${((now - t0) / 1000).toFixed(1)}s — remaining distance ${remaining} ticks (${detail || "unreadable motors"}).`);
        return false;
      }
      await this.sleep(RETURN_POLL_MS);
    }
    this.log(`Fallback: return hit the ${ceilingMs / 1000}s ceiling with ${remaining} ticks remaining.`);
    return false;
  }

  /** _graceful_stop (script :298): freeze, return (or hold STOP_HOLD), strictly best-effort. */
  async gracefulStop(restPose: ReadonlyMap<So101Motor, number>): Promise<void> {
    try {
      this.log("Stop requested: freezing the arm in place...");
      await this.freezeArm();
      let returned = false;
      if (restPose.size) {
        this.log("Returning the arm to its starting position...");
        returned = await this.returnToRestPose(restPose, RETURN_TO_REST_BUDGET_MS);
      } else this.log("Fallback: no start pose was captured at startup.");
      if (returned) this.log("Arm returned to its starting position.");
      else {
        this.log(`Holding position for ${STOP_HOLD_MS / 1000}s before releasing...`);
        await this.sleep(STOP_HOLD_MS);
      }
    } catch {
      // Anything: straight on to the release.
    }
  }
}

/** Runs the whole procedure on an open bus; resolves with the calibration (already written to the motors with
write_calibration semantics, torque released) keyed by motor name in SO101_MOTORS order. Rejects with AutoCalStopped
after the stop sequence when `signal` aborts, or with the error that ended the run (torque released either way). */
export async function autoCalibrateSo101(bus: FeetechBus, options: AutoCalOptions): Promise<Record<string, MotorCalibration>> {
  const run = new AutoCal(bus, options);
  if (options.signal?.aborted || options.halt?.aborted) throw new AutoCalStopped();
  if (run.torqueLimit !== DEFAULT_TORQUE_LIMIT) run.log(`Working torque: Torque_Limit=${run.torqueLimit} (default ${DEFAULT_TORQUE_LIMIT})`);
  try {
    run.progress(0, "Connecting to the servos");
    await run.connectAndClear();
  } catch (e) {
    if (!(e instanceof StopRequested)) throw e;
    run.stopping = true;
    run.releasing = true;
    await run.safeDisableAll();
    throw new AutoCalStopped();
  }
  let restPose = new Map<So101Motor, number>();
  try {
    // Captured before any motion, so a Stop can drive the arm back to where the operator left it (script :679).
    restPose = await run.captureRestPose();
    const result = await run.body();
    // Done and released: a Stop from here on has nothing left to stop (Python's would escape the `finally`).
    run.stopping = true;
    run.releasing = true;
    await run.disconnect();
    return result;
  } catch (e) {
    run.stopping = true;
    if (e instanceof StopRequested) {
      // An e-stop skips the return: torque off at once.
      if (!run.halt?.aborted) await run.gracefulStop(restPose);
      run.releasing = true;
      await run.safeDisableAll();
      run.log("User interrupt; releasing all servos...");
      await run.disconnect();
      throw new AutoCalStopped();
    }
    run.log(`Exception: ${(e as Error).message}`);
    run.releasing = true;
    await run.safeDisableAll();
    await run.disconnect();
    throw e;
  }
}
