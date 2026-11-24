/** Calibration of one SO-101 arm (follower or leader), by hand or automatically, and where calibrations are kept.
Lifted from the console's hardware/so101-calibrate.ts (MakerLab calibrate.py's range sweep, which is LeRobot's
SOFollower.calibrate with a page in place of the terminal; dreamlabs so101_calibration.ArmCalibration does the
same) and so101-autocal.ts (MakerLab's motorized auto-calibration, ./autocal.ts).

By hand: the servos are switched off and put in position mode; on start the arm is in its middle pose, so each
servo's homing offset is reset and then set so it reads the half turn (2047; set_half_turn_homings); status() then
shows each joint's recorded range (raw ticks) while the operator moves every joint end to end; complete() checks
every joint went far enough, gives the wrist roll the whole turn, and writes the calibration to the servos and the
store. Cancelled or failed after the homing writes, the servos get back the calibration they held before, so their
old calibration still fits them.

A follower's calibration is then aligned to Dream's reference arm (alignToReference): every model trained on an
SO-101 reads joints relative to the arm it was recorded on, and the ends a joint is moved between are its
mechanical stops, the same on every SO-101, so each joint's zero is moved by how far its range's middle is from the
reference's. A leader is left as recorded.

Refusals: a servo found with its torque on refuses (another program may hold the arm up, and calibration switches
everything off); an incomplete-calibration marker is saved before the first homing write and cleared only once the
calibration is saved (So101Arm refuses to enable while it is set); the servos' calibration is read back and must be
what was written. */

import type { Clock } from "@makermods/arm-core";
import { ArmError } from "@makermods/arm-core";
import { AutoCalStopped, autoCalibrateSo101 } from "./autocal.js";
import { openSo101Link, portLabel, type FeetechBus, type SerialPortLike, type So101Link } from "./bus.js";
import type { MsClock } from "./clock-ms.js";
import { MODE_POSITION, STS3215_MODEL, errorText, isFault } from "./protocol.js";
import {
  HALF_TURN_TICK,
  JOINT_WORDS,
  MAX_HOMING_OFFSET,
  MAX_TICK,
  SO101_IDS,
  SO101_MOTORS,
  So101Error,
  alignToReference,
  calibrationDifferences,
  parseSo101Calibration,
  readHeldCalibration,
  shortJoints,
  switchOff,
  unlockEeprom,
  type So101Calibration,
} from "./so101.js";

// ---------- Stores ----------

/** Where calibrations are kept, by name. The application chooses (a server, IndexedDB, localStorage, memory). */
export interface CalibrationStore {
  /** The calibration saved under `name`, or null. */
  load(name: string): Promise<So101Calibration | null>;
  save(name: string, calibration: So101Calibration): Promise<void>;
  /** Marks `name` as mid-calibration: the servos may hold something else than the saved calibration. */
  markIncomplete(name: string): Promise<void>;
  clearIncomplete(name: string): Promise<void>;
  isIncomplete(name: string): Promise<boolean>;
}

/** Calibrations in memory (lost on reload). */
export class MemoryCalibrationStore implements CalibrationStore {
  readonly calibrations = new Map<string, So101Calibration>();
  readonly incomplete = new Set<string>();

  constructor(initial: Record<string, So101Calibration> = {}) {
    for (const [name, c] of Object.entries(initial)) this.calibrations.set(name, parseSo101Calibration(c, name));
  }

  async load(name: string): Promise<So101Calibration | null> {
    const c = this.calibrations.get(name);
    return c ? structuredClone(c) : null;
  }

  async save(name: string, calibration: So101Calibration): Promise<void> {
    this.calibrations.set(name, structuredClone(parseSo101Calibration(calibration, name)));
  }

  async markIncomplete(name: string): Promise<void> {
    this.incomplete.add(name);
  }

  async clearIncomplete(name: string): Promise<void> {
    this.incomplete.delete(name);
  }

  async isIncomplete(name: string): Promise<boolean> {
    return this.incomplete.has(name);
  }
}

/** The parts of the Web Storage API LocalStorageCalibrationStore uses. */
export interface StorageLike {
  getItem(key: string): string | null;
  setItem(key: string, value: string): void;
  removeItem(key: string): void;
}

/** Calibrations in localStorage, as LeRobot JSON under `<prefix><name>`. Where storage is unavailable (a Web Worker,
a private window, blocked site data, a full quota) it keeps them in memory instead and says so in `persistent` and
`lastError`; it never throws for storage itself. A stored entry that does not parse as an SO-101 calibration loads
as null. */
export class LocalStorageCalibrationStore implements CalibrationStore {
  readonly prefix: string;
  /** False once storage could not be used: calibrations from then on live in memory only. */
  persistent: boolean;
  /** Why storage was given up, if it was. */
  lastError: string | null = null;
  private readonly storage: StorageLike | null;
  private readonly memory = new MemoryCalibrationStore();

  constructor(options: { prefix?: string; storage?: StorageLike | null } = {}) {
    this.prefix = options.prefix ?? "makermods.so101.calibration.";
    let storage: StorageLike | null = null;
    try {
      storage = options.storage === undefined ? ((globalThis as { localStorage?: StorageLike }).localStorage ?? null) : options.storage;
    } catch (exc) {
      this.lastError = exc instanceof Error ? exc.message : String(exc);
    }
    this.storage = storage;
    this.persistent = storage !== null;
    if (!storage && !this.lastError) this.lastError = "localStorage is not available here";
  }

  private attempt<T>(op: (s: StorageLike) => T): { ok: true; value: T } | { ok: false } {
    if (!this.storage || !this.persistent) return { ok: false };
    try {
      return { ok: true, value: op(this.storage) };
    } catch (exc) {
      this.persistent = false;
      this.lastError = exc instanceof Error ? exc.message : String(exc);
      return { ok: false };
    }
  }

  async load(name: string): Promise<So101Calibration | null> {
    const memory = await this.memory.load(name);
    if (memory) return memory;
    const got = this.attempt((s) => s.getItem(this.prefix + name));
    if (!got.ok || got.value === null) return null;
    try {
      return parseSo101Calibration(JSON.parse(got.value), name);
    } catch {
      return null;
    }
  }

  async save(name: string, calibration: So101Calibration): Promise<void> {
    const checked = parseSo101Calibration(calibration, name);
    const saved = this.attempt((s) => s.setItem(this.prefix + name, JSON.stringify(checked, null, 4)));
    if (saved.ok) await this.memory.clearIncomplete(name).then(() => this.memory.calibrations.delete(name));
    else await this.memory.save(name, checked);
  }

  async markIncomplete(name: string): Promise<void> {
    await this.memory.markIncomplete(name);
    this.attempt((s) => s.setItem(`${this.prefix}${name}.incomplete`, "1"));
  }

  async clearIncomplete(name: string): Promise<void> {
    await this.memory.clearIncomplete(name);
    this.attempt((s) => s.removeItem(`${this.prefix}${name}.incomplete`));
  }

  async isIncomplete(name: string): Promise<boolean> {
    if (await this.memory.isIncomplete(name)) return true;
    const got = this.attempt((s) => s.getItem(`${this.prefix}${name}.incomplete`));
    return got.ok && got.value !== null;
  }
}

// ---------- Session ----------

/** The range is read at 20 Hz; readings outside (0, 5000) are glitches; a jump this big between two reads of a joint
with stops is an encoder wrap (the middle pose was set too far round). */
export const RECORD_MS = 50;
export const GLITCH_LOW = 0;
export const GLITCH_HIGH = 5000;
export const WRAP_JUMP_TICKS = 2000;
/** calibrate.py's centering check (a leader's; a follower's is the reference alignment's 25°). */
export const CENTERING_SHARE = 0.2;
/** A hand recording ends unsaved after this long. */
export const POSE_TIMEOUT_MS = 15 * 60_000;
/** stop() waits this long for an automatic run to bring the arm back before it lets go. */
export const STOP_JOIN_MS = 25_000;

/** By hand ("manual") or motorized ("auto"). */
export type So101Method = "manual" | "auto";

/** What a session reports (the console's CalibrationStatus, for an SO-101). */
export interface CalibrationStatus {
  status: "connecting" | "recording" | "running" | "saving" | "completed" | "error" | "stopped";
  message: string;
  /** recording: each joint's raw ticks so far. */
  recordedRanges?: Record<string, { min: number; max: number; current: number }>;
  /** running: 0..1. */
  progress?: number;
  /** completed: the name it was saved under. */
  name?: string;
  /** completed: what was saved. */
  calibration?: So101Calibration;
  error?: string;
}

/** How to calibrate. */
export interface So101CalibrationOptions {
  /** Saved under this name in `store` (So101Arm's `calibrationName`). */
  name: string;
  store: CalibrationStore;
  side: "left" | "right";
  /** A follower is aligned to Dream's reference arm; a leader is kept as recorded. Default follower. */
  role?: "leader" | "follower";
  method: So101Method;
  /** Auto only: Torque_Limit while probing (MakerLab: motor power % × 10, default 380). */
  torqueLimit?: number;
  /** Operator-facing events ("system", "error"). */
  onEvent?: (kind: "system" | "error", message: string) => void;
  /** Diagnostic lines. */
  log?: (message: string) => void;
}

type Phase = "connecting" | "recording" | "running" | "saving" | "stopping" | "idle" | "completed" | "error";

const message = (exc: unknown) => (exc instanceof Error ? exc.message : String(exc));

class Stopped extends Error {
  override name = "Stopped";
}

/** A servo holds its torque on: another program may hold the arm up. Nothing was switched. */
export class TorqueOnError extends ArmError {}

/** One calibration run on one arm's open link. Poll status(); complete() saves a hand calibration; stop() cancels.
`done` settles when the session has ended (it never rejects; status() says how it ended). */
export class So101CalibrationSession {
  readonly name: string;
  readonly side: "left" | "right";
  readonly role: "leader" | "follower";
  readonly method: So101Method;
  readonly done: Promise<void>;

  private phase: Phase = "connecting";
  private activeFlag = true;
  private messageText = "Connecting to the SO-101...";
  private errorText: string | null = null;
  private progress = 0;
  private stopRequested = false;
  private finished = false;
  private readonly abort = new AbortController();
  private readonly haltAbort = new AbortController();
  /** The servos' calibration before the homing writes (rolled back on a cancelled or failed hand calibration). */
  private baseline: So101Calibration | null = null;
  private homed = false;
  /** The automatic run's step and its last log lines, for the error a failure reports. */
  private step = "";
  private readonly tail: string[] = [];
  /** Set once anything was written to the servos: a refusal before that (a servo holding torque) leaves them be. */
  private touched = false;
  private homing: Record<string, number> = {};
  private low: Record<string, number> = {};
  private high: Record<string, number> = {};
  private now: Record<string, number> = {};
  private saved: So101Calibration | null = null;
  private resolveSave: ((ok: boolean) => void) | null = null;
  private readonly store: CalibrationStore;
  private readonly clock: MsClock;

  /** Runs on `link` (left open at the end unless `ownsLink`). */
  constructor(
    private readonly link: So101Link,
    private readonly options: So101CalibrationOptions,
    private readonly ownsLink = false,
    private readonly portName = "the arm's port",
  ) {
    if (options.method !== "manual" && options.method !== "auto") throw new ArmError(`Unknown SO-101 calibration method ${String(options.method)}`);
    if (options.side !== "left" && options.side !== "right") throw new ArmError(`side must be "left" or "right", got ${String(options.side)}`);
    this.name = options.name;
    this.side = options.side;
    this.role = options.role ?? "follower";
    this.method = options.method;
    this.store = options.store;
    this.clock = link.clock;
    options.onEvent?.("system", `Calibrating the ${this.side} ${this.role} (SO-101, ${this.method === "auto" ? "automatic" : "by hand"}) on ${portName}.`);
    this.done = this.worker();
  }

  get active(): boolean {
    return this.activeFlag;
  }

  /** The session's state now. */
  status(): CalibrationStatus {
    const base = { message: this.messageText };
    if (this.activeFlag) {
      if (this.phase === "recording") {
        const recordedRanges = Object.fromEntries(
          SO101_MOTORS.filter((m) => this.now[m] !== undefined).map((m) => [m, { min: this.low[m]!, max: this.high[m]!, current: this.now[m]! }]),
        );
        return { ...base, status: "recording", recordedRanges };
      }
      if (this.phase === "running") return { ...base, status: "running", progress: this.progress };
      return { ...base, status: this.phase === "connecting" ? "connecting" : "saving" };
    }
    if (this.phase === "completed") return { ...base, status: "completed", name: this.name, calibration: structuredClone(this.saved!) };
    if (this.phase === "error") return { ...base, status: "error", error: this.errorText || this.messageText };
    return { ...base, status: "stopped" };
  }

  /** Save, by hand: every joint has been moved end to end. */
  complete(): void {
    if (!this.activeFlag) throw new ArmError("No calibration active");
    if (this.phase !== "recording") throw new ArmError(`Cannot complete step in status: ${this.phase}`);
    this.resolveSave?.(true);
  }

  /** Cancels. An automatic run brings the arm back to where it started before it lets go. */
  async stop(): Promise<void> {
    if (!this.activeFlag) return;
    this.stopRequested = true;
    this.abort.abort();
    this.resolveSave?.(false);
    this.messageText = "Stopping calibration...";
    if (this.phase !== "running") this.phase = "stopping";
    const timer = new AbortController();
    await Promise.race([this.done, this.clock.sleep(STOP_JOIN_MS, timer.signal)]);
    timer.abort();
    if (this.activeFlag) await this.finish("Calibration stopped", "idle");
  }

  /** An e-stop: an automatic run switches every servo off at once instead of returning to its start pose (the arm
  drops); a hand calibration is cancelled as by stop(). Resolves once the session has ended. */
  async halt(): Promise<void> {
    if (!this.activeFlag) return;
    this.haltAbort.abort();
    await this.stop();
  }

  private async worker(): Promise<void> {
    // Let the constructor return before the first exchange.
    await Promise.resolve();
    try {
      const bus = this.link.bus;
      await this.link.arbiter.run(async () => {
        await this.handshake(bus);
        if (this.stopRequested) throw new Stopped("Calibration cancelled");
        // Every servo must answer, fault-free, with its torque off before anything is written: a servo whose state is
        // unknown may be holding the arm up.
        const states = await bus.syncReadState(SO101_IDS);
        const silent = SO101_MOTORS.filter((_, i) => !states.has(SO101_IDS[i]!));
        if (silent.length) throw new So101Error(`${silent.join(", ")} did not report ${silent.length > 1 ? "their" : "its"} torque state; nothing was switched. Check the cables and try again`);
        const faulted = SO101_MOTORS.flatMap((m, i) => (isFault(states.get(SO101_IDS[i]!)!) ? [`${m} (${errorText(states.get(SO101_IDS[i]!)!.error || 0x20)})`] : []));
        if (faulted.length) throw new So101Error(`${faulted.join(", ")} reports a fault; nothing was switched. Support the arm, then switch its power off and on`);
        const on = SO101_MOTORS.filter((_, i) => states.get(SO101_IDS[i]!)!.torque !== 0);
        if (on.length) {
          throw new TorqueOnError(
            `The ${this.side} ${this.role}'s ${on.join(", ")} ${on.length > 1 ? "have" : "has"} torque on: another program may be holding this arm up, and calibration switches it off. Close it, or support the arm and switch its power off and on, then try again`,
          );
        }
        this.baseline = await readHeldCalibration(bus);
      }, { label: "calibration checks" });
      this.touched = true;
      const entries = this.method === "auto" ? await this.auto(bus) : await this.manual(bus);
      // No stop from here on: the servos are about to hold the new calibration.
      this.phase = "saving";
      this.messageText = "Saving calibration...";
      const final = this.role === "follower" ? this.aligned(entries) : entries;
      await this.link.arbiter.run(async () => {
        await this.writeToServos(bus, final);
        const held = await readHeldCalibration(bus);
        const differ = calibrationDifferences(final, held);
        if (differ.length) throw new So101Error(`The servos did not keep the calibration: ${differ.join(", ")}. Calibrate the arm again`);
      }, { label: "calibration write" });
      this.baseline = null; // the servos hold the new calibration: nothing to roll back
      await this.store.save(this.name, final);
      await this.store.clearIncomplete(this.name);
      this.saved = final;
      await this.finish("Calibration completed successfully", "completed");
    } catch (exc) {
      if ((exc instanceof Stopped || exc instanceof AutoCalStopped) && this.stopRequested) return await this.finish("Calibration cancelled", "idle");
      this.options.log?.(`SO-101 calibration error: ${message(exc)}`);
      // Where an automatic run was, and what it last did: the operator sees this in the error, not only in a log.
      const where = this.method === "auto" && this.step ? ` (at: ${this.step}; last: ${this.tail.slice(-4).join(" | ")})` : "";
      await this.finish(`Calibration failed: ${message(exc)}${where}`, "error", `${message(exc)}${where}`);
    } finally {
      if (this.activeFlag) await this.finish("Calibration stopped", "idle");
    }
  }

  /** LeRobot's handshake on every servo: a ping and the STS3215's model number. */
  private async handshake(bus: FeetechBus): Promise<void> {
    const missing: string[] = [];
    for (const [i, id] of SO101_IDS.entries()) {
      try {
        if (!(await bus.ping(id))) throw new Error("no reply");
        const model = await bus.read(id, "Model_Number");
        if (model !== STS3215_MODEL) throw new Error(`model ${model}, not an STS3215`);
      } catch (exc) {
        missing.push(`servo ${id} (${SO101_MOTORS[i]}): ${message(exc)}`);
      }
    }
    if (missing.length === SO101_IDS.length) throw new So101Error(`No SO-101 servo answered on ${this.portName}. Check that the arm is powered and plugged in`);
    if (missing.length) throw new So101Error(`${missing.join("; ")}. Reseat that servo's cable at both ends and check the arm's power supply`);
  }

  /** Every servo off and its EEPROM unlocked (Lock read back 0, so the calibration writes that follow persist across
  a power cycle); throws naming one that won't, before anything is written. */
  private async disableAll(bus: FeetechBus): Promise<void> {
    const refused: string[] = [];
    const locked: string[] = [];
    for (const [i, id] of SO101_IDS.entries()) {
      let ok = false;
      for (let attempt = 0; attempt < 3 && !ok; attempt++) ok = await switchOff(bus, id).catch(() => false);
      if (!ok) refused.push(`${SO101_MOTORS[i]} (id ${id})`);
      else if (!(await unlockEeprom(bus, id))) locked.push(`${SO101_MOTORS[i]} (id ${id})`);
    }
    if (refused.length) throw new So101Error(`${refused.join(", ")} did not switch off: support the arm, then switch its power off and on`);
    if (locked.length) throw new So101Error(`${locked.join(", ")} did not unlock its EEPROM, so a calibration would not survive a power cycle. Check the cables and calibrate again`);
  }

  // ---------- by hand ----------

  /** Throws Stopped once stop() was called (checked after every step before the recording). */
  private checkStop(): void {
    if (this.stopRequested) throw new Stopped("Calibration cancelled");
  }

  private async manual(bus: FeetechBus): Promise<So101Calibration> {
    const arbiter = this.link.arbiter;
    // Settled by complete(), stop() (now or later) or the pose timeout.
    const saved = new Promise<boolean>((resolve) => (this.resolveSave = resolve));
    this.checkStop();
    await arbiter.run(() => this.disableAll(bus));
    this.checkStop();
    await arbiter.run(async () => {
      for (const id of SO101_IDS) await bus.write(id, "Operating_Mode", MODE_POSITION);
    });
    await this.store.markIncomplete(this.name);
    this.checkStop();
    // set_half_turn_homings: the factory's offset and limits back, the raw positions, each offset making its servo
    // read the half turn.
    this.homed = true;
    await arbiter.run(async () => {
      for (const id of SO101_IDS) {
        await bus.write(id, "Homing_Offset", 0);
        await bus.write(id, "Min_Position_Limit", 0);
        await bus.write(id, "Max_Position_Limit", MAX_TICK);
      }
    });
    const raw = await this.readRaw(bus, 3);
    const far = SO101_MOTORS.filter((m) => Math.abs(raw[m]! - HALF_TURN_TICK) > MAX_HOMING_OFFSET);
    if (far.length) throw new So101Error(`${far.map((m) => JOINT_WORDS[m]).join(", ")} cannot be zeroed here: put the arm in its middle pose and start again`);
    this.checkStop();
    await arbiter.run(async () => {
      for (const [i, motor] of SO101_MOTORS.entries()) {
        this.homing[motor] = raw[motor]! - HALF_TURN_TICK;
        await bus.write(SO101_IDS[i]!, "Homing_Offset", this.homing[motor]!);
      }
    });
    this.checkStop();
    this.now = await this.readRaw(bus, 3);
    this.low = { ...this.now };
    this.high = { ...this.now };
    this.phase = "recording";
    this.messageText = "Move every joint through its full range, both ways.";
    this.checkStop();
    const timer = new AbortController();
    void this.clock.sleep(POSE_TIMEOUT_MS, timer.signal).then(() => {
      if (!timer.signal.aborted) this.resolveSave?.(false);
    });
    let ended = false;
    void saved.then(() => (ended = true));
    let error: unknown = null;
    while (!ended && !this.stopRequested) {
      try {
        this.record(await this.readRaw(bus, 1));
      } catch (exc) {
        if (exc instanceof So101Error) {
          error = exc;
          break;
        }
        // a missed read: the next one counts
      }
      await this.clock.sleep(RECORD_MS, this.abort.signal);
    }
    timer.abort();
    if (error) throw error;
    if (!(await saved) || this.stopRequested) throw new Stopped("Calibration cancelled");
    this.record(await this.readRaw(bus, 3).catch(() => this.now));
    return this.recorded();
  }

  /** One raw position per servo (sync read), retried. */
  private async readRaw(bus: FeetechBus, attempts: number): Promise<Record<string, number>> {
    for (let a = 0; ; a++) {
      const read = await this.link.arbiter.run(() => bus.syncRead(SO101_IDS, "Present_Position"));
      if (SO101_IDS.every((id) => read.has(id))) return Object.fromEntries(SO101_MOTORS.map((m, i) => [m, read.get(SO101_IDS[i]!)!]));
      if (a + 1 >= attempts) throw new Error(`No position from servo ${SO101_IDS.filter((id) => !read.has(id)).join(", ")}`);
      await this.clock.sleep(20);
    }
  }

  /** calibrate.py's recording: glitches skipped, a wrap refused, min and max widened. */
  private record(now: Record<string, number>): void {
    for (const motor of SO101_MOTORS) {
      const t = now[motor]!;
      if (t <= GLITCH_LOW || t >= GLITCH_HIGH) continue;
      if (motor !== "wrist_roll" && Math.abs(t - this.now[motor]!) > WRAP_JUMP_TICKS) {
        throw new So101Error(`The ${JOINT_WORDS[motor]} jumped from ${this.now[motor]} to ${t}: it crossed the encoder's wrap. Put the arm in its middle pose and start again`);
      }
      this.now[motor] = t;
      this.low[motor] = Math.min(this.low[motor]!, t);
      this.high[motor] = Math.max(this.high[motor]!, t);
    }
  }

  /** ArmCalibration.finish's checks (and calibrate.py's centering for a leader); the wrist roll gets the whole turn. */
  private recorded(): So101Calibration {
    const out = {} as So101Calibration;
    for (const [i, motor] of SO101_MOTORS.entries()) {
      let low = this.low[motor]!;
      let high = this.high[motor]!;
      if (motor === "wrist_roll") {
        low = 0;
        high = MAX_TICK;
      } else if (this.role === "leader" && motor !== "gripper" && Math.abs(HALF_TURN_TICK - (low + high) / 2) > CENTERING_SHARE * (high - low)) {
        throw new So101Error(`The ${JOINT_WORDS[motor]} wasn't near the middle of its range when you pressed Start: put the arm in the middle pose and calibrate again`);
      }
      out[motor] = { id: SO101_IDS[i]!, drive_mode: 0, homing_offset: this.homing[motor]!, range_min: low, range_max: high };
    }
    const short = shortJoints(out, this.side, this.role === "follower" ? "side" : null);
    if (short.length) {
      throw new So101Error(`Move ${short.join(", ")} all the way from one end to the other: each bar fills when it has gone far enough`);
    }
    return out;
  }

  // ---------- automatic ----------

  private async auto(bus: FeetechBus): Promise<So101Calibration> {
    await this.store.markIncomplete(this.name);
    this.homed = true;
    this.phase = "running";
    this.messageText = "Finding joint limits. Keep the area clear.";
    // The whole procedure is one transaction: nothing else may talk on the bus while it drives the arm.
    const result = await this.link.arbiter.run(() =>
      autoCalibrateSo101(bus, {
        clock: this.link.seconds,
        signal: this.abort.signal,
        halt: this.haltAbort.signal,
        torqueLimit: this.options.torqueLimit,
        onProgress: (progress, text) => {
          this.progress = progress;
          this.messageText = text;
          this.step = text;
        },
        log: (line) => {
          this.options.log?.(line);
          this.tail.push(line);
          if (this.tail.length > 12) this.tail.shift();
        },
      }),
    );
    // The procedure leaves the servos holding its result; an interrupted one leaves them as they were then (the
    // marker stays: calibrate again before running).
    this.baseline = null;
    const calibration = Object.fromEntries(SO101_MOTORS.map((m, i) => [m, { ...result[m]!, id: SO101_IDS[i]!, drive_mode: 0 }])) as So101Calibration;
    // A probe that stalled early (an obstruction, a cable caught) found a false stop: refused, the marker kept.
    const short = shortJoints(calibration, this.side, "either");
    if (short.length) {
      throw new So101Error(`The ${short.join(", ")} stopped well short of an SO-101's range (something held ${short.length > 1 ? "them" : "it"}?). Clear the arm's way and calibrate it again`);
    }
    return calibration;
  }

  // ---------- both ----------

  private aligned(entries: So101Calibration): So101Calibration {
    const { calibration, movedDeg, notes } = alignToReference(this.side, entries);
    const moved = Object.entries(movedDeg).map(([m, d]) => `${JOINT_WORDS[m]} ${d > 0 ? "+" : ""}${d}°`);
    if (moved.length) this.options.onEvent?.("system", `${this.side} follower: zero matched to Dream's SO-101 (${moved.join(", ")}).`);
    for (const note of notes) this.options.onEvent?.("system", `${this.side} follower: ${note}.`);
    return calibration;
  }

  /** write_calibration: each servo's homing offset and limits (EEPROM, torque off, Lock 0 so they persist). */
  private async writeToServos(bus: FeetechBus, calibration: So101Calibration): Promise<void> {
    await this.disableAll(bus);
    for (const motor of SO101_MOTORS) {
      const c = calibration[motor];
      await bus.write(c.id, "Homing_Offset", c.homing_offset);
      await bus.write(c.id, "Min_Position_Limit", c.range_min);
      await bus.write(c.id, "Max_Position_Limit", c.range_max);
    }
  }

  /** Every exit: a hand calibration cancelled or failed after its homing writes gets the servos' old calibration back
  (and the marker cleared once that is confirmed); torque off; the port closed if the session owns it. The first
  finish wins. */
  private async finish(text: string, phase: "idle" | "completed" | "error", error?: string): Promise<void> {
    if (this.finished) return;
    this.finished = true;
    const bus = this.link.bus;
    try {
      await this.link.arbiter.run(async () => {
        if (this.homed && this.baseline && this.method === "manual") {
          await this.writeToServos(bus, this.baseline);
          if (!calibrationDifferences(this.baseline, await readHeldCalibration(bus)).length) await this.store.clearIncomplete(this.name).catch(() => undefined);
        } else if (this.touched) {
          await this.disableAll(bus);
        }
      }, { priority: true, label: "calibration cleanup" });
    } catch (exc) {
      this.options.onEvent?.("error", `${this.side} ${this.role}: releasing the arm after calibration: ${message(exc)}`);
    }
    if (this.ownsLink) await this.link.close().catch(() => undefined);
    this.activeFlag = false;
    this.phase = phase;
    this.messageText = text;
    if (error !== undefined) this.errorText = error;
    if (phase === "completed") this.options.onEvent?.("system", `Calibration saved as ${this.name}.`);
    else if (phase === "error") this.options.onEvent?.("error", `Calibration failed: ${this.errorText || text}`);
  }
}

/** Opens `port`, calibrates the arm on it, and closes the port when the session ends. */
export async function calibrateSo101(port: SerialPortLike, clock: Clock, options: So101CalibrationOptions): Promise<So101CalibrationSession> {
  const link = await openSo101Link(port, clock);
  return new So101CalibrationSession(link, options, true, portLabel(port));
}
