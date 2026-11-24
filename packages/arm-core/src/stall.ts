/**
 * Motor-state recording and persistent stall detection for powered control loops.
 *
 * The recording is CSV text with one row per motor per tick, handed to a caller-supplied sink (a
 * browser has no filesystem): collect it, stream it to a download, or write it to a file in Node.
 * The columns and their formatting are the Python SDK's.
 */

import { ValueError } from "./errors.js";

/** When a motor counts as stalled. */
export class StallThresholds {
  readonly errorRad: number;
  readonly velocityRadS: number;
  readonly torqueNm: number;
  readonly persistenceS: number;
  readonly maxFeedbackAgeS: number;

  constructor(options: Partial<Pick<StallThresholds, "errorRad" | "velocityRadS" | "torqueNm" | "persistenceS" | "maxFeedbackAgeS">> = {}) {
    this.errorRad = options.errorRad ?? 0.12;
    this.velocityRadS = options.velocityRadS ?? 0.04;
    this.torqueNm = options.torqueNm ?? 0.5;
    this.persistenceS = options.persistenceS ?? 0.5;
    this.maxFeedbackAgeS = options.maxFeedbackAgeS ?? 0.15;
    Object.freeze(this);
  }
}

/** A confirmed stall. */
export class StallEvent {
  constructor(
    readonly motorId: number,
    readonly candidateSinceS: number,
    readonly confirmedAtS: number,
    readonly errorRad: number,
    readonly velocityRadS: number,
    readonly torqueNm: number,
  ) {
    Object.freeze(this);
  }
}

/** One motor's sample for StallDetector. */
export interface StallSample {
  error: number;
  velocity: number;
  torque: number;
  feedbackAge: number;
  faulted: boolean;
  /** Default true. */
  enabled?: boolean;
}

/** Track independent stall candidates and preserve their confirmation order. */
export class StallDetector {
  readonly thresholds: StallThresholds;
  /** Motor id -> when it became a candidate (s), or null. */
  readonly candidateSince: Map<number, number | null>;
  readonly events: StallEvent[] = [];
  #confirmed = new Set<number>();

  constructor(motorIds: readonly number[], thresholds: StallThresholds) {
    this.thresholds = thresholds;
    this.candidateSince = new Map(motorIds.map((id) => [id, null]));
  }

  /** Large error, little motion, real torque, fresh feedback, enabled and not faulted. */
  isCandidate(sample: StallSample): boolean {
    const { error, velocity, torque, feedbackAge, faulted, enabled = true } = sample;
    const t = this.thresholds;
    return [error, velocity, torque, feedbackAge].every((v) => Number.isFinite(v))
      && Math.abs(error) >= t.errorRad
      && Math.abs(velocity) <= t.velocityRadS
      && Math.abs(torque) >= t.torqueNm
      && feedbackAge <= t.maxFeedbackAgeS
      && enabled
      && !faulted;
  }

  /** Feed one motor's sample at `nowS`; true while it is a candidate. Appends to `events` on confirmation. */
  update(nowS: number, motorId: number, sample: StallSample): boolean {
    const candidate = this.isCandidate(sample);
    if (this.#confirmed.has(motorId)) return candidate;
    if (!candidate) {
      this.candidateSince.set(motorId, null);
      return false;
    }
    if (!this.candidateSince.has(motorId)) throw new ValueError(`motor ${motorId} is not tracked`);
    const since = this.candidateSince.get(motorId)!;
    if (since === null) {
      this.candidateSince.set(motorId, nowS);
      return true;
    }
    if (nowS - since >= this.thresholds.persistenceS - 1e-9) {
      this.#confirmed.add(motorId);
      this.events.push(new StallEvent(motorId, since, nowS, sample.error, sample.velocity, sample.torque));
    }
    return true;
  }
}

/** Python's `f"{x:.{digits}f}"`: exact decimal rounding, ties to even. */
export function pyFixed(x: number, digits: number): string {
  if (Number.isNaN(x)) return "nan";
  if (x === Infinity) return "inf";
  if (x === -Infinity) return "-inf";
  const negative = x < 0 || Object.is(x, -0);
  const a = Math.abs(x);
  let s = a.toFixed(digits);
  if (a < 1e21) {
    // toFixed rounds an exact tie away from zero; Python rounds it to even.
    const exact = a.toFixed(100);
    const point = exact.indexOf(".");
    const rest = exact.slice(point + 1 + digits);
    if (/^50*$/.test(rest)) {
      const kept = exact.slice(0, point + 1 + digits).replace(/\.$/, "");
      const lastDigit = Number(kept.replace(".", "").slice(-1));
      if (lastDigit % 2 === 0) s = kept;
    }
  }
  return (negative ? "-" : "") + s;
}

/** Python's str() of a float. */
function pyFloat(x: number): string {
  if (Number.isNaN(x)) return "nan";
  if (!Number.isFinite(x)) return x > 0 ? "inf" : "-inf";
  if (Number.isInteger(x) && Math.abs(x) < 1e16) return (Object.is(x, -0) ? "-" : "") + Math.abs(x).toFixed(1);
  return String(x);
}

/** A CSV field as Python's csv module writes it (minimal quoting). */
function csvField(value: string): string {
  return /[",\r\n]/.test(value) ? `"${value.replace(/"/g, '""')}"` : value;
}

/** Local time as Python's `datetime.now().astimezone().isoformat(timespec=...)`. */
function isoLocal(d: Date, milliseconds: boolean): string {
  const p = (n: number, w = 2): string => String(n).padStart(w, "0");
  const offset = -d.getTimezoneOffset();
  const sign = offset >= 0 ? "+" : "-";
  const abs = Math.abs(offset);
  return `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())}T${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}`
    + (milliseconds ? `.${p(d.getMilliseconds(), 3)}` : "") + `${sign}${p(Math.floor(abs / 60))}:${p(abs % 60)}`;
}

/** Python's `f"{x:g}"`. */
function pyG(x: number): string {
  if (Number.isNaN(x)) return "nan";
  if (!Number.isFinite(x)) return x > 0 ? "inf" : "-inf";
  if (x === 0) return Object.is(x, -0) ? "-0" : "0";
  const exp = Number(x.toExponential(5).split("e")[1]);
  if (exp < -4 || exp >= 6) {
    const [m, e] = x.toExponential(5).split("e") as [string, string];
    const mant = m.includes(".") ? m.replace(/0+$/, "").replace(/\.$/, "") : m;
    const n = Number(e);
    return `${mant}e${n < 0 ? "-" : "+"}${String(Math.abs(n)).padStart(2, "0")}`;
  }
  const s = x.toFixed(Math.max(0, 5 - exp));
  return s.includes(".") ? s.replace(/0+$/, "").replace(/\.$/, "") : s;
}

/** The name the Python SDK gives a recording: `<directory>/teleop_motor_states_<local time>_<payload>kg.csv`. */
export function defaultStallLogPath(payloadKg: number, directory = "logs", now: Date = new Date()): string {
  const p = (n: number): string => String(n).padStart(2, "0");
  const stamp = `${now.getFullYear()}${p(now.getMonth() + 1)}${p(now.getDate())}_${p(now.getHours())}${p(now.getMinutes())}${p(now.getSeconds())}`;
  return `${directory}/teleop_motor_states_${stamp}_${pyG(payloadKg)}kg.csv`;
}

/** Where MotorStateRecorder writes. */
export interface RecordSink {
  /** CSV text: the header line first, then rows (each line ends with "\r\n"). */
  write(text: string): void;
  /** Called at most every 0.25 s of recording time. */
  flush?(): void;
  /** Called once by MotorStateRecorder.close(), after the last row. */
  close?(): void;
  /** The stall summary as JSON text, once, at close. */
  summary?(json: string): void;
}

/** What MotorStateRecorder reads of an Arm. */
export interface RecordedArm {
  readonly state: unknown;
  readonly config: { readonly joints: readonly { readonly motorId: number; readonly model: string }[] };
  readonly motors: readonly {
    readonly feedback: { readonly enabled: boolean | null } | null;
    readonly feedbackAge: number;
    readonly status: { readonly code: number; readonly hasFault: boolean; readonly text: string } | null;
    readonly statusAge: number;
    readonly faultLatched: string | null;
    readonly torqueLimit: number;
  }[];
  getCommandedPositions(): number[];
  getJointPositions(): number[];
  getJointVelocities(): number[];
  getJointTorques(): number[];
  getTemperatures(): number[];
}

/** Options of the MotorStateRecorder constructor. */
export interface MotorStateRecorderOptions {
  /** The recording's name or path, for the summary's "csv" entry (default ""). */
  name?: string;
  /** Wall time for the rows and the summary (default: `() => new Date()`). */
  now?: () => Date;
}

interface Peak {
  max_abs_error_rad: number;
  max_abs_torque_nm: number;
  max_temperature_c: number;
}

/** The summary close() returns (keys as in the Python SDK's JSON file). */
export interface StallSummary {
  started_at: string;
  payload_kg: number;
  stop_reason: string;
  thresholds: Record<string, number>;
  first_stall: Record<string, number> | null;
  stall_events: Array<Record<string, number>>;
  motor_peaks: Record<string, Record<string, number | null>>;
  csv: string;
}

const eventJson = (e: StallEvent): Record<string, number> => ({
  motor_id: e.motorId, candidate_since_s: e.candidateSinceS, confirmed_at_s: e.confirmedAtS, error_rad: e.errorRad,
  velocity_rad_s: e.velocityRadS, torque_nm: e.torqueNm,
});

/** Record one row per motor per teleop tick and produce a stall summary. */
export class MotorStateRecorder {
  static readonly FIELDS = [
    "elapsed_s", "wall_time", "payload_kg", "joint", "motor_id", "model",
    "leader_target_rad", "command_rad", "position_rad", "error_rad",
    "velocity_rad_s", "torque_nm", "torque_fraction", "temperature_c",
    "enabled", "status_code", "fault_text", "feedback_age_s",
    "stall_candidate", "stall_confirmed", "status_age_s",
  ] as const;

  readonly sink: RecordSink;
  readonly payloadKg: number;
  readonly arm: RecordedArm;
  readonly thresholds: StallThresholds;
  readonly detector: StallDetector;
  readonly startedAt: Date;
  /** Motor id -> peak |error|, |torque| and temperature. */
  readonly peaks: Map<number, Peak>;
  readonly name: string;
  #now: () => Date;
  #lastFlushS = -Infinity;
  #summary: StallSummary | null = null;

  constructor(sink: RecordSink, payloadKg: number, arm: RecordedArm, thresholds: StallThresholds,
              options: MotorStateRecorderOptions = {}) {
    this.sink = sink;
    this.payloadKg = payloadKg;
    this.arm = arm;
    this.thresholds = thresholds;
    this.name = options.name ?? "";
    this.#now = options.now ?? (() => new Date());
    sink.write(MotorStateRecorder.FIELDS.join(",") + "\r\n");
    this.detector = new StallDetector(arm.config.joints.map((j) => j.motorId), thresholds);
    this.startedAt = this.#now();
    this.peaks = new Map(arm.config.joints.map((j) => [j.motorId, {
      max_abs_error_rad: 0.0, max_abs_torque_nm: 0.0, max_temperature_c: -Infinity,
    }]));
  }

  /** Record one tick; returns the stalls confirmed in it. */
  record(elapsedS: number, leaderTargets: readonly number[]): StallEvent[] {
    const arm = this.arm;
    const commanded = arm.getCommandedPositions();
    const positions = arm.getJointPositions();
    const velocities = arm.getJointVelocities();
    const torques = arm.getJointTorques();
    const temperatures = arm.getTemperatures();
    const eventCount = this.detector.events.length;
    const enabledArm = String(arm.state) === "enabled";
    const n = Math.min(arm.config.joints.length, arm.motors.length);
    const samples: Array<[number, string, boolean, number]> = [];
    for (let index = 0; index < n; index++) {
      const joint = arm.config.joints[index]!;
      const motor = arm.motors[index]!;
      const feedback = motor.feedback;
      const enabled = feedback === null || feedback.enabled === null ? "" : feedback.enabled ? "1" : "0";
      const status = motor.status;
      const faulted = Boolean(motor.faultLatched) || Boolean(status && status.hasFault);
      const error = commanded[index]! - positions[index]!;
      const candidate = this.detector.update(elapsedS, joint.motorId, {
        error, velocity: velocities[index]!, torque: torques[index]!, feedbackAge: motor.feedbackAge, faulted,
        enabled: enabledArm,
      });
      const peak = this.peaks.get(joint.motorId)!;
      peak.max_abs_error_rad = pyMaxNum(peak.max_abs_error_rad, Math.abs(error));
      peak.max_abs_torque_nm = pyMaxNum(peak.max_abs_torque_nm, Math.abs(torques[index]!));
      peak.max_temperature_c = pyMaxNum(peak.max_temperature_c, temperatures[index]!);
      samples.push([index, enabled, candidate, error]);
    }

    const confirmed = new Set(this.detector.events.map((e) => e.motorId));
    const wallTime = isoLocal(this.#now(), true);
    let text = "";
    for (const [index, enabled, candidate, error] of samples) {
      const joint = arm.config.joints[index]!;
      const motor = arm.motors[index]!;
      const status = motor.status;
      const torqueLimit = motor.torqueLimit;
      if (torqueLimit === 0) throw new ValueError("float division by zero");
      const statusText = motor.faultLatched || (status ? status.text : "");
      const row: Record<(typeof MotorStateRecorder.FIELDS)[number], string> = {
        elapsed_s: pyFixed(elapsedS, 6),
        wall_time: wallTime,
        payload_kg: pyFloat(this.payloadKg),
        joint: String(index + 1),
        motor_id: String(joint.motorId),
        model: joint.model,
        leader_target_rad: pyFixed(leaderTargets[index]!, 7),
        command_rad: pyFixed(commanded[index]!, 7),
        position_rad: pyFixed(positions[index]!, 7),
        error_rad: pyFixed(error, 7),
        velocity_rad_s: pyFixed(velocities[index]!, 7),
        torque_nm: pyFixed(torques[index]!, 7),
        torque_fraction: pyFixed(Math.abs(torques[index]!) / torqueLimit, 6),
        temperature_c: pyFixed(temperatures[index]!, 2),
        enabled,
        status_code: status === null ? "" : `0x${status.code.toString(16).toUpperCase().padStart(8, "0")}`,
        fault_text: statusText,
        feedback_age_s: pyFixed(motor.feedbackAge, 6),
        stall_candidate: candidate ? "1" : "0",
        stall_confirmed: confirmed.has(joint.motorId) ? "1" : "0",
        status_age_s: pyFixed(motor.statusAge, 6),
      };
      text += MotorStateRecorder.FIELDS.map((f) => csvField(row[f])).join(",") + "\r\n";
    }
    if (text) this.sink.write(text);
    if (elapsedS - this.#lastFlushS >= 0.25) {
      this.sink.flush?.();
      this.#lastFlushS = elapsedS;
    }
    return this.detector.events.slice(eventCount);
  }

  /** End the recording and return the stall summary (also handed to `sink.summary` as JSON). Repeat calls return the first summary. */
  close(stopReason: string): StallSummary {
    if (this.#summary !== null) return this.#summary;
    this.sink.flush?.();
    this.sink.close?.();
    const events = [...this.detector.events].sort((a, b) =>
      a.candidateSinceS - b.candidateSinceS || a.confirmedAtS - b.confirmedAtS || a.motorId - b.motorId);
    const cleanPeaks: Record<string, Record<string, number | null>> = {};
    for (const [motorId, values] of this.peaks) {
      cleanPeaks[String(motorId)] = Object.fromEntries(
        Object.entries(values).map(([name, value]) => [name, Number.isFinite(value) ? value : null]));
    }
    const t = this.thresholds;
    this.#summary = {
      started_at: isoLocal(this.startedAt, false),
      payload_kg: this.payloadKg,
      stop_reason: stopReason,
      thresholds: {
        error_rad: t.errorRad, velocity_rad_s: t.velocityRadS, torque_nm: t.torqueNm, persistence_s: t.persistenceS,
        max_feedback_age_s: t.maxFeedbackAgeS,
      },
      first_stall: events.length ? eventJson(events[0]!) : null,
      stall_events: events.map(eventJson),
      motor_peaks: cleanPeaks,
      csv: this.name,
    };
    this.sink.summary?.(JSON.stringify(this.#summary, null, 2));
    return this.#summary;
  }
}

/** Python's max(a, b) on floats (a NaN `b` never replaces `a`). */
const pyMaxNum = (a: number, b: number): number => (b > a ? b : a);
