/**
 * RobStride motor behind the Motor seam: one CAN ID = one motor. Commands write a frame, queries
 * wait for the reply, feedback is timestamped. Everything RobStride-specific that the arm layer
 * needs lives here (frame routing, fault query, RUN_MODE check, mode bits, CAN-timeout watchdog,
 * firmware parameter support); protocol.ts stays a pure codec.
 *
 * Fault evidence is preserved: once a motor reports a fault (fault word or feedback fault flag),
 * every command that would clear it is refused until the caller explicitly runs clearFault().
 */

import {
  ArmError, type CanBackend, type Clock, ConnectError, FaultLatched, type JointConfig, type Logger, type Motor,
  type MotorFeedback, type MotorStatus, Mutex, type ParamSpec, ParamTimeout, Signal, ValueError,
} from "@makermods/arm-core";
import * as protocol from "./protocol.js";

export const MODELS: readonly string[] = Object.freeze(Object.keys(protocol.MOTOR_PARAMS));
export const DEFAULT_MODEL = "RS00";
export const KP_RANGE: readonly [number, number] = [protocol.KP_MIN, protocol.KP_MAX];
export const KD_RANGE: readonly [number, number] = [protocol.KD_MIN, protocol.KD_MAX];
export const ID_RANGE: readonly [number, number] = [1, 127];
export const DEFAULT_HOST_ID = protocol.HOST_CAN_ID;

/**
 * Motor ID a received frame belongs to, or null. Parameter replies carry it in the CAN ID (their
 * data[0] is a register byte); MIT replies arrive on the host ID with it in data[0].
 */
export function route(canId: number, data: Uint8Array, hostId: number = DEFAULT_HOST_ID, extended = false): number | null {
  if (extended || !(canId >= 0 && canId <= 0x7ff)) return null;
  if ((canId & 0x700) === 0x300 || (canId & 0x700) === 0x400) return canId & 0xff;
  if (canId === hostId && data.length) return data[0]!;
  return null;
}

/** One feedback sample, motor coordinates, with the RobStride mode bits. */
export interface RobStrideFeedback extends MotorFeedback {
  /** RobStride: 0 reset, 1 calibration, 2 run. */
  readonly mode: number;
  readonly faultFlag: boolean;
}

/** Named parameters readParam/writeParam accept. */
export const PARAMS: Readonly<Record<string, ParamSpec>> = Object.freeze({
  run_mode: { index: protocol.ParamIndex.RUN_MODE, dtype: "u8", doc: "control mode, 0 = MIT" },
  mech_pos: { index: protocol.ParamIndex.MECH_POS, dtype: "f", doc: "mechanical position, rad" },
  can_timeout: {
    index: protocol.ParamIndex.CAN_TIMEOUT, dtype: "u32", doc: "CAN timeout, 20 ticks per ms; 0 disables the watchdog",
  },
});

/**
 * A RobStride motor on the MIT protocol (the Python SDK's maker_arm.motor.Motor).
 */
export class RobStrideMotor implements Motor {
  /** Consecutive health checks a motor may report mode != run. */
  static readonly MODE_FAULT_TICKS = 5;
  static readonly PARAMS = PARAMS;

  readonly joint: JointConfig;
  motorId: number;
  readonly params: Readonly<Record<string, ParamSpec>> = PARAMS;
  readonly changed = new Signal();
  /** The MIT mapping ranges of this model. */
  readonly scale: protocol.MotorParams;
  /**
   * null = unknown, true/false after probeParams(). Typed parameter access needs RS00 >= 0.0.3.27 /
   * RS02 >= 0.2.3.32; that firmware also reports mode/fault bits.
   */
  paramsSupported: boolean | null = null;

  #backend: CanBackend;
  #hostId: number;
  #clock: Clock;
  #log: Logger;
  #feedback: RobStrideFeedback | null = null;
  #fbTime: number | null = null;
  #feedbackSequence = 0;
  #status: MotorStatus | null = null;
  #statusTime: number | null = null;
  #statusSequence = 0;
  #latched: string | null = null;
  #queryLock = new Mutex();
  #paramReply: protocol.ParamReply | null = null;
  #paramExpect: number | null = null;
  #modeBad = 0;

  constructor(joint: JointConfig, backend: CanBackend, hostId: number, clock: Clock, logger: Logger = console) {
    this.joint = joint;
    this.motorId = joint.motorId;
    this.#backend = backend;
    this.#hostId = hostId;
    this.#clock = clock;
    this.#log = logger;
    const scale = protocol.MOTOR_PARAMS[joint.model];
    if (!scale) throw new ValueError(`unknown model '${joint.model}'`);
    this.scale = scale;
  }

  /** Largest torque the MIT frame can carry for this model, N·m. */
  get torqueLimit(): number {
    return this.scale.tMax;
  }

  // -- receive path (invoked by the RX dispatcher) --
  handleFrame(canId: number, data: Uint8Array, extended = false): void {
    const msg = protocol.parseFrame(canId, data, { params: this.scale, hostId: this.#hostId, extended });
    if (msg === null || msg.motorId !== this.motorId) return;
    if (msg.kind === "feedback") {
      this.#feedback = {
        motorId: msg.motorId, position: msg.position, velocity: msg.velocity, torque: msg.torque,
        temperature: msg.temperature, temperatures: { motor: msg.temperature },
        enabled: this.paramsSupported ? msg.mode === protocol.MODE_RUN : null,
        warning: msg.warningFlag, mode: msg.mode, faultFlag: msg.faultFlag,
      };
      this.#fbTime = this.#clock.now();
      this.#feedbackSequence += 1;
      if (msg.faultFlag && this.#latched === null) this.#latched = "feedback fault flag set";
    } else if (msg.kind === "status") {
      this.#status = {
        motorId: msg.motorId, hasFault: msg.hasFault, code: msg.word, text: msg.hasFault ? msg.text : "", enabled: null,
      };
      this.#statusTime = this.#clock.now();
      this.#statusSequence += 1;
      if (msg.hasFault && this.#latched === null) {
        this.#latched = `fault word 0x${msg.word.toString(16).toUpperCase().padStart(8, "0")} (${msg.text})`;
      }
    } else if (!msg.write && msg.index === this.#paramExpect) {
      this.#paramReply = msg;
    } else {
      return;
    }
    this.changed.notify();
  }

  // -- cached state --
  get feedback(): RobStrideFeedback | null {
    return this.#feedback;
  }

  get feedbackAge(): number {
    return this.#fbTime === null ? Infinity : this.#clock.now() - this.#fbTime;
  }

  /** Monotonic count used to distinguish a fresh reply from cached feedback. */
  get feedbackSequence(): number {
    return this.#feedbackSequence;
  }

  get status(): MotorStatus | null {
    return this.#status;
  }

  get statusAge(): number {
    return this.#statusTime === null ? Infinity : this.#clock.now() - this.#statusTime;
  }

  /** Reason a fault was latched, or null. */
  get faultLatched(): string | null {
    return this.#latched;
  }

  /**
   * Does the motor report only the state its CAN watchdog leaves after commands stopped? Never on
   * RobStride: the timeout puts it in reset mode with no fault, which reads as torque off.
   */
  get watchdogReleased(): boolean {
    return false;
  }

  /** Test hook: when the last status reply arrived (clock seconds). */
  set statusTime(t: number | null) {
    this.#statusTime = t;
  }

  resetCache(): void {
    this.#feedback = null;
    this.#fbTime = null;
    this.#status = null;
    this.#statusTime = null;
    this.#latched = null;
    this.#modeBad = 0;
  }

  #send(frame: protocol.Frame): Promise<void> {
    return this.#backend.send(frame[0], frame[1], false);
  }

  // -- commands --
  enable(): Promise<void> {
    this.#modeBad = 0;
    return this.#send(protocol.encodeEnable(this.motorId));
  }

  disable(): Promise<void> {
    return this.#send(protocol.encodeDisable(this.motorId));
  }

  sendCommand(pos: number, vel: number, kp: number, kd: number, tau: number): Promise<void> {
    return this.#send(protocol.encodeControl(this.motorId, pos, vel, kp, kd, tau, this.scale));
  }

  #waitFeedback(sequence: number, timeout: number): Promise<boolean> {
    return this.changed.wait(() => this.#feedbackSequence !== sequence, timeout, this.#clock);
  }

  /**
   * Let late replies to earlier requests arrive, so the next reply answers ours (MIT replies carry
   * no transaction ID).
   */
  async #waitQuiet(quiet = 0.005, limit = 0.05): Promise<void> {
    const deadline = this.#clock.now() + limit;
    while (this.#clock.now() < deadline) {
      const sequence = this.#feedbackSequence;
      await this.#clock.sleep(quiet);
      if (this.#feedbackSequence === sequence) return;
    }
  }

  /** Zero-gain command at `pos` (motor coordinates); true once the motor replies. */
  async preload(pos: number, timeout = 0.05): Promise<boolean> {
    if (!Number.isFinite(pos)) return false;
    await this.#waitQuiet();
    const sequence = this.#feedbackSequence;
    await this.sendCommand(pos, 0.0, 0.0, 0.0, 0.0);
    return this.#waitFeedback(sequence, timeout);
  }

  /** Enable; true once a reply arrives that confirms torque on (any reply on old firmware). */
  async enableConfirmed(timeout = 0.05): Promise<boolean> {
    const sequence = this.#feedbackSequence;
    await this.enable();
    return (await this.#waitFeedback(sequence, timeout)) && this.confirms("enabled");
  }

  /** Send a non-clearing fault query; the reply updates status. */
  pollStatus(): Promise<void> {
    return this.#send(protocol.encodeFaultQuery(this.motorId));
  }

  /** Request feedback without changing torque state. That frame also clears faults, so it is refused once a fault is latched. */
  async requestState(): Promise<void> {
    if (this.#latched !== null) {
      throw new FaultLatched(`motor ${this.motorId}: ${this.#latched}; not sending a fault-clearing read`);
    }
    await this.#send(protocol.encodeClearFault(this.motorId));
  }

  // -- queries --
  /** Fresh, non-clearing fault query. Never changes torque; does not refresh feedback. */
  async check(timeout = 0.05, attempts = 3): Promise<MotorStatus> {
    if (timeout <= 0) throw new ValueError("timeout must be positive");
    return this.#queryLock.run(async () => {
      for (let i = 0; i < attempts; i++) {
        const sequence = this.#statusSequence;
        await this.pollStatus();
        if (await this.changed.wait(() => this.#statusSequence !== sequence, timeout, this.#clock)) {
          return this.#status!;
        }
      }
      throw new ParamTimeout(`motor ${this.motorId} fault query timed out`);
    });
  }

  /**
   * Fresh feedback (not cached). Throws ParamTimeout after every attempt misses.
   *
   * RobStride's state read also clears faults, so it throws FaultLatched once a fault is latched;
   * run check() first so a fault the motor reports is latched, not cleared.
   */
  async readState(timeout = 0.05, attempts = 3): Promise<RobStrideFeedback> {
    for (let attempt = 0; attempt < attempts; attempt++) {
      const sequence = this.#feedbackSequence;
      await this.requestState();
      if (await this.#waitFeedback(sequence, timeout)) return this.#feedback!;
      if (attempt + 1 < attempts) await this.#clock.sleep(0.05);
    }
    throw new ParamTimeout(`motor ${this.motorId} did not reply to a state request`);
  }

  /** Connect-time check: firmware with parameter access must be in MIT control mode. */
  async validate(timeout = 0.12): Promise<void> {
    if (await this.probeParams(timeout, 2)) {
      const runMode = await this.readParam("run_mode", undefined, timeout);
      if (runMode !== 0) throw new ConnectError(`motor ${this.motorId} RUN_MODE is ${runMode}, not 0 (MIT control)`);
    }
  }

  /**
   * Does the latest feedback confirm "enabled" / "disabled"? Old firmware has no mode bits, so any
   * reply counts there.
   */
  confirms(expected: "enabled" | "disabled"): boolean {
    const modes = { enabled: protocol.MODE_RUN, disabled: protocol.MODE_RESET } as Record<string, number>;
    const want = modes[expected];
    if (want === undefined) throw new ValueError(`unknown state '${expected}'`);
    const fb = this.#feedback;
    if (fb === null) return false;
    return !this.paramsSupported || fb.mode === want;
  }

  /**
   * Per-tick check while enabled: the motor left run mode for MODE_FAULT_TICKS ticks. Mode bits
   * only exist on firmware with parameter access (old firmware reports 0); calibration (1) and
   * undefined (3) modes are never legitimate.
   */
  healthProblem(): string | null {
    const fb = this.#feedback;
    if (fb === null) return null;
    const bad = fb.mode === protocol.MODE_CALIBRATION || fb.mode === 3
      || (Boolean(this.paramsSupported) && fb.mode !== protocol.MODE_RUN);
    this.#modeBad = bad ? this.#modeBad + 1 : 0;
    if (this.#modeBad >= RobStrideMotor.MODE_FAULT_TICKS) {
      return `left run mode (mode=${fb.mode} for ${this.#modeBad} ticks)`;
    }
    return null;
  }

  // -- operator actions --
  /** Explicit operator action: clear the motor fault and the latched evidence. */
  clearFault(): Promise<void> {
    this.#latched = null;
    return this.#send(protocol.encodeClearFault(this.motorId));
  }

  /** Make the current position the motor's zero. Persists in the motor. */
  setZero(): Promise<void> {
    return this.#send(protocol.encodeSetZero(this.motorId));
  }

  async setId(_newId: number, _timeout = 0.5): Promise<void> {
    throw new ArmError(
      "changing a RobStride motor's CAN ID over the MIT protocol is not supported: the ID-change frame is "
      + "not documented in the sources this SDK follows. Set the ID with RobStride Motor Studio "
      + "(docs/motor-replacement.md)");
  }

  /**
   * Set and verify the motor-side CAN timeout. Returns the previous value as the restore token, or
   * null when the firmware has no parameter access.
   */
  async armWatchdog(timeoutMs: number, timeout = 0.12): Promise<number | null> {
    if (!this.paramsSupported) {
      this.#log.warn(`${this.joint.label}: firmware has no parameter access (needs RS00 >=0.0.3.27 / RS02 >=0.2.3.32); `
        + `running without its ${timeoutMs} ms motor watchdog`);
      return null;
    }
    const ticks = timeoutMs * protocol.CAN_TIMEOUT_TICKS_PER_MS;
    const original = await this.readParam("can_timeout", undefined, timeout);
    try {
      const readback = await this.writeParam("can_timeout", ticks, undefined, timeout);
      if (readback !== ticks) {
        throw new ConnectError(`${this.joint.label}: CAN timeout read back ${readback}, expected ${ticks}`);
      }
    } catch (e) {
      try {
        await this.restoreWatchdog(original, timeout);
      } catch (e2) {
        this.#log.error(`could not restore ${this.joint.label} CAN timeout: ${errorText(e2)}`);
      }
      throw e;
    }
    return original;
  }

  async restoreWatchdog(token: unknown, timeout = 0.12): Promise<void> {
    await this.writeParam("can_timeout", token as number, undefined, timeout);
  }

  // -- engineering --
  #param(nameOrIndex: string | number, dtype: string | undefined): [number, string] {
    if (typeof nameOrIndex === "string") {
      const spec = PARAMS[nameOrIndex];
      if (!spec) {
        throw new ValueError(`unknown parameter '${nameOrIndex}' (known: ${Object.keys(PARAMS).sort().join(", ")})`);
      }
      return [spec.index, dtype || spec.dtype];
    }
    for (const spec of Object.values(PARAMS)) {
      if (spec.index === nameOrIndex) return [spec.index, dtype || spec.dtype];
    }
    return [Math.trunc(nameOrIndex), dtype || "f"];
  }

  async readParam(nameOrIndex: string | number, dtype?: string, timeout = 0.12): Promise<number> {
    const [index, type] = this.#param(nameOrIndex, dtype);
    return this.#queryLock.run(async () => {
      this.#paramReply = null;
      this.#paramExpect = index;
      try {
        await this.#send(protocol.encodeReadParam(this.motorId, index));
        if (!(await this.changed.wait(() => this.#paramReply !== null, timeout, this.#clock))) {
          throw new ParamTimeout(
            `motor ${this.motorId} read param 0x${index.toString(16).toUpperCase().padStart(4, "0")} timed out after ${timeout}s`);
        }
        return this.#paramReply!.value(type);
      } finally {
        this.#paramExpect = null;
      }
    });
  }

  /** Write a parameter and return the read-back value (throws ParamTimeout). */
  async writeParam(nameOrIndex: string | number, value: number, dtype?: string, timeout = 0.12, save = false): Promise<number> {
    if (save) throw new ArmError("saving parameters to flash is not supported over the RobStride MIT protocol");
    const [index, type] = this.#param(nameOrIndex, dtype);
    await this.#send(protocol.encodeWriteParam(this.motorId, index, value, type));
    await this.#clock.sleep(0.005);
    return this.readParam(index, type, timeout);
  }

  /** Detect typed parameter support (newer firmware). Read-only. */
  async probeParams(timeout = 0.12, attempts = 2): Promise<boolean> {
    this.paramsSupported = false;
    for (let i = 0; i < attempts; i++) {
      try {
        await this.readParam("run_mode", undefined, timeout);
        this.paramsSupported = true;
        break;
      } catch (e) {
        if (e instanceof ParamTimeout) continue;
        throw e;
      }
    }
    return this.paramsSupported;
  }

  /** Disable and confirm the reply (and mode 0 where the firmware reports mode). */
  async disableConfirmed(timeout = 0.05, attempts = 3): Promise<boolean> {
    for (let i = 0; i < attempts; i++) {
      const sequence = this.#feedbackSequence;
      await this.disable();
      if ((await this.#waitFeedback(sequence, timeout)) && this.confirms("disabled")) return true;
    }
    return false;
  }
}

function errorText(e: unknown): string {
  return e instanceof Error ? e.message : String(e);
}
