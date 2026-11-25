/**
 * Damiao motor behind the Motor seam: one send id = one Motor. Commands are fire-and-forget,
 * queries wait for their reply, feedback is timestamped. Everything Damiao-specific that the arm
 * layer needs lives here (reply routing, status nibble, register checks, watchdog register, id
 * change); protocol.ts stays a pure codec.
 *
 * Every Damiao state frame (each MIT reply) carries the motor's status; the read-only refresh
 * (0xCC, sent by pollStatus/check/readState) never clears anything. A fault status (overvoltage,
 * comm lost, ...) is latched until the caller explicitly runs clearFault().
 *
 * Late replies. The Metal gripper motor (id 7) often emits its reply to request N only when the
 * next frame appears on the bus, sometimes delivers that held reply twice, and occasionally never
 * answers (measured on two arms and two adapters). Every query here therefore (1) matches replies
 * by content, (2) after FLUSH_AFTER_S of silence from this motor puts a harmless flush frame on the
 * bus (a refresh of a neighbouring id) to release a held reply, (3) for state reads, which have no
 * content to match, sends one trailing flush and keeps the newest reply, so a stale reply released
 * by the request itself is superseded by the fresh one, and (4) re-sends the request on a miss.
 */

import {
  type CanBackend, type Clock, ConnectError, type JointConfig, type Logger, type Motor as CoreMotor,
  type MotorFeedback as CoreMotorFeedback, type MotorStatus, Mutex, ParamTimeout, type ParamSpec, RealClock, Signal,
  StateError, ValueError,
} from "@makermods/arm-core";
import { formatG, reprFloat, reprStr, roundHalfEven } from "./format.js";
import * as protocol from "./protocol.js";
import { type Frame, type MotorModel, Reg, RegisterReply, StateFrame } from "./protocol.js";

/** Motor models this package drives. */
export const MODELS: readonly string[] = Object.freeze(Object.keys(protocol.MODELS));
export const DEFAULT_MODEL = "jlo";
export const KP_RANGE: readonly [number, number] = [protocol.KP_MIN, protocol.KP_MAX];
export const KD_RANGE: readonly [number, number] = [protocol.KD_MIN, protocol.KD_MAX];
export const ID_RANGE: readonly [number, number] = [protocol.ID_MIN, protocol.ID_MAX];
/** Reply id = hostId + motor id unless set per joint. */
export const DEFAULT_HOST_ID = protocol.DEFAULT_MASTER_OFFSET;

/** A punctual reply arrives well inside this; a held one needs a frame. */
export const FLUSH_AFTER_S = 0.003;
export const FLUSHES_PER_ATTEMPT = 3;
/** Flash write time before the motor is asked anything again. */
export const SAVE_SETTLE_S = 0.1;

/**
 * Motor id a received frame belongs to, or null. Register replies carry the send id in bytes 0-1;
 * state frames carry its low nibble in byte 0. Frames addressed to motors (another host's
 * commands) are never replies. Motor.handleFrame additionally checks the exact reply id.
 */
export function route(canId: number, data: Uint8Array, hostId: number = DEFAULT_HOST_ID, extended = false): number | null {
  if (extended || !(0 <= canId && canId <= 0x7ff) || data.length !== 8) return null;
  if (protocol.isCommandId(canId)) return null;
  if (protocol.isRegisterReply(data)) return data[0]!;
  if (!Object.hasOwn(protocol.STATUS_NAMES, data[0]! >> 4)) return null;
  const nibble = data[0]! & 0xf;
  const offset = canId - hostId;
  if (ID_RANGE[0] <= offset && offset <= ID_RANGE[1] && (offset & 0xf) === nibble) return offset;
  return ID_RANGE[0] <= nibble && nibble <= ID_RANGE[1] ? nibble : null;
}

/** One feedback sample, motor coordinates, with the Damiao status nibble. */
export interface MotorFeedback extends CoreMotorFeedback {
  /** {"mos": .., "rotor": ..} */
  readonly temperatures: Readonly<Record<string, number>>;
  /** Damiao status nibble, see protocol.STATUS_NAMES. */
  readonly status: number;
}

export type { MotorStatus, ParamSpec };

/** What a query's match function returns for a frame it accepts. */
type Match<T> = (canId: number, data: Uint8Array) => T | null;

/** Collects replies to one query; fed from the receive callback. */
class Waiter<T> {
  readonly replies: T[] = [];
  /** Set by every accepted reply, cleared by the query loop after each wait (a threading.Event). */
  isSet = false;
  readonly signal = new Signal();

  constructor(readonly match: Match<T>) {}

  feed(canId: number, data: Uint8Array): void {
    const msg = this.match(canId, data);
    if (msg !== null && msg !== undefined) {
      this.replies.push(msg);
      this.isSet = true;
      this.signal.notify();
    }
  }

  since(index: number): T[] {
    return this.replies.slice(index);
  }
}

interface RegisterOptions {
  value?: number;
  dtype?: string;
  timeout?: number;
  attempts?: number;
  sendId?: number;
  /** Accepted master ids (undefined = this motor's, [] = any). */
  replyIds?: readonly number[];
  /** Motor ids accepted inside the reply (undefined = the one addressed). */
  echoIds?: readonly number[];
  /** The query lock is already held (setId). */
  locked?: boolean;
}

interface QueryOptions<T> {
  want?: ((msg: T) => boolean) | null;
  timeout?: number;
  attempts?: number;
  newest?: boolean;
  what?: string;
  locked?: boolean;
}

const bytesEqual = (a: Uint8Array, b: Uint8Array): boolean => a.length === b.length && a.every((x, i) => x === b[i]);

const PARAMS: Readonly<Record<string, ParamSpec>> = Object.freeze(Object.fromEntries(
  Object.entries(protocol.REGISTERS).map(([name, s]) => [name, Object.freeze({ index: s.index, dtype: s.dtype, doc: s.doc })]),
));

const reprValue = (value: number, dtype: string): string => (dtype === "f" ? reprFloat(value) : String(value));

/** A Damiao motor on the Metal Arm. */
export class Motor implements CoreMotor {
  /** Consecutive health checks an enabled motor may report "disabled". */
  static readonly STATUS_DISABLED_TICKS = 5;
  /** The Damiao register map by name. */
  static readonly PARAMS: Readonly<Record<string, ParamSpec>> = PARAMS;

  readonly joint: JointConfig;
  motorId: number;
  /** CAN id this motor replies on. */
  replyId: number;
  /** Refreshing a neighbouring id flushes a held reply without asking this motor again. */
  flushId: number;
  /** MIT scaling of this motor's model. */
  readonly scaling: MotorModel;
  readonly params: Readonly<Record<string, ParamSpec>> = PARAMS;
  readonly changed = new Signal();

  #backend: CanBackend;
  #hostId: number;
  #clock: Clock;
  #log: Logger;
  #feedback: MotorFeedback | null = null;
  #fbTime: number | null = null;
  #feedbackSequence = 0;
  #status: MotorStatus | null = null;
  #latched: string | null = null;
  #disabledTicks = 0;
  #queryLock = new Mutex();
  #waiter: Waiter<unknown> | null = null;
  #tapping = false;

  /** `logger` is the arm's (default: console). */
  constructor(joint: JointConfig, backend: CanBackend, hostId: number = DEFAULT_HOST_ID, clock: Clock = new RealClock(),
              logger: Logger = console) {
    if (!Object.hasOwn(protocol.MODELS, joint.model)) {
      throw new ValueError(`motor ${joint.motorId}: unknown model ${reprStr(joint.model)} (supported: (${MODELS.map(reprStr).join(", ")}))`);
    }
    this.joint = joint;
    this.motorId = joint.motorId;
    protocol.encodeRefresh(this.motorId); // validates the id range
    this.#backend = backend;
    this.#hostId = hostId;
    this.#clock = clock;
    this.#log = logger;
    this.scaling = protocol.MODELS[joint.model]!;
    this.replyId = joint.replyId || hostId + this.motorId;
    this.flushId = this.motorId > ID_RANGE[0] ? this.motorId - 1 : this.motorId + 1;
  }

  /** Largest torque the MIT frame can carry for this model, N·m. */
  get torqueLimit(): number {
    return this.scaling.tMax;
  }

  // -- receive path (invoked by the receive dispatcher) --
  handleFrame(canId: number, data: Uint8Array, extended = false): void {
    data = Uint8Array.from(data);
    if (canId !== this.replyId || extended) return;
    const msg = this.#decodeOwn(canId, data);
    if (msg instanceof StateFrame) this.#store(msg);
    const waiter = this.#waiter;
    if (waiter !== null && !this.#tapping) waiter.feed(canId, data);
    this.changed.notify();
  }

  #decodeOwn(canId: number, data: Uint8Array): StateFrame | RegisterReply | null {
    const msg = protocol.decode(canId, data, this.scaling);
    if (msg instanceof StateFrame && msg.motorId === (this.motorId & 0xf)) return msg;
    if (msg instanceof RegisterReply && msg.motorId === this.motorId) return msg;
    return null;
  }

  #store(frame: StateFrame): void {
    const fault = protocol.isFaultStatus(frame.status);
    this.#feedback = Object.freeze({
      motorId: this.motorId, position: frame.position, velocity: frame.velocity, torque: frame.torque,
      temperature: Math.max(frame.tMos, frame.tRotor), temperatures: Object.freeze({ mos: frame.tMos, rotor: frame.tRotor }),
      enabled: frame.status === protocol.STATUS_ENABLED, warning: false, status: frame.status,
    });
    this.#status = Object.freeze({
      motorId: this.motorId, hasFault: fault, code: frame.status, text: fault ? protocol.statusText(frame.status) : "",
      enabled: frame.status === protocol.STATUS_ENABLED,
    });
    this.#fbTime = this.#clock.now();
    this.#feedbackSequence += 1;
    if (fault && this.#latched === null) this.#latched = protocol.statusText(frame.status);
  }

  // -- cached state --
  get feedback(): MotorFeedback | null {
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

  /** Every state frame carries the status. */
  get statusAge(): number {
    return this.feedbackAge;
  }

  /** Reason a fault was latched, or null. */
  get faultLatched(): string | null {
    return this.#latched;
  }

  /**
   * Does the motor report only the state its watchdog leaves after commands stopped? Status
   * "communication lost": torque is off and the status must be cleared before an enable.
   */
  get watchdogReleased(): boolean {
    return this.#status !== null && this.#status.code === protocol.STATUS_COMM_LOST;
  }

  resetCache(): void {
    this.#feedback = null;
    this.#fbTime = null;
    this.#status = null;
    this.#latched = null;
    this.#disabledTicks = 0;
  }

  async #send(frame: Frame): Promise<void> {
    await this.#backend.send(frame[0], frame[1], false);
  }

  // -- commands: the frame is written, no reply is awaited --
  async enable(): Promise<void> {
    this.#disabledTicks = 0;
    await this.#send(protocol.encodeEnable(this.motorId));
  }

  async disable(): Promise<void> {
    await this.#send(protocol.encodeDisable(this.motorId));
  }

  async sendCommand(pos: number, vel: number, kp: number, kd: number, tau: number): Promise<void> {
    await this.#send(protocol.encodeMit(this.motorId, pos, vel, kp, kd, tau, this.scaling));
  }

  /** The refresh pollStatus() sends is answered with a state frame. */
  readonly pollAnsweredByFeedback = true;

  /** Send the read-only refresh; its state frame updates status (as every MIT reply does). */
  async pollStatus(): Promise<void> {
    await this.#send(protocol.encodeRefresh(this.motorId));
  }

  // -- query engine --
  /**
   * Send `frame` and return the reply that answers it (see the module comment).
   *
   * `match` returns the decoded reply or null; `want` narrows a match to the content this request
   * must produce, so a late duplicate of an earlier answer is skipped. A match that is not wanted
   * is returned only if nothing better arrives (the caller's check then fails closed). `newest`
   * keeps the newest wanted reply after one trailing flush.
   */
  async #query<T>(frame: Frame, match: Match<T>, options: QueryOptions<T> = {}): Promise<T> {
    const { want = null, timeout = 0.05, attempts = 3, newest = false, what = "request", locked = false } = options;
    if (timeout <= 0 || attempts < 1) throw new ValueError("timeout must be positive and attempts >= 1");
    const flush = protocol.encodeRefresh(this.flushId);
    const run = async (): Promise<T> => {
      let unwanted: T | null = null;
      for (let i = 0; i < attempts; i++) {
        const waiter = new Waiter<T>(match);
        this.#waiter = waiter as Waiter<unknown>;
        let best: T | null;
        try {
          best = await this.#attempt(waiter, frame, flush, want, timeout, newest);
        } finally {
          this.#waiter = null;
        }
        if (best !== null && (want === null || want(best))) return best;
        unwanted = best ?? unwanted;
      }
      if (unwanted !== null) return unwanted;
      throw new ParamTimeout(`motor ${this.motorId} did not answer ${what} (${attempts} attempts of ${formatG(timeout)} s)`);
    };
    return locked ? run() : this.#queryLock.run(run);
  }

  async #attempt<T>(waiter: Waiter<T>, frame: Frame, flush: Frame, want: ((msg: T) => boolean) | null,
                    timeout: number, newest: boolean): Promise<T | null> {
    await this.#send(frame);
    let now = this.#clock.now();
    let quietSince = now;
    const deadline = now + timeout;
    let flushes = 0;
    let seen = 0;
    let best: T | null = null;
    let unwanted: T | null = null;
    let trailingUntil: number | null = null;
    for (;;) {
      const fresh = waiter.since(seen);
      seen += fresh.length;
      if (fresh.length) quietSince = now;
      for (const msg of fresh) {
        if (want === null || want(msg)) best = msg;
        else unwanted = msg;
      }
      if (best !== null) {
        if (!newest) return best;
        if (trailingUntil === null) {
          await this.#send(flush);
          trailingUntil = now + FLUSH_AFTER_S;
        }
      }
      if (trailingUntil !== null && now >= trailingUntil) return best;
      if (now >= deadline) return best ?? unwanted;
      if (best === null && flushes < FLUSHES_PER_ATTEMPT && now - quietSince >= FLUSH_AFTER_S) {
        await this.#send(flush);
        flushes += 1;
        quietSince = now;
      }
      await waiter.signal.wait(() => waiter.isSet, Math.max(0.0, Math.min(FLUSH_AFTER_S, deadline - now)), this.#clock);
      waiter.isSet = false;
      now = this.#clock.now();
    }
  }

  #matchState = (canId: number, data: Uint8Array): StateFrame | null => {
    const msg = canId === this.replyId ? this.#decodeOwn(canId, data) : null;
    return msg instanceof StateFrame ? msg : null;
  };

  #refresh(timeout: number, attempts: number, locked = false): Promise<StateFrame> {
    return this.#query(protocol.encodeRefresh(this.motorId), this.#matchState,
      { timeout, attempts, newest: true, what: "a state refresh", locked });
  }

  /** One register transaction. */
  #register(op: number, rid: number, options: RegisterOptions = {}): Promise<RegisterReply> {
    const { value, dtype = "f", timeout = 0.12, attempts = 2, locked = false } = options;
    const sendId = options.sendId ?? this.motorId;
    const echoIds = options.echoIds ?? [sendId];
    const replyIds = options.replyIds ?? [this.replyId];
    const frame = op === protocol.OP_READ
      ? protocol.encodeReadRegister(sendId, rid)
      : protocol.encodeWriteRegister(sendId, rid, value as number, dtype);

    const match = (canId: number, data: Uint8Array): RegisterReply | null => {
      if (replyIds.length && !replyIds.includes(canId)) return null;
      if (protocol.isCommandId(canId) || !protocol.isRegisterReply(data)) return null;
      const reply = new RegisterReply(data[0]!, data[2]!, data[3]!, data.slice(4, 8));
      return reply.op === op && reply.rid === rid && echoIds.includes(reply.motorId) ? reply : null;
    };

    // A write echoes its value: a late duplicate of an earlier echo is skipped.
    const echoed = frame[1].slice(4, 8);
    const want = op === protocol.OP_READ ? null : (r: RegisterReply) => bytesEqual(r.raw, echoed);
    const verb = op === protocol.OP_READ ? "read" : "write";
    return this.#query(frame, match, { want, timeout, attempts, what: `register ${rid} ${verb}`, locked });
  }

  // -- queries --
  /** Fresh status via the read-only refresh: never clears a fault, never changes torque. */
  async check(timeout = 0.05, attempts = 3): Promise<MotorStatus> {
    return this.#check(timeout, attempts, false);
  }

  async #check(timeout: number, attempts: number, locked: boolean): Promise<MotorStatus> {
    await this.#refresh(timeout, attempts, locked);
    return this.#status!;
  }

  /**
   * Fresh feedback (not cached). The Damiao refresh clears nothing, so this never needs to refuse;
   * a fault status in the reply is latched as usual.
   */
  async readState(timeout = 0.05, attempts = 3): Promise<MotorFeedback> {
    await this.#refresh(timeout, attempts);
    return this.#feedback!;
  }

  /** Connect-time checks: MIT control mode and MIT scaling registers equal to the model. */
  async validate(timeout = 0.12): Promise<void> {
    const label = this.joint.label;
    let mode: number;
    const scaling: number[] = [];
    try {
      mode = await this.readParam("ctrl_mode", undefined, timeout);
      for (const name of ["pmax", "vmax", "tmax"]) scaling.push(await this.readParam(name, undefined, timeout));
    } catch (e) {
      if (e instanceof ParamTimeout) {
        throw new ConnectError(`${label}: no reply to a register read (${e.message}); check power and wiring`, { cause: e });
      }
      throw e;
    }
    if (mode !== protocol.MODE_MIT) {
      throw new ConnectError(
        `${label}: control mode is ${mode}, not 1 (MIT). Another program left it there; run `
        + `\`<cli> params write ${this.motorId} ctrl_mode 1\` or power-cycle the arm`);
    }
    const expected = [this.scaling.pMax, this.scaling.vMax, this.scaling.tMax];
    if (!scaling.every((a, i) => protocol.isClose(a, expected[i]!, 1e-5))) {
      const found = protocol.matchingModel(scaling[0]!, scaling[1]!, scaling[2]!);
      const fix = found
        ? `set \`model: ${found}\` for this joint in the profile`
        : "the motor was reconfigured; restore PMAX/VMAX/TMAX (registers 21-23) with the "
          + "Damiao tool or use a matching profile";
      throw new ConnectError(
        `${label}: MIT scaling PMAX/VMAX/TMAX = ${scaling.map(formatG).join(", ")} does not `
        + `match model ${reprStr(this.joint.model)} (${expected.map(formatG).join(", ")}); ${fix}`);
    }
  }

  /**
   * Does the latest feedback confirm "enabled" / "disabled"? Enabled is status 1; disabled is any
   * status but 1 (a faulted motor carries no torque).
   */
  confirms(expected: "enabled" | "disabled"): boolean {
    if (expected !== "enabled" && expected !== "disabled") {
      throw new ValueError(`expected 'enabled' or 'disabled', got ${reprStr(String(expected))}`);
    }
    const fb = this.#feedback;
    if (fb === null) return false;
    return (fb.status === protocol.STATUS_ENABLED) === (expected === "enabled");
  }

  /**
   * Per-tick check while enabled: a fault status at once; status "disabled" (the motor dropped
   * torque, e.g. after a supply dip reboot) once it persists STATUS_DISABLED_TICKS ticks.
   */
  healthProblem(): string | null {
    if (this.#latched !== null) return this.#latched;
    const fb = this.#feedback;
    if (fb === null) return null;
    this.#disabledTicks = fb.status === protocol.STATUS_DISABLED ? this.#disabledTicks + 1 : 0;
    if (this.#disabledTicks >= Motor.STATUS_DISABLED_TICKS) {
      return `motor reports disabled while it should be enabled (${this.#disabledTicks} ticks)`;
    }
    return null;
  }

  /** Zero-gain MIT command at `pos` (motor coordinates); true once its state reply arrives. */
  async preload(pos: number, timeout = 0.05): Promise<boolean> {
    if (!Number.isFinite(pos)) return false;
    try {
      await this.#query(protocol.encodeMit(this.motorId, pos, 0.0, 0.0, 0.0, 0.0, this.scaling), this.#matchState,
        { timeout, attempts: 2, newest: true, what: "the neutral preload" });
    } catch (e) {
      if (e instanceof ParamTimeout) return false;
      throw e;
    }
    return true;
  }

  /** Enable; true once a reply reports status enabled. */
  async enableConfirmed(timeout = 0.05): Promise<boolean> {
    this.#disabledTicks = 0;
    let reply: StateFrame;
    try {
      reply = await this.#query(protocol.encodeEnable(this.motorId), this.#matchState,
        { want: (m) => m.status === protocol.STATUS_ENABLED, timeout, attempts: 2, what: "enable" });
    } catch (e) {
      if (e instanceof ParamTimeout) return false;
      throw e;
    }
    return reply.status === protocol.STATUS_ENABLED;
  }

  /** Disable and confirm a non-enabled status in the reply. */
  async disableConfirmed(timeout = 0.05, attempts = 3): Promise<boolean> {
    try {
      await this.#query(protocol.encodeDisable(this.motorId), this.#matchState,
        { want: (m) => m.status !== protocol.STATUS_ENABLED, timeout, attempts, what: "disable" });
    } catch (e) {
      if (e instanceof ParamTimeout) return false;
      throw e;
    }
    return this.confirms("disabled");
  }

  // -- operator actions --
  /** Explicit operator action: clear the motor's error (0xFB) and the latched evidence. */
  async clearFault(): Promise<void> {
    this.#latched = null;
    await this.#send(protocol.encodeClearError(this.motorId));
  }

  /** Make the current position the motor's zero (0xFE). Persists in the motor. */
  async setZero(): Promise<void> {
    await this.#send(protocol.encodeSetZero(this.motorId));
  }

  /**
   * Set and verify the communication watchdog (register 9, 50 us ticks) in RAM only, never flash.
   * Returns the previous tick count as the token.
   */
  async armWatchdog(timeoutMs: number, timeout = 0.12): Promise<number> {
    if (!(0 < timeoutMs && timeoutMs <= 200_000)) {
      throw new ValueError("timeout_ms must be in (0, 200000]; 0 would disable the watchdog");
    }
    const ticks = roundHalfEven(timeoutMs * protocol.TIMEOUT_TICKS_PER_MS);
    const original = await this.readParam("timeout", undefined, timeout);
    try {
      const readback = await this.writeParam("timeout", ticks, undefined, timeout);
      if (readback !== ticks) throw new ConnectError(`${this.joint.label}: watchdog read back ${readback} ticks, expected ${ticks}`);
    } catch (e) {
      try {
        await this.restoreWatchdog(original, timeout);
      } catch (e2) {
        this.#log.error(`could not restore ${this.joint.label} watchdog: ${(e2 as Error).message}`);
      }
      throw e;
    }
    return original;
  }

  async restoreWatchdog(token: unknown, timeout = 0.12): Promise<void> {
    if (token === null || token === undefined) return;
    const ticks = Math.trunc(Number(token));
    const readback = await this.writeParam("timeout", ticks, undefined, timeout);
    if (readback !== ticks) {
      throw new StateError(`${this.joint.label}: watchdog restore read back ${readback}, expected ${String(token)}`);
    }
  }

  /**
   * Run `fn` holding the query lock with every received frame (not only this motor's) fed to the
   * active waiter: an id change must hear the new id. Chains the backend's existing callback.
   */
  async #withTap<T>(fn: () => Promise<T>): Promise<T> {
    const backend = this.#backend;
    return this.#queryLock.run(async () => {
      const previous = backend.recvCallback;
      const tap = (canId: number, data: Uint8Array, extended: boolean): void => {
        const waiter = this.#waiter;
        if (waiter !== null) waiter.feed(canId, Uint8Array.from(data));
        if (previous !== null) previous(canId, data, extended);
      };
      this.#tapping = true;
      backend.setRecvCallback(tap);
      try {
        return await fn();
      } finally {
        backend.setRecvCallback(previous);
        this.#tapping = false;
      }
    });
  }

  /** ESC_ID register as read through `sendId`, or null when nothing answers. Lock held. */
  async #answersAt(sendId: number, replyIds: readonly number[], timeout: number): Promise<number | null> {
    try {
      return (await this.#register(protocol.OP_READ, Reg.ESC_ID, { timeout, attempts: 2, sendId, replyIds, locked: true })).value("u32");
    } catch (e) {
      if (e instanceof ParamTimeout) return null;
      throw e;
    }
  }

  /** Lock held. */
  async #idInUse(newId: number, timeout: number): Promise<boolean> {
    const anyState = (canId: number, data: Uint8Array): Uint8Array | null => {
      if (protocol.isCommandId(canId) || protocol.isRegisterReply(data)) return null;
      return (data[0]! & 0xf) === newId && Object.hasOwn(protocol.STATUS_NAMES, data[0]! >> 4) ? data : null;
    };
    try {
      await this.#query(protocol.encodeRefresh(newId), anyState, { timeout, attempts: 2, locked: true });
      return true;
    } catch (e) {
      if (!(e instanceof ParamTimeout)) throw e;
    }
    return (await this.#answersAt(newId, [], timeout)) !== null; // [] = any reply id
  }

  /**
   * Change the send id to newId and the master id to hostId + newId, saved to flash.
   *
   * Damiao motors report whether torque is on in every reply, so the fresh status read here decides;
   * `torqueOffConfirmed` is only needed if a reply ever leaves that unknown.
   *
   * Procedure (Damiao register map, DM_CAN change_motor_param/save_motor_param): refuse if anything
   * answers at newId; require the motor disabled (flash save is ignored while enabled); write MST_ID
   * then ESC_ID through the old id. The firmware applies both at once, so each echo is accepted on
   * the old or the new master id, and the ESC_ID echo may name either id or not arrive at all. Then
   * find the id the motor now answers at and save (0xAA) through it; verify by reading ESC_ID and
   * MST_ID through the new id. A motor that keeps answering at the old id has the change saved but
   * not applied: StateError asks for a power cycle.
   */
  async setId(newId: number, timeout = 0.5, torqueOffConfirmed = false): Promise<void> {
    const [lo, hi] = ID_RANGE;
    if (!(Number.isInteger(newId) && lo <= newId && newId <= hi)) {
      throw new ValueError(`motor id must be in [${lo}, ${hi}], got ${newId}`); // whole numbers only
    }
    if (newId === this.motorId) return;
    const oldId = this.motorId;
    const oldReply = this.replyId;
    const newReply = this.#hostId + newId;
    if (protocol.isCommandId(newReply) || !(0 < newReply && newReply < 0x7ff)) {
      throw new ValueError(`master id 0x${newReply.toString(16).toUpperCase()} would collide with command ids`);
    }
    const step = Math.min(timeout, 0.12);
    const replies = [oldReply, newReply];
    await this.#withTap(async () => {
      if (await this.#idInUse(newId, step)) {
        throw new StateError(`id ${newId} is already used by a motor on this bus; pick a free id or disconnect that motor`);
      }
      const status = await this.#check(step, 3, true);
      if (status.enabled) {
        throw new StateError(`${this.joint.label}: the motor reports torque on; disable it before changing its id`);
      }
      if (status.enabled === null && !torqueOffConfirmed) {
        throw new StateError(`${this.joint.label}: the motor did not report whether torque is on. Make sure it `
          + "is de-energized, then confirm (torque_off_confirmed=True; the CLI asks)");
      }
      const reply = await this.#register(protocol.OP_WRITE, Reg.MST_ID,
        { value: newReply, dtype: "u32", timeout: step, attempts: 2, sendId: oldId, replyIds: replies, locked: true });
      if (reply.value("u32") !== newReply) {
        throw new StateError(`master id echoed ${reply.value("u32")}, expected ${newReply}; `
          + "nothing was saved, power-cycle the motor to undo");
      }
      try {
        await this.#register(protocol.OP_WRITE, Reg.ESC_ID, {
          value: newId, dtype: "u32", timeout: step, attempts: 1, sendId: oldId, replyIds: replies,
          echoIds: [oldId, newId], locked: true,
        });
      } catch (e) {
        if (!(e instanceof ParamTimeout)) throw e; // the read through the new id below decides
      }
      let saveId: number;
      if ((await this.#answersAt(newId, [newReply], step)) === newId) saveId = newId;
      else if ((await this.#answersAt(oldId, replies, step)) === newId) saveId = oldId;
      else {
        throw new StateError(`${this.joint.label}: motor stopped answering after the id write; `
          + "nothing was saved, power-cycle the motor to undo");
      }
      await this.#send(protocol.encodeSave(saveId));
      await this.#clock.sleep(SAVE_SETTLE_S);
      const esc = await this.#answersAt(newId, [newReply], timeout);
      if (esc !== newId) {
        throw new StateError(
          `${this.joint.label}: ids written and saved, but the motor does not answer at id `
          + `${newId} yet; power-cycle the arm and check with a scan`);
      }
      const mst = (await this.#register(protocol.OP_READ, Reg.MST_ID,
        { timeout, sendId: newId, replyIds: [newReply], locked: true })).value("u32");
      if (mst !== newReply) {
        throw new StateError(`${this.joint.label}: master id reads 0x${mst.toString(16).toUpperCase()}, `
          + `expected 0x${newReply.toString(16).toUpperCase()}`);
      }
    });
    this.motorId = newId;
    this.replyId = newReply;
    this.flushId = newId > lo ? newId - 1 : newId + 1;
    this.resetCache();
  }

  // -- engineering --
  #param(nameOrIndex: string | number, dtype?: string | null): [number, string] {
    if (typeof nameOrIndex === "string") {
      const key = nameOrIndex.toLowerCase();
      if (!Object.hasOwn(PARAMS, key)) {
        const known = Object.keys(PARAMS).sort().map(reprStr).join(", ");
        throw new ValueError(`unknown parameter ${reprStr(nameOrIndex)} (known: [${known}])`);
      }
      const s = PARAMS[key]!;
      return [s.index, dtype || s.dtype];
    }
    const index = Math.trunc(nameOrIndex);
    return [index, dtype || protocol.registerDtype(index)];
  }

  async readParam(nameOrIndex: string | number, dtype?: string | null, timeout = 0.12): Promise<number> {
    const [index, dt] = this.#param(nameOrIndex, dtype);
    return (await this.#register(protocol.OP_READ, index, { timeout })).value(dt);
  }

  /**
   * Write a register (RAM), verify the echo and a read-back, and return the read-back value.
   * save=true then stores all registers to flash (motor must be disabled; flash wears out).
   */
  async writeParam(nameOrIndex: string | number, value: number, dtype?: string | null, timeout = 0.12,
                   save = false): Promise<number> {
    const [index, dt] = this.#param(nameOrIndex, dtype);
    const packed = protocol.packValue(value, dt);
    if (save && (await this.check(Math.min(timeout, 0.05))).enabled) {
      throw new StateError(`motor ${this.motorId}: disable it before saving registers to flash`);
    }
    const echo = await this.#register(protocol.OP_WRITE, index, { value, dtype: dt, timeout });
    const readback = await this.#register(protocol.OP_READ, index, { timeout });
    if (!bytesEqual(echo.raw, packed) || !bytesEqual(readback.raw, packed)) {
      throw new StateError(`motor ${this.motorId} register ${index} kept `
        + `${reprValue(readback.value(dt), dt)}, not ${reprValue(protocol.unpackValue(packed, dt), dt)}`);
    }
    if (save) {
      await this.#send(protocol.encodeSave(this.motorId));
      await this.#clock.sleep(SAVE_SETTLE_S);
    }
    return readback.value(dt);
  }
}
