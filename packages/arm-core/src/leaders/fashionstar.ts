/**
 * FashionStar UART smart servos (the Star Arm 102 leader), the part of motorbridge-smart-servo 0.0.4
 * the Star leader uses: ping, unlock, multi-turn reset, set origin and the code-25 sync monitor, on a
 * SerialPortLike. The bytes on the wire, the reply timing rules (input cleared before each request,
 * 100 ms per reply, 10 ms reads), the angle reliability filter and the consecutive-loss limit are
 * the library's (checked against it by contract/fixtures/teleop_star.json).
 *
 * All multi-byte fields are little-endian. Angles are f32 in the library (`raw as f32 / 10.0`), so
 * they are rounded to f32 here too. A Star leader has encoders only: the only stop-mode command sent
 * is unlock (unloaded), and nothing here can energize anything.
 */

import { type Clock, Mutex, RealClock, Signal, yieldToIo } from "../clock.js";
import { ArmError } from "../errors.js";
import type { SerialPortLike } from "../transport/serial.js";

/** First two bytes of every request ... */
export const REQ_HEADER = [0x12, 0x4c] as const;
/** ... and of every reply. */
export const RESP_HEADER = [0x05, 0x1c] as const;

/** Command codes. */
export const CODE_PING = 1;
/** Reset the multi-turn counter (no reply). */
export const CODE_RESET_MTURN = 17;
/** Monitor: voltage, current, power, temperature, status, angle, turns. */
export const CODE_QUERY_MONITOR = 22;
/** Make the present position the origin (no reply). */
export const CODE_SET_ORIGIN = 23;
/** Stop mode and power (no reply). */
export const CODE_SET_STOP_MODE = 24;
/** One request for several servos; each answers with the sub-command's reply. */
export const CODE_SYNC_COMMAND = 25;

/** Stop mode "unloaded" (power 0): the servo turns freely. */
export const STOP_MODE_UNLOCKED = 0x10;
/** The Star Arm 102's UART rate. */
export const FASHIONSTAR_BAUDRATE = 1_000_000;
/** s: each reply is waited for up to this long overall ... */
export const REPLY_TIMEOUT = 0.1;
/** ... in reads of up to this long. */
export const READ_TIMEOUT = 0.01;
/** s: the most a request's write may take; the library's serial writes have the same kind of bound. */
export const WRITE_TIMEOUT = 0.1;
/** Bytes kept beyond the largest expected reply set during an exchange (resynchronisation room). */
export const RESYNC_ROOM = 256;
/** Bytes of one code-22 monitor reply. */
const MONITOR_REPLY_BYTES = 5 + 16;
/** Bytes of one ping reply. */
const PING_REPLY_BYTES = 5 + 1;
/** Consecutive missed replies of one servo that make syncMonitor throw (the library's default). */
export const LOSS_THRESHOLD = 20;

/** A bus operation failed (the library's ServoBusError). */
export class ServoBusError extends ArmError {}

/** One parsed reply. */
export interface Packet {
  code: number;
  params: Uint8Array;
}

/** Sum of header, code, length and parameters, low byte. */
export function checksum(header: readonly number[], code: number, params: ArrayLike<number>): number {
  let sum = header[0]! + header[1]! + code + params.length;
  for (let i = 0; i < params.length; i++) sum += params[i]!;
  return sum & 0xff;
}

/** `12 4C code len params... checksum`. */
export function packRequest(code: number, params: ArrayLike<number>): Uint8Array {
  if (params.length > 255) throw new ServoBusError("packet params too large");
  const out = new Uint8Array(5 + params.length);
  out.set(REQ_HEADER, 0);
  out[2] = code;
  out[3] = params.length;
  out.set(Array.from(params), 4);
  out[4 + params.length] = checksum(REQ_HEADER, code, params);
  return out;
}

/** The replies in `data`, and the checksum mismatches (code, expected, got). */
export interface ParseReport {
  packets: Packet[];
  errors: Array<{ code: number; expected: number; got: number }>;
}

/** Finds each `05 1C`, takes len + 5 bytes, and drops (reports) a bad checksum. */
export function parseResponseStream(data: Uint8Array): ParseReport {
  const report: ParseReport = { packets: [], errors: [] };
  let i = 0;
  while (i + 5 <= data.length) {
    if (data[i] === RESP_HEADER[0] && data[i + 1] === RESP_HEADER[1]) {
      const code = data[i + 2]!;
      const size = data[i + 3]!;
      const end = i + 5 + size;
      if (end > data.length) break;
      const params = data.slice(i + 4, i + 4 + size);
      const got = data[end - 1]!;
      const expected = checksum(RESP_HEADER, code, params);
      if (got === expected) report.packets.push({ code, params });
      else report.errors.push({ code, expected, got });
      i = end;
    } else {
      i += 1;
    }
  }
  return report;
}

const checkId = (id: number): number => {
  if (!Number.isInteger(id) || id < 0 || id > 253) throw new RangeError("servo_id must be in range 0..253");
  return id;
};

/** Request bytes for each command. */
export const encodePing = (id: number): Uint8Array => packRequest(CODE_PING, [checkId(id)]);
/** Reset the turn counter. */
export const encodeResetMultiTurn = (id: number): Uint8Array => packRequest(CODE_RESET_MTURN, [checkId(id)]);
/** Two bytes `[id, 0]`: the firmware ignores set-origin with only `[id]`. */
export const encodeSetOrigin = (id: number): Uint8Array => packRequest(CODE_SET_ORIGIN, [checkId(id), 0]);
/** Stop mode `mode` at `power` (u16). */
export function encodeSetStopMode(id: number, mode: number, power: number): Uint8Array {
  return packRequest(CODE_SET_STOP_MODE, [checkId(id), mode & 0xff, power & 0xff, (power >> 8) & 0xff]);
}
/** Stop mode unloaded, power 0: the servo turns freely. */
export const encodeUnlock = (id: number): Uint8Array => encodeSetStopMode(id, STOP_MODE_UNLOCKED, 0);
/** Code 25 with sub-command 22: one request, one code-22 reply per servo. */
export function encodeSyncMonitor(ids: readonly number[]): Uint8Array {
  if (!ids.length) throw new ServoBusError("sync_monitor: ids list must not be empty");
  return packRequest(CODE_SYNC_COMMAND, [CODE_QUERY_MONITOR, 1, ids.length, ...ids.map(checkId)]);
}

/** The id a ping reply carries, or null. */
export function decodePing(packet: Packet): number | null {
  return packet.code === CODE_PING && packet.params.length === 1 ? packet.params[0]! : null;
}

const f32 = Math.fround;
const view = (p: Uint8Array): DataView => new DataView(p.buffer, p.byteOffset, p.byteLength);

/** One servo's monitor reply. */
export interface ServoMonitor {
  id: number;
  voltageMv: number;
  currentMa: number;
  powerMw: number;
  tempRaw: number;
  status: number;
  /** Multi-turn degrees (f32), after the reliability filter. */
  angleDeg: number;
  turn: number;
  /** true: a fresh, trusted reading this cycle; false: held from the last trusted one. */
  reliable: boolean;
}

/** Code 22 reply, at least 14 bytes (normally 16): angle i32 / 10 at bytes 10-13, turn i16 at 14-15. */
export function decodeMonitor(packet: Packet): ServoMonitor | null {
  const p = packet.params;
  if (packet.code !== CODE_QUERY_MONITOR || p.length < 14) return null;
  const v = view(p);
  return {
    id: p[0]!,
    voltageMv: v.getUint16(1, true),
    currentMa: v.getUint16(3, true),
    powerMw: v.getUint16(5, true),
    tempRaw: v.getUint16(7, true),
    status: p[9]!,
    angleDeg: f32(f32(v.getInt32(10, true)) / 10),
    turn: p.length >= 16 ? v.getInt16(14, true) : 0,
    reliable: true,
  };
}

/**
 * The library's angle filter against power-cycle glitches: a near-zero reading (|x| <= 0.2°) is
 * accepted only after 30 in a row, and |x| > 3,686,400° is garbage; both hold the last good value,
 * marked unreliable. Compared in f32 as the library does.
 */
export class AngleReliability {
  lastGood: number | null = null;
  zeroCandidates = 0;

  constructor(readonly zeroEpsDeg = f32(0.2), readonly zeroConfirmSamples = 30, readonly validRangeDeg = f32(3_686_400)) {}

  /** [angle to report, reliable]. */
  filter(rawDeg: number): [number, boolean] {
    const raw = f32(rawDeg);
    const nearZero = Math.abs(raw) <= this.zeroEpsDeg;
    const inRange = Math.abs(raw) <= this.validRangeDeg;
    if (this.lastGood === null) {
      if (!inRange) return [0.0, false];
      this.lastGood = raw;
      return [raw, true];
    }
    if (!inRange) {
      this.zeroCandidates = 0;
      return [this.lastGood, false];
    }
    if (nearZero) {
      this.zeroCandidates = Math.min(0xffff, this.zeroCandidates + 1);
      if (this.zeroCandidates >= this.zeroConfirmSamples) {
        this.zeroCandidates = 0;
        this.lastGood = raw;
        return [raw, true];
      }
      return [this.lastGood, false];
    }
    this.zeroCandidates = 0;
    this.lastGood = raw;
    return [raw, true];
  }
}

/** Consecutive missed replies per servo; `threshold` in a row throws (0 = never). */
export class LossTracker {
  readonly #counts = new Map<number, number>();

  constructor(public threshold = LOSS_THRESHOLD) {}

  ok(id: number): void {
    this.#counts.set(id, 0);
  }

  miss(id: number): void {
    const count = (this.#counts.get(id) ?? 0) + 1;
    this.#counts.set(id, count);
    if (this.threshold > 0 && count >= this.threshold) {
      throw new ServoBusError(`servo ${id} lost ${count} consecutive responses`);
    }
  }

  missCount(id: number): number {
    return this.#counts.get(id) ?? 0;
  }
}

/** Options of FashionStarBus.open(). */
export interface FashionStarBusOptions {
  /** Default FASHIONSTAR_BAUDRATE. */
  baudrate?: number;
  /** Default: a RealClock. */
  clock?: Clock;
}

/**
 * A FashionStar servo bus on one serial port (the library's FashionStarServo, the leader's part).
 * Each method is one request (and its reply); calls are serialized, as the library holds a lock
 * per bus. Every call is bounded: its write by WRITE_TIMEOUT, its reply by REPLY_TIMEOUT. A call
 * stops early with ServoBusError when its `signal` aborts or the bus is closed. Bytes that arrive
 * outside an exchange are dropped.
 */
export class FashionStarBus {
  readonly port: SerialPortLike;
  readonly baudrate: number;
  readonly clock: Clock;
  readonly filters = new Map<number, AngleReliability>();
  readonly loss = new LossTracker();
  readonly lastMonitors = new Map<number, ServoMonitor>();

  #reader: ReadableStreamDefaultReader<Uint8Array>;
  #writer: WritableStreamDefaultWriter<Uint8Array>;
  #input: number[] = [];
  #arrived = new Signal();
  #lock = new Mutex();
  #pump: Promise<void>;
  #readError: unknown = null;
  #closed: Promise<void> | null = null;
  /** Aborted by close(): wakes every waiting call. */
  #closing = new AbortController();
  /** Bytes the current exchange may still take in; 0 outside an exchange (arrivals are dropped). */
  #budget = 0;

  private constructor(port: SerialPortLike, baudrate: number, clock: Clock) {
    this.port = port;
    this.baudrate = baudrate;
    this.clock = clock;
    this.#reader = port.readable!.getReader();
    this.#writer = port.writable!.getWriter();
    this.#pump = this.#readLoop();
  }

  /** Opens `port` at the baud rate and starts reading it. */
  static async open(port: SerialPortLike, options: FashionStarBusOptions = {}): Promise<FashionStarBus> {
    const baudrate = options.baudrate ?? FASHIONSTAR_BAUDRATE;
    await port.open({ baudRate: baudrate });
    if (!port.readable || !port.writable) {
      await port.close().catch(() => undefined);
      throw new ServoBusError("the serial port opened without streams");
    }
    return new FashionStarBus(port, baudrate, options.clock ?? new RealClock());
  }

  get isOpen(): boolean {
    return this.#closed === null;
  }

  /** Received bytes not yet taken by the current exchange (0 outside one). */
  get bufferedBytes(): number {
    return this.#input.length;
  }

  async #readLoop(): Promise<void> {
    try {
      for (;;) {
        const { value, done } = await this.#reader.read();
        if (done) return;
        if (value?.length && this.#budget > 0) {
          const take = Math.min(value.length, this.#budget);
          for (let i = 0; i < take; i++) this.#input.push(value[i]!);
          this.#budget -= take;
          this.#arrived.notify();
        }
      }
    } catch (e) {
      if (this.#closed === null) this.#readError = e;
    } finally {
      this.#arrived.notify();
    }
  }

  #checkOpen(signal?: AbortSignal): void {
    if (this.#closed !== null) throw new ServoBusError("the port is closed");
    if (this.#readError !== null) throw new ServoBusError(`the port failed: ${String(this.#readError)}`);
    if (signal?.aborted) throw new ServoBusError("aborted");
  }

  /** Write `bytes`, giving up (ServoBusError) after WRITE_TIMEOUT, on `signal` or on close. */
  async #write(bytes: Uint8Array, signal?: AbortSignal): Promise<void> {
    this.#checkOpen(signal);
    const writing = this.#writer.write(bytes);
    writing.catch(() => undefined); // observed here even when the race below is lost
    let cancelTimer: () => void = () => {};
    let unlisten: () => void = () => {};
    try {
      await Promise.race([
        writing,
        new Promise<never>((_, reject) => {
          // Checked after pending I/O is delivered: a write that completed during a stall is not late.
          let written = false;
          void writing.then(() => (written = true), () => undefined);
          cancelTimer = this.clock.timer(WRITE_TIMEOUT, () => {
            void yieldToIo(this.clock).then(() => {
              if (!written) reject(new ServoBusError("serial write timeout"));
            });
          });
          const stop = (): void => reject(new ServoBusError(this.#closed !== null ? "the port is closed" : "aborted"));
          this.#closing.signal.addEventListener("abort", stop, { once: true });
          signal?.addEventListener("abort", stop, { once: true });
          unlisten = () => {
            this.#closing.signal.removeEventListener("abort", stop);
            signal?.removeEventListener("abort", stop);
          };
        }),
      ]);
    } finally {
      cancelTimer();
      unlisten();
    }
  }

  /** What arrived within `timeout` s (empty on silence, an abort or close). */
  async #read(timeout: number, signal?: AbortSignal): Promise<Uint8Array> {
    if (!this.#input.length) {
      const wake = (): void => this.#arrived.notify();
      signal?.addEventListener("abort", wake, { once: true });
      try {
        await this.#arrived.wait(() => this.#input.length > 0 || this.#readError !== null || this.#closed !== null
          || Boolean(signal?.aborted), timeout, this.clock);
      } finally {
        signal?.removeEventListener("abort", wake);
      }
    }
    const out = Uint8Array.from(this.#input);
    this.#input = [];
    return out;
  }

  /**
   * Read until `done`, after a silence once data arrived (`idleGap`), or at the reply timeout. Data
   * that arrived counts even when incomplete; only silence is a timeout (null).
   */
  async #readUntil(idleGap: boolean, done: (buf: Uint8Array) => boolean, signal?: AbortSignal): Promise<Uint8Array | null> {
    const started = this.clock.now();
    let out = new Uint8Array(0);
    while (this.clock.now() - started < REPLY_TIMEOUT) {
      this.#checkOpen(signal);
      const chunk = await this.#read(READ_TIMEOUT, signal);
      if (chunk.length) {
        const next = new Uint8Array(out.length + chunk.length);
        next.set(out);
        next.set(chunk, out.length);
        out = next;
        if (done(out)) break;
      } else if (idleGap && out.length) {
        break;
      }
    }
    return out.length ? out : null;
  }

  /**
   * Clear unread input, send `request`, read the replies: at most `replyBytes` + RESYNC_ROOM bytes
   * are taken in.
   */
  async #exchange(request: Uint8Array, idleGap: boolean, expected: number, replyBytes: number,
                  signal?: AbortSignal): Promise<ParseReport | null> {
    this.#input = [];
    this.#budget = replyBytes + RESYNC_ROOM;
    try {
      await this.#write(request, signal);
      const data = await this.#readUntil(idleGap, (buf) => parseResponseStream(buf).packets.length >= expected, signal);
      return data ? parseResponseStream(data) : null;
    } finally {
      this.#budget = 0;
      this.#input = [];
    }
  }

  #filter(id: number): AngleReliability {
    let f = this.filters.get(id);
    if (!f) this.filters.set(id, (f = new AngleReliability()));
    return f;
  }

  /** Does servo `id` answer? false on silence; a reply with only a bad checksum throws. */
  ping(id: number, signal?: AbortSignal): Promise<boolean> {
    const request = encodePing(id);
    return this.#lock.run(async () => {
      const report = await this.#exchange(request, true, 1, PING_REPLY_BYTES, signal);
      if (!report) return false;
      if (!report.packets.length && report.errors.length) {
        const e = report.errors[0]!;
        throw new ServoBusError(`checksum mismatch: code=${e.code} expected=0x${hex2(e.expected)} got=0x${hex2(e.got)}`);
      }
      return report.packets.some((p) => decodePing(p) === id);
    });
  }

  /**
   * One code-25 request for all `ids`, waiting for every reply (no early exit on silence) within the
   * reply timeout. A servo that did not answer gets its last monitor marked unreliable, or null if
   * it never answered. Throws ServoBusError once a servo missed LOSS_THRESHOLD replies in a row
   * (checked in `ids` order, the first one stops the count).
   */
  syncMonitor(ids: readonly number[], signal?: AbortSignal): Promise<Map<number, ServoMonitor | null>> {
    const request = encodeSyncMonitor(ids);
    return this.#lock.run(async () => {
      const report = await this.#exchange(request, false, ids.length, ids.length * MONITOR_REPLY_BYTES, signal);
      const held = (id: number): ServoMonitor | null => {
        const last = this.lastMonitors.get(id);
        return last ? { ...last, reliable: false } : null;
      };
      const result = new Map<number, ServoMonitor | null>();
      if (!report) {
        for (const id of ids) this.loss.miss(id);
        for (const id of ids) result.set(id, held(id));
        return result;
      }
      const got = new Map<number, ServoMonitor>();
      for (const packet of report.packets) {
        const m = decodeMonitor(packet);
        if (!m) continue;
        const [filtered, ok] = this.#filter(m.id).filter(m.angleDeg);
        m.angleDeg = filtered;
        m.reliable = ok;
        this.loss.ok(m.id);
        this.lastMonitors.set(m.id, m);
        got.set(m.id, m);
      }
      for (const id of ids) {
        const m = got.get(id);
        if (m) {
          result.set(id, m);
        } else {
          this.loss.miss(id);
          result.set(id, held(id));
        }
      }
      return result;
    });
  }

  /** Set the stop mode (write only, no reply is read). */
  setStopMode(id: number, mode: number, power: number, signal?: AbortSignal): Promise<void> {
    const request = encodeSetStopMode(id, mode, power);
    return this.#lock.run(() => this.#write(request, signal));
  }

  /** Stop mode "unloaded": the servo turns freely (write only). */
  unlock(id: number, signal?: AbortSignal): Promise<void> {
    return this.setStopMode(id, STOP_MODE_UNLOCKED, 0, signal);
  }

  /** Reset the servo's turn counter (write only). */
  resetMultiTurn(id: number, signal?: AbortSignal): Promise<void> {
    const request = encodeResetMultiTurn(id);
    return this.#lock.run(() => this.#write(request, signal));
  }

  /** Make the present position the servo's origin (write only). */
  setOriginPoint(id: number, signal?: AbortSignal): Promise<void> {
    const request = encodeSetOrigin(id);
    return this.#lock.run(() => this.#write(request, signal));
  }

  /** Stop reading, release the streams and close the port. */
  close(): Promise<void> {
    this.#closed ??= (async () => {
      this.#closing.abort();
      this.#arrived.notify();
      await this.#reader.cancel().catch(() => undefined);
      await this.#pump;
      this.#reader.releaseLock();
      this.#writer.releaseLock();
      await this.port.close();
    })();
    return this.#closed;
  }
}

const hex2 = (n: number): string => n.toString(16).padStart(2, "0");
