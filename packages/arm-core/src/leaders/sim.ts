/**
 * A simulated Star Arm 102 leader: a serial port (SerialPortLike) with simulated FashionStar servos
 * behind it, for tests, application development and a browser demo. A test or a UI slider sets
 * each servo's angle; the StarLeader on the port reads it through the real protocol code.
 *
 *     const [leader, port] = simulatedStarLeader({ product: MAKER_ARM, clock });
 *     await leader.connect();
 *     port.setAngle(0, 30);            // servo 0 turned to 30 degrees
 *     const targets = await leader.read();
 *
 * Each servo reports `physical - origin - 360 * turns` (degrees): set origin makes the present
 * position its zero, and the multi-turn reset drops whole turns so the reading lands in
 * [-180, 180]. The servos answer at once; a servo set offline never answers.
 */

import type { SerialPortLike } from "../transport/serial.js";
import {
  CODE_PING, CODE_QUERY_MONITOR, CODE_RESET_MTURN, CODE_SET_ORIGIN, CODE_SET_STOP_MODE, CODE_SYNC_COMMAND, REQ_HEADER,
  RESP_HEADER, checksum,
} from "./fashionstar.js";
import { StarLeader, type StarLeaderOptions } from "./star.js";

/** Python's round(): half to even, an integer. */
function roundHalfEven(x: number): number {
  const r = Math.round(x);
  return (r - x === 0.5 && r % 2 !== 0 ? r - 1 : r) + 0;
}

function packReply(code: number, params: ArrayLike<number>): Uint8Array {
  const out = new Uint8Array(5 + params.length);
  out.set(RESP_HEADER, 0);
  out[2] = code;
  out[3] = params.length;
  out.set(Array.from(params), 4);
  out[4 + params.length] = checksum(RESP_HEADER, code, params);
  return out;
}

/** One simulated FashionStar servo. */
export class SimulatedServo {
  readonly id: number;
  /** Shaft angle, degrees (what a slider moves). */
  physicalDeg: number;
  /** The shaft angle set origin made zero. */
  originDeg = 0.0;
  /** Whole turns the multi-turn reset dropped. */
  turns = 0;
  /** false: the servo never answers. */
  online = true;
  /** The last stop mode set (0x10: unloaded), or null. */
  stopMode: number | null = null;
  voltageMv = 12000;
  currentMa = 0;
  powerMw = 0;
  tempRaw = 0;
  status = 0;

  constructor(id: number, physicalDeg = 0.0) {
    this.id = id;
    this.physicalDeg = physicalDeg;
  }

  /** What the servo reports, degrees. */
  get angleDeg(): number {
    return this.physicalDeg - this.originDeg - 360 * this.turns;
  }

  /** Move the shaft so the servo reports `deg`. */
  set angleDeg(deg: number) {
    this.physicalDeg = deg + this.originDeg + 360 * this.turns;
  }

  setOrigin(): void {
    this.originDeg = this.physicalDeg;
    this.turns = 0;
  }

  resetMultiTurn(): void {
    this.turns = roundHalfEven((this.physicalDeg - this.originDeg) / 360);
  }

  /** The 16-byte code-22 monitor reply's parameters. */
  monitorParams(): Uint8Array {
    const p = new Uint8Array(16);
    const v = new DataView(p.buffer);
    p[0] = this.id;
    v.setUint16(1, this.voltageMv, true);
    v.setUint16(3, this.currentMa, true);
    v.setUint16(5, this.powerMw, true);
    v.setUint16(7, this.tempRaw, true);
    p[9] = this.status;
    v.setInt32(10, roundHalfEven(this.angleDeg * 10), true);
    v.setInt16(14, 0, true);
    return p;
  }
}

/** A serial port with simulated FashionStar servos on it. */
export class SimulatedStarPort implements SerialPortLike {
  readonly servos = new Map<number, SimulatedServo>();
  /** Every chunk written to the port, in order. */
  readonly written: Uint8Array[] = [];
  /** Every reply chunk the servos sent, in order. */
  readonly replies: Uint8Array[] = [];
  /** Bytes sent ahead of every reply set (line noise; default none). */
  noise: Uint8Array | null = null;
  /** The baud rate of the last open, or null while closed. */
  baudRate: number | null = null;
  #readable: ReadableStream<Uint8Array> | null = null;
  #writable: WritableStream<Uint8Array> | null = null;
  #controller: ReadableStreamDefaultController<Uint8Array> | null = null;
  #pending: number[] = [];

  /** `angles`: servo id -> reported degrees (default: servos 0-6 at 0). */
  constructor(angles: ReadonlyMap<number, number> | Readonly<Record<number, number>> | readonly number[] = [0, 0, 0, 0, 0, 0, 0]) {
    const entries: Array<[number, number]> = Array.isArray(angles)
      ? (angles as readonly number[]).map((a, i) => [i, a])
      : angles instanceof Map
        ? [...(angles as ReadonlyMap<number, number>)]
        : Object.entries(angles as Record<number, number>).map(([k, a]) => [Number(k), a]);
    for (const [id, a] of entries) this.servos.set(id, new SimulatedServo(id, a));
  }

  get readable(): ReadableStream<Uint8Array> | null {
    return this.#readable;
  }

  get writable(): WritableStream<Uint8Array> | null {
    return this.#writable;
  }

  get isOpen(): boolean {
    return this.baudRate !== null;
  }

  /** The servo with this id (throws for an unknown id). */
  servo(id: number): SimulatedServo {
    const s = this.servos.get(id);
    if (!s) throw new RangeError(`no simulated servo ${id}`);
    return s;
  }

  /** What servo `id` reports, degrees. */
  angle(id: number): number {
    return this.servo(id).angleDeg;
  }

  /** Turn servo `id` so it reports `deg` (a slider's value). */
  setAngle(id: number, deg: number): void {
    this.servo(id).angleDeg = deg;
  }

  /** Set several servos: an array is servo ids 0, 1, ... */
  setAngles(angles: ReadonlyMap<number, number> | Readonly<Record<number, number>> | readonly number[]): void {
    if (Array.isArray(angles)) (angles as readonly number[]).forEach((a, i) => this.setAngle(i, a));
    else if (angles instanceof Map) for (const [id, a] of angles as ReadonlyMap<number, number>) this.setAngle(id, a);
    else for (const [id, a] of Object.entries(angles as Record<number, number>)) this.setAngle(Number(id), a);
  }

  /** Send `bytes` to the host now, unasked (a chattering device, line noise). */
  inject(bytes: Uint8Array): void {
    this.#controller?.enqueue(Uint8Array.from(bytes));
  }

  /** Take servo `id` off the bus (false) or put it back. */
  setOnline(id: number, online: boolean): void {
    this.servo(id).online = online;
  }

  async open(options: { baudRate: number }): Promise<void> {
    if (this.baudRate !== null) throw new DOMException("The port is already open.", "InvalidStateError");
    this.baudRate = options.baudRate;
    this.#pending = [];
    this.#readable = new ReadableStream<Uint8Array>({
      start: (controller) => {
        this.#controller = controller;
      },
      cancel: () => {
        this.#controller = null;
      },
    });
    this.#writable = new WritableStream<Uint8Array>({
      write: (chunk) => this.#receive(chunk),
    });
  }

  async close(): Promise<void> {
    if (this.baudRate === null) throw new DOMException("The port is already closed.", "InvalidStateError");
    if (this.#readable?.locked || this.#writable?.locked) {
      throw new TypeError("Cannot close a serial port with a locked stream");
    }
    try {
      this.#controller?.close();
    } catch {
      // already cancelled
    }
    this.#controller = null;
    this.#readable = null;
    this.#writable = null;
    this.baudRate = null;
  }

  #receive(chunk: Uint8Array): void {
    this.written.push(Uint8Array.from(chunk));
    for (const b of chunk) this.#pending.push(b);
    const buf = this.#pending;
    while (buf.length >= 5) {
      if (buf[0] !== REQ_HEADER[0] || buf[1] !== REQ_HEADER[1]) {
        buf.shift();
        continue;
      }
      const size = buf[3]!;
      if (buf.length < 5 + size) break;
      const code = buf[2]!;
      const params = buf.slice(4, 4 + size);
      const ok = buf[4 + size] === checksum(REQ_HEADER, code, params);
      buf.splice(0, 5 + size);
      if (ok) this.#handle(code, params);
    }
  }

  #handle(code: number, params: number[]): void {
    const live = (id: number | undefined): SimulatedServo | null => {
      const s = id === undefined ? undefined : this.servos.get(id);
      return s && s.online ? s : null;
    };
    const replies: Uint8Array[] = [];
    if (code === CODE_PING) {
      const s = live(params[0]);
      if (s) replies.push(packReply(CODE_PING, [s.id]));
    } else if (code === CODE_SET_STOP_MODE) {
      const s = live(params[0]);
      if (s) s.stopMode = params[1]!;
    } else if (code === CODE_RESET_MTURN) {
      live(params[0])?.resetMultiTurn();
    } else if (code === CODE_SET_ORIGIN) {
      live(params[0])?.setOrigin();
    } else if (code === CODE_SYNC_COMMAND && params[0] === CODE_QUERY_MONITOR) {
      for (const id of params.slice(3, 3 + params[2]!)) {
        const s = live(id);
        if (s) replies.push(packReply(CODE_QUERY_MONITOR, s.monitorParams()));
      }
    }
    if (!replies.length) return;
    if (this.noise?.length) this.#controller?.enqueue(Uint8Array.from(this.noise));
    const out = new Uint8Array(replies.reduce((n, r) => n + r.length, 0));
    let at = 0;
    for (const r of replies) {
      out.set(r, at);
      at += r.length;
    }
    this.replies.push(out);
    this.#controller?.enqueue(out);
  }
}

/** Options of simulatedStarLeader(): StarLeader's, and the servos' starting angles. */
export interface SimulatedStarLeaderOptions extends StarLeaderOptions {
  /** Servo id -> reported degrees (default: every mapped servo at 0). */
  angles?: ReadonlyMap<number, number> | Readonly<Record<number, number>> | readonly number[];
}

/**
 * A StarLeader on a SimulatedStarPort with one servo per mapped joint. Needs `mapping` or `product`,
 * as StarLeader does; pass the app's clock (a VirtualClock in tests).
 */
export function simulatedStarLeader(options: SimulatedStarLeaderOptions = {}): [StarLeader, SimulatedStarPort] {
  const { angles, ...leaderOptions } = options;
  const port = new SimulatedStarPort([]);
  const leader = new StarLeader(port, leaderOptions);
  for (const id of leader.servoIds) port.servos.set(id, new SimulatedServo(id, 0.0));
  if (angles !== undefined) port.setAngles(angles);
  return [leader, port];
}
