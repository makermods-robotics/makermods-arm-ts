/**
 * Lawicel SLCAN adapters (CANable and compatibles running slcan firmware) over a serial port: Web
 * Serial in a page or a Web Worker, the `serialport` package in Node (see "@makermods/arm-core/node"),
 * or FakeSerialPort in tests. The userspace equivalent of Linux `slcand`.
 *
 * Wire format: ASCII records ending in CR. The host opens the channel with `C`, `S<n>` (bitrate),
 * `A0`/`A1` (canable-fw's auto-retransmit switch) and `O` in one write, sends `t<iii><l><dd…>` (11-bit
 * id) or `T<iiiiiiii><l><dd…>` (29-bit id) per frame, and closes with `C`. Received records that are
 * not frames (command acknowledgements, status replies) are ignored.
 *
 * The normaldotcom canable-fw this targets has no USB TX buffering, a three-frame CAN RX FIFO and a
 * 28-frame CAN TX queue drained one frame per main-loop pass, so every frame is spaced txInterval
 * (1.5 ms) after the previous write and the backend asks the control loop for feedback-paced control.
 */

import { boundAfterIo, type Clock, Mutex, RealClock, Signal } from "../clock.js";
import { ConnectError, TimeoutError, ValueError } from "../errors.js";
import { CanBackend } from "./base.js";
import type { TransportLogger } from "./gs-usb.js";
import { grantedSerialPorts, requestSerialPort, type SerialLike, type SerialPortLike } from "./serial.js";

/** CAN bitrate (bit/s) -> the command that selects it. */
export const BITRATE_COMMANDS: Readonly<Record<number, string>> = Object.freeze({
  10_000: "S0", 20_000: "S1", 50_000: "S2", 100_000: "S3",
  125_000: "S4", 250_000: "S5", 500_000: "S6", 800_000: "S7",
  1_000_000: "S8",
});

/** Record terminator. */
const CR = 0x0d;

/** Python's `{n:#x}`. */
const pyHex = (n: number) => (n < 0 ? "-0x" + (-n).toString(16) : "0x" + n.toString(16));

const hexUpper = (n: number, width: number) => n.toString(16).toUpperCase().padStart(width, "0");

const ascii = (text: string): Uint8Array<ArrayBuffer> => {
  const out = new Uint8Array(text.length);
  for (let i = 0; i < text.length; i++) out[i] = text.charCodeAt(i);
  return out;
};

/**
 * One classic-CAN data frame in Lawicel SLCAN format, CR included. Throws ValueError for more than 8
 * data bytes or an id outside 0..0x7FF (standard) / 0..0x1FFFFFFF (extended).
 */
export function slcanEncode(canId: number, data: Uint8Array, extended = true): Uint8Array<ArrayBuffer> {
  if (data.length > 8) throw new ValueError(`classic CAN payload is limited to 8 bytes, got ${data.length}`);
  if (!Number.isInteger(canId)) throw new ValueError(`CAN ID must be an integer, got ${canId}`);
  let head: string;
  if (extended) {
    if (!(canId >= 0 && canId <= 0x1fffffff)) throw new ValueError(`extended CAN ID out of range: ${pyHex(canId)}`);
    head = "T" + hexUpper(canId, 8) + data.length.toString(16).toUpperCase();
  } else {
    if (!(canId >= 0 && canId <= 0x7ff)) throw new ValueError(`standard CAN ID out of range: ${pyHex(canId)}`);
    head = "t" + hexUpper(canId, 3) + data.length.toString(16).toUpperCase();
  }
  let text = head;
  for (const b of data) text += hexUpper(b, 2);
  return ascii(text + "\r");
}

// ---------- Parsing, exactly as Python's SlcanFrameParser (int(x, 16) and bytes.fromhex included) ----------

/** ASCII whitespace as Python's int() and bytes.fromhex() skip it. */
const isSpace = (c: number) => c === 0x20 || (c >= 0x09 && c <= 0x0d);

const hexValue = (c: number): number =>
  c >= 0x30 && c <= 0x39 ? c - 0x30 : c >= 0x41 && c <= 0x46 ? c - 0x37 : c >= 0x61 && c <= 0x66 ? c - 0x57 : -1;

/** Python's int(raw, 16) on bytes (surrounding whitespace, a sign, a 0x prefix, single underscores), or null for ValueError. */
function pyIntHex(raw: Uint8Array): number | null {
  let i = 0, j = raw.length;
  while (i < j && isSpace(raw[i]!)) i++;
  while (j > i && isSpace(raw[j - 1]!)) j--;
  let negative = false;
  if (i < j && (raw[i] === 0x2b || raw[i] === 0x2d)) {
    negative = raw[i] === 0x2d;
    i++;
  }
  if (j - i >= 2 && raw[i] === 0x30 && (raw[i + 1] === 0x78 || raw[i + 1] === 0x58)) {
    i += 2;
    if (i < j && raw[i] === 0x5f) i++;
  }
  if (i >= j) return null;
  let value = 0, digit = false;
  for (; i < j; i++) {
    const c = raw[i]!;
    if (c === 0x5f) {
      if (!digit) return null;
      digit = false;
      continue;
    }
    const v = hexValue(c);
    if (v < 0) return null;
    value = value * 16 + v;
    digit = true;
  }
  if (!digit) return null;
  return negative && value !== 0 ? -value : value;
}

/** Python's bytes.fromhex(raw.decode("ascii")): pairs of hex digits, whitespace between pairs; null for an error. */
function pyFromHex(raw: Uint8Array): Uint8Array | null {
  const out: number[] = [];
  let i = 0;
  for (;;) {
    while (i < raw.length && isSpace(raw[i]!)) i++;
    if (i >= raw.length) break;
    const hi = hexValue(raw[i]!);
    const lo = i + 1 < raw.length ? hexValue(raw[i + 1]!) : -1;
    if (hi < 0 || lo < 0) return null;
    out.push(hi * 16 + lo);
    i += 2;
  }
  return Uint8Array.from(out);
}

/** A received frame: [canId, data, extended]. */
export type SlcanFrame = [canId: number, data: Uint8Array, extended: boolean];

/**
 * Incremental SLCAN stream parser. Records end with CR; records other than `t`/`T` frames (command
 * acknowledgements, status replies) and malformed frames are skipped. Without a CR for more than 256
 * bytes the buffer is cut to its last 32, bounding garbage when the stream loses framing.
 */
export class SlcanFrameParser {
  #buf: Uint8Array = new Uint8Array(0);

  /** Bytes held while waiting for a CR. */
  get buffered(): number {
    return this.#buf.length;
  }

  /** The frames completed by `chunk`, in order. */
  feed(chunk: Uint8Array): SlcanFrame[] {
    const joined = new Uint8Array(this.#buf.length + chunk.length);
    joined.set(this.#buf);
    joined.set(chunk, this.#buf.length);
    let buf = joined;
    const out: SlcanFrame[] = [];
    for (;;) {
      const end = buf.indexOf(CR);
      if (end < 0) {
        // A valid classic-CAN record is at most 30 bytes including a timestamp.
        if (buf.length > 256) buf = buf.slice(buf.length - 32);
        break;
      }
      const raw = buf.subarray(0, end);
      buf = buf.subarray(end + 1);
      const frame = parseRecord(raw);
      if (frame) out.push(frame);
    }
    this.#buf = buf.slice();
    return out;
  }
}

function parseRecord(raw: Uint8Array): SlcanFrame | null {
  if (!raw.length || (raw[0] !== 0x74 && raw[0] !== 0x54)) return null;
  const extended = raw[0] === 0x54;
  const idLen = extended ? 8 : 3;
  if (raw.length < 2 + idLen) return null;
  const canId = pyIntHex(raw.subarray(1, 1 + idLen));
  if (canId === null) return null;
  const dlc = pyIntHex(raw.subarray(1 + idLen, 2 + idLen));
  if (dlc === null || dlc > 8) return null;
  const payloadEnd = 2 + idLen + dlc * 2;
  if (raw.length < payloadEnd) return null;
  const data = pyFromHex(raw.subarray(2 + idLen, payloadEnd));
  if (data === null) return null;
  return [canId, data, extended];
}

// ---------- Browser helpers (page or Web Worker) ----------

/**
 * Shows the browser's serial port chooser (unfiltered: SLCAN adapters are plain USB serial devices
 * with many vendor ids). Must be called on the page from a user gesture (a click). Resolves to the
 * chosen port, or null when the chooser was dismissed.
 */
export function requestSlcanPort(serial?: SerialLike): Promise<SerialPortLike | null> {
  return serial ? requestSerialPort([], serial) : requestSerialPort();
}

/** The serial ports this page was already granted (works in a page and in a Web Worker). */
export function grantedSlcanPorts(serial?: SerialLike): Promise<SerialPortLike[]> {
  return serial ? grantedSerialPorts([], serial) : grantedSerialPorts();
}

// ---------- Backend ----------

/** SlcanSerialBackend settings; the defaults are the Python SDK's. */
export interface SlcanSerialBackendOptions {
  /** Serial baud rate (default 115200; USB CDC adapters ignore it). */
  baudrate?: number;
  /** CAN bitrate, one of BITRATE_COMMANDS (default 1 000 000). */
  bitrate?: number;
  /** Seconds to wait after the port opens before the first command: USB serial bridges may reset on open (default 2.5). */
  startupDelay?: number;
  /** `A1` instead of `A0`: let the adapter retry a frame nothing acknowledges (default false). */
  autoRetransmit?: boolean;
  /** Minimum seconds between the end of one write and the next frame (default 0.0015). */
  txInterval?: number;
  /** Bound on every write, in seconds (default 1.0). */
  writeTimeout?: number;
  /** How the port is named in messages (default: its USB ids, or "serial port"). */
  name?: string;
  clock?: Clock;
  logger?: TransportLogger;
}

/**
 * An open channel: what send() writes to. A send accepted in one session never writes in another.
 * A failed write errors a Web Serial writable stream (the port then hands out a new one), so the
 * writer is replaced before the next write.
 */
interface Session {
  writer: WritableStreamDefaultWriter<Uint8Array>;
  broken: boolean;
}

/** An operation that outlived its time bound (the operation itself may still complete). */
class BoundExpired extends Error {}

const reason = (e: unknown) => (e instanceof Error ? e.message : String(e)).replace(/\.$/, "");

const OPEN_HINT =
  "Close other programs using the port (a serial monitor, slcand, another tab); unplug and replug the adapter if it " +
  "stopped answering. On Linux without root, add yourself to the dialout group.";

/** A CAN backend on one SLCAN adapter. The port is opened by open() and closed by close(). */
export class SlcanSerialBackend extends CanBackend {
  static readonly BITRATE_COMMANDS = BITRATE_COMMANDS;

  readonly serial: SerialPortLike;
  readonly baudrate: number;
  readonly bitrate: number;
  readonly startupDelay: number;
  readonly autoRetransmit: boolean;
  readonly txInterval: number;
  readonly writeTimeout: number;
  readonly clock: Clock;
  /** Why receiving stopped (a read failure, an unplug), or null. Sends are still attempted. */
  error: string | null = null;
  /**
   * What went wrong in the last close (or in the cleanup of a failed open): a `C` that could not be
   * written or a port that would not close, so the adapter's channel may still be open. Null when
   * clean; reset by open().
   */
  override closeError: string | null = null;

  #name: string | undefined;
  #log: TransportLogger;
  #session: Session | null = null;
  #reader: ReadableStreamDefaultReader<Uint8Array> | null = null;
  #writer: WritableStreamDefaultWriter<Uint8Array> | null = null;
  #portOpen = false;
  #running = false;
  #rxGen = 0;
  #rxDone: Promise<void> = Promise.resolve();
  /** Received bytes are dropped until open() has waited out the startup delay (pyserial's reset_input_buffer). */
  #discard = true;
  #parser = new SlcanFrameParser();
  /** When the last write to the port settled (resolved or failed): frames are paced from here. */
  #lastTx = 0.0;
  /** The one write handed to the port and not yet settled, or null. Nothing else is handed over meanwhile. */
  #inflight: Promise<void> | null = null;
  /** Set when a write outlived its caller's timeout while in the port's stream; cleared when it settles. */
  #stalled: string | null = null;
  #settled = new Signal();
  #lifecycle = new Mutex();
  #txLock = new Mutex();

  constructor(port: SerialPortLike, options: SlcanSerialBackendOptions = {}) {
    super();
    const bitrate = options.bitrate ?? 1_000_000;
    if (!Object.hasOwn(BITRATE_COMMANDS, bitrate)) {
      const choices = Object.keys(BITRATE_COMMANDS).map(Number).sort((a, b) => a - b);
      throw new ValueError(`unsupported SLCAN bitrate ${bitrate}; choices: [${choices.join(", ")}]`);
    }
    const txInterval = options.txInterval ?? 0.0015;
    const writeTimeout = options.writeTimeout ?? 1.0;
    if (txInterval < 0) throw new ValueError("SLCAN tx_interval must be non-negative");
    if (!(writeTimeout > 0)) throw new ValueError("SLCAN write_timeout must be positive");
    this.serial = port;
    this.baudrate = options.baudrate ?? 115200;
    this.bitrate = bitrate;
    this.startupDelay = options.startupDelay ?? 2.5;
    this.autoRetransmit = options.autoRetransmit ?? false;
    this.txInterval = txInterval;
    this.writeTimeout = writeTimeout;
    this.clock = options.clock ?? new RealClock();
    this.#name = options.name;
    this.#log = options.logger ?? console;
    // canable-fw has no USB-TX buffering and its hardware CAN RX FIFO holds only three frames: the
    // control loop waits for each motor's reply before commanding the next, so a burst of replies
    // cannot hide the later motors.
    this.feedbackPacedControl = true;
    this.feedbackWaitTimeout = 0.015;
  }

  /** The port's name in messages: the `name` option, else `usb:VVVV:PPPP`, else "serial port". */
  get port(): string {
    if (this.#name) return this.#name;
    const info = this.serial.getInfo?.() ?? {};
    if (info.usbVendorId !== undefined && info.usbProductId !== undefined) {
      return `usb:${hexUpper(info.usbVendorId, 4)}:${hexUpper(info.usbProductId, 4)}`;
    }
    return "serial port";
  }

  /**
   * Why the backend is in an unknown state, or null: a frame write timed out after the port had
   * taken it, so it may still reach the bus once the adapter recovers. Sends wait for it to settle
   * (up to writeTimeout) and write nothing until it has. Cleared when it settles, or by close().
   */
  get stalled(): string | null {
    return this.#stalled;
  }

  /** True between a successful open() and close(). */
  get isOpen(): boolean {
    return this.#session !== null;
  }

  /** The commands open() writes, in one write: close, bitrate, retransmit mode, open. */
  get openCommands(): string {
    return `C\r${BITRATE_COMMANDS[this.bitrate]}\r${this.autoRetransmit ? "A1" : "A0"}\rO\r`;
  }

  /** `work` bounded by `seconds`, checked only after pending I/O has been delivered (boundAfterIo). */
  #bounded<T>(work: Promise<T>, seconds: number, what: string): Promise<T> {
    return boundAfterIo(this.clock, work, seconds, () => new BoundExpired(`${what} timed out after ${seconds} s`));
  }

  /**
   * Hands one write to the port, bounded by writeTimeout. Only called with no write in flight, so
   * the port's stream never holds more than this one write. A write that outlives the bound stays
   * in flight (and marks the backend stalled) until it settles; pacing restarts from its settlement.
   */
  #write(writer: WritableStreamDefaultWriter<Uint8Array>, bytes: Uint8Array, what: string): Promise<void> {
    let write: Promise<void>;
    try {
      write = writer.write(bytes);
    } catch (e) {
      write = Promise.reject(e);
    }
    this.#inflight = write;
    const done = () => {
      this.#lastTx = this.clock.now();
      if (this.#inflight === write) {
        this.#inflight = null;
        this.#stalled = null;
      }
      this.#settled.notify();
    };
    write.then(done, done);
    return this.#bounded(write, this.writeTimeout, what).catch((e: unknown) => {
      if (e instanceof BoundExpired && this.#inflight === write) {
        this.#stalled = `a ${what} was taken by the port but not completed within ${this.writeTimeout} s; ` +
          "it may still reach the bus if the adapter recovers";
      }
      throw e;
    });
  }

  /** Waits, until `deadline` (clock time), for no write to be in flight. False when one still is. */
  async #idle(deadline: number, stop: () => boolean = () => false): Promise<boolean> {
    if (this.#inflight === null) return true;
    await this.#settled.wait(() => this.#inflight === null || stop(), deadline - this.clock.now(), this.clock);
    return this.#inflight === null;
  }

  /**
   * Opens the port, waits startupDelay, drops what arrived meanwhile and opens the CAN channel
   * (`C`, `S<n>`, `A0`/`A1`, `O` in one write). Serialised with close().
   */
  open(): Promise<void> {
    return this.#lifecycle.run(() => this.#open());
  }

  async #open(): Promise<void> {
    if (this.#session) return;
    const port = this.serial;
    this.closeError = null;
    let writer: WritableStreamDefaultWriter<Uint8Array>;
    try {
      try {
        await port.open({ baudRate: this.baudrate });
      } catch (e) {
        throw new ConnectError(`cannot open ${this.port}: ${reason(e)}. ${OPEN_HINT}`, { cause: e });
      }
      this.#portOpen = true;
      const readable = port.readable, writable = port.writable;
      if (!readable || !writable) throw new Error("the port opened without streams");
      this.#reader = readable.getReader();
      writer = this.#writer = writable.getWriter();
      this.error = null;
      this.#discard = true;
      this.#running = true;
      this.#rxDone = this.#receive(this.#reader, ++this.#rxGen);
      // USB serial bridges may reset when the port opens; commands sent during that reset are lost.
      if (this.startupDelay) await this.clock.sleep(this.startupDelay);
      if (this.error) throw new Error(this.error);
      this.#parser = new SlcanFrameParser();
      this.#discard = false;
      // A0 (canable-fw) stops an unacknowledged frame from being retried forever: no stale commands,
      // and the adapter's small TX queue stays responsive while motor power is off.
      await this.#txLock.run(async () => {
        await this.#write(writer, ascii(this.openCommands), "write of the open commands");
      });
    } catch (e) {
      await this.#shutdown(false);
      if (e instanceof ConnectError) throw e;
      throw new ConnectError(`cannot open the SLCAN channel on ${this.port}: ${reason(e)}. ${OPEN_HINT}`, { cause: e });
    }
    this.#session = this.#watch({ writer, broken: false });
  }

  /** Marks the session broken once its writer's stream errors. */
  #watch(session: Session): Session {
    const writer = session.writer;
    writer.closed.catch(() => {
      if (session.writer === writer) session.broken = true;
    });
    return session;
  }

  /** Takes a writer on the stream the port hands out after a failed write. Throws when it has none. */
  #renewWriter(session: Session): void {
    try {
      session.writer.releaseLock();
    } catch {
      // already released
    }
    const writable = this.serial.writable;
    if (!writable || writable.locked) throw new Error("the port has no usable writable stream");
    session.writer = this.#writer = writable.getWriter();
    session.broken = false;
    this.#watch(session);
  }

  /**
   * Stops receiving, closes the CAN channel (`C`) and closes the port. Never throws for a failed
   * `C` or port close: see closeError. Idempotent; serialised with open().
   */
  close(): Promise<void> {
    return this.#lifecycle.run(async () => {
      const session = this.#session;
      this.#session = null;
      if (session?.broken) {
        try {
          this.#renewWriter(session);
        } catch {
          // no usable stream (unplugged): the C below fails and closeError says so
        }
      }
      await this.#shutdown(session !== null);
    });
  }

  #closeProblem(message: string, error: unknown): void {
    this.closeError = this.closeError ? `${this.closeError}; ${message}` : message;
    this.#log.warn(`${message}; the adapter's CAN channel may still be open`, error);
  }

  async #shutdown(sendClose: boolean): Promise<void> {
    this.#running = false;
    this.#rxGen++;
    const reader = this.#reader, writer = this.#writer;
    this.#reader = null;
    // Stop receiving first (the Python receive thread's join, up to 1 s).
    if (reader) {
      const cancelled = reader.cancel().catch(() => undefined);
      await this.#bounded(Promise.all([cancelled, this.#rxDone]), 1.0, "stopping receive").catch(() => undefined);
      try {
        reader.releaseLock();
      } catch {
        // a read still pending on a stream that never ended: the port close below fails and says so
      }
    }
    if (writer) {
      await this.#txLock.run(async () => {
        // A write still in flight (a stalled adapter) gets writeTimeout to settle, as Python's close
        // waits for the TX lock; then C goes out (unpaced, as in Python).
        const idle = await this.#idle(this.clock.now() + this.writeTimeout);
        if (!idle) {
          // Abandon it: nothing more is written, and the port close below interrupts it.
          void this.#bounded(writer.abort(new Error("port closing")), this.writeTimeout, "abort").catch(() => undefined);
          if (sendClose) {
            this.#closeProblem(
              `C not sent on ${this.port}: a write was still pending after ${this.writeTimeout} s and was abandoned`, null);
          }
          return;
        }
        if (!sendClose) return;
        try {
          await this.#write(this.#writer ?? writer, ascii("C\r"), "write of C");
        } catch (e) {
          this.#closeProblem(`could not close the SLCAN channel on ${this.port}: ${reason(e)}`, e);
          if (this.#inflight) void this.#bounded(writer.abort(new Error("port closing")), this.writeTimeout, "abort").catch(() => undefined);
        }
      });
      try {
        (this.#writer ?? writer).releaseLock();
      } catch {
        // already released
      }
      this.#writer = null;
    }
    if (this.#portOpen) {
      this.#portOpen = false;
      try {
        await this.#bounded(this.serial.close(), this.writeTimeout + 1.0, "closing the port");
      } catch (e) {
        this.#closeProblem(`could not close ${this.port}: ${reason(e)}`, e);
      }
    }
    // A write abandoned above belongs to the closed port; the next session starts clean.
    this.#inflight = null;
    this.#stalled = null;
  }

  async #receive(reader: ReadableStreamDefaultReader<Uint8Array>, gen: number): Promise<void> {
    const current = () => this.#running && this.#rxGen === gen;
    while (current()) {
      let result: ReadableStreamReadResult<Uint8Array>;
      try {
        result = await reader.read();
      } catch (e) {
        if (current()) this.#fail(`serial read failed: ${reason(e)}`, e);
        break;
      }
      if (!current()) break;
      if (result.done) {
        this.#fail("the serial port stopped delivering data (closed or unplugged)");
        break;
      }
      const chunk = result.value;
      if (this.#discard || !chunk?.length) continue;
      try {
        const cb = this.cb;
        for (const [canId, data, extended] of this.#parser.feed(chunk)) {
          if (cb) cb(canId, data, extended);
        }
      } catch (e) {
        this.#log.error("SLCAN RX frame-processing exception (continuing to receive frames)", e);
      }
    }
    if (this.#rxGen === gen) this.#running = false;
  }

  #fail(message: string, cause?: unknown): void {
    this.error = message;
    this.#log.error(`${this.port}: ${message}; receive stopped (unplug and replug the adapter)`, cause);
  }

  #notOpen(): ConnectError {
    return new ConnectError(`SLCAN backend is not open (${this.port})` + (this.error ? ` (${this.error})` : ""));
  }

  /**
   * Writes one frame, at least txInterval after the previous write settled, every frame (motion,
   * enable/disable, parameters) included so callers cannot burst-fill the adapter's queue. At most
   * one write is ever handed to the port: while one is in flight (a stalled adapter) this send waits
   * for it, up to writeTimeout, and fails with TimeoutError without writing anything if it does not
   * settle. Refused only when the backend is not open: after a receive failure frames are still
   * attempted (a torque-off must reach the bus).
   */
  async send(canId: number, data: Uint8Array, extended = true): Promise<void> {
    const session = this.#session;
    if (session === null) throw this.#notOpen();
    const frame = slcanEncode(canId, Uint8Array.from(data), extended);
    await this.#txLock.run(async () => {
      if (this.#session !== session) throw this.#notOpen();
      const deadline = this.clock.now() + this.writeTimeout;
      if (!(await this.#idle(deadline, () => this.#session !== session))) {
        if (this.#session !== session) throw this.#notOpen();
        throw new TimeoutError(
          `${this.port}: frame not sent: the previous write has not completed (${this.#stalled ?? "still pending"}); ` +
            "the adapter may need unplugging and replugging",
        );
      }
      if (this.#session !== session) throw this.#notOpen();
      const remain = this.txInterval - (this.clock.now() - this.#lastTx);
      if (remain > 0) {
        await this.clock.sleep(remain);
        if (this.#session !== session) throw this.#notOpen();
      }
      try {
        if (session.broken) this.#renewWriter(session);
        await this.#write(session.writer, frame, "serial write");
      } catch (e) {
        throw new TimeoutError(
          `${this.port}: serial write failed (${reason(e)}); the adapter may need unplugging and replugging`, { cause: e },
        );
      }
    });
  }
}
