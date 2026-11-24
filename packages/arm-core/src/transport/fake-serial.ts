/**
 * A fake SLCAN adapter on a fake serial port: a Web Serial-shaped port (open, readable, writable,
 * close, getInfo) with a canable-fw style command interpreter and a scriptable CAN bus behind it.
 * Frames written once the channel is open (`O`) are handed to `responder`, whose replies arrive on
 * the readable stream as SLCAN records. Everything resolves in microtasks unless a write delay is
 * set, so it runs on VirtualClock without real time passing. No real serial port is involved.
 */

import type { Clock } from "../clock.js";
import type { FakeBusResponder } from "./fake-gs-usb.js";
import type { SerialPortLike } from "./serial.js";
import { slcanEncode } from "./slcan-serial.js";

/** One write the host made, as the port saw it. */
export interface FakeSerialWrite {
  /** Clock time when the write started (null without a clock). */
  at: number | null;
  bytes: Uint8Array;
  /** The bytes as latin-1 text. */
  text: string;
}

/** A frame the adapter put on the bus. */
export interface FakeSlcanFrame {
  /** Clock time when the adapter took it (null without a clock). */
  at: number | null;
  canId: number;
  extended: boolean;
  data: Uint8Array;
  /** The record as written, without its CR. */
  text: string;
}

/** FakeSerialPort settings; every field has a CANable default. */
export interface FakeSerialPortOptions {
  /** USB ids getInfo() reports (default a CANable on canable-fw, AD50:60C4; null: not a USB port). */
  usbVendorId?: number | null;
  usbProductId?: number | null;
  responder?: FakeBusResponder;
  /** Timestamps writes and frames; needed for writeDelay. */
  clock?: Clock;
  /** Lawicel acknowledgements: CR after an accepted command, BELL after a refused one (canable-fw sends neither). */
  acks?: boolean;
  /** Received bytes are delivered in chunks of at most this many bytes (default: one chunk per record). */
  chunkSize?: number;
  /**
   * Frames reaching the adapter closer together than this many seconds are recorded in
   * protocolErrors (canable-fw's TX queue fills under a burst). Needs a clock. Default: no check.
   */
  minFrameInterval?: number;
}

const BELL = 0x07;
const CR = 0x0d;

const latin1 = (bytes: Uint8Array) => String.fromCharCode(...bytes);

const domError = (message: string, name: string) => new DOMException(message, name);

const FRAME = /^(?:t([0-9A-Fa-f]{3})|T([0-9A-Fa-f]{8}))([0-8])([0-9A-Fa-f]*)$/;

/** A fake serial port with an SLCAN adapter behind it. Its failure switches and records are public, for tests to script and check. */
export class FakeSerialPort implements SerialPortLike {
  readonly usbVendorId: number | null;
  readonly usbProductId: number | null;
  readonly clock: Clock | null;
  readonly acks: boolean;
  readonly chunkSize: number | null;
  readonly minFrameInterval: number | null;

  /** The port is open (Web Serial's open state). */
  opened = false;
  baudRate: number | null = null;
  /** The adapter's CAN channel (set by `O`, cleared by `C`; closing the port does not close it). */
  channelOpen = false;
  /** The last bitrate command taken (`S0`..`S8`), or null. */
  bitrateCommand: string | null = null;
  /** Auto-retransmit as the host last set it (`A1`: true, `A0`: false), or null. */
  autoRetransmit: boolean | null = null;

  /** The CAN bus behind the adapter. */
  responder: FakeBusResponder | null;
  /** The bus acknowledges frames (false: nothing ACKs, so nothing is answered). */
  ack = true;

  /** Thrown by the next open() (the port is busy or missing). */
  openError: Error | null = null;
  /** Writes reject with this error. */
  writeError: Error | null = null;
  /** Each write completes this many seconds after it starts (a slow adapter). Needs a clock. */
  writeDelay = 0;
  /** Writes do not complete until releaseWrites(), close() or unplug(). */
  hangWrites = false;
  /** The bus answers this many seconds after the adapter took the frame (a motor's reply latency). Needs a clock. */
  replyDelay = 0;

  /** Counts and records. */
  opens = 0;
  closes = 0;
  readonly writes: FakeSerialWrite[] = [];
  /** Every record the adapter took, without its CR, in order. */
  readonly commands: string[] = [];
  readonly frames: FakeSlcanFrame[] = [];
  /** Things a real adapter would not accept or a careful host would not do. */
  readonly protocolErrors: string[] = [];

  #readable: ReadableStream<Uint8Array> | null = null;
  #rc: ReadableStreamDefaultController<Uint8Array> | null = null;
  #writable: WritableStream<Uint8Array> | null = null;
  #hung: { bytes: Uint8Array; at: number | null; resolve: () => void; reject: (e: unknown) => void }[] = [];
  #cmd: number[] = [];
  #gone = false;
  #lastFrameAt: number | null = null;

  constructor(options: FakeSerialPortOptions = {}) {
    this.usbVendorId = options.usbVendorId === undefined ? 0xad50 : options.usbVendorId;
    this.usbProductId = options.usbProductId === undefined ? 0x60c4 : options.usbProductId;
    this.responder = options.responder ?? null;
    this.clock = options.clock ?? null;
    this.acks = options.acks ?? false;
    this.chunkSize = options.chunkSize ?? null;
    this.minFrameInterval = options.minFrameInterval ?? null;
  }

  /** True once unplug() was called. */
  get unplugged(): boolean {
    return this.#gone;
  }

  /** Writes stuck on hangWrites. */
  get pendingWrites(): number {
    return this.#hung.length;
  }

  getInfo(): { usbVendorId?: number; usbProductId?: number } {
    const info: { usbVendorId?: number; usbProductId?: number } = {};
    if (this.usbVendorId !== null) info.usbVendorId = this.usbVendorId;
    if (this.usbProductId !== null) info.usbProductId = this.usbProductId;
    return info;
  }

  get readable(): ReadableStream<Uint8Array> | null {
    if (!this.opened || this.#gone) return null;
    if (!this.#readable) {
      this.#readable = new ReadableStream<Uint8Array>({
        start: (c) => {
          this.#rc = c;
        },
        cancel: () => {
          this.#rc = null;
          this.#readable = null;
        },
      });
    }
    return this.#readable;
  }

  get writable(): WritableStream<Uint8Array> | null {
    if (!this.opened || this.#gone) return null;
    if (!this.#writable) {
      this.#writable = new WritableStream<Uint8Array>({
        write: (chunk) => this.#hostWrite(chunk),
        abort: () => this.#rejectHung(domError("The write was aborted.", "AbortError")),
      });
    }
    return this.#writable;
  }

  async open(options: { baudRate: number }): Promise<void> {
    if (this.#gone) throw domError("Failed to open serial port.", "NetworkError");
    if (this.opened) throw domError("The port is already open.", "InvalidStateError");
    if (this.openError) throw this.openError;
    if (!(options.baudRate > 0)) throw new TypeError("Requested baud rate must be greater than zero.");
    this.opened = true;
    this.baudRate = options.baudRate;
    this.opens++;
    this.#cmd = [];
  }

  async close(): Promise<void> {
    if (!this.opened) throw domError("The port is already closed.", "InvalidStateError");
    if (this.#readable?.locked || this.#writable?.locked) throw new TypeError("Cannot close a port with a locked stream.");
    this.#rejectHung(domError("The port was closed.", "AbortError"));
    const readable = this.#readable, writable = this.#writable;
    this.#readable = this.#rc = null;
    this.#writable = null;
    await readable?.cancel().catch(() => undefined);
    await writable?.abort().catch(() => undefined);
    this.opened = false;
    this.baudRate = null;
    this.closes++;
  }

  /** The stuck writes complete in order, as normal writes, and later writes are no longer held. */
  releaseWrites(): void {
    this.hangWrites = false;
    const hung = this.#hung;
    this.#hung = [];
    for (const h of hung) {
      this.#take(h.bytes);
      h.resolve();
    }
  }

  /**
   * The adapter stalls now and recovers after `seconds`: writes hang until then, and the stuck ones
   * then complete in order (releaseWrites). Needs a clock.
   */
  stallFor(seconds: number): void {
    if (!this.clock) throw new Error("FakeSerialPort.stallFor needs a clock");
    this.hangWrites = true;
    this.clock.timer(seconds, () => this.releaseWrites());
  }

  /** A frame from the bus to the host, as an SLCAN record (with a 4-digit timestamp when given). Dropped while the channel is closed. */
  receive(canId: number, data: Uint8Array, extra: { extended?: boolean; timestamp?: number } = {}): void {
    if (!this.channelOpen) return;
    let record = slcanEncode(canId, data, extra.extended ?? false);
    if (extra.timestamp !== undefined) {
      const stamp = (extra.timestamp & 0xffff).toString(16).toUpperCase().padStart(4, "0");
      const out = new Uint8Array(record.length + 4);
      out.set(record.subarray(0, record.length - 1));
      for (let i = 0; i < 4; i++) out[record.length - 1 + i] = stamp.charCodeAt(i);
      out[out.length - 1] = CR;
      record = out;
    }
    this.inject(record);
  }

  /** Raw bytes to the host (status lines, garbage, split records), chunked by chunkSize. Dropped while the port is closed. */
  inject(bytes: Uint8Array): void {
    if (!this.opened || this.#gone) return;
    void this.readable; // created on first data, as the port buffers until read
    const c = this.#rc;
    if (!c) return;
    const size = this.chunkSize ?? Math.max(1, bytes.length);
    for (let i = 0; i < bytes.length; i += size) c.enqueue(bytes.slice(i, i + size));
  }

  /** The readable stream fails with `error` (a serial read error). */
  failRead(error: Error): void {
    const c = this.#rc;
    if (!c) {
      this.protocolErrors.push("failRead() with no readable stream");
      return;
    }
    this.#rc = null;
    this.#readable = null;
    c.error(error);
  }

  /** The readable stream ends without an error. */
  endRead(): void {
    const c = this.#rc;
    this.#rc = null;
    this.#readable = null;
    c?.close();
  }

  /** The adapter is pulled out: reads and pending writes fail, later writes and opens fail. */
  unplug(): void {
    this.#gone = true;
    this.channelOpen = false;
    const lost = domError("The device has been lost.", "NetworkError");
    const c = this.#rc;
    this.#rc = null;
    this.#readable = null;
    c?.error(lost);
    this.#rejectHung(lost);
  }

  #now(): number | null {
    return this.clock ? this.clock.now() : null;
  }

  #hostWrite(chunk: Uint8Array): Promise<void> {
    const bytes = Uint8Array.from(chunk);
    const at = this.#now();
    this.writes.push({ at, bytes, text: latin1(bytes) });
    // A failed write errors the stream; the port hands out a new one, as Web Serial does.
    if (this.#gone) return this.#failWrite(domError("The device has been lost.", "NetworkError"));
    if (this.writeError) return this.#failWrite(this.writeError);
    if (this.hangWrites) return new Promise<void>((resolve, reject) => this.#hung.push({ bytes, at, resolve, reject }));
    if (this.writeDelay > 0) {
      if (!this.clock) throw new Error("FakeSerialPort.writeDelay needs a clock");
      return this.clock.sleep(this.writeDelay).then(() => {
        if (this.#gone) throw domError("The device has been lost.", "NetworkError");
        this.#take(bytes);
      });
    }
    this.#take(bytes);
    return Promise.resolve();
  }

  #failWrite(error: unknown): Promise<void> {
    this.#writable = null;
    return Promise.reject(error);
  }

  #rejectHung(error: unknown): void {
    const hung = this.#hung;
    this.#hung = [];
    for (const h of hung) h.reject(error);
  }

  /** The adapter reads bytes from USB: records end with CR. */
  #take(bytes: Uint8Array): void {
    for (const b of bytes) {
      if (b !== CR) {
        this.#cmd.push(b);
        continue;
      }
      const text = latin1(Uint8Array.from(this.#cmd));
      this.#cmd = [];
      this.commands.push(text);
      const ok = this.#command(text);
      if (this.acks) this.inject(Uint8Array.of(ok ? CR : BELL));
    }
  }

  #refuse(message: string): false {
    this.protocolErrors.push(message);
    return false;
  }

  #command(text: string): boolean {
    if (text === "") return true;
    if (text === "C") {
      this.channelOpen = false;
      return true;
    }
    if (text === "O") {
      if (this.bitrateCommand === null) return this.#refuse("O before a bitrate command");
      this.channelOpen = true;
      this.#lastFrameAt = null;
      return true;
    }
    if (/^S[0-8]$/.test(text)) {
      if (this.channelOpen) return this.#refuse(`${text} while the channel is open`);
      this.bitrateCommand = text;
      return true;
    }
    if (text === "A0" || text === "A1") {
      this.autoRetransmit = text === "A1";
      return true;
    }
    const m = FRAME.exec(text);
    if (text[0] === "t" || text[0] === "T") {
      if (!m || m[4]!.length !== Number(m[3]) * 2) return this.#refuse(`malformed frame record ${JSON.stringify(text)}`);
      if (!this.channelOpen) return this.#refuse(`frame before O: ${text}`);
      const extended = text[0] === "T";
      const canId = parseInt((extended ? m[2] : m[1])!, 16);
      const hex = m[4]!;
      const data = new Uint8Array(hex.length / 2);
      for (let i = 0; i < data.length; i++) data[i] = parseInt(hex.slice(2 * i, 2 * i + 2), 16);
      const at = this.#now();
      if (this.minFrameInterval !== null && at !== null && this.#lastFrameAt !== null
          && at - this.#lastFrameAt < this.minFrameInterval - 1e-12) {
        this.protocolErrors.push(`frames ${(at - this.#lastFrameAt).toFixed(6)} s apart (minimum ${this.minFrameInterval})`);
      }
      this.#lastFrameAt = at;
      this.frames.push({ at, canId, extended, data, text });
      if (this.ack) {
        const replies = this.responder?.(canId, data, extended) ?? [];
        const deliver = () => {
          for (const reply of replies) this.receive(reply[0], reply[1], { extended: reply[2] ?? false });
        };
        if (this.replyDelay > 0 && replies.length) {
          if (!this.clock) throw new Error("FakeSerialPort.replyDelay needs a clock");
          this.clock.timer(this.replyDelay, deliver);
        } else deliver();
      }
      return true;
    }
    return this.#refuse(`unknown command ${JSON.stringify(text)}`);
  }
}
