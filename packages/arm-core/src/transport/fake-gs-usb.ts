/**
 * A fake candleLight adapter: a WebUSB-shaped device (open, configuration, claim, vendor control
 * transfers, bulk IN/OUT) that speaks the gs_usb protocol, with a scriptable CAN bus behind it.
 * Frames written once the channel is started are echoed back (when the bus acknowledges them) and
 * handed to `responder`, whose replies arrive as received frames. Everything resolves in microtasks,
 * so it runs on VirtualClock without real time passing. No real USB device is involved.
 */

import {
  BREQ_BITTIMING, BREQ_BT_CONST, BREQ_MODE, CAN_EFF_FLAG, CAN_EFF_MASK, ECHO_RX, EP_IN, EP_OUT,
  FRAME_SIZE, INTERFACE, MODE_START, type UsbControlSetup, type UsbDeviceLike,
} from "./gs-usb.js";
import type { Clock } from "../clock.js";

type Status = "ok" | "stall" | "babble";
type InResult = { status: Status; data?: DataView };
type OutResult = { status: Status; bytesWritten: number };

/** One frame from the bus: [canId, data] (a standard id) or [canId, data, extended]. */
export type FakeBusFrame = [number, Uint8Array] | [number, Uint8Array, boolean];

/** Called for every frame the adapter puts on the bus; returns the frames the bus answers with. */
export type FakeBusResponder = (canId: number, data: Uint8Array, extended: boolean) => FakeBusFrame[] | null | undefined | void;

/** A control transfer as the device saw it. */
export interface FakeControl {
  dir: "in" | "out";
  request: number;
  value: number;
  index: number;
  /** The data stage (out) or the requested length (in). */
  data: Uint8Array | number;
}

/** A frame the host wrote, decoded. */
export interface FakeWrite {
  echoId: number;
  /** can_id as written, flags included. */
  ident: number;
  canId: number;
  extended: boolean;
  data: Uint8Array;
  /** The raw bulk OUT bytes. */
  raw: Uint8Array;
}

/** FakeGsUsbDevice settings; every field has a candleLight default. */
export interface FakeGsUsbDeviceOptions {
  serialNumber?: string | null;
  productName?: string | null;
  manufacturerName?: string | null;
  vendorId?: number;
  productId?: number;
  /** BT_CONST: feature word, clock and timing limits (defaults: a candleLight on an STM32F072, 48 MHz). */
  feature?: number;
  fclk?: number;
  limits?: [tseg1Min: number, tseg1Max: number, tseg2Min: number, tseg2Max: number, sjwMax: number, brpMin: number, brpMax: number, brpInc: number];
  /** The device starts unconfigured (configuration null). */
  unconfigured?: boolean;
  responder?: FakeBusResponder;
}

const CANDLELIGHT_LIMITS: [number, number, number, number, number, number, number, number] = [1, 16, 1, 8, 4, 1, 1024, 1];

const bytesOf = (data?: BufferSource): Uint8Array =>
  data === undefined
    ? new Uint8Array(0)
    : ArrayBuffer.isView(data)
      ? new Uint8Array(data.buffer, data.byteOffset, data.byteLength).slice()
      : new Uint8Array(data).slice();

const domError = (message: string, name: string) => new DOMException(message, name);

/** A fake gs_usb adapter. Its failure switches and records are public, for tests to script and check. */
export class FakeGsUsbDevice implements UsbDeviceLike {
  readonly vendorId: number;
  readonly productId: number;
  readonly serialNumber: string | null;
  readonly productName: string | null;
  readonly manufacturerName: string | null;
  readonly configurations = [{ configurationValue: 1 }];
  readonly fclk: number;
  readonly feature: number;
  readonly limits: number[];

  opened = false;
  configuration: { configurationValue: number } | null;
  claimed = false;
  /** MODE as the host last set it. */
  started = false;
  modeFlags = 0;
  /** The last BITTIMING payload as five words. */
  bitTiming: number[] | null = null;

  /** The CAN bus behind the adapter. */
  responder: FakeBusResponder | null;
  /** The bus acknowledges frames (false: nothing ACKs, so nothing is echoed or answered). */
  ack = true;

  /** Thrown by the next open(). */
  openError: Error | null = null;
  /** Thrown by claimInterface() (another program holds the interface). */
  claimError: Error | null = null;
  /** Control transfers with this request number throw this error, answer this status, or never complete. */
  controlFailure: { request: number; error?: Error; status?: Status; hang?: boolean } | null = null;
  /** Thrown by transferOut(). */
  writeError: Error | null = null;
  /** transferOut() answers this status instead of writing. */
  writeStatus: Status | null = null;
  /** transferOut() reports this many bytes written (a short write; the frame does not go out). */
  writeBytes: number | null = null;
  /** transferOut() does not complete until the device is closed, its interface released or it is unplugged. */
  hangWrites = false;
  /**
   * Bulk IN reads end after `seconds` when nothing arrives, like a libusb transfer with a timeout:
   * they reject with `error()` when given, else answer ok with no data. Null (default): WebUSB, no timeout.
   */
  readTimeout: { clock: Clock; seconds: number; error?: () => unknown } | null = null;
  /** close() throws while bulk IN reads are pending, as node-usb does. */
  refuseCloseWithPendingReads = false;

  /** Counts and records. */
  opens = 0;
  closes = 0;
  claims = 0;
  releases = 0;
  /** USB resets requested (a gs_usb host must never do one). */
  resets = 0;
  readonly controls: FakeControl[] = [];
  readonly writes: FakeWrite[] = [];
  /** Things a real candleLight would not accept or a careful host would not do. */
  readonly protocolErrors: string[] = [];

  #rx: Uint8Array[] = [];
  #reads: { resolve: (r: InResult) => void; reject: (e: unknown) => void }[] = [];
  #hung: { endpoint: number; bytes: Uint8Array; resolve: (r: OutResult) => void; reject: (e: unknown) => void }[] = [];
  #gone = false;

  constructor(options: FakeGsUsbDeviceOptions = {}) {
    this.serialNumber = options.serialNumber === undefined ? "FAKE0001" : options.serialNumber;
    this.productName = options.productName === undefined ? "candleLight USB to CAN adapter" : options.productName;
    this.manufacturerName = options.manufacturerName === undefined ? "bytewerk" : options.manufacturerName;
    this.vendorId = options.vendorId ?? 0x1d50;
    this.productId = options.productId ?? 0x606f;
    this.feature = options.feature ?? 0x13;
    this.fclk = options.fclk ?? 48_000_000;
    this.limits = [...(options.limits ?? CANDLELIGHT_LIMITS)];
    this.configuration = options.unconfigured ? null : { configurationValue: 1 };
    this.responder = options.responder ?? null;
  }

  /** Bulk IN transfers the host has queued and the device has not answered yet. */
  get pendingReads(): number {
    return this.#reads.length;
  }

  /** True once unplug() was called. */
  get unplugged(): boolean {
    return this.#gone;
  }

  async open(): Promise<void> {
    this.#check();
    if (this.openError) throw this.openError;
    if (!this.opened) this.opens++;
    this.opened = true;
  }

  async close(): Promise<void> {
    this.#check();
    if (!this.opened) return;
    if (this.refuseCloseWithPendingReads && this.#reads.length) throw new Error("close error: Can't close device with a pending request");
    this.opened = false;
    this.claimed = false;
    this.closes++;
    this.#abortReads(domError("The transfer was cancelled.", "AbortError"));
  }

  async reset(): Promise<void> {
    this.resets++;
    this.protocolErrors.push("USB reset");
  }

  async selectConfiguration(value: number): Promise<void> {
    this.#checkOpen();
    if (value !== 1) throw domError("The configuration value provided is not supported by the device.", "NotFoundError");
    this.configuration = { configurationValue: value };
  }

  async claimInterface(n: number): Promise<void> {
    this.#checkOpen();
    if (this.claimError) throw this.claimError;
    if (this.configuration === null) throw domError("The device must have a configuration selected.", "InvalidStateError");
    if (n !== INTERFACE) throw domError("The interface number provided is not supported by the device.", "NotFoundError");
    if (!this.claimed) this.claims++;
    this.claimed = true;
  }

  async releaseInterface(n: number): Promise<void> {
    this.#checkOpen();
    if (n !== INTERFACE || !this.claimed) throw domError("The interface is not claimed.", "InvalidStateError");
    this.claimed = false;
    this.releases++;
    // Releasing an interface cancels the transfers queued on its endpoints.
    this.#abortReads(domError("The transfer was cancelled.", "AbortError"));
  }

  async controlTransferIn(setup: UsbControlSetup, length: number): Promise<InResult> {
    this.#checkControl(setup);
    this.controls.push({ dir: "in", request: setup.request, value: setup.value, index: setup.index, data: length });
    const failed = await this.#controlFailed(setup.request);
    if (failed) return failed;
    if (setup.request !== BREQ_BT_CONST) return { status: "stall" };
    const out = new Uint8Array(40);
    const view = new DataView(out.buffer);
    [this.feature, this.fclk, ...this.limits].forEach((v, i) => view.setUint32(4 * i, v, true));
    return { status: "ok", data: new DataView(out.buffer, 0, Math.min(40, length)) };
  }

  async controlTransferOut(setup: UsbControlSetup, data?: BufferSource): Promise<OutResult> {
    this.#checkControl(setup);
    const bytes = bytesOf(data);
    this.controls.push({ dir: "out", request: setup.request, value: setup.value, index: setup.index, data: bytes });
    const failed = await this.#controlFailed(setup.request);
    if (failed) return { status: failed.status, bytesWritten: 0 };
    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    if (setup.request === BREQ_BITTIMING && bytes.length === 20) {
      this.bitTiming = [0, 1, 2, 3, 4].map((i) => view.getUint32(4 * i, true));
      return { status: "ok", bytesWritten: bytes.length };
    }
    if (setup.request === BREQ_MODE && bytes.length === 8) {
      const mode = view.getUint32(0, true);
      if (mode === MODE_START && !this.bitTiming) this.protocolErrors.push("MODE start before BITTIMING");
      this.started = mode === MODE_START;
      this.modeFlags = this.started ? view.getUint32(4, true) : 0;
      return { status: "ok", bytesWritten: bytes.length };
    }
    return { status: "stall", bytesWritten: 0 };
  }

  async transferOut(endpoint: number, data: BufferSource): Promise<OutResult> {
    this.#checkOpen();
    const bytes = bytesOf(data);
    if (!this.claimed) throw domError("The interface is not claimed.", "InvalidStateError");
    if (this.writeError) throw this.writeError;
    if (this.hangWrites || this.#hung.length) {
      // a stalled endpoint: this transfer queues behind the ones already stuck
      return new Promise<OutResult>((resolve, reject) => this.#hung.push({ endpoint, bytes, resolve, reject }));
    }
    return this.#write(endpoint, bytes);
  }

  /** Bulk OUT transfers stuck on a stalled endpoint (see hangWrites). */
  get pendingWrites(): number {
    return this.#hung.length;
  }

  /** The stalled endpoint moves again: stuck transfers complete in order, as normal writes. */
  releaseWrites(): void {
    this.hangWrites = false;
    const hung = this.#hung;
    this.#hung = [];
    for (const h of hung) h.resolve(this.#write(h.endpoint, h.bytes));
  }

  #write(endpoint: number, bytes: Uint8Array): OutResult {
    if (this.writeStatus) return { status: this.writeStatus, bytesWritten: 0 };
    if (this.writeBytes !== null) return { status: "ok", bytesWritten: this.writeBytes };
    if (endpoint !== EP_OUT) this.protocolErrors.push(`bulk OUT on endpoint ${endpoint}`);
    else if (!this.started) this.protocolErrors.push("frame before MODE start");
    else if (bytes.length !== FRAME_SIZE) this.protocolErrors.push(`${bytes.length}-byte frame, expected ${FRAME_SIZE}`);
    else {
      const view = new DataView(bytes.buffer);
      const echoId = view.getUint32(0, true);
      const ident = view.getUint32(4, true);
      const dlc = bytes[8]!;
      if (dlc > 8 || bytes[9] !== 0 || bytes[10] !== 0) this.protocolErrors.push(`frame with dlc ${dlc}, channel ${bytes[9]}, flags ${bytes[10]}`);
      const extended = (ident & CAN_EFF_FLAG) !== 0;
      const canId = extended ? ident & CAN_EFF_MASK : ident & 0x7ff;
      const frame = bytes.slice(12, 12 + Math.min(dlc, 8));
      this.writes.push({ echoId, ident, canId, extended, data: frame, raw: bytes });
      if (this.ack) {
        this.inject(this.#pack(echoId, ident, frame, 0, 0, dlc));
        for (const reply of this.responder?.(canId, frame, extended) ?? []) {
          this.receive(reply[0], reply[1], { extended: reply[2] ?? false });
        }
      }
    }
    return { status: "ok", bytesWritten: bytes.length };
  }

  transferIn(endpoint: number, _length: number): Promise<InResult> {
    try {
      this.#checkOpen();
      if (!this.claimed) throw domError("The interface is not claimed.", "InvalidStateError");
    } catch (e) {
      return Promise.reject(e);
    }
    if (endpoint !== EP_IN) return Promise.resolve({ status: "stall" });
    const queued = this.#rx.shift();
    if (queued) return Promise.resolve({ status: "ok", data: new DataView(queued.buffer) });
    return new Promise((resolve, reject) => {
      const t = this.readTimeout;
      if (!t) {
        this.#reads.push({ resolve, reject });
        return;
      }
      const entry = {
        resolve: (r: InResult) => (cancel(), resolve(r)),
        reject: (e: unknown) => (cancel(), reject(e)),
      };
      const cancel = t.clock.timer(t.seconds, () => {
        const i = this.#reads.indexOf(entry);
        if (i < 0) return;
        this.#reads.splice(i, 1);
        if (t.error) reject(t.error());
        else resolve({ status: "ok", data: new DataView(new ArrayBuffer(0)) });
      });
      this.#reads.push(entry);
    });
  }

  /**
   * A frame from the bus to the host, as candleLight packs it. Dropped while the channel is not
   * started, as on the adapter.
   */
  receive(
    canId: number,
    data: Uint8Array,
    extra: { extended?: boolean; echoId?: number; flags?: number; channel?: number; dlc?: number; ident?: number } = {},
  ): void {
    if (!this.started) return;
    const ident = extra.ident ?? (extra.extended ? (canId | CAN_EFF_FLAG) >>> 0 : canId);
    this.inject(this.#pack(extra.echoId ?? ECHO_RX, ident, data, extra.channel ?? 0, extra.flags ?? 0, extra.dlc ?? data.length));
  }

  /** Raw bytes as one bulk IN transfer (any length). */
  inject(bytes: Uint8Array): void {
    const read = this.#reads.shift();
    if (read) read.resolve({ status: "ok", data: new DataView(bytes.slice().buffer) });
    else this.#rx.push(bytes.slice());
  }

  /** The next queued bulk IN transfer fails with `error`, or answers `status`. */
  failRead(failure: Error | Status): void {
    const read = this.#reads.shift();
    if (!read) {
      this.protocolErrors.push("failRead() with no read queued");
      return;
    }
    if (typeof failure === "string") read.resolve({ status: failure });
    else read.reject(failure);
  }

  /** The adapter is pulled out: queued transfers fail and every later call throws. */
  unplug(): void {
    this.#gone = true;
    this.opened = false;
    this.claimed = false;
    this.started = false;
    this.#abortReads(domError("A transfer error has occurred.", "NetworkError"));
  }

  #pack(echoId: number, ident: number, data: Uint8Array, channel: number, flags: number, dlc: number): Uint8Array {
    const out = new Uint8Array(FRAME_SIZE);
    const view = new DataView(out.buffer);
    view.setUint32(0, echoId >>> 0, true);
    view.setUint32(4, ident >>> 0, true);
    out[8] = dlc;
    out[9] = channel;
    out[10] = flags;
    out.set(data.subarray(0, 8), 12);
    return out;
  }

  #controlFailed(request: number): Promise<{ status: Status } | null> {
    const f = this.controlFailure;
    if (!f || f.request !== request) return Promise.resolve(null);
    if (f.hang) return new Promise<never>(() => undefined);
    if (f.error) return Promise.reject(f.error);
    return Promise.resolve({ status: f.status ?? "stall" });
  }

  #abortReads(error: DOMException): void {
    const reads = this.#reads;
    this.#reads = [];
    this.#rx.length = 0;
    for (const r of reads) r.reject(error);
    const hung = this.#hung;
    this.#hung = [];
    for (const h of hung) h.reject(error);
  }

  #check(): void {
    if (this.#gone) throw domError("The device was disconnected.", "NotFoundError");
  }

  #checkOpen(): void {
    this.#check();
    if (!this.opened) throw domError("The device must be opened first.", "InvalidStateError");
  }

  #checkControl(setup: UsbControlSetup): void {
    this.#checkOpen();
    if (setup.requestType !== "vendor" || setup.recipient !== "interface" || setup.index !== INTERFACE) {
      this.protocolErrors.push(`control ${setup.requestType}/${setup.recipient} index ${setup.index}`);
    }
    if (!this.claimed) throw domError("The interface is not claimed.", "InvalidStateError");
  }
}
