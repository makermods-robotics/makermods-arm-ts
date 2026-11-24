// The SLCAN serial transport: the Python SDK's tests/unit/test_slcan_codec.py, the codec, lifecycle and
// pacing recorded from Python (contract/fixtures/slcan.json, tools/gen_slcan.py), and the browser-side
// behaviour (Web Serial streams, read failure, unplug, close during a send, reopen) on a fake port.
import { describe, expect, it, vi } from "vitest";
import { readFileSync } from "node:fs";
import { ConnectError, TimeoutError, ValueError } from "@makermods/arm-core";
import { VirtualClock } from "@makermods/arm-core/testing";
import { FakeSerialPort, type FakeSerialPortOptions } from "@makermods/arm-core/transport/fake-serial";
import type { SerialLike, SerialPortLike } from "@makermods/arm-core/transport/serial";
import {
  BITRATE_COMMANDS, grantedSlcanPorts, requestSlcanPort, SlcanFrameParser, SlcanSerialBackend, type SlcanSerialBackendOptions,
  slcanEncode,
} from "@makermods/arm-core/transport/slcan-serial";

const F = JSON.parse(readFileSync(new URL("../../../contract/fixtures/slcan.json", import.meta.url), "utf8"));

const enc = new TextEncoder();
const fromHex = (hex: string) => Uint8Array.from(hex.match(/../g) ?? [], (h) => parseInt(h, 16));
const toHex = (b: Uint8Array) => Array.from(b, (x) => x.toString(16).padStart(2, "0")).join("");
const text = (b: Uint8Array) => String.fromCharCode(...b);
/** Microtasks only: virtual time does not move. */
const flush = async (turns = 50) => {
  for (let i = 0; i < turns; i++) await Promise.resolve();
};
/** Event-loop turns: virtual time may move to the next timer. */
const settle = async (turns = 5) => {
  for (let i = 0; i < turns; i++) await new Promise<void>((r) => setImmediate(r));
};

type Received = [number, string, boolean];

function quietLogger() {
  return { warn: vi.fn(), error: vi.fn() };
}

function setup(portOptions: FakeSerialPortOptions = {}, options: SlcanSerialBackendOptions = {}) {
  const clock = new VirtualClock();
  const port = new FakeSerialPort({ clock, ...portOptions });
  const logger = quietLogger();
  const backend = new SlcanSerialBackend(port, { clock, logger, ...options });
  const received: Received[] = [];
  backend.setRecvCallback((canId, data, extended) => received.push([canId, toHex(data), extended]));
  return { clock, port, backend, logger, received };
}

/** Python kwargs -> options. */
function optionsFrom(kwargs: Record<string, number | boolean>): SlcanSerialBackendOptions {
  const out: SlcanSerialBackendOptions = {};
  if ("bitrate" in kwargs) out.bitrate = kwargs.bitrate as number;
  if ("baudrate" in kwargs) out.baudrate = kwargs.baudrate as number;
  if ("startup_delay" in kwargs) out.startupDelay = kwargs.startup_delay as number;
  if ("auto_retransmit" in kwargs) out.autoRetransmit = kwargs.auto_retransmit as boolean;
  if ("write_timeout" in kwargs) out.writeTimeout = kwargs.write_timeout as number;
  if ("tx_interval" in kwargs) out.txInterval = kwargs.tx_interval as number;
  return out;
}

describe("test_slcan_codec.py", () => {
  it("encodes extended and standard frames", () => {
    expect(text(slcanEncode(0x0400fd01, new Uint8Array(8)))).toBe("T0400FD0180000000000000000\r");
    expect(text(slcanEncode(0x123, Uint8Array.of(0xab, 0xcd), false))).toBe("t1232ABCD\r");
  });

  it("parser: fragmentation, ack, garbage and timestamp", () => {
    const parser = new SlcanFrameParser();
    const ts = slcanEncode(0x028001fd, fromHex("8000800080000159"));
    const timestamped = new Uint8Array([...ts.subarray(0, ts.length - 1), ...enc.encode("1234\r")]);
    const stream = new Uint8Array([...enc.encode("\rgarbage\r"), ...slcanEncode(0x0400fd01, new Uint8Array(8)), ...timestamped]);
    expect(parser.feed(stream.subarray(0, 19))).toEqual([]);
    expect(parser.feed(stream.subarray(19)).map(([id, d, x]) => [id, toHex(d), x])).toEqual([
      [0x0400fd01, "0000000000000000", true],
      [0x028001fd, "8000800080000159", true],
    ]);
  });

  it("parser recovers after malformed records", () => {
    const parser = new SlcanFrameParser();
    const stream = new Uint8Array([...enc.encode("T00000001900\rTZZZZZZZZ1AA\r"), ...slcanEncode(0x101, Uint8Array.of(1))]);
    expect(parser.feed(stream).map(([id, d, x]) => [id, toHex(d), x])).toEqual([[0x101, "01", true]]);
  });

  it("encode rejects an invalid id and payload", () => {
    expect(() => slcanEncode(0x20000000, new Uint8Array(0))).toThrow(ValueError);
    expect(() => slcanEncode(0x800, new Uint8Array(0), false)).toThrow(ValueError);
    expect(() => slcanEncode(1, new Uint8Array(9))).toThrow(ValueError);
  });

  for (const [autoRetransmit, command] of [[false, "C\rS8\rA0\rO\r"], [true, "C\rS8\rA1\rO\r"]] as const) {
    it(`configures the CANable retry mode (auto_retransmit ${autoRetransmit})`, async () => {
      const { port, backend } = setup({}, { startupDelay: 0, autoRetransmit });
      await backend.open();
      await backend.close();
      expect(port.baudRate).toBe(null);
      expect(port.opens).toBe(1);
      expect(port.writes.map((w) => w.text)).toEqual([command, "C\r"]);
    });
  }
});

describe("codec (golden: Python slcan_encode and SlcanFrameParser)", () => {
  it("encodes every payload length and id boundary, refusals with Python's messages", () => {
    expect(F.encode.length).toBeGreaterThan(300);
    for (const c of F.encode) {
      const call = () => text(slcanEncode(c.id, fromHex(c.data), c.extended));
      if (c.error) expect(call, JSON.stringify(c)).toThrow(c.error);
      else expect(call(), JSON.stringify(c)).toBe(c.out);
    }
  });

  it("parses every recorded stream chunk by chunk: frames, ids, payloads and the bytes held", () => {
    expect(F.parse.length).toBeGreaterThan(400);
    for (const c of F.parse) {
      const parser = new SlcanFrameParser();
      c.chunks.forEach((chunk: string, i: number) => {
        const frames = parser.feed(fromHex(chunk)).map(([id, d, x]) => [id, x, toHex(d)]);
        expect(frames, `${JSON.stringify(c.chunks)} feed ${i}`).toEqual(c.feeds[i].frames);
        expect(parser.buffered, `${JSON.stringify(c.chunks)} feed ${i}`).toBe(c.feeds[i].buffered);
      });
    }
  });

  it("bitrate commands, defaults and constructor refusals match", () => {
    expect(Object.fromEntries(Object.entries(BITRATE_COMMANDS))).toEqual(F.constants.bitrate_commands);
    const d = F.constants.defaults;
    const backend = new SlcanSerialBackend(new FakeSerialPort());
    expect([backend.baudrate, backend.bitrate, backend.startupDelay, backend.autoRetransmit, backend.txInterval, backend.writeTimeout])
      .toEqual([d.baudrate, d.bitrate, d.startup_delay, d.auto_retransmit, d.tx_interval, d.write_timeout]);
    expect([backend.feedbackPacedControl, backend.feedbackWaitTimeout]).toEqual([d.feedback_paced_control, d.feedback_wait_timeout]);
    for (const r of F.refusals) {
      if (r.kwargs) {
        expect(() => new SlcanSerialBackend(new FakeSerialPort(), optionsFrom(r.kwargs))).toThrow(ValueError);
        expect(() => new SlcanSerialBackend(new FakeSerialPort(), optionsFrom(r.kwargs))).toThrow(r.error);
      }
    }
  });
});

describe("lifecycle and pacing (golden: Python open, send, close)", () => {
  it("open, sends with gaps and close write Python's bytes at Python's times", async () => {
    for (const c of F.lifecycle) {
      const { clock, port, backend } = setup({}, optionsFrom(c.kwargs));
      await backend.open();
      expect(port.baudRate).toBe(c.serial_args[1]);
      expect(c.serial_kwargs.rtscts).toBe(false);
      await backend.send(0x0400fd01, new Uint8Array(8));
      await backend.send(0x123, Uint8Array.of(0xab, 0xcd), false);
      await clock.sleep(0.01);
      await backend.send(0x7ff, new Uint8Array(0), false);
      await clock.sleep(0.0005);
      await backend.send(0x1fffffff, Uint8Array.of(0, 1, 2, 3, 4, 5, 6, 7));
      await backend.close();
      expect(port.writes.map((w) => w.text), JSON.stringify(c.kwargs)).toEqual(c.writes.map((w: [number, string]) => w[1]));
      port.writes.forEach((w, i) => expect(Math.abs(w.at! - c.writes[i][0]), JSON.stringify(c.kwargs)).toBeLessThanOrEqual(1e-12));
      expect(backend.feedbackPacedControl).toBe(c.feedback_paced_control);
      expect(backend.feedbackWaitTimeout).toBe(c.feedback_wait_timeout);
      expect(port.protocolErrors).toEqual([]);
    }
  });

  it("each frame starts txInterval after the previous write ended, with a slow adapter", async () => {
    const { clock, port, backend } = setup({}, { startupDelay: 0 });
    await backend.open();
    port.writeDelay = F.pacing.write_cost;
    for (const s of F.pacing.sends) {
      if (s.gap) await clock.sleep(s.gap);
      expect(Math.abs(clock.now() - s.start)).toBeLessThanOrEqual(1e-12);
      await backend.send(0x10, Uint8Array.of(0), false);
      expect(Math.abs(port.writes.at(-1)!.at! - s.written_at)).toBeLessThanOrEqual(1e-12);
    }
    port.writeDelay = 0;
    await backend.close();
    for (const tx of [0.0, 0.004]) {
      const { port: p, backend: b } = setup({}, { startupDelay: 0, txInterval: tx });
      await b.open();
      for (let i = 0; i < 3; i++) await b.send(0x10, new Uint8Array(0), false);
      await b.close();
      const expected = F.pacing[tx ? "tx_interval_0.004" : "tx_interval_0.0"].writes as [number, string][];
      expect(p.writes.map((w) => [w.at, w.text])).toEqual(expected);
    }
  });
});

describe("open", () => {
  it("waits the startup delay, drops what arrived meanwhile, then opens the channel in one write", async () => {
    const { clock, port, backend, received } = setup();
    const opening = backend.open();
    await flush();
    expect(port.opened).toBe(true);
    expect(port.writes).toEqual([]);
    port.channelOpen = true; // an adapter left open by a previous program keeps sending
    port.receive(0x11, Uint8Array.of(1));
    port.inject(enc.encode("t01")); // half a record before the reset
    await opening;
    expect(clock.now()).toBe(2.5);
    expect(port.writes.map((w) => [w.at, w.text])).toEqual([[2.5, "C\rS8\rA0\rO\r"]]);
    expect([port.channelOpen, port.bitrateCommand, port.autoRetransmit]).toEqual([true, "S8", false]);
    port.receive(0x12, Uint8Array.of(2));
    await settle();
    expect(received).toEqual([[0x12, "02", false]]);
    expect(backend.isOpen).toBe(true);
    await backend.close();
    expect(port.protocolErrors).toEqual([]);
  });

  it("a port that cannot be opened is a ConnectError that says what to do", async () => {
    const { port, backend } = setup();
    port.openError = new DOMException("Failed to open serial port.", "NetworkError");
    const err = await backend.open().catch((e: unknown) => e);
    expect(err).toBeInstanceOf(ConnectError);
    expect(String(err)).toMatch(/cannot open usb:AD50:60C4: Failed to open serial port\. Close other programs/);
    expect(backend.isOpen).toBe(false);
    await expect(backend.send(1, new Uint8Array(0))).rejects.toThrow(ConnectError);
  });

  it("a failed open-command write closes the port again", async () => {
    const { port, backend } = setup({}, { startupDelay: 0 });
    port.writeError = new Error("write failed");
    await expect(backend.open()).rejects.toThrow(/cannot open the SLCAN channel on usb:AD50:60C4: write failed/);
    expect(port.opened).toBe(false);
    port.writeError = null;
    await backend.open();
    expect(port.channelOpen).toBe(true);
    await backend.close();
  });

  it("a read failure during the startup delay fails the open", async () => {
    const { port, backend } = setup();
    const opening = backend.open();
    await flush();
    port.failRead(new Error("framing error"));
    await expect(opening).rejects.toThrow(/serial read failed: framing error/);
    expect(port.opened).toBe(false);
    expect(port.writes).toEqual([]);
  });

  it("other bitrates and baud rates", async () => {
    const { port, backend } = setup({}, { bitrate: 125_000, baudrate: 2_000_000, startupDelay: 0 });
    await backend.open();
    expect(port.baudRate).toBe(2_000_000);
    expect(port.bitrateCommand).toBe("S4");
    await backend.close();
  });

  it("names the port by option, USB ids, or not at all", () => {
    expect(new SlcanSerialBackend(new FakeSerialPort()).port).toBe("usb:AD50:60C4");
    expect(new SlcanSerialBackend(new FakeSerialPort(), { name: "/dev/ttyACM0" }).port).toBe("/dev/ttyACM0");
    expect(new SlcanSerialBackend(new FakeSerialPort({ usbVendorId: null, usbProductId: null })).port).toBe("serial port");
  });
});

describe("send", () => {
  it("un-awaited sends go out strictly in call order, each txInterval after the previous", async () => {
    const { port, backend } = setup({ minFrameInterval: 0.0015 }, { startupDelay: 0 });
    await backend.open();
    const sends = Array.from({ length: 20 }, (_, i) => backend.send(i, Uint8Array.of(i), i % 2 === 0));
    await Promise.all(sends);
    expect(port.frames.map((f) => [f.canId, f.extended, toHex(f.data)])).toEqual(
      Array.from({ length: 20 }, (_, i) => [i, i % 2 === 0, toHex(Uint8Array.of(i))]),
    );
    for (let i = 1; i < port.frames.length; i++) {
      expect(port.frames[i]!.at! - port.frames[i - 1]!.at!).toBeGreaterThanOrEqual(0.0015 - 1e-12);
    }
    expect(port.protocolErrors).toEqual([]);
    await backend.close();
  });

  it("resolves once the bytes are written", async () => {
    const { clock, port, backend } = setup({}, { startupDelay: 0, txInterval: 0 });
    await backend.open();
    port.writeDelay = 0.01;
    const t0 = clock.now();
    await backend.send(0x10, new Uint8Array(0), false);
    expect(clock.now() - t0).toBeCloseTo(0.01, 12);
    expect(port.frames).toHaveLength(1);
    port.writeDelay = 0;
    await backend.close();
  });

  it("invalid frames are refused before anything is written; a closed backend says so", async () => {
    const { port, backend } = setup({}, { startupDelay: 0 });
    await expect(backend.send(1, new Uint8Array(0))).rejects.toThrow(/SLCAN backend is not open/);
    await backend.open();
    await expect(backend.send(0x800, new Uint8Array(0), false)).rejects.toThrow("standard CAN ID out of range: 0x800");
    await expect(backend.send(1, new Uint8Array(9))).rejects.toThrow("classic CAN payload is limited to 8 bytes, got 9");
    expect(port.frames).toEqual([]);
    await backend.close();
  });

  it("a failed write is a TimeoutError and the next send still goes out", async () => {
    const { port, backend } = setup({}, { startupDelay: 0 });
    await backend.open();
    port.writeError = new Error("I/O error");
    await expect(backend.send(1, new Uint8Array(0))).rejects.toThrow(TimeoutError);
    port.writeError = null;
    await backend.send(2, new Uint8Array(0));
    expect(port.frames.map((f) => f.canId)).toEqual([2]);
    port.writeError = new Error("I/O error");
    await expect(backend.send(3, new Uint8Array(0))).rejects.toThrow(TimeoutError);
    port.writeError = null;
    await backend.close(); // C still goes out on the port's new stream
    expect(port.commands.at(-1)).toBe("C");
    expect(backend.closeError).toBe(null);
  });

  it("a stalled write times out, marks the backend stalled, and later sends write nothing until it settles", async () => {
    const { clock, port, backend } = setup({}, { startupDelay: 0 });
    await backend.open();
    port.hangWrites = true;
    const t0 = clock.now();
    const first = backend.send(1, new Uint8Array(0)).catch((e: unknown) => e);
    const second = backend.send(2, new Uint8Array(0)).catch((e: unknown) => e);
    const third = backend.send(3, new Uint8Array(0)).catch((e: unknown) => e);
    const e1 = await first;
    expect(e1).toBeInstanceOf(TimeoutError);
    expect(String(e1)).toMatch(/serial write failed \(serial write timed out after 1 s\)/);
    expect(clock.now() - t0).toBeCloseTo(1.0015, 9);
    expect(backend.stalled).toMatch(/may still reach the bus if the adapter recovers/);
    const e2 = await second;
    expect(e2).toBeInstanceOf(TimeoutError);
    expect(String(e2)).toMatch(/frame not sent: the previous write has not completed/);
    expect(clock.now() - t0).toBeCloseTo(2.0015, 9); // waited its own writeTimeout for the stalled write
    expect(await third).toBeInstanceOf(TimeoutError);
    // Only the stalled frame was ever handed to the port.
    expect(port.writes.map((w) => w.text)).toEqual(["C\rS8\rA0\rO\r", "T000000010\r"]);
    await backend.close();
    expect(port.opened).toBe(false);
    expect(backend.closeError).toMatch(/C not sent on usb:AD50:60C4: a write was still pending after 1 s and was abandoned/);
    expect(backend.stalled).toBe(null);
    expect(port.frames).toEqual([]);
    port.hangWrites = false;
    await backend.open(); // a new session starts clean
    await backend.send(4, new Uint8Array(0));
    expect(port.frames.map((f) => f.canId)).toEqual([4]);
    await backend.close();
    expect(backend.closeError).toBe(null);
  });

  it("an adapter that recovers: no burst, no stale frames, the minimum gap holds", async () => {
    const { clock, port, backend } = setup({ minFrameInterval: 0.0015 }, { startupDelay: 0 });
    await backend.open();
    await backend.send(0x10, Uint8Array.of(1), false); // accepted before the stall
    port.stallFor(4.5);
    const stalled = backend.send(0x11, Uint8Array.of(2), false).catch((e: unknown) => e);
    const stale = [0x12, 0x13, 0x14].map((id) => backend.send(id, Uint8Array.of(3), false).catch((e: unknown) => e));
    expect(await stalled).toBeInstanceOf(TimeoutError);
    for (const r of await Promise.all(stale)) expect(String(r)).toMatch(/frame not sent/); // each waited 1 s
    const recovered = port.frames.find((f) => f.canId === 0x11);
    expect(recovered).toBeUndefined(); // still stalled: 0x12..0x14 timed out before the adapter recovered
    await clock.sleep(1.0);
    expect(backend.stalled).toBe(null);
    // The torque-off after recovery goes out at once (paced), not behind the stale frames.
    await backend.send(0x15, Uint8Array.of(0), false);
    await backend.send(0x16, Uint8Array.of(0), false);
    expect(port.frames.map((f) => f.canId)).toEqual([0x10, 0x11, 0x15, 0x16]);
    const [, f11, f15, f16] = port.frames;
    expect(f15!.at! - f11!.at!).toBeGreaterThanOrEqual(0.0015 - 1e-12);
    expect(f16!.at! - f15!.at!).toBeGreaterThanOrEqual(0.0015 - 1e-12);
    expect(port.protocolErrors).toEqual([]);
    await backend.close();
    expect(backend.closeError).toBe(null);
  });

  it("a send waiting on a stalled write goes out txInterval after the adapter recovers", async () => {
    const { clock, port, backend } = setup({ minFrameInterval: 0.0015 }, { startupDelay: 0 });
    await backend.open();
    port.hangWrites = true;
    const stalled = backend.send(1, new Uint8Array(0)).catch((e: unknown) => e);
    const waiting = backend.send(2, new Uint8Array(0));
    expect(await stalled).toBeInstanceOf(TimeoutError);
    await clock.sleep(0.3);
    port.releaseWrites();
    await waiting;
    expect(port.frames.map((f) => f.canId)).toEqual([1, 2]);
    expect(port.frames[1]!.at! - port.frames[0]!.at!).toBeCloseTo(0.0015, 12);
    expect(port.protocolErrors).toEqual([]);
    await backend.close();
  });

  it("close while a write is stalled waits writeTimeout for it; if it settles, C still goes out", async () => {
    const { port, backend } = setup({}, { startupDelay: 0 });
    await backend.open();
    port.stallFor(1.5);
    expect(await backend.send(1, new Uint8Array(0)).catch((e: unknown) => e)).toBeInstanceOf(TimeoutError);
    await backend.close(); // the stall ends 0.5 s into close's wait
    expect(backend.closeError).toBe(null);
    expect(port.commands.slice(-2)).toEqual(["T000000010", "C"]);
  });
});

describe("receive", () => {
  it("delivers frames from the bus with their id format, through the responder", async () => {
    const { port, backend, received } = setup({
      responder: (canId, data, extended) =>
        extended ? [[canId + 0x100, data, true]] : [[canId + 0x100, data], [canId, Uint8Array.of(9), true]],
    }, { startupDelay: 0 });
    await backend.open();
    await backend.send(0x01, Uint8Array.of(0xaa), false);
    await backend.send(0x1234567, new Uint8Array(0), true);
    await settle();
    expect(received).toEqual([[0x101, "aa", false], [0x01, "09", true], [0x1234667, "", true]]);
    expect(port.protocolErrors).toEqual([]);
    await backend.close();
  });

  it("reassembles records split across chunks, one byte or a few at a time, timestamps included", async () => {
    for (const chunkSize of [1, 3, 7]) {
      const { port, backend, received } = setup({ chunkSize }, { startupDelay: 0 });
      await backend.open();
      port.receive(0x10, Uint8Array.of(1, 2, 3));
      port.receive(0x1fffffff, Uint8Array.of(0xff), { extended: true, timestamp: 0x1234 });
      port.receive(0x7ff, new Uint8Array(0));
      await settle(20);
      expect(received, `chunk ${chunkSize}`).toEqual([[0x10, "010203", false], [0x1fffffff, "ff", true], [0x7ff, "", false]]);
      await backend.close();
    }
  });

  it("ignores acknowledgements and status replies; a BELL glued to the next record loses it, as in Python", async () => {
    const { port, backend, received } = setup({ acks: true }, { startupDelay: 0 });
    await backend.open();
    port.inject(enc.encode("\rz\rZ\rV1013\rN1234\rF00\r"));
    port.receive(0x10, Uint8Array.of(1));
    port.inject(Uint8Array.of(0x07));
    port.receive(0x11, Uint8Array.of(2)); // "\x07t11102": not a frame record
    port.receive(0x12, Uint8Array.of(3));
    await settle();
    expect(received).toEqual([[0x10, "01", false], [0x12, "03", false]]);
    expect(backend.error).toBe(null);
    await backend.close();
  });

  it("a callback that throws is logged and receiving continues", async () => {
    const { port, backend, logger } = setup({}, { startupDelay: 0 });
    const got: number[] = [];
    backend.setRecvCallback((canId) => {
      if (canId === 0x10) throw new Error("boom");
      got.push(canId);
    });
    await backend.open();
    port.receive(0x10, new Uint8Array(0));
    port.receive(0x11, new Uint8Array(0));
    await settle();
    expect(got).toEqual([0x11]);
    expect(logger.error).toHaveBeenCalledWith("SLCAN RX frame-processing exception (continuing to receive frames)", expect.any(Error));
    await backend.close();
  });

  it("a read failure stops receiving and is reported; sends still go out (a torque-off must)", async () => {
    const { port, backend, logger, received } = setup({}, { startupDelay: 0 });
    await backend.open();
    port.failRead(new Error("buffer overrun"));
    await settle();
    expect(backend.error).toBe("serial read failed: buffer overrun");
    expect(logger.error).toHaveBeenCalledWith(
      "usb:AD50:60C4: serial read failed: buffer overrun; receive stopped (unplug and replug the adapter)", expect.any(Error));
    await backend.send(0x10, Uint8Array.of(0), false);
    expect(port.frames.map((f) => f.canId)).toEqual([0x10]);
    port.receive(0x11, new Uint8Array(0));
    await settle();
    expect(received).toEqual([]);
    expect(backend.isOpen).toBe(true);
    await backend.close();
    expect(backend.closeError).toBe(null);
    expect(port.commands.at(-1)).toBe("C");
    expect(port.opened).toBe(false);
  });

  it("a stream that ends without an error stops receiving", async () => {
    const { port, backend } = setup({}, { startupDelay: 0 });
    await backend.open();
    port.endRead();
    await settle();
    expect(backend.error).toMatch(/stopped delivering data/);
    await backend.close();
  });

  it("unplugged: error set, a send fails on the write, close releases the port without throwing", async () => {
    const { port, backend } = setup({}, { startupDelay: 0 });
    await backend.open();
    port.unplug();
    await settle();
    expect(backend.error).toBe("serial read failed: The device has been lost");
    await expect(backend.send(1, new Uint8Array(0))).rejects.toThrow(TimeoutError);
    await backend.close();
    expect(port.opened).toBe(false);
    expect(backend.closeError).toMatch(/could not close the SLCAN channel/);
    await expect(backend.open()).rejects.toThrow(ConnectError);
    expect(backend.closeError).toBe(null);
  });
});

describe("close and session boundary", () => {
  it("writes C and closes the port; idempotent; nothing when never opened", async () => {
    const { port, backend } = setup({}, { startupDelay: 0 });
    await backend.close();
    expect(port.writes).toEqual([]);
    await backend.open();
    await backend.close();
    await backend.close();
    expect(port.writes.map((w) => w.text)).toEqual(["C\rS8\rA0\rO\r", "C\r"]);
    expect([port.opened, port.channelOpen, port.closes]).toEqual([false, false, 1]);
  });

  it("close during sends: the frame being written completes, the rest fail with ConnectError, C is last", async () => {
    const { port, backend } = setup({}, { startupDelay: 0 });
    await backend.open();
    port.writeDelay = 0.0005;
    const sends = [1, 2, 3, 4].map((id) => backend.send(id, new Uint8Array(0)).then(() => "ok", (e: unknown) => e));
    await flush();
    const closing = backend.close();
    const results = await Promise.all(sends);
    await closing;
    expect(results.filter((r) => r === "ok").length).toBeLessThanOrEqual(1);
    for (const r of results.filter((x) => x !== "ok")) expect(r).toBeInstanceOf(ConnectError);
    expect(port.commands.at(-1)).toBe("C");
    expect(port.frames.length).toBe(results.filter((r) => r === "ok").length);
  });

  it("a send waiting out the pacing interval never writes once close has begun", async () => {
    const { port, backend } = setup({}, { startupDelay: 0, txInterval: 0.05 });
    await backend.open();
    const pending = backend.send(1, new Uint8Array(0)).catch((e: unknown) => e);
    await flush();
    await backend.close();
    expect(await pending).toBeInstanceOf(ConnectError);
    expect(port.frames).toEqual([]);
  });

  it("reopens after close", async () => {
    const { port, backend, received } = setup({ responder: (id, d) => [[id + 1, d]] }, { startupDelay: 0 });
    await backend.open();
    await backend.send(0x10, new Uint8Array(0), false);
    await backend.close();
    await backend.open();
    await backend.send(0x20, new Uint8Array(0), false);
    await settle();
    expect(received).toEqual([[0x11, "", false], [0x21, "", false]]);
    expect(port.writes.map((w) => w.text)).toEqual(["C\rS8\rA0\rO\r", "t0100\r", "C\r", "C\rS8\rA0\rO\r", "t0200\r"]);
    expect(port.opens).toBe(2);
    await backend.close();
  });

  it("sends queued in one session never write in the next, even when it opens at once", async () => {
    const { port, backend } = setup({}, { startupDelay: 0 });
    await backend.open();
    port.hangWrites = true;
    const stuck = backend.send(1, new Uint8Array(0)).catch((e: unknown) => e);
    const queued = backend.send(2, new Uint8Array(0)).catch((e: unknown) => e);
    const closing = backend.close();
    port.hangWrites = false;
    const reopening = backend.open();
    await Promise.all([closing, reopening]);
    expect(await stuck).toBeInstanceOf(Error);
    expect(await queued).toBeInstanceOf(ConnectError);
    await backend.send(3, new Uint8Array(0));
    expect(port.frames.map((f) => f.canId)).toEqual([3]);
    await backend.close();
  });

  it("a send during open is refused and nothing is written before the channel is open", async () => {
    const { port, backend } = setup();
    const opening = backend.open();
    await flush();
    await expect(backend.send(1, new Uint8Array(0))).rejects.toThrow(ConnectError);
    await opening;
    expect(port.writes.map((w) => w.text)).toEqual(["C\rS8\rA0\rO\r"]);
    await backend.close();
  });

  it("a second open() waits for the first, and a close() during open closes after it", async () => {
    const { port, backend } = setup();
    const a = backend.open();
    const b = backend.open();
    const c = backend.close();
    await Promise.all([a, b, c]);
    expect(port.opens).toBe(1);
    expect(port.writes.map((w) => w.text)).toEqual(["C\rS8\rA0\rO\r", "C\r"]);
    expect(backend.isOpen).toBe(false);
  });
});

describe("browser helpers", () => {
  function fakeSerial(ports: SerialPortLike[], choose: SerialPortLike | null) {
    const calls: unknown[] = [];
    const serial: SerialLike = {
      requestPort: async (options) => {
        calls.push(options);
        if (!choose) throw new DOMException("No port selected by the user.", "NotFoundError");
        return choose;
      },
      getPorts: async () => ports,
    };
    return { serial, calls };
  }

  it("requestSlcanPort opens an unfiltered chooser; a dismissed chooser is null", async () => {
    const port = new FakeSerialPort();
    const { serial, calls } = fakeSerial([], port);
    expect(await requestSlcanPort(serial)).toBe(port);
    expect(calls).toEqual([undefined]);
    expect(await requestSlcanPort(fakeSerial([], null).serial)).toBe(null);
  });

  it("grantedSlcanPorts lists every granted port", async () => {
    const ports = [new FakeSerialPort(), new FakeSerialPort({ usbVendorId: 0x1a86, usbProductId: 0x7523 })];
    expect(await grantedSlcanPorts(fakeSerial(ports, null).serial)).toEqual(ports);
  });

  it("without Web Serial both helpers throw ConnectError", async () => {
    await expect(requestSlcanPort()).rejects.toThrow(ConnectError);
    await expect(grantedSlcanPorts()).rejects.toThrow(ConnectError);
  });
});
