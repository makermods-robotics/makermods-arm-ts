import { afterEach, describe, expect, it, vi } from "vitest";
import { readFileSync } from "node:fs";
import { ConnectError, TimeoutError, ValueError } from "@makermods/arm-core";
import { VirtualClock } from "@makermods/arm-core/testing";
import {
  ACCESS_HINT, BREQ_BITTIMING, BREQ_BT_CONST, BREQ_MODE, CAN_ERR_FLAG, ECHO_RX, FLAG_OVERFLOW, GS_USB_FILTERS,
  GsUsbBackend, type GsUsbBackendOptions, MAX_IN_FLIGHT, adapterPort, bitTiming, encodeFrame, grantedAdapters,
  parseBtConst, parsePort, requestAdapter, selectAdapter,
} from "@makermods/arm-core/transport/gs-usb";
import { FakeGsUsbDevice, type FakeGsUsbDeviceOptions } from "@makermods/arm-core/transport/fake-gs-usb";

/** Recorded from maker_arm/transport/gs_usb.py by tools/gen_gs_usb.py. */
const F = JSON.parse(readFileSync(new URL("../../../contract/fixtures/gs_usb.json", import.meta.url), "utf8"));

const hex = (b: Uint8Array) => Array.from(b, (x) => x.toString(16).toUpperCase().padStart(2, "0")).join("");
const fromHex = (s: string) => Uint8Array.from(s.match(/../g) ?? [], (x) => parseInt(x, 16));
const PY_HINT_START = F.selection.find((s: { error?: string }) => s.error?.startsWith("no gs_usb")).error.indexOf("Install pyusb");

function silent() {
  return { warn: vi.fn(), error: vi.fn() };
}

type Received = Array<[number, string, boolean]>;

/**
 * An open backend on a fake adapter. `manual`: time moves only in clock.advance(), for tests that
 * let the receive loop run (flush) while a send waits on a timeout.
 */
async function opened(options: FakeGsUsbDeviceOptions & { ack?: boolean; manual?: boolean } = {}, backendOptions: GsUsbBackendOptions = {}) {
  const clock = new VirtualClock({ auto: !options.manual });
  const dev = new FakeGsUsbDevice({ serialNumber: "ABC123", ...options });
  if (options.ack === false) dev.ack = false;
  const logger = silent();
  const backend = new GsUsbBackend(dev, { clock, logger, ...backendOptions });
  const received: Received = [];
  backend.setRecvCallback((id, data, extended) => received.push([id, hex(data), extended]));
  await backend.open();
  return { clock, dev, backend, received, logger };
}

/** Lets the receive loop process everything the fake has queued (microtasks and one macrotask). */
const flush = () => new Promise<void>((resolve) => setImmediate(resolve));

/** The fixture's ["in"/"out", request, value, index, data, timeout] control calls, as the fake records them. */
const pyControls = (log: unknown[][]) =>
  log.filter((c) => c[0] === "in" || c[0] === "out").map(([dir, request, value, index, data]) => [dir, request, value, index, data]);
const fakeControls = (dev: FakeGsUsbDevice) =>
  dev.controls.map((c) => [c.dir, c.request, c.value, c.index, typeof c.data === "number" ? c.data : hex(c.data)]);

describe("bit timing (golden: Python bit_timing)", () => {
  it("matches every clock, bitrate and BT_CONST limit set Python was asked, refusals included", () => {
    for (const c of F.bit_timing) {
      const limits = F.limits[c.limits] as number[];
      const label = `${c.limits} ${c.fclk}/${c.bitrate}`;
      if (c.error) {
        expect(() => bitTiming(c.fclk, c.bitrate, ...(limits as [number, number, number, number, number, number, number, number])), label).toThrow(
          new ConnectError(c.error),
        );
      } else {
        expect(bitTiming(c.fclk, c.bitrate, ...(limits as [number, number, number, number, number, number, number, number])), label).toEqual(c.timing);
      }
    }
    expect(F.bit_timing.filter((c: { timing?: unknown }) => c.timing).length).toBeGreaterThan(300);
  });

  it("the Python unit test's vectors: candleLight 48 MHz, 80 MHz at 87.5 %, and a refusal", () => {
    expect(bitTiming(48_000_000, 1_000_000, 1, 16, 1, 8, 4, 1, 1024, 1)).toEqual([1, 12, 2, 1, 3]);
    expect(bitTiming(48_000_000, 500_000, 1, 16, 1, 8, 4, 1, 1024, 1)).toEqual([1, 12, 2, 1, 6]);
    const [prop, seg1, seg2, , brp] = bitTiming(80_000_000, 1_000_000, 1, 16, 1, 8, 4, 1, 1024, 1);
    const quanta = 1 + prop + seg1 + seg2;
    expect(Math.floor(80_000_000 / (brp * quanta))).toBe(1_000_000);
    expect((1 + prop + seg1) / quanta).toBe(0.875);
    expect(() => bitTiming(48_000_000, 1_000_000, 1, 4, 1, 2, 1, 1, 1, 1)).toThrow(ConnectError);
  });

  it("BT_CONST parses as <10I", () => {
    const bytes = new Uint8Array(40);
    const view = new DataView(bytes.buffer);
    [0x1f, 48e6, 1, 16, 1, 8, 4, 1, 1024, 1].forEach((v, i) => view.setUint32(4 * i, v, true));
    expect(parseBtConst(bytes)).toEqual({
      feature: 0x1f, fclk: 48e6, tseg1Min: 1, tseg1Max: 16, tseg2Min: 1, tseg2Max: 8, sjwMax: 4, brpMin: 1, brpMax: 1024, brpInc: 1,
    });
    expect(() => parseBtConst(bytes.slice(0, 12))).toThrow("expected 40");
  });
});

describe("ports and selection (golden: parse_port, GsUsbBackend._select)", () => {
  it("parsePort", () => {
    for (const c of F.parse_port) {
      if (c.error) expect(() => parsePort(c.port), String(c.port)).toThrow(new ValueError(c.error));
      else expect(parsePort(c.port), String(c.port)).toBe(c.serial);
    }
    expect(parsePort(undefined)).toBeNull();
  });

  it("selectAdapter: exactly one without a serial, else the matching serial, with Python's messages", () => {
    for (const c of F.selection) {
      const devices = c.serials.map((s: string) => new FakeGsUsbDevice({ serialNumber: s }));
      const label = `${c.serials} ${c.port}`;
      if (c.error) {
        const expected = c.error.startsWith("no gs_usb") ? c.error.slice(0, PY_HINT_START) + ACCESS_HINT : c.error;
        expect(() => selectAdapter(devices, c.port), label).toThrow(new ConnectError(expected));
      } else {
        expect(selectAdapter(devices, c.port), label).toBe(devices[c.chosen]);
      }
    }
  });

  it("selectAdapter ignores devices that are not gs_usb adapters", () => {
    const other = new FakeGsUsbDevice({ serialNumber: "B2", vendorId: 0x2341, productId: 0x0043 });
    const adapter = new FakeGsUsbDevice({ serialNumber: "A1", vendorId: 0x1209, productId: 0x2323 });
    expect(selectAdapter([other, adapter])).toBe(adapter);
    expect(() => selectAdapter([other], "B2")).toThrow("found 0 adapter(s)");
  });

  it("adapterPort and the backend's port name the USB serial", () => {
    expect(adapterPort({ serialNumber: "207A37AA45465006" })).toBe("gs_usb:207A37AA45465006");
    expect(adapterPort({ serialNumber: null })).toBe("gs_usb");
    expect(new GsUsbBackend(new FakeGsUsbDevice({ serialNumber: "XYZ" })).port).toBe("gs_usb:XYZ");
  });

  it("the filters are the Linux driver's vendor/product pairs", () => {
    expect(GS_USB_FILTERS.map((f) => [f.vendorId, f.productId]).sort()).toEqual([...F.constants.ids].sort());
    expect(F.constants.MAX_IN_FLIGHT).toBe(MAX_IN_FLIGHT);
  });
});

describe("open", () => {
  it("claims, reads BT_CONST, sets bit timing and starts the channel exactly as Python, without a USB reset", async () => {
    const { dev, backend } = await opened();
    expect(fakeControls(dev)).toEqual(pyControls(F.lifecycle.open));
    expect(F.lifecycle.open[0]).toEqual(["claim", 0]);
    expect(dev.claimed && dev.started).toBe(true);
    expect(dev.bitTiming).toEqual([1, 12, 2, 1, 3]);
    expect(dev.modeFlags).toBe(0);
    expect(dev.resets).toBe(0);
    expect(dev.protocolErrors).toEqual([]);
    await backend.close();
  });

  it("other bitrates, clocks and timeouts send Python's payloads", async () => {
    for (const [key, opts, bopts] of [
      ["bitrate_500k_open", {}, { bitrate: 500_000 }],
      ["fclk_80mhz_open", { fclk: 80_000_000 }, {}],
      ["io_timeout_0_25_open", {}, { ioTimeout: 0.25 }],
    ] as const) {
      const { dev, backend } = await opened(opts, bopts);
      await backend.close();
      expect(fakeControls(dev), key).toEqual(pyControls(F.lifecycle[key]));
    }
  });

  it("an unconfigured device gets its first configuration", async () => {
    const { dev, backend } = await opened({ unconfigured: true });
    expect(dev.configuration).toEqual({ configurationValue: 1 });
    await backend.close();
  });

  it("a failed transfer undoes the open and explains what to do", async () => {
    const clock = new VirtualClock();
    const dev = new FakeGsUsbDevice();
    dev.controlFailure = { request: BREQ_BT_CONST, error: new Error("Pipe error") };
    const backend = new GsUsbBackend(dev, { clock, logger: silent() });
    const err = await backend.open().catch((e: unknown) => e);
    expect(err).toBeInstanceOf(ConnectError);
    expect((err as Error).message).toMatch(/^cannot open gs_usb:FAKE0001: Pipe error\. Close other CAN software; unplug and replug/);
    expect(dev.claimed || dev.opened).toBe(false);
    expect([dev.claims, dev.releases, dev.closes, dev.resets]).toEqual([1, 1, 1, 0]);
    expect(dev.controls.map((c) => c.request)).toEqual([BREQ_BT_CONST]); // never started, so never stopped
    await expect(backend.send(1, new Uint8Array(8), false)).rejects.toThrow(ConnectError);
  });

  it("a stalled BITTIMING fails the open; a failed MODE start is undone with MODE reset", async () => {
    const clock = new VirtualClock();
    const dev = new FakeGsUsbDevice();
    dev.controlFailure = { request: BREQ_BITTIMING, status: "stall" };
    await expect(new GsUsbBackend(dev, { clock }).open()).rejects.toThrow("BITTIMING failed (stall)");
    expect(dev.controls.filter((c) => c.request === BREQ_MODE)).toEqual([]);

    const dev2 = new FakeGsUsbDevice();
    let modes = 0;
    const original = dev2.controlTransferOut.bind(dev2);
    dev2.controlTransferOut = async (setup, data) => {
      if (setup.request === BREQ_MODE && modes++ === 0) throw new Error("timeout");
      return original(setup, data);
    };
    await expect(new GsUsbBackend(dev2, { clock }).open()).rejects.toThrow("cannot open");
    expect(dev2.controls.filter((c) => c.request === BREQ_MODE).map((c) => hex(c.data as Uint8Array))).toEqual(["0000000000000000"]);
    expect(dev2.opened).toBe(false);
  });

  it("every control transfer is bounded by ioTimeout", async () => {
    const clock = new VirtualClock();
    const dev = new FakeGsUsbDevice();
    dev.controlFailure = { request: BREQ_BT_CONST, hang: true };
    const err = await new GsUsbBackend(dev, { clock, ioTimeout: 0.1 }).open().catch((e: unknown) => e);
    expect((err as Error).message).toContain("BT_CONST timed out after 100 ms");
    expect(clock.now()).toBeCloseTo(0.1, 12);
    expect(dev.opened).toBe(false);
  });

  it("a claim refused by another program says so", async () => {
    const dev = new FakeGsUsbDevice();
    dev.claimError = new DOMException("Unable to claim interface.", "NetworkError");
    const err = await new GsUsbBackend(dev, { clock: new VirtualClock() }).open().catch((e: unknown) => e);
    expect(err).toBeInstanceOf(ConnectError);
    expect((err as Error).message).toContain("Unable to claim interface.");
    expect((err as Error).message).toContain("another program or tab is using it");
    expect((err as Error).message).toContain("kernel gs_usb driver");
    expect(dev.opened).toBe(false);
  });

  it("a clock that cannot make the bitrate fails with Python's message and never starts", async () => {
    const dev = new FakeGsUsbDevice({ fclk: 7_500_000 });
    await expect(new GsUsbBackend(dev, { clock: new VirtualClock() }).open()).rejects.toThrow(
      new ConnectError("adapter clock 7500000 Hz cannot produce 1000000 bit/s"),
    );
    expect(dev.controls.map((c) => c.request)).toEqual([BREQ_BT_CONST]);
    expect(dev.opened || dev.claimed).toBe(false);
  });

  it("a device that cannot be opened is a ConnectError", async () => {
    const dev = new FakeGsUsbDevice();
    dev.openError = new DOMException("Access denied.", "SecurityError");
    await expect(new GsUsbBackend(dev, { clock: new VirtualClock() }).open()).rejects.toThrow(/cannot open gs_usb:FAKE0001: Access denied\./);
  });
});

describe("send", () => {
  it("lays out standard and extended frames byte for byte as Python", async () => {
    const { dev, backend } = await opened();
    for (const s of F.lifecycle.sends) await backend.send(s.id, fromHex(s.data), s.extended);
    expect(dev.writes.map((w) => hex(w.raw))).toEqual(F.lifecycle.sends.map((s: { bytes: string }) => s.bytes));
    expect(dev.protocolErrors).toEqual([]);
    await expect(backend.send(1, new Uint8Array(9), false)).rejects.toThrow(new ValueError("classic CAN carries at most 8 data bytes"));
    expect(hex(encodeFrame(ECHO_RX, 0x7ff, Uint8Array.of(1)))).toBe("FFFFFFFFFF0700000100000001000000" + "00000000");
    await backend.close();
  });

  it("echo ids count up from 0 and wrap at 0x10000", async () => {
    const { dev, backend } = await opened();
    for (let i = 0; i < 3; i++) await backend.send(1, new Uint8Array(0), false);
    expect(dev.writes.map((w) => w.echoId)).toEqual([0, 1, 2]);
    await backend.close();
  });

  it("frames never acknowledged stop further writes after ioTimeout, with Python's message", async () => {
    const { clock, dev, backend } = await opened({ ack: false }, { ioTimeout: 0.02 });
    for (let i = 0; i < MAX_IN_FLIGHT; i++) await backend.send(1, new Uint8Array(8), false);
    const start = clock.now();
    await expect(backend.send(1, new Uint8Array(8), false)).rejects.toThrow(
      new TimeoutError(F.never_acknowledged.message.replace("gs_usb:ABC123", backend.port)),
    );
    expect(clock.now() - start).toBeCloseTo(0.02, 12);
    expect(dev.writes.length).toBe(F.never_acknowledged.writes);
    expect(backend.inFlight).toBe(MAX_IN_FLIGHT);
    await backend.close();
  });

  it("echoes free the in-flight budget", async () => {
    const { dev, backend } = await opened({}, { ioTimeout: 0.05 });
    for (let i = 0; i < 5 * MAX_IN_FLIGHT; i++) await backend.send(1, new Uint8Array(8), false);
    expect(dev.writes.length).toBe(5 * MAX_IN_FLIGHT);
    await flush();
    expect(backend.inFlight).toBe(0);
    await backend.close();
  });

  it("a send blocked on the cap goes out as soon as an echo frees a slot", async () => {
    const { clock, dev, backend } = await opened({ ack: false, manual: true });
    for (let i = 0; i < MAX_IN_FLIGHT; i++) await backend.send(1, new Uint8Array(8), false);
    const blocked = backend.send(2, new Uint8Array(8), false);
    await clock.advance(0.05);
    expect(dev.writes.length).toBe(MAX_IN_FLIGHT);
    dev.receive(0x001, new Uint8Array(8), { echoId: 0 });
    await blocked;
    expect(dev.writes.at(-1)!.canId).toBe(2);
    expect(clock.now()).toBeLessThan(0.1);
    await backend.close();
  });

  it("un-awaited sends go out strictly in call order, and the cap holds across them", async () => {
    const { clock, dev, backend } = await opened({}, { ioTimeout: 1 });
    const transferOut = dev.transferOut.bind(dev);
    let n = 0;
    dev.transferOut = async (ep, data) => {
      await clock.sleep(0.001 * (5 - (n++ % 5))); // later writes would finish first if they overlapped
      return transferOut(ep, data);
    };
    const sends = Array.from({ length: 30 }, (_, i) => backend.send(i, Uint8Array.of(i), false));
    await Promise.all(sends);
    expect(dev.writes.map((w) => w.canId)).toEqual([...Array(30).keys()]);

    dev.ack = false;
    const results = await Promise.allSettled(Array.from({ length: 12 }, (_, i) => backend.send(0x100 + i, new Uint8Array(1), false)));
    await flush();
    expect(results.map((r) => r.status)).toEqual([...Array(10).fill("fulfilled"), "rejected", "rejected"]);
    expect((results[10] as PromiseRejectedResult).reason).toBeInstanceOf(TimeoutError);
    expect(dev.writes.slice(30).map((w) => w.canId)).toEqual([...Array(10).keys()].map((i) => 0x100 + i));
    await backend.close();
  });

  it("a failed USB write is a clear error and does not hold an echo slot", async () => {
    const { dev, backend } = await opened({ ack: false });
    dev.writeError = new DOMException("A transfer error has occurred.", "NetworkError");
    await expect(backend.send(1, new Uint8Array(8), false)).rejects.toThrow(
      new TimeoutError("gs_usb:ABC123: USB write failed (A transfer error has occurred.); the adapter may need unplugging and replugging"),
    );
    expect(backend.inFlight).toBe(0);
    dev.writeError = null;
    for (let i = 0; i < MAX_IN_FLIGHT; i++) await backend.send(1, new Uint8Array(8), false); // all ten slots still free
    await backend.close();
  });

  it("a stalled or short write throws and frees its slot", async () => {
    const { dev, backend } = await opened();
    dev.writeStatus = "stall";
    await expect(backend.send(1, new Uint8Array(8), false)).rejects.toThrow("USB write failed (stall)");
    dev.writeStatus = null;
    dev.writeBytes = 10;
    await expect(backend.send(1, new Uint8Array(8), false)).rejects.toThrow(
      new TimeoutError("gs_usb:ABC123: incomplete USB write (10/20 bytes)"),
    );
    expect(backend.inFlight).toBe(0);
    dev.writeBytes = null;
    await backend.send(1, new Uint8Array(8), false);
    await backend.close();
  });

  it("a write the adapter never completes fails after ioTimeout but keeps its slot while the transfer is queued", async () => {
    const { clock, dev, backend } = await opened({}, { ioTimeout: 0.1 });
    dev.hangWrites = true;
    const start = clock.now();
    await expect(backend.send(1, new Uint8Array(8), false)).rejects.toThrow("USB write failed (USB write timed out after 100 ms)");
    expect(clock.now() - start).toBeCloseTo(0.1, 12);
    expect(backend.inFlight).toBe(1);
    expect(dev.pendingWrites).toBe(1);
    // the transfer completes late: it goes out, its echo frees the slot
    dev.releaseWrites();
    await flush();
    expect(dev.writes.length).toBe(1);
    expect(backend.inFlight).toBe(0);
    await backend.send(2, new Uint8Array(8), false);
    await backend.close();
  });

  it("a stalled adapter never holds more than ten transfers: timed-out writes keep their slots until the cap refuses", async () => {
    const { dev, backend } = await opened({}, { ioTimeout: 0.1 });
    dev.hangWrites = true;
    await expect(backend.send(1, new Uint8Array(8), false)).rejects.toThrow("timed out");
    const results = [];
    for (let i = 0; i < 12; i++) results.push(await backend.send(1, new Uint8Array(8), false).catch((e: unknown) => e));
    expect(dev.pendingWrites).toBe(MAX_IN_FLIGHT);
    expect(backend.inFlight).toBe(MAX_IN_FLIGHT);
    expect(results.slice(0, 9).every((e) => (e as Error).message.includes("USB write failed (USB write timed out"))).toBe(true);
    for (const e of results.slice(9)) {
      expect(e).toBeInstanceOf(TimeoutError);
      expect((e as Error).message).toContain("10 frames were never acknowledged on the CAN bus");
    }
    // the stuck transfers end in an error (the device is closed): their slots are freed
    await backend.close();
    expect(dev.pendingWrites).toBe(0);
    expect(backend.inFlight).toBe(0);
  });

  it("a stuck transfer that later fails frees its slot", async () => {
    const { dev, backend } = await opened({ manual: true }, { ioTimeout: 0.1 });
    dev.hangWrites = true;
    const sending = backend.send(1, new Uint8Array(8), false).catch((e: unknown) => e);
    await (backend.clock as VirtualClock).advance(0.1);
    expect(await sending).toBeInstanceOf(TimeoutError);
    expect(backend.inFlight).toBe(1);
    dev.writeStatus = "stall";
    dev.releaseWrites();
    await flush();
    expect(backend.inFlight).toBe(0);
    await backend.close();
  });
});

describe("receive", () => {
  it("delivers, drops and counts every case exactly as Python's _handle", async () => {
    for (const r of F.receives) {
      const { dev, backend, received } = await opened();
      dev.inject(fromHex(r.bytes));
      await flush();
      const label = r.case;
      expect(received.map(([id, data, ext]) => [id, ext, data]), label).toEqual(r.received);
      expect([backend.errorFrames, backend.rxOverflows], label).toEqual([r.error_frames, r.rx_overflows]);
      await backend.close();
    }
  });

  it("the Python unit test's mixed burst: frames delivered in order, echoes, errors, other channels and FD dropped", async () => {
    const { dev, backend, received } = await opened();
    dev.receive(0x17, Uint8Array.from([0, 1, 2, 3, 4, 5, 6, 7]));
    dev.receive(0x1abcdef, Uint8Array.of(0xaa), { extended: true });
    dev.receive(0x11, new Uint8Array(8), { echoId: 3 });
    dev.receive(0, new Uint8Array(8), { ident: CAN_ERR_FLAG | 4 });
    dev.receive(0x12, new Uint8Array(8), { channel: 1 });
    dev.receive(0x13, new Uint8Array(8), { flags: 0x02 });
    dev.receive(0x14, Uint8Array.of(1), { flags: FLAG_OVERFLOW });
    await flush();
    expect(received).toEqual([
      [0x17, "0001020304050607", false],
      [0x1abcdef, "AA", true],
      [0x14, "01", false],
    ]);
    expect([backend.errorFrames, backend.rxOverflows]).toEqual([1, 1]);
    await backend.close();
  });

  it("the bus behind the fake answers through the callback", async () => {
    const { dev, backend, received } = await opened();
    dev.responder = (id, data, extended) => (extended ? [[id, data, true]] : [[0x580 + id, Uint8Array.of(data[0]! + 1)]]);
    await backend.send(0x05, Uint8Array.of(7), false);
    await backend.send(0x1234, Uint8Array.of(1, 2), true);
    await flush();
    expect(received).toEqual([
      [0x585, "08", false],
      [0x1234, "0102", true],
    ]);
    await backend.close();
  });

  it("a callback that throws is logged and receiving continues", async () => {
    const { dev, backend, logger } = await opened();
    const got: number[] = [];
    backend.setRecvCallback((id) => {
      if (id === 1) throw new Error("bad parse");
      got.push(id);
    });
    dev.receive(1, new Uint8Array(1));
    dev.receive(2, new Uint8Array(1));
    await flush();
    expect(got).toEqual([2]);
    expect(logger.error).toHaveBeenCalledOnce();
    expect(backend.error).toBeNull();
    await backend.close();
  });

  it("keeps several bulk IN transfers queued", async () => {
    const { dev, backend } = await opened();
    await flush();
    expect(dev.pendingReads).toBe(4);
    await backend.close();
    expect(dev.pendingReads).toBe(0);
  });

  it("a read failure stops receiving and is reported; sends still go out (a torque-off must) until the cap, which names it", async () => {
    const { dev, backend, logger } = await opened();
    await flush();
    dev.failRead(new Error("No such device"));
    await flush();
    expect(backend.error).toBe("USB read failed: No such device");
    expect(logger.error).toHaveBeenCalledWith("gs_usb:ABC123: USB read failed: No such device; receive stopped (unplug and replug the adapter)", expect.any(Error));
    for (let i = 0; i < MAX_IN_FLIGHT; i++) await backend.send(0x10 + i, Uint8Array.of(0xfd), false);
    expect(dev.writes.map((w) => w.canId)).toEqual([...Array(10).keys()].map((i) => 0x10 + i));
    const err = await backend.send(1, new Uint8Array(8), false).catch((e: unknown) => e);
    expect(err).toBeInstanceOf(TimeoutError);
    expect((err as Error).message).toMatch(/10 frames were never acknowledged .* \(receive had stopped: USB read failed: No such device\)$/);
    expect(dev.writes.length).toBe(MAX_IN_FLIGHT);
    await backend.close();
    await expect(backend.send(1, new Uint8Array(8), false)).rejects.toThrow(new ConnectError("gs_usb:ABC123 is not open (USB read failed: No such device)"));
  });

  it("a stalled read stops receiving", async () => {
    const { dev, backend } = await opened();
    await flush();
    dev.failRead("stall");
    await flush();
    expect(backend.error).toBe("USB read failed (stall)");
    await backend.close();
  });

  it("unplugged while receiving: error set, a send fails on the write, close still releases without throwing", async () => {
    const { dev, backend, logger } = await opened();
    await flush();
    dev.unplug();
    await flush();
    expect(backend.error).toBe("USB read failed: A transfer error has occurred.");
    await expect(backend.send(1, new Uint8Array(8), false)).rejects.toThrow(
      new TimeoutError("gs_usb:ABC123: USB write failed (The device was disconnected.); the adapter may need unplugging and replugging"),
    );
    expect(backend.inFlight).toBe(0);
    await backend.close();
    expect(logger.warn).toHaveBeenCalledWith(expect.stringContaining("gs_usb channel stop failed on gs_usb:ABC123"), expect.anything());
    expect(backend.closeError).toMatch(/^gs_usb channel stop failed on gs_usb:ABC123: /);
    expect(dev.resets).toBe(0);
  });

  it("unplugged during a send blocked on the cap: the send fails at once with the cap error naming the reason", async () => {
    const { clock, dev, backend } = await opened({ ack: false, manual: true }, { ioTimeout: 0.1 });
    for (let i = 0; i < MAX_IN_FLIGHT; i++) await backend.send(1, new Uint8Array(8), false);
    const blocked = backend.send(1, new Uint8Array(8), false).catch((e: unknown) => e);
    await clock.advance(0.05);
    dev.unplug();
    const err = await blocked;
    expect(err).toBeInstanceOf(TimeoutError);
    expect((err as Error).message).toContain("A transfer error has occurred.");
    expect(clock.now()).toBeLessThan(0.1);
    await backend.close();
  });

  it("unplugged during a write in progress: the write fails and frees its slot", async () => {
    const { clock, dev, backend } = await opened({ manual: true });
    dev.hangWrites = true;
    const sending = backend.send(1, new Uint8Array(8), false).catch((e: unknown) => e);
    await clock.advance(0.05);
    dev.unplug();
    const err = await sending;
    expect(err).toBeInstanceOf(TimeoutError);
    expect((err as Error).message).toContain("USB write failed (A transfer error has occurred.)");
    expect(backend.inFlight).toBe(0);
    await backend.close();
  });
});

describe("close", () => {
  it("stops the channel, releases and closes, never resets; idempotent", async () => {
    const { dev, backend } = await opened();
    await backend.close();
    expect(fakeControls(dev).slice(-1)).toEqual(pyControls(F.lifecycle.close));
    expect(F.lifecycle.close.slice(1)).toEqual([["release", 0], ["dispose"]]);
    expect([dev.started, dev.claimed, dev.opened]).toEqual([false, false, false]);
    expect([dev.releases, dev.closes, dev.resets]).toEqual([1, 1, 0]);
    const controls = dev.controls.length;
    await backend.close();
    await Promise.all([backend.close(), backend.close()]);
    expect(dev.controls.length).toBe(controls);
    await expect(backend.send(1, new Uint8Array(8), false)).rejects.toThrow(new ConnectError("gs_usb:ABC123 is not open"));
  });

  it("a failed channel stop is logged and recorded in closeError, never thrown; open() clears it", async () => {
    const { dev, backend, logger } = await opened();
    expect(backend.closeError).toBeNull();
    dev.controlFailure = { request: BREQ_MODE, error: new Error("Pipe error") };
    await backend.close();
    expect(logger.warn).toHaveBeenCalledOnce();
    expect(backend.closeError).toBe("gs_usb channel stop failed on gs_usb:ABC123: Pipe error");
    expect(dev.opened || dev.claimed).toBe(false);
    dev.controlFailure = null;
    await backend.open();
    expect(backend.closeError).toBeNull();
    await backend.close();
    expect(backend.closeError).toBeNull();
  });

  it("a device close that fails while the device stays open is recorded", async () => {
    const { dev, backend } = await opened();
    dev.close = async () => {
      throw new Error("close error: busy");
    };
    await backend.close();
    expect(backend.closeError).toBe("gs_usb device close failed on gs_usb:ABC123: close error: busy");
  });

  it("a hung channel stop is bounded by ioTimeout", async () => {
    const { clock, dev, backend, logger } = await opened();
    dev.controlFailure = { request: BREQ_MODE, hang: true };
    const start = clock.now();
    await backend.close();
    expect(clock.now() - start).toBeCloseTo(0.1, 12);
    expect(logger.warn).toHaveBeenCalledOnce();
    expect(dev.opened).toBe(false);
  });

  it("close during a blocked send: the send fails with ConnectError, not a timeout", async () => {
    const { clock, backend } = await opened({ ack: false, manual: true });
    for (let i = 0; i < MAX_IN_FLIGHT; i++) await backend.send(1, new Uint8Array(8), false);
    const blocked = backend.send(1, new Uint8Array(8), false).catch((e: unknown) => e);
    const queued = backend.send(2, new Uint8Array(8), false).catch((e: unknown) => e);
    await clock.advance(0.05);
    await backend.close();
    expect(await blocked).toEqual(new ConnectError("gs_usb:ABC123 is not open"));
    expect(await queued).toBeInstanceOf(ConnectError);
    expect(clock.now()).toBeLessThan(0.1);
    expect(backend.inFlight).toBe(0);
  });

  it("reopens after close", async () => {
    const { dev, backend, received } = await opened();
    await backend.close();
    await backend.open();
    dev.receive(0x21, Uint8Array.of(1));
    await backend.send(1, new Uint8Array(1), false);
    await flush();
    expect(received).toEqual([[0x21, "01", false]]);
    expect(dev.opens).toBe(2);
    await backend.close();
  });
});

describe("session boundary", () => {
  /** A device whose BT_CONST answer waits until `go()` is called. */
  function gated() {
    const clock = new VirtualClock({ auto: false });
    const dev = new FakeGsUsbDevice({ serialNumber: "ABC123" });
    let go!: () => void;
    const gate = new Promise<void>((resolve) => (go = resolve));
    const controlIn = dev.controlTransferIn.bind(dev);
    dev.controlTransferIn = async (setup, length) => {
      await gate;
      return controlIn(setup, length);
    };
    return { clock, dev, go, backend: new GsUsbBackend(dev, { clock, logger: silent() }) };
  }

  it("a send during open is refused and nothing is written before the channel has started", async () => {
    const { dev, go, backend } = gated();
    const opening = backend.open();
    await flush();
    expect(dev.claimed).toBe(true);
    await expect(backend.send(1, new Uint8Array(1), false)).rejects.toThrow(new ConnectError("gs_usb:ABC123 is not open"));
    expect(backend.isOpen).toBe(false);
    go();
    await opening;
    expect(backend.isOpen).toBe(true);
    await backend.send(1, new Uint8Array(1), false);
    expect(dev.writes.length).toBe(1);
    expect(dev.protocolErrors).toEqual([]);
    await backend.close();
  });

  it("a second open() waits for the first, and a close() during open closes after it", async () => {
    const { dev, go, backend } = gated();
    const first = backend.open();
    const second = backend.open();
    const closing = backend.close();
    await flush();
    go();
    await Promise.all([first, second, closing]);
    expect([dev.opens, dev.claims, dev.closes]).toEqual([1, 1, 1]);
    expect(dev.controls.filter((c) => c.request === BREQ_MODE).map((c) => hex(c.data as Uint8Array))).toEqual([
      "0100000000000000",
      "0000000000000000",
    ]);
    expect(backend.isOpen || dev.opened).toBe(false);
  });

  it("sends queued in one session never write in the next, even when it opens at once", async () => {
    const { clock, dev, backend } = await opened({ ack: false, manual: true });
    for (let i = 0; i < MAX_IN_FLIGHT; i++) await backend.send(1, new Uint8Array(8), false);
    const blocked = backend.send(2, new Uint8Array(8), false).catch((e: unknown) => e);
    const queued = backend.send(3, new Uint8Array(8), false).catch((e: unknown) => e);
    await clock.advance(0.01);
    const closing = backend.close();
    const reopening = backend.open();
    await closing;
    await reopening;
    expect(backend.isOpen).toBe(true);
    expect(await blocked).toBeInstanceOf(ConnectError);
    expect(await queued).toBeInstanceOf(ConnectError);
    expect(dev.writes.map((w) => w.canId)).toEqual(Array(10).fill(1));
    dev.ack = true;
    await backend.send(4, new Uint8Array(1), false);
    expect(dev.writes.at(-1)!.canId).toBe(4);
    await backend.close();
  });
});

describe("echo slots and device ownership", () => {
  it("two backends cannot have one USBDevice open at once", async () => {
    const clock = new VirtualClock();
    const dev = new FakeGsUsbDevice({ serialNumber: "ABC123" });
    const a = new GsUsbBackend(dev, { clock, logger: silent() });
    const b = new GsUsbBackend(dev, { clock, logger: silent() });
    await a.open();
    await expect(b.open()).rejects.toThrow(new ConnectError("gs_usb:ABC123 is already open in another GsUsbBackend; close that one first"));
    expect([dev.opens, dev.claims, dev.closes]).toEqual([1, 1, 0]);
    await a.send(1, Uint8Array.of(1), false);
    await a.close();
    await b.open();
    await b.send(2, Uint8Array.of(2), false);
    await b.close();
  });

  it("echo ids skip an id still reserved by a frame whose echo was lost", async () => {
    const { dev, backend } = await opened({}, { ioTimeout: 1 });
    dev.ack = false;
    await backend.send(0x100, Uint8Array.of(0), false); // echo id 0, never echoed
    dev.ack = true;
    for (let i = 1; i < 0x10000; i++) await backend.send(1, new Uint8Array(0), false);
    await flush();
    expect(backend.inFlight).toBe(1);
    await backend.send(0x200, Uint8Array.of(1), false);
    const last = dev.writes.at(-1)!;
    expect([last.canId, last.echoId]).toEqual([0x200, 1]); // 0 is still taken
    await flush();
    expect(backend.inFlight).toBe(1);
    dev.receive(0x100, Uint8Array.of(0), { echoId: 0 }); // the late echo frees only its own slot
    await flush();
    expect(backend.inFlight).toBe(0);
    await backend.close();
  });

  it("after a close that failed with a write unresolved, open() waits for it, then refuses, naming the reason", async () => {
    const { clock, dev, backend } = await opened({}, { ioTimeout: 0.1 });
    dev.hangWrites = true;
    await expect(backend.send(1, new Uint8Array(8), false)).rejects.toThrow("timed out");
    const close = dev.close.bind(dev);
    const release = dev.releaseInterface.bind(dev);
    // a release that does not cancel the stuck transfer, and a close that fails
    dev.releaseInterface = async () => {
      dev.claimed = false;
    };
    dev.close = async () => {
      throw new Error("close error: busy");
    };
    await backend.close();
    expect(backend.closeError).toBe("gs_usb device close failed on gs_usb:ABC123: close error: busy");
    expect(dev.pendingWrites).toBe(1);
    const start = clock.now();
    const err = await backend.open().catch((e: unknown) => e);
    expect(err).toBeInstanceOf(ConnectError);
    expect((err as Error).message).toBe(
      "cannot open gs_usb:ABC123: 1 USB write(s) from the previous session have not ended " +
        "(gs_usb device close failed on gs_usb:ABC123: close error: busy); unplug and replug the adapter",
    );
    expect(clock.now() - start).toBeCloseTo(1.1, 9);
    expect(dev.claims).toBe(1); // nothing was touched

    // the old transfer ends (late, and failed): the next open goes ahead with a clean budget
    dev.close = close;
    dev.releaseInterface = release;
    dev.writeStatus = "stall";
    dev.releaseWrites();
    dev.writeStatus = null;
    await backend.open();
    expect(backend.inFlight).toBe(0);
    dev.ack = false;
    for (let i = 0; i < MAX_IN_FLIGHT; i++) await backend.send(2, new Uint8Array(1), false);
    await expect(backend.send(2, new Uint8Array(1), false)).rejects.toThrow("10 frames were never acknowledged");
    await backend.close();
  });

  it("a late completion from the closed session does not touch the new session's reservations", async () => {
    const { clock, dev, backend } = await opened({ manual: true }, { ioTimeout: 0.1 });
    dev.hangWrites = true;
    const first = backend.send(1, new Uint8Array(8), false).catch((e: unknown) => e);
    await clock.advance(0.1);
    expect(await first).toBeInstanceOf(TimeoutError);
    dev.releaseInterface = async () => {
      dev.claimed = false;
    };
    dev.close = async () => {
      throw new Error("close error: busy");
    };
    const closing = backend.close();
    await clock.advance(1.2); // the uncancelled reads hold close for its full bound
    await closing;
    expect(backend.closeError).toContain("close error: busy");
    const reopening = backend.open();
    await clock.advance(0.05);
    expect(dev.claims).toBe(1); // still waiting for the old transfer
    dev.releaseWrites(); // the old transfer completes while open() waits
    await clock.advance(0.01);
    await reopening;
    expect(dev.claims).toBe(2);
    expect(backend.inFlight).toBe(0);
    dev.ack = false;
    await backend.send(2, new Uint8Array(1), false);
    expect(backend.inFlight).toBe(1);
    // the old frame's echo arrives now: it belongs to the closed session
    dev.receive(1, new Uint8Array(8), { echoId: 0 });
    await clock.advance(0.01);
    expect(backend.inFlight).toBe(1);
  });
});

describe("browser helpers", () => {
  afterEach(() => vi.unstubAllGlobals());

  it("requestAdapter asks the chooser with the gs_usb filters; a dismissed chooser is null", async () => {
    const device = new FakeGsUsbDevice();
    const requestDevice = vi.fn(async () => device as unknown as USBDevice);
    vi.stubGlobal("navigator", { usb: { requestDevice, getDevices: async () => [] } });
    expect(await requestAdapter()).toBe(device);
    expect(requestDevice).toHaveBeenCalledWith({ filters: GS_USB_FILTERS.map((f) => ({ ...f })) });
    requestDevice.mockRejectedValueOnce(new DOMException("No device selected.", "NotFoundError"));
    expect(await requestAdapter()).toBeNull();
    requestDevice.mockRejectedValueOnce(new DOMException("Must be handling a user gesture.", "SecurityError"));
    await expect(requestAdapter()).rejects.toThrow("user gesture");
  });

  it("in a worker (no chooser) requestAdapter explains; grantedAdapters lists only gs_usb devices", async () => {
    const a = new FakeGsUsbDevice({ serialNumber: "A1" });
    const other = new FakeGsUsbDevice({ serialNumber: "Z", vendorId: 0x0403, productId: 0x6001 });
    vi.stubGlobal("navigator", { usb: { getDevices: async () => [other, a] } });
    await expect(requestAdapter()).rejects.toThrow("grantedAdapters()");
    const granted = await grantedAdapters();
    expect(granted).toEqual([a]);
    expect(selectAdapter(granted as unknown as FakeGsUsbDevice[], "gs_usb:A1")).toBe(a);
  });

  it("without WebUSB both helpers throw ConnectError", async () => {
    vi.stubGlobal("navigator", {});
    await expect(requestAdapter()).rejects.toThrow(ConnectError);
    await expect(grantedAdapters()).rejects.toThrow("WebUSB is not available");
  });
});
