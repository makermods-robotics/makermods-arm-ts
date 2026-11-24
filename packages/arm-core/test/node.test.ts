import { describe, expect, it, vi } from "vitest";
import { ConnectError } from "@makermods/arm-core";
import { VirtualClock } from "@makermods/arm-core/testing";
import { NodeGsUsbDevice, type RawDevice, type RawEndpoint, listAdapters, loadUsb, openAdapter } from "@makermods/arm-core/node";
import { BREQ_MODE, EP_OUT, GsUsbBackend } from "@makermods/arm-core/transport/gs-usb";
import { FakeGsUsbDevice } from "@makermods/arm-core/transport/fake-gs-usb";

const LIBUSB_TRANSFER_TIMED_OUT = 2;

/**
 * A fake adapter as node-usb presents it: the WebUSB-shaped device, and the raw node-usb Device
 * behind it, typed as RawDevice (derived from node-usb's own Device/Interface/Endpoint types). The
 * kernel-driver calls are on the Interface, as in node-usb 2.19. Bulk IN reads go through the raw
 * IN endpoint, end after ioTimeout with libusb's timed-out error, and close() refuses while one is
 * pending, as node-usb does.
 */
function nodeFake(kernelDriver: boolean, clock: VirtualClock, ioTimeout = 0.1) {
  const web = new FakeGsUsbDevice({ serialNumber: "N1" });
  web.refuseCloseWithPendingReads = true;
  const calls: string[] = [];
  const out: RawEndpoint = { timeout: 0, direction: "out" };
  const inp: RawEndpoint = {
    timeout: 0,
    direction: "in",
    transferAsync: async (length: number) => {
      const r = await web.transferIn(1, length);
      if (r.data!.byteLength === 0) return undefined;
      return Buffer.from(r.data!.buffer, r.data!.byteOffset, r.data!.byteLength);
    },
  };
  web.readTimeout = { clock, seconds: ioTimeout, error: () => Object.assign(new Error("LIBUSB_TRANSFER_TIMED_OUT"), { errno: LIBUSB_TRANSFER_TIMED_OUT }) };
  const state = { kernelDriver };
  const raw: RawDevice = {
    timeout: 1000,
    get interfaces() {
      return web.opened ? [{}] : undefined;
    },
    interface: (n: number) => {
      if (n !== 0) throw new Error(`Interface not found for address: ${n}`);
      return {
        isKernelDriverActive: () => (calls.push("isKernelDriverActive"), state.kernelDriver),
        detachKernelDriver: () => {
          calls.push("detachKernelDriver");
          state.kernelDriver = false;
        },
        endpoint: (address: number) => (address === EP_OUT ? out : address === 0x81 ? inp : undefined),
      };
    },
  };
  return { web, raw, out, inp, calls, state };
}

describe("NodeGsUsbDevice", () => {
  it("on Linux detaches the kernel driver through the interface at open, leaves it detached, bounds transfers in libusb", async () => {
    const clock = new VirtualClock();
    const { web, raw, out, inp, calls, state } = nodeFake(true, clock);
    const backend = new GsUsbBackend(new NodeGsUsbDevice(web, raw, { platform: "linux", ioTimeout: 0.1 }), { clock });
    expect(backend.port).toBe("gs_usb:N1");
    await backend.open();
    expect(calls).toEqual(["isKernelDriverActive", "detachKernelDriver"]);
    expect([raw.timeout, out.timeout, inp.timeout]).toEqual([100, 100, 100]);
    await backend.send(0x7ff, Uint8Array.of(7, 0, 0xcc, 0, 0, 0, 0, 0), false);
    expect(web.writes.length).toBe(1);
    await backend.close();
    expect(backend.closeError).toBeNull();
    expect(state.kernelDriver).toBe(false);
    expect(web.resets).toBe(0);
    expect(web.protocolErrors).toEqual([]);
  });

  it("does not touch kernel drivers on other platforms, nor detach when none is bound", async () => {
    for (const [platform, active, expected] of [
      ["darwin", true, []],
      ["linux", false, ["isKernelDriverActive"]],
    ] as const) {
      const clock = new VirtualClock();
      const { web, raw, calls } = nodeFake(active, clock);
      const backend = new GsUsbBackend(new NodeGsUsbDevice(web, raw, { platform }), { clock });
      await backend.open();
      expect(calls).toEqual(expected);
      await backend.close();
    }
  });

  it("a failed detach fails the open cleanly", async () => {
    const clock = new VirtualClock();
    const { web, raw } = nodeFake(true, clock);
    const iface = raw.interface(0);
    raw.interface = () => ({
      ...iface,
      detachKernelDriver: () => {
        throw new Error("LIBUSB_ERROR_ACCESS");
      },
    });
    const backend = new GsUsbBackend(new NodeGsUsbDevice(web, raw, { platform: "linux" }), { clock });
    await expect(backend.open()).rejects.toThrow(/cannot open gs_usb:N1: LIBUSB_ERROR_ACCESS/);
    expect(web.opened || web.claimed).toBe(false);
  });

  it("read timeouts are 'nothing yet': receiving continues and frames still arrive", async () => {
    const clock = new VirtualClock({ auto: false });
    const { web, raw } = nodeFake(false, clock);
    const logger = { warn: vi.fn(), error: vi.fn() };
    const backend = new GsUsbBackend(new NodeGsUsbDevice(web, raw, { platform: "linux" }), { clock, logger });
    const got: number[] = [];
    backend.setRecvCallback((id) => got.push(id));
    await backend.open();
    for (let i = 0; i < 10; i++) await clock.advance(0.1); // forty reads time out
    expect(backend.error).toBeNull();
    expect(web.pendingReads).toBe(4);
    web.receive(0x55, Uint8Array.of(1));
    await clock.advance(0.01);
    expect(got).toEqual([0x55]);
    expect(logger.error).not.toHaveBeenCalled();
    const closing = backend.close();
    await clock.advance(0.2);
    await closing;
    expect(backend.closeError).toBeNull();
  });

  it("close waits for the reads to end before stopping the channel and closing the device", async () => {
    const clock = new VirtualClock({ auto: false });
    const { web, raw } = nodeFake(false, clock);
    const backend = new GsUsbBackend(new NodeGsUsbDevice(web, raw, { platform: "linux" }), { clock });
    await backend.open();
    await clock.advance(0.05);
    const pendingAt: Record<string, number> = {};
    const control = web.controlTransferOut.bind(web);
    web.controlTransferOut = async (setup, data) => {
      if (setup.request === BREQ_MODE) pendingAt.modeReset = web.pendingReads;
      return control(setup, data);
    };
    const close = web.close.bind(web);
    web.close = async () => {
      pendingAt.close = web.pendingReads;
      return close();
    };
    const start = clock.now();
    const closing = backend.close();
    await clock.advance(0.2);
    await closing;
    expect(pendingAt).toEqual({ modeReset: 0, close: 0 });
    expect(clock.now() - start).toBeLessThanOrEqual(0.2 + 1e-9);
    expect(web.opened).toBe(false);
    expect(backend.closeError).toBeNull();
  });

  it("a device close node-usb refuses is recorded in closeError and logged", async () => {
    const clock = new VirtualClock();
    const { web, raw } = nodeFake(false, clock);
    web.readTimeout = null; // reads that never end
    // a release that does not cancel the reads
    web.releaseInterface = async () => {
      web.claimed = false;
    };
    const logger = { warn: vi.fn(), error: vi.fn() };
    const backend = new GsUsbBackend(new NodeGsUsbDevice(web, raw, { platform: "linux" }), { clock, logger });
    await backend.open();
    await backend.close();
    expect(backend.closeError).toMatch(/^gs_usb device close failed on gs_usb:N1: close error: Can't close device with a pending request/);
    expect(logger.warn).toHaveBeenCalledWith(expect.stringContaining("may not have stopped cleanly"), expect.any(Error));
    expect(web.opened).toBe(true);
  });
});

describe("without the usb package", () => {
  it("listAdapters and openAdapter throw a ConnectError that says how to install it", async () => {
    vi.resetModules();
    vi.doMock("usb", () => {
      throw new Error("Cannot find package 'usb'");
    });
    try {
      const node = await import("@makermods/arm-core/node");
      const errors = await import("@makermods/arm-core");
      const err = await node.listAdapters().catch((e: unknown) => e);
      expect(err).toBeInstanceOf(errors.ConnectError);
      expect((err as Error).message).toContain("npm install usb");
      await expect(node.openAdapter()).rejects.toThrow("needs the optional 'usb' package");
    } finally {
      vi.doUnmock("usb");
      vi.resetModules();
    }
  });
});

describe("Node enumeration (read-only)", () => {
  it("loads the usb package", async () => {
    const usb = await loadUsb();
    expect(typeof usb.getDeviceList).toBe("function");
    expect(usb.usb.LIBUSB_TRANSFER_TIMED_OUT).toBe(LIBUSB_TRANSFER_TIMED_OUT);
  });

  // Reads USB descriptors of connected gs_usb adapters only: no interface is claimed, nothing is
  // written, and no backend is opened.
  it("lists connected adapters by serial; an unknown serial is refused; a known one gives an unopened backend", async (ctx) => {
    let adapters;
    try {
      adapters = await listAdapters();
    } catch (e) {
      ctx.skip(`USB enumeration unavailable: ${(e as Error).message}`);
      return;
    }
    if (adapters.length === 0) {
      ctx.skip("no gs_usb adapter connected");
      return;
    }
    for (const a of adapters) {
      expect(a.vidPid).toMatch(/^[0-9a-f]{4}:[0-9a-f]{4}$/);
      expect(a.usbPath).toMatch(/^\d+[-:][\d.]+$/);
      if (a.serial) expect(a.port).toBe(`gs_usb:${a.serial}`);
    }
    const sorted = [...adapters].sort((x, y) => (x.serial < y.serial ? -1 : x.serial > y.serial ? 1 : 0));
    expect(adapters.map((a) => a.serial)).toEqual(sorted.map((a) => a.serial));

    await expect(openAdapter("gs_usb:NO-SUCH-SERIAL")).rejects.toThrow(ConnectError);
    await expect(openAdapter("gs_usb:NO-SUCH-SERIAL")).rejects.toThrow(`found ${adapters.length} adapter(s)`);
    const named = adapters.find((a) => a.serial);
    if (named) {
      const backend = await openAdapter(named.port);
      expect(backend.port).toBe(named.port);
      expect(backend.device.opened).toBe(false); // never opened here
      expect(backend.isOpen).toBe(false);
    }
  });
});

// ---------- A faithful double of node-usb 2.19 (dist/usb/device.js, src/device.cc, dist/webusb/webusb-device.js) ----------

const STRINGS = ["", "bytewerk", "candleLight USB to CAN adapter"];

/** node-usb's Device: getDeviceList() hands out this same object while it lives; open() resets `interfaces`. */
class FakeRawDevice {
  readonly deviceDescriptor = { idVendor: 0x1d50, idProduct: 0x606f, iManufacturer: 1, iProduct: 2, iSerialNumber: 3 };
  readonly deviceAddress = 7;
  timeout = 1000;
  interfaces: FakeInterface[] | undefined = undefined;
  handle = false;
  pendingTransfers = 0;
  nativeOpens = 0;
  nativeCloses = 0;

  constructor(
    readonly gs: FakeGsUsbDevice,
    readonly busNumber: number,
    readonly portNumbers: number[],
  ) {}

  open(defaultConfig = true) {
    if (!this.handle) {
      this.handle = true; // Device::Open: libusb_open only when there is no handle yet
      this.nativeOpens++;
      void this.gs.open();
    }
    this.interfaces = []; // device.js: reset even when the handle was already open
    if (defaultConfig !== false) this.interfaces[0] = new FakeInterface(this);
  }

  close() {
    if (this.pendingTransfers) throw new Error("Can't close device with a pending request");
    if (this.handle) {
      this.handle = false;
      this.nativeCloses++;
      void this.gs.close();
    }
    this.interfaces = undefined;
  }

  interface(addr: number) {
    if (!this.interfaces) throw new Error("Device must be open before searching for interfaces");
    const found = this.interfaces.find((i) => i.interfaceNumber === addr);
    if (!found) throw new Error(`Interface not found for address: ${addr}`);
    return found;
  }

  getStringDescriptor(index: number, cb: (error?: Error, value?: string) => void) {
    if (!this.handle) cb(new Error("LIBUSB_ERROR_IO"));
    else cb(undefined, index === 3 ? (this.gs.serialNumber ?? "") : STRINGS[index]);
  }
}

class FakeInterface {
  readonly interfaceNumber = 0;
  readonly out: RawEndpoint & { address: number } = { address: 0x02, direction: "out", timeout: 0 };
  readonly in: RawEndpoint & { address: number };

  constructor(readonly device: FakeRawDevice) {
    const gs = device.gs;
    this.in = {
      address: 0x81,
      direction: "in",
      timeout: 0,
      transferAsync: async (length: number) => {
        device.pendingTransfers++;
        try {
          const r = await gs.transferIn(1, length);
          return r.data!.byteLength ? Buffer.from(r.data!.buffer, r.data!.byteOffset, r.data!.byteLength) : undefined;
        } finally {
          device.pendingTransfers--;
        }
      },
    };
  }

  claim() {
    void this.device.gs.claimInterface(0);
  }
  isKernelDriverActive() {
    return false;
  }
  detachKernelDriver() {}
  endpoint(address: number) {
    return [this.out, this.in].find((e) => e.address === address);
  }
}

/** node-usb's WebUSBDevice over a FakeRawDevice, including createInstance()'s open-read-close. */
class FakeWebUSBDevice {
  readonly vendorId = 0x1d50;
  readonly productId = 0x606f;
  readonly configurations = [{ configurationValue: 1 }];
  serialNumber: string | null = null;
  productName: string | null = null;
  manufacturerName: string | null = null;

  constructor(readonly raw: FakeRawDevice) {}

  static async createInstance(raw: FakeRawDevice) {
    const web = new FakeWebUSBDevice(raw);
    try {
      if (!web.opened) raw.open();
      const read = (i: number) => new Promise<string>((resolve) => raw.getStringDescriptor(i, (e, v) => resolve(e ? "" : v!)));
      web.manufacturerName = await read(1);
      web.productName = await read(2);
      web.serialNumber = await read(3);
    } finally {
      if (web.opened) raw.close(); // closes whatever it found open
    }
    return web;
  }

  get opened() {
    return !!this.raw.interfaces;
  }
  get configuration() {
    return this.raw.gs.configuration;
  }
  async open() {
    if (!this.opened) this.raw.open();
  }
  async close() {
    if (!this.opened) return;
    try {
      if (this.raw.gs.claimed) await this.raw.gs.releaseInterface(0);
    } catch {
      // ignored, as node-usb does
    }
    this.raw.close();
  }
  selectConfiguration(value: number) {
    return this.raw.gs.selectConfiguration(value);
  }
  async claimInterface(n: number) {
    if (!this.opened) throw new Error("claimInterface error: invalid state");
    this.raw.interface(n).claim();
  }
  releaseInterface(n: number) {
    return this.raw.gs.releaseInterface(n);
  }
  controlTransferIn(setup: Parameters<FakeGsUsbDevice["controlTransferIn"]>[0], length: number) {
    return this.raw.gs.controlTransferIn(setup, length);
  }
  controlTransferOut(setup: Parameters<FakeGsUsbDevice["controlTransferOut"]>[0], data?: BufferSource) {
    return this.raw.gs.controlTransferOut(setup, data);
  }
  async transferIn(): Promise<never> {
    throw new Error("transferIn goes through the raw endpoint");
  }
  async transferOut(endpoint: number, data: BufferSource) {
    const ep = (this.raw.interfaces ?? []).map((i) => i.endpoint(endpoint)).find(Boolean);
    if (!ep) throw new Error("transferOut error: TypeError: Cannot read properties of undefined (reading 'transferAsync')");
    return this.raw.gs.transferOut(endpoint, data);
  }
}

function twoAdapters(clock: VirtualClock) {
  const make = (serial: string, bus: number) => {
    const gs = new FakeGsUsbDevice({ serialNumber: serial });
    gs.readTimeout = { clock, seconds: 0.1, error: () => Object.assign(new Error("LIBUSB_TRANSFER_TIMED_OUT"), { errno: LIBUSB_TRANSFER_TIMED_OUT }) };
    return new FakeRawDevice(gs, bus, [2]);
  };
  return [make("A1", 1), make("B1", 3)] as const;
}

async function withFakeUsb(devices: readonly FakeRawDevice[]) {
  vi.resetModules();
  vi.doMock("usb", () => ({
    getDeviceList: () => [...devices],
    WebUSBDevice: FakeWebUSBDevice,
    usb: { LIBUSB_TRANSFER_TIMED_OUT, LIBUSB_TRANSFER_STALL: 4, LIBUSB_TRANSFER_OVERFLOW: 6 },
  }));
  return import("@makermods/arm-core/node");
}

describe("two adapters in one Node process (node-usb double)", () => {
  it("the double reproduces the hazard: opening an owned device for discovery breaks the arm using it", async () => {
    const clock = new VirtualClock();
    const [a] = twoAdapters(clock);
    const web = await FakeWebUSBDevice.createInstance(a);
    const backend = new GsUsbBackend(new NodeGsUsbDevice(web as never, a as never, { platform: "linux" }), { clock, logger: { warn: vi.fn(), error: vi.fn() } });
    await backend.open();
    await backend.send(1, Uint8Array.of(1), false);
    // what discovery used to do: open(false), read the strings, close
    try {
      a.open(false);
    } finally {
      try {
        a.close();
      } catch {
        // pending reads: node-usb refuses
      }
    }
    await expect(backend.send(2, Uint8Array.of(2), false)).rejects.toThrow("Cannot read properties of undefined");
    await backend.close().catch(() => undefined);
  });

  it("discovery leaves an open adapter alone; the other adapter opens beside it; a second open of the first is refused", async () => {
    const clock = new VirtualClock();
    const [a, b] = twoAdapters(clock);
    try {
      const node = await withFakeUsb([a, b]);
      const backendA = await node.openAdapter("gs_usb:A1", { clock });
      await backendA.open();
      await backendA.send(1, Uint8Array.of(1), false);
      const opensA = [a.nativeOpens, a.nativeCloses];

      const listed = await node.listAdapters();
      expect(listed.map((x) => [x.serial, x.port, x.usbPath, x.product])).toEqual([
        ["A1", "gs_usb:A1", "1-2", "candleLight USB to CAN adapter"],
        ["B1", "gs_usb:B1", "3-2", "candleLight USB to CAN adapter"],
      ]);
      expect([a.nativeOpens, a.nativeCloses]).toEqual(opensA);
      expect(a.interfaces?.length).toBe(1);

      const again = await node.openAdapter("gs_usb:A1", { clock }).catch((e: unknown) => e);
      expect((again as Error).name).toBe("ConnectError");
      expect((again as Error).message).toContain("already open in this process");

      const backendB = await node.openAdapter("gs_usb:B1", { clock });
      await backendB.open();
      await backendA.send(2, Uint8Array.of(2), false);
      await backendB.send(3, Uint8Array.of(3), false);
      expect(a.gs.writes.map((w) => w.canId)).toEqual([1, 2]);
      expect(b.gs.writes.map((w) => w.canId)).toEqual([3]);
      expect([a.nativeOpens, a.nativeCloses]).toEqual(opensA);

      await backendA.close();
      await backendB.close();
      expect(backendA.closeError).toBeNull();
      expect(a.handle || b.handle).toBe(false);
      // released: discovery reads it again, and it can be opened anew
      await node.listAdapters();
      expect(a.handle).toBe(false);
      const reopened = await node.openAdapter("gs_usb:A1", { clock });
      await reopened.open();
      await reopened.close();
    } finally {
      vi.doUnmock("usb");
      vi.resetModules();
    }
  });

  it("two backends made for one adapter before either opened: the second open() is refused", async () => {
    const clock = new VirtualClock();
    const [a] = twoAdapters(clock);
    try {
      const node = await withFakeUsb([a]);
      const first = await node.openAdapter(undefined, { clock });
      const second = await node.openAdapter(undefined, { clock });
      await first.open();
      const err = await second.open().catch((e: unknown) => e);
      expect((err as Error).name).toBe("ConnectError");
      expect((err as Error).message).toContain("gs_usb:A1 is already open in this process");
      await first.send(1, Uint8Array.of(1), false);
      await first.close();
      await second.open();
      await second.close();
    } finally {
      vi.doUnmock("usb");
      vi.resetModules();
    }
  });
});
