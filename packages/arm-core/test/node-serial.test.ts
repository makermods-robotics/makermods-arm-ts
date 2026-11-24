// The Node serial helpers on a mocked `serialport` module (no serial port is opened).
import { describe, expect, it, vi } from "vitest";
import { ConnectError } from "@makermods/arm-core";
import { VirtualClock } from "@makermods/arm-core/testing";

type Listener = (...args: unknown[]) => void;

const state = vi.hoisted(() => ({
  ports: [] as { path: string; vendorId?: string; productId?: string; manufacturer?: string; serialNumber?: string }[],
  instances: [] as unknown[],
  openError: null as Error | null,
  /** Writes do not complete until the port closes (a device that stopped draining). */
  hangWrites: false,
  /** close() never calls back (a native close that does not finish). */
  hangClose: false,
}));

vi.mock("serialport", () => {
  class SerialPort {
    options: { path: string; baudRate: number; rtscts?: boolean };
    isOpen = false;
    written: Uint8Array[] = [];
    listeners = new Map<string, Set<Listener>>();
    constructor(options: { path: string; baudRate: number; autoOpen: false; rtscts?: boolean }) {
      this.options = options;
      state.instances.push(this);
    }
    static async list() {
      return state.ports;
    }
    open(cb: (e: Error | null) => void) {
      if (state.openError) return cb(state.openError);
      this.isOpen = true;
      cb(null);
    }
    stuck: ((e: Error) => void)[] = [];
    queuedUntilOpen = 0;
    write(data: Uint8Array, cb: (e?: Error | null) => void) {
      // As @serialport/stream's _write: on a closed port the write waits for the next 'open'.
      if (!this.isOpen) {
        this.queuedUntilOpen++;
        return false;
      }
      if (state.hangWrites) {
        // The binding's write only settles when the port closes (canceled).
        this.stuck.push((e) => cb(e));
        return false;
      }
      this.written.push(Uint8Array.from(data));
      // canable-fw sends nothing back for commands; answer frames with one reply
      const text = String.fromCharCode(...data);
      if (text.startsWith("t")) this.emit("data", new TextEncoder().encode("t1110\r"));
      cb(null);
      return true;
    }
    close(cb: (e: Error | null) => void) {
      if (!this.isOpen) return cb(new Error("Port is not open"));
      if (state.hangClose) return;
      this.isOpen = false;
      for (const fail of this.stuck.splice(0)) fail(Object.assign(new Error("Writing to COM port (GetOverlappedResult): Operation aborted"), { canceled: true }));
      this.emit("close", null);
      cb(null);
    }
    /** The device is pulled out: serialport closes the port and emits a disconnect. */
    unplug() {
      this.isOpen = false;
      this.emit("close", Object.assign(new Error("Disconnected"), { disconnected: true }));
    }
    on(event: string, l: Listener) {
      if (!this.listeners.has(event)) this.listeners.set(event, new Set());
      this.listeners.get(event)!.add(l);
      return this;
    }
    off(event: string, l: Listener) {
      this.listeners.get(event)?.delete(l);
      return this;
    }
    emit(event: string, ...args: unknown[]) {
      for (const l of [...(this.listeners.get(event) ?? [])]) l(...args);
    }
  }
  return { SerialPort };
});

const { listSerialPorts, openSerialPort, openSlcanAdapter, slcanCandidates } = await import("@makermods/arm-core/node");
const { SlcanSerialBackend } = await import("@makermods/arm-core/transport/slcan-serial");

interface MockPort {
  options: { path: string; baudRate: number; rtscts?: boolean };
  isOpen: boolean;
  written: Uint8Array[];
  stuck: unknown[];
  queuedUntilOpen: number;
  emit(event: string, ...args: unknown[]): void;
  unplug(): void;
}

const flush = async (turns = 50) => {
  for (let i = 0; i < turns; i++) await Promise.resolve();
};

describe("node serial", () => {
  it("lists ports sorted by path with their USB ids, and picks SLCAN candidates as Python does", async () => {
    state.ports = [
      { path: "/dev/ttyUSB0", vendorId: "1a86", productId: "7523" },
      { path: "/dev/ttyACM0", vendorId: "ad50", productId: "60c4", manufacturer: "Protofusion Labs", serialNumber: "0042" },
      { path: "/dev/ttyTHS1" },
    ];
    const ports = await listSerialPorts();
    expect(ports.map((p) => [p.path, p.vidPid, p.vendorId])).toEqual([
      ["/dev/ttyACM0", "ad50:60c4", 0xad50], ["/dev/ttyTHS1", "", null], ["/dev/ttyUSB0", "1a86:7523", 0x1a86],
    ]);
    expect(slcanCandidates(ports, "linux").map((p) => p.path)).toEqual(["/dev/ttyACM0", "/dev/ttyUSB0"]);
    expect(slcanCandidates([...ports, { ...ports[0]!, path: "/dev/cu.usbmodem1" }], "darwin").map((p) => p.path))
      .toEqual(["/dev/cu.usbmodem1"]);
    expect(slcanCandidates(ports, "win32")).toHaveLength(3);
  });

  it("a SerialPortLike with Web Serial's shape: streams while open, close refused while locked", async () => {
    state.ports = [{ path: "/dev/ttyACM0", vendorId: "ad50", productId: "60c4" }];
    state.instances = [];
    const port = await openSerialPort("/dev/ttyACM0", { rtscts: true });
    expect(port.getInfo?.()).toEqual({ usbVendorId: 0xad50, usbProductId: 0x60c4 });
    expect(port.readable).toBe(null);
    await port.open({ baudRate: 115200 });
    const raw = state.instances[0] as MockPort;
    expect(raw.options).toMatchObject({ path: "/dev/ttyACM0", baudRate: 115200, rtscts: true });
    const reader = port.readable!.getReader();
    const writer = port.writable!.getWriter();
    await expect(port.close()).rejects.toThrow(/locked/);
    await writer.write(new TextEncoder().encode("t0100\r"));
    expect(String.fromCharCode(...raw.written[0]!)).toBe("t0100\r");
    expect(String.fromCharCode(...(await reader.read()).value!)).toBe("t1110\r");
    reader.releaseLock();
    writer.releaseLock();
    await port.close();
    expect(raw.isOpen).toBe(false);
    expect(port.readable).toBe(null);
  });

  it("an unplug errors the readable stream", async () => {
    state.instances = [];
    const port = await openSerialPort("/dev/ttyACM0", { info: {} });
    await port.open({ baudRate: 115200 });
    const reader = port.readable!.getReader();
    const raw = state.instances[0] as MockPort;
    raw.emit("close", Object.assign(new Error("Disconnected"), { disconnected: true }));
    await expect(reader.read()).rejects.toThrow("/dev/ttyACM0 was disconnected");
  });

  it("openSlcanAdapter: the only candidate, or a clear error; the backend opens and talks over it", async () => {
    state.ports = [];
    await expect(openSlcanAdapter()).rejects.toThrow(ConnectError);
    await expect(openSlcanAdapter()).rejects.toThrow(/no USB CAN adapter found/);
    state.ports = [{ path: "/dev/ttyACM0" }, { path: "/dev/ttyACM1" }];
    await expect(openSlcanAdapter()).rejects.toThrow("several serial devices found (/dev/ttyACM0, /dev/ttyACM1); pass the port path");
    state.ports = [{ path: "/dev/ttyACM1", vendorId: "ad50", productId: "60c4" }, { path: "/dev/ttyS0" }];
    state.instances = [];
    const clock = new VirtualClock();
    const backend = await openSlcanAdapter(null, { clock, startupDelay: 0 });
    expect(backend).toBeInstanceOf(SlcanSerialBackend);
    expect(backend.port).toBe("/dev/ttyACM1");
    const got: number[] = [];
    backend.setRecvCallback((id) => got.push(id));
    await backend.open();
    await backend.send(0x10, new Uint8Array(0), false);
    await flush();
    await backend.close();
    const raw = state.instances[0] as MockPort;
    expect(raw.written.map((b) => String.fromCharCode(...b))).toEqual(["C\rS8\rA0\rO\r", "t0100\r", "C\r"]);
    expect(got).toEqual([0x111]);
    expect(raw.isOpen).toBe(false);
    expect(backend.closeError).toBe(null);
  });

  it("a port that will not open is a ConnectError naming the path", async () => {
    state.openError = new Error("Error: Permission denied, cannot open /dev/ttyACM0");
    try {
      const backend = await openSlcanAdapter("/dev/ttyACM0", { clock: new VirtualClock(), startupDelay: 0 });
      await expect(backend.open()).rejects.toThrow(/cannot open \/dev\/ttyACM0: Error: Permission denied/);
    } finally {
      state.openError = null;
    }
  });

  it("a write that only settles when the port closes does not keep the port from closing", async () => {
    state.instances = [];
    state.hangWrites = true;
    try {
      const port = await openSerialPort("/dev/ttyACM0", { info: {} });
      await port.open({ baudRate: 115200 });
      const raw = state.instances[0] as MockPort;
      const writer = port.writable!.getWriter();
      const write = writer.write(new TextEncoder().encode("t0100\r")).catch((e: unknown) => e);
      await flush();
      expect(raw.stuck).toHaveLength(1);
      writer.releaseLock();
      await port.close();
      expect(raw.isOpen).toBe(false);
      expect(String(await write)).toMatch(/closed|aborted/);
      await port.open({ baudRate: 115200 }); // the port is free again
      await port.close();
    } finally {
      state.hangWrites = false;
    }
  });

  it("a port closed underneath: pending and later writes fail at once instead of waiting for a reopen", async () => {
    state.instances = [];
    state.hangWrites = true;
    const port = await openSerialPort("/dev/ttyACM0", { info: {} });
    await port.open({ baudRate: 115200 });
    const raw = state.instances[0] as MockPort;
    const writer = port.writable!.getWriter();
    const pending = writer.write(Uint8Array.of(0x43, 0x0d)).catch((e: unknown) => e);
    await flush();
    state.hangWrites = false;
    raw.unplug();
    expect(String(await pending)).toMatch(/\/dev\/ttyACM0 was disconnected/);
    writer.releaseLock();
    const later = port.writable!.getWriter();
    await expect(later.write(Uint8Array.of(0x43, 0x0d))).rejects.toThrow();
    expect(raw.queuedUntilOpen).toBe(0);
    later.releaseLock();
    await port.close(); // nothing native left to close
  });

  it("close is bounded, and the port stays owned until the native close finishes", async () => {
    state.instances = [];
    state.hangClose = true;
    try {
      const clock = new VirtualClock();
      const port = await openSerialPort("/dev/ttyACM0", { info: {}, clock, closeTimeout: 2.0 });
      await port.open({ baudRate: 115200 });
      await expect(port.close()).rejects.toThrow("closing /dev/ttyACM0 timed out after 2 s");
      expect(clock.now()).toBe(2.0);
      await expect(port.open({ baudRate: 115200 })).rejects.toThrow(/already open/);
    } finally {
      state.hangClose = false;
    }
  });

  it("the SLCAN backend over a Node port whose writes stall: send times out, close still closes the port", async () => {
    state.ports = [{ path: "/dev/ttyACM0" }];
    state.instances = [];
    const clock = new VirtualClock();
    const backend = await openSlcanAdapter("/dev/ttyACM0", { clock, startupDelay: 0, logger: { warn() {}, error() {} } });
    await backend.open();
    const raw = state.instances[0] as MockPort;
    state.hangWrites = true;
    try {
      await expect(backend.send(0x10, new Uint8Array(0), false)).rejects.toThrow(/timed out after 1 s/);
      expect(backend.stalled).not.toBe(null);
      await backend.close();
      expect(raw.isOpen).toBe(false);
      expect(backend.closeError).toMatch(/C not sent/);
      expect(raw.written.map((b) => String.fromCharCode(...b))).toEqual(["C\rS8\rA0\rO\r"]);
    } finally {
      state.hangWrites = false;
    }
  });
});
