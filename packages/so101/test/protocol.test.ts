// The Feetech codec (lerobot FeetechMotorsBus over scservo_sdk protocol 0) and the bus's reply matching.
import { VirtualClock } from "@makermods/arm-core/testing";
import { describe, expect, it } from "vitest";
import { Arbiter, FeetechBus, StaleEpochError, grantedPorts, requestPort, type SerialLike } from "../src/bus.js";
import { msClock } from "../src/clock-ms.js";
import {
  FeetechServoError,
  checksum,
  decodeSignMagnitude,
  decodeValue,
  encodeSignMagnitude,
  encodeValue,
  errorText,
  isFault,
  packet,
  parseStatus,
  percentToRaw,
  rawToPercent,
  readPacket,
  syncReadPacket,
  syncWritePacket,
  writePacket,
} from "../src/protocol.js";
import { SimFeetechBus } from "../src/sim.js";

describe("Feetech codec", () => {
  it("encodes sign-magnitude fields at their sign bits", () => {
    expect(encodeSignMagnitude(-200, 11)).toBe(0x800 | 200);
    expect(decodeSignMagnitude(0x800 | 200, 11)).toBe(-200);
    expect(encodeValue("Homing_Offset", -1365)).toBe(0x800 | 1365);
    expect(decodeValue("Goal_Velocity", 0x8000 | 1000)).toBe(-1000);
    expect(decodeValue("Present_Load", 0x400 | 25)).toBe(-25);
    expect(encodeValue("Goal_Position", 2048)).toBe(2048);
    expect(() => encodeSignMagnitude(2048, 11)).toThrow(RangeError);
  });

  it("builds packets as scservo_sdk does (FF FF id len instr params checksum)", () => {
    expect(Array.from(readPacket(1, 56, 2))).toEqual([0xff, 0xff, 1, 4, 0x02, 56, 2, ~(1 + 4 + 2 + 56 + 2) & 0xff]);
    expect(Array.from(writePacket(6, 42, [0x00, 0x08]))).toEqual([0xff, 0xff, 6, 5, 0x03, 42, 0, 8, ~(6 + 5 + 3 + 42 + 8) & 0xff]);
    const sr = syncReadPacket([1, 2, 3, 4, 5, 6], 40, 27);
    expect(Array.from(sr.subarray(0, 7))).toEqual([0xff, 0xff, 0xfe, 10, 0x82, 40, 27]);
    const sw = syncWritePacket(42, 2, [[1, 2048], [2, 1024]]);
    expect(Array.from(sw)).toEqual([0xff, 0xff, 0xfe, 10, 0x83, 42, 2, 1, 0, 8, 2, 0, 4, checksum(0xfe, 10, 0x83, [42, 2, 1, 0, 8, 2, 0, 4])]);
  });

  it("parses status packets, skipping noise and resyncing past a bad checksum", () => {
    const good = (id: number, error: number, params: number[]) => [0xff, 0xff, id, params.length + 2, error, ...params, checksum(id, params.length + 2, error, params)];
    const bad = good(3, 0, [1, 2]);
    bad[bad.length - 1] = bad.at(-1)! ^ 0xff;
    const { packets, badChecksums } = parseStatus(Uint8Array.from([0x00, 0x12, ...good(1, 0, [0x00, 0x08]), ...bad, ...good(2, 0x20, [])]));
    expect(badChecksums).toBe(1);
    expect(packets.map((p) => [p.id, p.error, Array.from(p.params)])).toEqual([[1, 0, [0, 8]], [2, 0x20, []]]);
  });

  it("names error bits and treats input voltage as a warning, not a fault", () => {
    expect(errorText(0x04)).toBe("overheat");
    expect(errorText(0x01 | 0x20)).toBe("input voltage, overload");
    expect(isFault({ error: 0x01, status: 0 })).toBe(false);
    expect(isFault({ error: 0x08, status: 0 })).toBe(true);
    expect(isFault({ error: 0, status: 0x20 })).toBe(true);
  });

  it("normalizes the gripper 0..100 over its range (lerobot RANGE_0_100)", () => {
    const c = { id: 6, drive_mode: 0, homing_offset: 0, range_min: 1000, range_max: 3000 };
    expect(rawToPercent(2000, c)).toBe(50);
    expect(rawToPercent(500, c)).toBe(0);
    expect(percentToRaw(25, c)).toBe(1500);
    expect(percentToRaw(25, { ...c, drive_mode: 1 })).toBe(2500);
  });
});

describe("FeetechBus on the simulated servos", () => {
  it("reads and writes registers, ignores an echoed request and a late reply, and raises a servo's error byte", async () => {
    const clock = new VirtualClock();
    const sim = new SimFeetechBus(clock, [1, 2]);
    // The adapter echoes every request (half duplex without echo cancellation).
    const write = sim.write.bind(sim);
    sim.write = async (bytes) => {
      sim.uart.feed(bytes);
      return write(bytes);
    };
    const bus = new FeetechBus(sim.uart, msClock(clock));
    expect(await bus.ping(1)).toBe(true);
    expect(await bus.ping(3)).toBe(false);
    expect(await bus.read(2, "Model_Number")).toBe(777);
    await bus.write(1, "Goal_Velocity", -300);
    expect(sim.reg(1, "Goal_Velocity")).toBe(-300);
    const states = await bus.syncReadState([1, 2]);
    expect([...states.keys()]).toEqual([1, 2]);
    expect(states.get(1)).toMatchObject({ torque: 0, position: 2048, voltage: 120, temperatureC: 30, status: 0, error: 0, velocity: 0 });
    sim.servo(2).forcedError = 0x04;
    await expect(bus.read(2, "Present_Position")).rejects.toBeInstanceOf(FeetechServoError);
    expect(bus.lastErrors.get(2)).toBe(0x04);
    sim.setOnline(2, false);
    await expect(bus.read(2, "Present_Position")).rejects.toThrow(/No reply from servo 2/);
    expect(bus.timeouts).toBe(2);
  });

  it("raw packets go out byte for byte", async () => {
    const clock = new VirtualClock();
    const sim = new SimFeetechBus(clock, [1]);
    const bus = new FeetechBus(sim.uart, msClock(clock));
    await bus.syncWrite("Goal_Position", [[1, 1000]]);
    await bus.action();
    expect(sim.requests).toEqual([Array.from(syncWritePacket(42, 2, [[1, 1000]])), Array.from(packet(0xfe, 0x05))]);
  });
});

describe("Arbiter", () => {
  it("runs one transaction at a time, priority first, and drops a stale epoch", async () => {
    const arbiter = new Arbiter();
    const order: string[] = [];
    let release!: () => void;
    const first = arbiter.run(() => new Promise<void>((r) => (release = () => (order.push("first"), r()))));
    const epoch = arbiter.epoch;
    const normal = arbiter.run(async () => void order.push("normal"), { epoch });
    const urgent = arbiter.run(async () => void order.push("urgent"), { priority: true });
    arbiter.bumpEpoch();
    release();
    await first;
    await urgent;
    await expect(normal).rejects.toBeInstanceOf(StaleEpochError);
    expect(order).toEqual(["first", "urgent"]);
    arbiter.close("gone");
    await expect(arbiter.run(async () => 1)).rejects.toThrow("gone");
  });
});

describe("getting a port", () => {
  const ch343 = { getInfo: () => ({ usbVendorId: 0x1a86, usbProductId: 0x55d3 }) } as unknown as SerialPort;
  const other = { getInfo: () => ({ usbVendorId: 0x0403, usbProductId: 0x6001 }) } as unknown as SerialPort;

  it("requestPort filters to the CH343 and answers null when the chooser is dismissed", async () => {
    const filters: unknown[] = [];
    const serial: SerialLike = {
      requestPort: async (options) => (filters.push(options?.filters), ch343),
      getPorts: async () => [ch343, other],
    };
    expect(await requestPort(serial)).toBe(ch343);
    expect(filters).toEqual([[{ usbVendorId: 0x1a86, usbProductId: 0x55d3 }]]);
    const dismissed: SerialLike = {
      requestPort: async () => {
        throw new DOMException("No port selected by the user.", "NotFoundError");
      },
      getPorts: async () => [],
    };
    expect(await requestPort(dismissed)).toBeNull();
    expect(await grantedPorts(serial)).toEqual([ch343]);
  });

  it("says so where Web Serial is missing", async () => {
    await expect(requestPort()).rejects.toThrow(/Web Serial is not available/);
  });
});
