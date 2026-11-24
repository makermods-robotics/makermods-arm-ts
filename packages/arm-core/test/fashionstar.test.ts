// FashionStarBus in the promise-based setting: every call is bounded and abortable, and the receive
// buffer cannot grow without bound.
import { describe, expect, it } from "vitest";
import { VirtualClock } from "@makermods/arm-core/testing";
import { FashionStarBus, RESYNC_ROOM, ServoBusError, WRITE_TIMEOUT } from "../src/leaders/fashionstar.js";
import { SimulatedStarPort } from "../src/leaders/sim.js";
import { StarLeader } from "../src/leaders/star.js";
import { JointMapper } from "../src/mapping.js";
import type { SerialPortLike } from "../src/transport/serial.js";
import { RecordingLogger } from "./shared/fakes.js";

/** A port whose writes never complete and which never sends anything (a wedged USB serial adapter). */
class WedgedPort implements SerialPortLike {
  readable: ReadableStream<Uint8Array> | null = null;
  writable: WritableStream<Uint8Array> | null = null;
  async open(): Promise<void> {
    this.readable = new ReadableStream<Uint8Array>();
    this.writable = new WritableStream<Uint8Array>({ write: () => new Promise<void>(() => {}) });
  }
  async close(): Promise<void> {}
}

describe("FashionStarBus bounds", () => {
  it("a write that never completes fails after WRITE_TIMEOUT", async () => {
    const clock = new VirtualClock({ start: 10 });
    const bus = await FashionStarBus.open(new WedgedPort(), { clock });
    await expect(bus.ping(0)).rejects.toThrow(/serial write timeout/);
    expect(clock.now()).toBeCloseTo(10 + WRITE_TIMEOUT, 12);
    await expect(bus.unlock(0)).rejects.toThrow(ServoBusError);
    await bus.close();
  });

  it("a Star leader on a wedged port counts misses and ends the session instead of hanging", async () => {
    const clock = new VirtualClock();
    const leader = new StarLeader(new WedgedPort(), {
      mapping: new JointMapper([{ servo: 0, direction: 1, range_deg: [-90, 90] }]), missLimit: 3, clock,
      logger: new RecordingLogger(),
    });
    leader.bus = await FashionStarBus.open(new WedgedPort(), { clock }); // connect() itself would fail its pings
    expect(await leader.read()).toBeNull();
    expect(await leader.read()).toBeNull();
    await expect(leader.read()).rejects.toThrow(/missed 3 reads in a row \(last error: serial write timeout\)/);
    await leader.close();
  });

  it("an aborted signal stops a call waiting for its reply, and counts no miss in the leader", async () => {
    const clock = new VirtualClock({ start: 10 });
    const port = new SimulatedStarPort([0]);
    port.setOnline(0, false);
    const bus = await FashionStarBus.open(port, { clock });
    const stop = new AbortController();
    clock.timer(0.03, () => stop.abort());
    await expect(bus.syncMonitor([0], stop.signal)).rejects.toThrow(/aborted/);
    expect(clock.now()).toBeLessThan(10.05);
    const leader = new StarLeader(port, { mapping: new JointMapper([{ servo: 0, direction: 1, range_deg: [-90, 90] }]), clock });
    leader.bus = bus;
    await expect(leader.read(stop.signal)).rejects.toThrow(ServoBusError);
    expect(leader.misses.get(0)).toBe(0);
    await bus.close();
  });

  it("close ends a call in progress", async () => {
    const clock = new VirtualClock({ start: 10 });
    const bus = await FashionStarBus.open(new WedgedPort(), { clock });
    const pinging = bus.ping(0);
    const closing = bus.close();
    await expect(pinging).rejects.toThrow(ServoBusError);
    await closing;
    expect(clock.now()).toBeLessThan(10 + WRITE_TIMEOUT);
  });

  it("bytes that arrive outside an exchange are dropped", async () => {
    const clock = new VirtualClock();
    const port = new SimulatedStarPort([0]);
    const bus = await FashionStarBus.open(port, { clock });
    port.inject(new Uint8Array(100_000).fill(0x55));
    await clock.advance(0.01);
    expect(bus.bufferedBytes).toBe(0);
    expect(await bus.ping(0)).toBe(true);
    await bus.close();
  });

  it("an exchange takes in at most its expected replies plus resynchronisation room", async () => {
    const clock = new VirtualClock();
    const port = new SimulatedStarPort([0, 1]);
    const bus = await FashionStarBus.open(port, { clock });
    port.noise = new Uint8Array(RESYNC_ROOM - 10).fill(0x00); // fits: the replies still parse
    expect([...(await bus.syncMonitor([0, 1])).keys()]).toEqual([0, 1]);
    expect((await bus.syncMonitor([0, 1])).get(1)!.reliable).toBe(true);
    port.noise = new Uint8Array(50_000).fill(0x00); // a chattering device: the replies fall beyond the cap
    let largest = 0;
    const sampling = setInterval(() => (largest = Math.max(largest, bus.bufferedBytes)), 0);
    const result = await bus.syncMonitor([0, 1]);
    clearInterval(sampling);
    expect(result.get(0)!.reliable).toBe(false); // held: its reply was dropped with the overflow
    expect(largest).toBeLessThanOrEqual(2 * 21 + RESYNC_ROOM);
    expect(bus.bufferedBytes).toBe(0);
    await bus.close();
  });
});
