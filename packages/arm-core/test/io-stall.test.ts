// A blocked event loop runs expired timers before it polls for I/O: a JavaScript bound around an I/O
// operation must not report a completion that happened during the stall as a timeout. Real time
// (RealClock) and real I/O callbacks (fs, delivered in libuv's poll phase); no hardware. The tests
// assert outcomes, not durations, apart from the deliberate stalls.
import { stat } from "node:fs";
import { describe, expect, it } from "vitest";
import { RealClock, Signal } from "../src/clock.js";
import { TimeoutError } from "../src/errors.js";
import { FakeGsUsbDevice } from "../src/transport/fake-gs-usb.js";
import { GsUsbBackend } from "../src/transport/gs-usb.js";

/** Blocks the event loop for `ms` (an application's synchronous work, a GC pause). */
function stall(ms: number): void {
  const end = performance.now() + ms;
  while (performance.now() < end) {
    // busy
  }
}

/** Calls `fn` from a real I/O completion (an fs callback, delivered in libuv's poll phase). */
function viaIo(fn: () => void): void {
  stat(".", () => fn());
}

/**
 * In the check phase (after this turn's poll), starts `io` and then blocks for `ms`: the I/O
 * completes during the stall, and the next loop iteration runs the expired timers before it polls,
 * which is the order a stalled application sees.
 */
function ioThenStall(io: () => void, ms: number): Promise<void> {
  return new Promise<void>((resolve) => setImmediate(() => {
    io();
    stall(ms);
    resolve();
  }));
}

/** Runs microtasks until `ready()` (no macrotask turn, so no timer or I/O callback runs meanwhile). */
async function untilIssued(ready: () => boolean): Promise<void> {
  for (let i = 0; i < 1000 && !ready(); i++) await Promise.resolve();
  expect(ready()).toBe(true);
}

async function backendWith(transfer: "io" | "never") {
  const clock = new RealClock();
  const dev = new FakeGsUsbDevice({ serialNumber: "STALL1" });
  const backend = new GsUsbBackend(dev, { clock, ioTimeout: 0.1 });
  await backend.open();
  let issued = false;
  let complete: () => void = () => {};
  const transferOut = dev.transferOut.bind(dev);
  dev.transferOut = (endpoint: number, data: BufferSource) => {
    issued = true;
    if (transfer === "never") return new Promise(() => {});
    // completes through a real I/O callback once the test starts it
    return new Promise((resolve, reject) => {
      complete = () => viaIo(() => transferOut(endpoint, data).then(resolve, reject));
    });
  };
  return { backend, issued: () => issued, complete: () => complete() };
}

describe("a stalled event loop does not turn a completed I/O into a timeout", () => {
  for (const ms of [150, 700]) {
    it(`gs_usb: a write that completed during a ${ms} ms stall resolves`, async () => {
      const { backend, issued, complete } = await backendWith("io");
      try {
        const sending = backend.send(1, new Uint8Array(8), false);
        const outcome = sending.then(() => "sent", (e: unknown) => `rejected: ${e instanceof Error ? e.message : e}`);
        await untilIssued(issued);
        await ioThenStall(complete, ms); // the transfer completes during the stall; the bound's timer expires
        expect(await outcome).toBe("sent");
      } finally {
        await backend.close();
      }
    });
  }

  it("gs_usb: a write that never completes is still rejected after the bound", async () => {
    const { backend, issued } = await backendWith("never");
    try {
      const sending = backend.send(1, new Uint8Array(8), false);
      await untilIssued(issued);
      await expect(sending).rejects.toThrow(TimeoutError);
      await expect(sending).rejects.toThrow(/timed out/);
    } finally {
      await backend.close();
    }
  });

  for (const ms of [150, 700]) {
    it(`Signal.wait: a reply delivered by I/O during a ${ms} ms stall is seen`, async () => {
      const clock = new RealClock();
      const signal = new Signal();
      let replied = false;
      const waiting = signal.wait(() => replied, 0.1, clock);
      await ioThenStall(() => viaIo(() => {
        replied = true;
        signal.notify();
      }), ms);
      expect(await waiting).toBe(true);
    });
  }

  it("Signal.wait: no reply is still a timeout", async () => {
    const clock = new RealClock();
    const signal = new Signal();
    expect(await signal.wait(() => false, 0.05, clock)).toBe(false);
  });
});
