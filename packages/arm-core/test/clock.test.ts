import { describe, expect, it } from "vitest";
import { Mutex, RealClock, Signal } from "@makermods/arm-core";
import { VirtualClock } from "@makermods/arm-core/testing";

describe("VirtualClock", () => {
  it("moves time by exactly the slept amount, in deadline order", async () => {
    const clock = new VirtualClock();
    const order: string[] = [];
    const a = clock.sleep(0.005).then(() => order.push(`a@${clock.now()}`));
    const b = clock.sleep(0.002).then(() => order.push(`b@${clock.now()}`));
    await Promise.all([a, b]);
    expect(order).toEqual(["b@0.002", "a@0.005"]);
    await clock.sleep(0.25);
    expect(clock.now()).toBe(0.005 + 0.25);
  });

  it("cancelled timers neither fire nor move time", async () => {
    const clock = new VirtualClock();
    let fired = false;
    clock.timer(10, () => (fired = true))();
    await clock.sleep(0.001);
    expect(fired).toBe(false);
    expect(clock.now()).toBe(0.001);
  });

  it("manual mode only moves in advance()", async () => {
    const clock = new VirtualClock({ auto: false });
    let woke = -1;
    void clock.sleep(0.01).then(() => (woke = clock.now()));
    await clock.advance(0.005);
    expect(woke).toBe(-1);
    await clock.advance(0.01);
    expect(woke).toBe(0.01);
    expect(clock.now()).toBe(0.015);
  });
});

describe("Signal", () => {
  it("resolves on notify, and false on timeout without leaving a timer", async () => {
    const clock = new VirtualClock();
    const signal = new Signal();
    let seq = 0;
    const hit = signal.wait(() => seq === 1, 0.05, clock);
    seq = 1;
    signal.notify();
    expect(await hit).toBe(true);
    expect(clock.pending).toBe(0);
    expect(clock.now()).toBe(0);
    expect(await signal.wait(() => false, 0.05, clock)).toBe(false);
    expect(clock.now()).toBe(0.05);
  });
});

describe("Mutex", () => {
  it("runs holders one at a time in call order", async () => {
    const clock = new VirtualClock();
    const mutex = new Mutex();
    const log: string[] = [];
    await Promise.all(["a", "b", "c"].map((name, i) => mutex.run(async () => {
      log.push(`${name}+`);
      await clock.sleep(0.01 * (3 - i));
      log.push(`${name}-`);
    })));
    expect(log).toEqual(["a+", "a-", "b+", "b-", "c+", "c-"]);
  });
});

describe("RealClock", () => {
  it("sleeps at least the requested time and close to it", async () => {
    const clock = new RealClock();
    const errors: number[] = [];
    for (let i = 0; i < 20; i++) {
      const t0 = clock.now();
      await clock.sleep(0.005);
      errors.push(clock.now() - t0 - 0.005);
    }
    expect(Math.min(...errors)).toBeGreaterThanOrEqual(0);
    errors.sort((x, y) => x - y);
    // Node's timers on the Windows and macOS runners wake ~10 ms late (Windows' tick is 15.6 ms); Linux keeps 1 ms.
    expect(errors[10]!).toBeLessThan(process.platform === "linux" ? 0.001 : 0.016);
  });
});

describe("RealClock.sleep(0)", () => {
  it("lets the event loop run", async () => {
    const clock = new RealClock();
    let ran = false;
    setImmediate(() => (ran = true));
    await clock.sleep(0);
    expect(ran).toBe(true);
  });
});
