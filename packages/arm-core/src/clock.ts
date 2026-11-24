/**
 * Time source for everything that waits. Hardware runs on RealClock; tests run on the VirtualClock
 * from "@makermods/arm-core/testing". Times are monotonic seconds.
 */
export interface Clock {
  now(): number;
  /** Resolves after `seconds`, never earlier. */
  sleep(seconds: number): Promise<void>;
  /** Calls `fn` once after `seconds`; the returned function cancels it. */
  timer(seconds: number, fn: () => void): () => void;
}

type Yield = () => Promise<void>;

function makeYield(): Yield {
  const g = globalThis as { setImmediate?: (fn: () => void) => unknown; MessageChannel?: typeof MessageChannel };
  if (typeof g.setImmediate === "function") {
    const setImmediate = g.setImmediate;
    return () => new Promise<void>((resolve) => setImmediate(resolve));
  }
  if (typeof g.MessageChannel === "function") {
    // Browser timers are clamped to 4 ms once nested; a message round trip is not.
    const channel = new g.MessageChannel();
    const waiting: Array<() => void> = [];
    channel.port1.onmessage = () => waiting.shift()?.();
    return () =>
      new Promise<void>((resolve) => {
        waiting.push(resolve);
        channel.port2.postMessage(0);
      });
  }
  return () => new Promise<void>((resolve) => setTimeout(resolve, 0));
}

export interface RealClockOptions {
  /**
   * The last `spinMs` of every sleep are spent yielding to the event loop instead of in a timer,
   * because timers fire up to a few milliseconds late. 0 trades precision for an idle CPU.
   */
  spinMs?: number;
}

export class RealClock implements Clock {
  readonly spinMs: number;
  #yield: Yield = makeYield();

  constructor(options: RealClockOptions = {}) {
    this.spinMs = options.spinMs ?? 1.5;
  }

  now(): number {
    return performance.now() / 1000;
  }

  timer(seconds: number, fn: () => void): () => void {
    const handle = setTimeout(fn, Math.max(0, seconds * 1000));
    return () => clearTimeout(handle);
  }

  /** Always gives the event loop at least one turn, sleep(0) included, so received frames are handled. */
  async sleep(seconds: number): Promise<void> {
    const deadline = this.now() + seconds;
    const coarse = seconds * 1000 - this.spinMs;
    if (coarse >= 1 || this.spinMs <= 0) {
      await new Promise<void>((resolve) => setTimeout(resolve, Math.max(0, coarse)));
    } else {
      await this.#yield();
    }
    while (this.now() < deadline) await this.#yield();
  }
}

/**
 * Wakes waiters when something changed (a frame arrived, a tick ran). Replaces the Python SDK's
 * short polling sleeps: `await signal.wait(() => seq !== before, timeout, clock)`.
 */
export class Signal {
  #waiters = new Set<() => void>();

  notify(): void {
    for (const check of [...this.#waiters]) check();
  }

  /** True as soon as `predicate()` holds (now, or after a notify); false once `timeout` seconds passed. */
  wait(predicate: () => boolean, timeout: number, clock: Clock): Promise<boolean> {
    if (predicate()) return Promise.resolve(true);
    if (!(timeout > 0)) return Promise.resolve(false);
    return new Promise<boolean>((resolve) => {
      let cancel: () => void = () => {};
      const done = (ok: boolean) => {
        this.#waiters.delete(check);
        cancel();
        resolve(ok);
      };
      const check = () => {
        if (predicate()) done(true);
      };
      this.#waiters.add(check);
      cancel = clock.timer(timeout, () => done(predicate()));
    });
  }
}

/** One holder at a time across awaits: the port of a Python lock that is held while waiting on the bus. */
export class Mutex {
  #tail: Promise<void> = Promise.resolve();

  async run<T>(fn: () => Promise<T> | T): Promise<T> {
    const previous = this.#tail;
    let release!: () => void;
    this.#tail = new Promise<void>((resolve) => (release = resolve));
    await previous;
    try {
      return await fn();
    } finally {
      release();
    }
  }
}
