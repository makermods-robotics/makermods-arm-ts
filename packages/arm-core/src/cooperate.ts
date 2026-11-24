/**
 * Cooperative computation: a long solve written once as a generator that yields estimated work
 * (microseconds of computation on a Jetson Orin class CPU) at fixed points. `run` drains it in one
 * go; `runAsync` drains the same generator but gives the event loop a turn whenever about `SLICE`
 * of work has accumulated, so receive callbacks and the control loop keep running. Both execute the
 * same arithmetic in the same order, so they return the same numbers. Pausing depends only on the
 * yielded estimates, never on a clock.
 */

/** A computation that yields work estimates (µs) and returns T. */
export type Work<T> = Generator<number, T, void>;

/** Options of the asynchronous (cooperative) solves. */
export interface CooperativeOptions {
  /** Gives the event loop a turn (default: a macrotask, so I/O callbacks run). */
  pause?: () => Promise<void>;
  /** Checked at every pause; once aborted the solve rejects with the signal's reason. */
  signal?: AbortSignal | null;
}

/** Work (µs) between pauses. */
export const SLICE = 2000;

type Yield = () => Promise<void>;

function makeYield(): Yield {
  const g = globalThis as { setImmediate?: (fn: () => void) => unknown; MessageChannel?: typeof MessageChannel };
  if (typeof g.setImmediate === "function") {
    const setImmediate = g.setImmediate;
    return () => new Promise<void>((resolve) => setImmediate(resolve));
  }
  if (typeof g.MessageChannel === "function") {
    // A message round trip is a macrotask that is not clamped like nested timers.
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

let defaultPause: Yield | null = null;

/** The default pause: `setImmediate` in Node, a `MessageChannel` round trip in browsers. */
export function macrotask(): Promise<void> {
  defaultPause ??= makeYield();
  return defaultPause();
}

/** Runs `work` to the end without pausing. */
export function run<T>(work: Work<T>): T {
  for (;;) {
    const step = work.next();
    if (step.done) return step.value;
  }
}

function checkAbort(signal: AbortSignal | null | undefined): void {
  if (signal?.aborted) throw signal.reason;
}

/** Runs `work` to the end, pausing every `SLICE` µs of estimated work. */
export async function runAsync<T>(work: Work<T>, options: CooperativeOptions = {}): Promise<T> {
  const pause = options.pause ?? macrotask;
  const signal = options.signal ?? null;
  checkAbort(signal);
  let spent = 0;
  try {
    for (;;) {
      const step = work.next();
      if (step.done) return step.value;
      spent += step.value;
      if (spent >= SLICE) {
        spent = 0;
        await pause();
        checkAbort(signal);
      }
    }
  } finally {
    work.return(undefined as never);
  }
}
