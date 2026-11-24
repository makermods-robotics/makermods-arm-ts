/** The bus, driver, calibration and identify code is lifted from the console, which counts time in milliseconds and
sleeps with an AbortSignal. MsClock is that view of the injected seconds Clock, so the lifted code keeps its
constants (`REPLY_TIMEOUT_MS`, `MAX_MISSED_MS`, ...) and its arithmetic unchanged. */

import type { Clock } from "@makermods/arm-core";

/** Milliseconds on the injected clock, and a sleep an AbortSignal cuts short (resolving, never rejecting). */
export interface MsClock {
  now(): number;
  sleep(ms: number, signal?: AbortSignal): Promise<void>;
}

const views = new WeakMap<Clock, MsClock>();

/** The millisecond view of `clock` (one per clock). */
export function msClock(clock: Clock): MsClock {
  let view = views.get(clock);
  if (!view) {
    view = {
      now: () => clock.now() * 1000,
      sleep: (ms, signal) =>
        new Promise<void>((resolve) => {
          if (signal?.aborted) return resolve();
          const done = () => {
            signal?.removeEventListener("abort", done);
            cancel();
            resolve();
          };
          const cancel = clock.timer(Math.max(0, ms) / 1000, done);
          signal?.addEventListener("abort", done, { once: true });
        }),
    };
    views.set(clock, view);
  }
  return view;
}
