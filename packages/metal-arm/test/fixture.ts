/** Helpers for the cross-language fixtures in contract/fixtures/. */

import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";

export function loadFixture<T = Record<string, unknown>>(name: string): T {
  const path = fileURLToPath(new URL(`../../../contract/fixtures/${name}`, import.meta.url));
  return JSON.parse(readFileSync(path, "utf8")) as T;
}

/** A fixture float: NaN and ±inf are written as strings. */
export function num(x: unknown): number {
  if (x === "nan") return NaN;
  if (x === "inf") return Infinity;
  if (x === "-inf") return -Infinity;
  return x as number;
}

/** A float for comparison with a fixture value. */
export function enc(x: number): number | string {
  if (Number.isNaN(x)) return "nan";
  if (x === Infinity) return "inf";
  if (x === -Infinity) return "-inf";
  return x;
}

export const hex = (b: Uint8Array): string => Array.from(b, (x) => x.toString(16).padStart(2, "0")).join("");

export function unhex(s: string): Uint8Array {
  const out = new Uint8Array(s.length / 2);
  for (let i = 0; i < out.length; i++) out[i] = parseInt(s.slice(2 * i, 2 * i + 2), 16);
  return out;
}

export type Outcome = { ok: unknown } | { error: string; message: string };

/** Run `fn` as the fixture recorded it: {ok: result} or {error, message}. */
export function outcome(fn: () => unknown, conv: (v: never) => unknown = (v) => v): Outcome {
  try {
    return { ok: conv(fn() as never) };
  } catch (e) {
    return { error: (e as Error).name, message: (e as Error).message };
  }
}

/**
 * Compare with a recorded outcome. Errors match by "it threw"; a Python ValueError must also carry
 * the same message (other Python exception classes have no TypeScript counterpart).
 */
export function sameOutcome(actual: Outcome, expected: Outcome): boolean {
  if ("ok" in expected) return "ok" in actual && JSON.stringify(actual.ok) === JSON.stringify(expected.ok);
  if (!("error" in actual)) return false;
  return expected.error !== "ValueError" || (actual.error === "ValueError" && actual.message === expected.message);
}
