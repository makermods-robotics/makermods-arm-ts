/** Test support for the math modules: contract fixtures from tools/gen_math_fixtures.py, a stand-in Product, ULP distance. */

import { readFileSync } from "node:fs";
import { ArmConfig, type Product, type RawProfile } from "@makermods/arm-core";

const FIXTURES = new URL("../../../../contract/fixtures/", import.meta.url);

/** Strings the generator writes for non-finite floats. */
const SPECIAL: Record<string, number> = { inf: Infinity, "-inf": -Infinity, nan: NaN };

// eslint-disable-next-line @typescript-eslint/no-explicit-any
export type Json = any;

/** A contract fixture, non-finite floats revived. */
export function loadFixture(name: string): Json {
  const text = readFileSync(new URL(name, FIXTURES), "utf8");
  return JSON.parse(text, (key, value) =>
    typeof value === "string" && key !== "urdf" && Object.hasOwn(SPECIAL, value) ? SPECIAL[value] : value);
}

export const ARMS = ["maker_arm", "metal_arm"] as const;
export type ArmName = (typeof ARMS)[number];

/** The generator's per-arm file. */
export function armFixture(arm: ArmName): Json {
  return loadFixture(`math_${arm}.json`);
}

interface ProductInfo {
  models: string[];
  default_model: string;
  kp_range: [number, number];
  kd_range: [number, number];
  id_range: [number, number];
  default_host_id: number;
}

/** A Product with the limits profile validation reads; everything that talks to motors throws. */
export function fakeProduct(info: ProductInfo, profile: RawProfile = {}): Product {
  const unsupported = (): never => {
    throw new Error("fake product: no motors");
  };
  return {
    packageName: "@makermods/fake-arm",
    displayName: "Fake Arm",
    motorVendor: "none",
    zeroPose: "anywhere",
    models: info.models,
    defaultModel: info.default_model,
    kpRange: info.kp_range,
    kdRange: info.kd_range,
    idRange: info.id_range,
    defaultHostId: info.default_host_id,
    route: () => null,
    createMotor: unsupported,
    defaultProfile: profile,
    defaultStarMapping: { joints: [] },
    dreamEmbodiment: "fake",
    dreamJointSign: [],
    dreamJointOffset: [],
  };
}

/** The arm's shipped profile as an ArmConfig, from its fixture. */
export function configOf(fx: Json, changes: Record<string, unknown> = {}): ArmConfig {
  const raw = structuredClone({ ...fx.profile, ...changes }) as Record<string, unknown>;
  return ArmConfig.fromRaw(raw, fakeProduct(fx.product, fx.profile));
}

const F = new Float64Array(1);
const I = new BigInt64Array(F.buffer);

function ordered(x: number): bigint {
  F[0] = x;
  const v = I[0]!;
  return v < 0n ? -(v & 0x7fffffffffffffffn) : v;
}

/** Distance in units in the last place (0 = same bits, -0 and +0 count as 1 apart; NaN = NaN counts as 0). */
export function ulps(a: number, b: number): number {
  if (Number.isNaN(a) && Number.isNaN(b)) return 0;
  if (Number.isNaN(a) || Number.isNaN(b)) return Infinity;
  if (Object.is(a, b)) return 0;
  if (a === 0 && b === 0) return 1;
  const d = ordered(a) - ordered(b);
  return Number(d < 0n ? -d : d);
}

/** One unit in the last place at |x|. */
export function ulpOf(x: number): number {
  const a = Math.abs(x);
  if (!Number.isFinite(a)) return NaN;
  if (a < 2 ** -1022) return 2 ** -1074;
  return 2 ** (Math.floor(Math.log2(a)) - 52) * (2 ** Math.floor(Math.log2(a)) > a ? 0.5 : 1);
}

/** Differences between nested arrays of numbers. */
export interface Diff {
  /** Largest element-wise ULP distance. */
  ulps: number;
  /** Largest absolute difference. */
  abs: number;
  /** Largest absolute difference in ULPs of the largest |value| of `b` (the result's scale). */
  scaled: number;
}

/** Largest ULP distance, absolute difference and scale-relative ULPs between two nested arrays of numbers. */
export function compare(a: unknown, b: unknown): Diff {
  let worst = 0;
  let abs = 0;
  let scale = 0;
  const walk = (x: unknown, y: unknown): void => {
    if (Array.isArray(x) && Array.isArray(y)) {
      if (x.length !== y.length) throw new Error(`length ${x.length} != ${y.length}`);
      x.forEach((v, i) => walk(v, y[i]));
    } else if (typeof x === "number" && typeof y === "number") {
      worst = Math.max(worst, ulps(x, y));
      if (Number.isFinite(y)) scale = Math.max(scale, Math.abs(y));
      if (!(Number.isNaN(x) && Number.isNaN(y)) && x !== y) abs = Math.max(abs, Math.abs(x - y));
    } else if (typeof x === "boolean" && typeof y === "boolean") {
      if (x !== y) worst = Infinity;
    } else {
      throw new Error(`cannot compare ${typeof x} with ${typeof y}`);
    }
  };
  walk(a, b);
  return { ulps: worst, abs, scaled: abs === 0 ? 0 : abs / ulpOf(scale || Number.MIN_VALUE) };
}

/** Records the worst difference seen per label, printed by `report()`. */
export class Tally {
  readonly worst = new Map<string, Diff>();

  add(label: string, d: Diff): void {
    const prev = this.worst.get(label) ?? { ulps: 0, abs: 0, scaled: 0 };
    this.worst.set(label, { ulps: Math.max(prev.ulps, d.ulps), abs: Math.max(prev.abs, d.abs), scaled: Math.max(prev.scaled, d.scaled) });
  }

  report(title: string): void {
    const rows = [...this.worst].map(([k, v]) => `  ${k.padEnd(40)} element ${String(v.ulps).padStart(20)} ulp   `
      + `of scale ${v.scaled.toFixed(1).padStart(14)} ulp   |diff| ${v.abs.toExponential(2)}`);
    console.log(`${title}\n${rows.join("\n")}`);
  }
}
