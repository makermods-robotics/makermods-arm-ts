import { describe, expect, it } from "vitest";
import {
  bandwidth, ddot, dot3, dotEinsum3, dotFma, fma, gemvRow, inv, LinAlgError, luFactor, luSolve, mat3Mul, norm, projection, solve,
} from "../src/linalg.js";
import { type Json, loadFixture } from "./helpers/math-fixtures.js";

const fmaCases: Json = loadFixture("math_fma.json");

function rng(seed: number): () => number {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296 - 0.5;
  };
}

describe("fma: correctly rounded fused multiply-add", () => {
  it("matches exact rational arithmetic", () => {
    const bad = (fmaCases.fma as number[][]).filter(([a, b, c, r]) => !Object.is(fma(a!, b!, c!), r));
    expect(bad).toEqual([]);
  });

  it("keeps IEEE semantics for zeros and non-finite values", () => {
    expect(Object.is(fma(-0, 1, 0), 0)).toBe(true);
    expect(Object.is(fma(-0, 1, -0), -0)).toBe(true);
    expect(fma(Infinity, 0, 1)).toBeNaN();
    expect(fma(Infinity, 1, -Infinity)).toBeNaN();
    expect(fma(1e308, 10, -Infinity)).toBe(-Infinity);
    expect(fma(1e308, 10, -1e308)).toBe(Infinity);
    // The product overflows a double, the exact sum does not.
    expect(fma(2 ** 512, 2 ** 512, -Number.MAX_VALUE)).toBe(2 ** 971);
    // Subnormal results.
    expect(fma(2 ** -540, 2 ** -540, 0)).toBe(0);
    expect(fma(2 ** -537, 2 ** -537, 0)).toBe(2 ** -1074);
  });

  it("differs from a * b + c where the product's rounding matters", () => {
    const a = 1 + 2 ** -30;
    expect(fma(a, a, -1)).not.toBe(a * a - 1);
    expect(fma(a, a, -1)).toBe(2 ** -29 + 2 ** -60);
  });
});

describe("BLAS-order dot products", () => {
  const r = rng(1);
  const vec = (n: number) => Float64Array.from({ length: n }, () => r() * 10);

  it("ddot and gemvRow reduce to the fused chain on short vectors", () => {
    for (let n = 1; n < 8; n++) {
      const a = vec(n), b = vec(n);
      if (n < 4) expect(ddot(a, b, n)).toBe(dotFma(a, b, n));
      expect(gemvRow(a, 0, b, n)).toBe(dotFma(a, b, n));
    }
  });

  it("gemvRow visiting only a band equals the full row when the rest is zero", () => {
    for (const m of [9, 17, 40, 77]) {
      const row = new Float64Array(m);
      const x = vec(m);
      for (let j = 3; j < Math.min(m, 30); j++) row[j] = r();
      expect(gemvRow(row, 0, x, m, 3, Math.min(m, 30))).toBe(gemvRow(row, 0, x, m));
    }
  });

  it("is close to the exact dot product", () => {
    for (const n of [3, 4, 6, 9, 33]) {
      const a = vec(n), b = vec(n);
      let exact = 0;
      for (let i = 0; i < n; i++) exact += a[i]! * b[i]!;
      expect(Math.abs(ddot(a, b, n) - exact)).toBeLessThan(1e-12);
      expect(Math.abs(gemvRow(a, 0, b, n) - exact)).toBeLessThan(1e-12);
    }
    expect(norm([3, 4, 12])).toBe(13);
  });

  it("never returns -0 (BLAS and einsum accumulate from +0)", () => {
    expect(Object.is(dot3(-0, 0, 0, 1, 1, 1), 0)).toBe(true);
    expect(Object.is(dotEinsum3(-0, -0, -0, 1, 1, 1), 0)).toBe(true);
    expect(Object.is(dotFma([-1, 0], [0, 1], 2), 0)).toBe(true);
  });

  it("mat3Mul is the matrix product", () => {
    const a = vec(9), b = vec(9);
    const c = mat3Mul(a, b);
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        expect(c[3 * i + j]).toBeCloseTo(a[3 * i]! * b[j]! + a[3 * i + 1]! * b[3 + j]! + a[3 * i + 2]! * b[6 + j]!, 12);
      }
    }
  });
});

describe("solvers", () => {
  const r = rng(2);

  /** Dense LU with partial pivoting, the loops luFactor restricts to the band. */
  function denseLu(a: Float64Array, n: number): Int32Array {
    const piv = new Int32Array(n);
    for (let k = 0; k < n; k++) {
      let p = k;
      for (let i = k + 1; i < n; i++) if (Math.abs(a[i * n + k]!) > Math.abs(a[p * n + k]!)) p = i;
      piv[k] = p;
      for (let j = 0; j < n; j++) [a[k * n + j], a[p * n + j]] = [a[p * n + j]!, a[k * n + j]!];
      for (let i = k + 1; i < n; i++) {
        const f = (a[i * n + k]! /= a[k * n + k]!);
        if (f !== 0) for (let j = k + 1; j < n; j++) a[i * n + j]! -= f * a[k * n + j]!;
      }
    }
    return piv;
  }

  function banded(n: number, kl: number, ku: number): Float64Array {
    const a = new Float64Array(n * n);
    for (let i = 0; i < n; i++) for (let j = Math.max(0, i - kl); j <= Math.min(n - 1, i + ku); j++) a[i * n + j] = r() + (i === j ? 0.1 : 0);
    return a;
  }

  it("bandwidth finds the band of nonzeros", () => {
    expect(bandwidth(banded(20, 3, 5), 20)).toEqual([3, 5]);
    expect(bandwidth(new Float64Array(16), 4)).toEqual([0, 0]);
  });

  it("banded elimination gives the dense result bit for bit", () => {
    for (const [n, kl, ku] of [[30, 4, 4], [48, 18, 18], [12, 11, 11], [25, 2, 7]] as const) {
      const a = banded(n, kl, ku);
      const x = Float64Array.from(a);
      const y = Float64Array.from(a);
      expect(Array.from(luFactor(x, n))).toEqual(Array.from(denseLu(y, n)));
      expect(Array.from(x, (v) => v + 0)).toEqual(Array.from(y, (v) => v + 0)); // zeros outside the band may differ in sign
    }
  });

  it("solve and inv solve", () => {
    const n = 7;
    const a = banded(n, n - 1, n - 1);
    const b = Float64Array.from({ length: n }, () => r());
    const x = solve(a, n, b);
    for (let i = 0; i < n; i++) {
      let s = 0;
      for (let j = 0; j < n; j++) s += a[i * n + j]! * x[j]!;
      expect(s).toBeCloseTo(b[i]!, 12);
    }
    const ai = inv(a, n);
    for (let i = 0; i < n; i++) {
      for (let j = 0; j < n; j++) {
        let s = 0;
        for (let k = 0; k < n; k++) s += a[i * n + k]! * ai[k * n + j]!;
        expect(s).toBeCloseTo(i === j ? 1 : 0, 12);
      }
    }
    const lu = Float64Array.from(a);
    const piv = luFactor(lu, n);
    expect(Array.from(luSolve(lu, piv, n, Float64Array.from(b)))).toEqual(Array.from(x));
  });

  it("a singular matrix is refused", () => {
    expect(() => solve(new Float64Array([1, 2, 2, 4]), 2, new Float64Array([1, 1]))).toThrow(LinAlgError);
  });

  it("projection is V pinv(V): symmetric, idempotent, keeps the columns of V", () => {
    const V = Array.from({ length: 15 }, (_, i) => [1, i - 7, (i - 7) ** 2, (i - 7) ** 3]);
    const P = projection(V);
    for (let i = 0; i < 15; i++) {
      for (let j = 0; j < 15; j++) {
        expect(P[i]![j]).toBeCloseTo(P[j]![i]!, 14);
        let pp = 0;
        for (let k = 0; k < 15; k++) pp += P[i]![k]! * P[k]![j]!;
        expect(pp).toBeCloseTo(P[i]![j]!, 13);
      }
      for (let c = 0; c < 4; c++) {
        let pv = 0;
        for (let k = 0; k < 15; k++) pv += P[i]![k]! * V[k]![c]!;
        expect(pv).toBeCloseTo(V[i]![c]!, 10);
      }
    }
  });
});
