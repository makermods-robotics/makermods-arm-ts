/**
 * Small dense linear algebra for kinematics and IK, in the operation order of the numpy build the
 * Python SDK runs on, so the same inputs round the same way.
 *
 * numpy's `@` on float64 goes to OpenBLAS, whose kernels accumulate each dot product with fused
 * multiply-adds from the first term: `fma(a[k-1], b[k-1], ... fma(a[1], b[1], a[0] * b[0]))`
 * (`dotFma`). `np.einsum` over a length-3 axis uses two SIMD lanes: `fma(a2, b2, a0 * b0) + a1 * b1`
 * (`dotEinsum3`). A 1-D `a @ b` (`ddot`) and a matrix times a vector (`gemvRow`) follow the lane
 * layouts of OpenBLAS's ARMv8 kernels. JavaScript has no FMA instruction, so `fma` emulates a
 * correctly rounded one (Dekker's exact product, then the tail rounded to odd as Boldo and
 * Melquiond describe).
 *
 * Matrices are row-major `number[][]` at the API and `Float64Array` (row-major, explicit sizes)
 * inside the solvers.
 */

import { run, type Work } from "./cooperate.js";

// ── exact fused multiply-add ──
const SPLIT = 134217729; // 2^27 + 1
const F64 = new Float64Array(1);
const U32 = new Uint32Array(F64.buffer);
const LITTLE_ENDIAN = new Uint8Array(new Uint16Array([1]).buffer)[0] === 1;
const LO = LITTLE_ENDIAN ? 0 : 1;
const HI = LITTLE_ENDIAN ? 1 : 0;
const SAFE_MAX = 2 ** 400;
const SAFE_MIN = 2 ** -400;

/** x + y rounded to odd: exact when representable, else the neighbour with an odd last bit. */
function addRoundOdd(x: number, y: number): number {
  const s = x + y;
  const bp = s - x;
  const e = (x - (s - bp)) + (y - bp);
  if (e === 0 || !Number.isFinite(s)) return s;
  F64[0] = s;
  if (U32[LO]! & 1) return s;
  // Step one unit in the last place toward the exact sum.
  const away = (e > 0) === (s > 0);
  let lo = U32[LO]!;
  let hi = U32[HI]!;
  if (away) {
    lo = (lo + 1) >>> 0;
    if (lo === 0) hi = (hi + 1) >>> 0;
  } else {
    if (lo === 0) hi = (hi - 1) >>> 0;
    lo = (lo - 1) >>> 0;
  }
  U32[LO] = lo;
  U32[HI] = hi;
  return F64[0]!;
}

function decompose(x: number): [bigint, number] {
  // x = m * 2^e exactly, m an integer (sign included).
  F64[0] = x;
  const hi = U32[HI]!;
  const lo = U32[LO]!;
  const exp = (hi >>> 20) & 0x7ff;
  let m = (BigInt(hi & 0xfffff) << 32n) | BigInt(lo);
  let e: number;
  if (exp === 0) {
    e = -1074;
  } else {
    m |= 1n << 52n;
    e = exp - 1075;
  }
  return [hi >>> 31 ? -m : m, e];
}

function bigToDouble(m: bigint, e: number): number {
  // Round m * 2^e to the nearest double, ties to even.
  if (m === 0n) return 0;
  const neg = m < 0n;
  let a = neg ? -m : m;
  const bits = a.toString(2).length;
  // Keep 53 significant bits, or fewer where the result is subnormal.
  let shift = bits - 53;
  if (e + shift < -1074) shift = -1074 - e;
  if (shift > 0) {
    const half = 1n << BigInt(shift - 1);
    const rem = a & ((1n << BigInt(shift)) - 1n);
    a >>= BigInt(shift);
    if (rem > half || (rem === half && (a & 1n) === 1n)) a += 1n;
    e += shift;
  }
  const v = Number(a) * 2 ** e;
  const r = Number.isFinite(v) ? v : Infinity;
  return neg ? -r : r;
}

function fmaExact(a: number, b: number, c: number): number {
  const [ma, ea] = decompose(a);
  const [mb, eb] = decompose(b);
  const [mc, ec] = decompose(c);
  const mp = ma * mb;
  const ep = ea + eb;
  const e = Math.min(ep, ec);
  const sum = (mp << BigInt(ep - e)) + (mc << BigInt(ec - e));
  if (sum === 0n) return a * b + c; // exact zero: IEEE sign rules
  return bigToDouble(sum, e);
}

/** Correctly rounded a * b + c (IEEE-754 fusedMultiplyAdd), as the FMA instruction computes it. */
export function fma(a: number, b: number, c: number): number {
  const p = a * b;
  if (!Number.isFinite(p) || !Number.isFinite(c)) {
    if (!Number.isFinite(a) || !Number.isFinite(b)) return p + c;
    // a * b is finite in exact arithmetic, even when its rounding overflows.
    return Number.isFinite(c) ? fmaExact(a, b, c) : c;
  }
  if (a === 0 || b === 0) return p + c;
  const aa = Math.abs(a);
  const ab = Math.abs(b);
  const ac = Math.abs(c);
  if (aa > SAFE_MAX || aa < SAFE_MIN || ab > SAFE_MAX || ab < SAFE_MIN
      || (c !== 0 && (ac > SAFE_MAX * SAFE_MAX || ac < SAFE_MIN * SAFE_MIN))) {
    return fmaExact(a, b, c);
  }
  if (c === 0) return p + c;
  // Exact product (Dekker), exact sum with c, then the tail rounded to odd (Boldo & Melquiond).
  let t = SPLIT * a;
  const ah = t - (t - a);
  const al = a - ah;
  t = SPLIT * b;
  const bh = t - (t - b);
  const bl = b - bh;
  const pl = ((ah * bh - p) + ah * bl + al * bh) + al * bl;
  const sh = p + c;
  const bp = sh - p;
  const sl = (p - (sh - bp)) + (c - bp);
  return sh + addRoundOdd(sl, pl);
}

// ── dot products in numpy's order ──
// The BLAS kernels and einsum accumulate from +0, so a zero sum is never -0: each result below is
// normalized with `+ 0` (which changes nothing else).

/** numpy `@` / BLAS order over three terms. */
export function dot3(a0: number, a1: number, a2: number, b0: number, b1: number, b2: number): number {
  return fma(a2, b2, fma(a1, b1, a0 * b0)) + 0;
}

/** numpy `@` / BLAS order: fma chain from the first term. */
export function dotFma(a: ArrayLike<number>, b: ArrayLike<number>, n: number, aOff = 0, aStride = 1,
                       bOff = 0, bStride = 1): number {
  let s = a[aOff]! * b[bOff]!;
  for (let k = 1; k < n; k++) s = fma(a[aOff + k * aStride]!, b[bOff + k * bStride]!, s);
  return s + 0;
}

/** `np.einsum` order over a length-3 axis. */
export function dotEinsum3(a0: number, a1: number, a2: number, b0: number, b1: number, b2: number): number {
  return fma(a2, b2, a0 * b0 + 0) + (a1 * b1 + 0);
}

/** 3x3 (row-major, 9 values) times 3x3, numpy `@` order. */
export function mat3Mul(a: ArrayLike<number>, b: ArrayLike<number>, out = new Float64Array(9)): Float64Array {
  for (let i = 0; i < 3; i++) {
    const a0 = a[3 * i]!, a1 = a[3 * i + 1]!, a2 = a[3 * i + 2]!;
    for (let j = 0; j < 3; j++) out[3 * i + j] = dot3(a0, a1, a2, b[j]!, b[3 + j]!, b[6 + j]!);
  }
  return out;
}

/** 3x3 times a 3-vector, numpy `@` order. */
export function mat3Vec(a: ArrayLike<number>, v: ArrayLike<number>, out = new Float64Array(3), vOff = 0): Float64Array {
  const v0 = v[vOff]!, v1 = v[vOff + 1]!, v2 = v[vOff + 2]!;
  for (let i = 0; i < 3; i++) out[i] = dot3(a[3 * i]!, a[3 * i + 1]!, a[3 * i + 2]!, v0, v1, v2);
  return out;
}

/** 4x4 times 4x4 (`number[][]`), numpy `@` order. */
export function mat4Mul(a: number[][], b: number[][]): number[][] {
  const out: number[][] = [];
  for (let i = 0; i < 4; i++) {
    const ai = a[i]!;
    const row: number[] = [];
    for (let j = 0; j < 4; j++) {
      let s = ai[0]! * b[0]![j]!;
      for (let k = 1; k < 4; k++) s = fma(ai[k]!, b[k]![j]!, s);
      row.push(s + 0);
    }
    out.push(row);
  }
  return out;
}

/**
 * 1-D `a @ b` of contiguous vectors (BLAS ddot, ARMv8 kernel): two lanes over blocks of four
 * (multiply, then add), their sum, then a fused tail.
 */
export function ddot(a: ArrayLike<number>, b: ArrayLike<number>, n: number, aOff = 0, bOff = 0): number {
  let i = 0;
  let d = 0.0;
  if (n >= 4) {
    let l0 = 0.0;
    let l1 = 0.0;
    for (; i + 4 <= n; i += 4) {
      l0 = (l0 + a[aOff + i]! * b[bOff + i]!) + a[aOff + i + 2]! * b[bOff + i + 2]!;
      l1 = (l1 + a[aOff + i + 1]! * b[bOff + i + 1]!) + a[aOff + i + 3]! * b[bOff + i + 3]!;
    }
    d = l0 + l1;
  }
  for (; i < n; i++) d = fma(a[aOff + i]!, b[bOff + i]!, d);
  return d;
}

/**
 * One row of a row-major matrix times a contiguous vector (BLAS gemv_t, ARMv8 kernel): eight
 * fused lanes over the first multiple of eight columns, summed pairwise, then a fused tail. Only
 * columns [from, to) are visited; the others must hold zeros.
 */
export function gemvRow(a: ArrayLike<number>, aOff: number, x: ArrayLike<number>, m: number,
                        from = 0, to = m): number {
  const full = m - (m % 8);
  const lanes = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
  let e = from;
  for (; e < to && e < full; e++) {
    const v = a[aOff + e]!;
    if (v !== 0) lanes[e & 7] = fma(v, x[e]!, lanes[e & 7]!);
  }
  const [l0, l1, l2, l3, l4, l5, l6, l7] = lanes as [number, number, number, number, number, number, number, number];
  let t = ((l0 + l1) + (l2 + l3)) + ((l4 + l5) + (l6 + l7));
  for (e = Math.max(e, full); e < to; e++) t = fma(a[aOff + e]!, x[e]!, t);
  return t;
}

/** `np.linalg.norm` of a vector: sqrt of the BLAS dot product with itself. */
export function norm(v: ArrayLike<number>, n = v.length, off = 0): number {
  if (n === 0) return 0;
  return Math.sqrt(ddot(v, v, n, off, off));
}

/** `np.linalg.norm(x, axis=-1)` of a 3-vector: the squares summed in order, no BLAS. */
export function norm3(x: number, y: number, z: number): number {
  return Math.sqrt((x * x + y * y) + z * z);
}

// ── elementwise and reductions in numpy's semantics ──
/** np.clip of one value. */
export function clip(v: number, lo: number, hi: number): number {
  return npMinimum(npMaximum(v, lo), hi);
}

/** np.minimum / np.maximum of two values: NaN propagates. */
export function npMinimum(a: number, b: number): number {
  return Number.isNaN(a) || Number.isNaN(b) ? NaN : a < b ? a : b;
}
export function npMaximum(a: number, b: number): number {
  return Number.isNaN(a) || Number.isNaN(b) ? NaN : a > b ? a : b;
}

/** np.sum over a contiguous float64 array: numpy's pairwise summation. */
export function pairwiseSum(a: ArrayLike<number>, off = 0, n = a.length - off): number {
  if (n < 8) {
    let res = 0.0;
    for (let i = 0; i < n; i++) res += a[off + i]!;
    return res;
  }
  if (n <= 128) {
    const r = new Float64Array(8);
    for (let j = 0; j < 8; j++) r[j] = a[off + j]!;
    let i = 8;
    for (; i < n - (n % 8); i += 8) for (let j = 0; j < 8; j++) r[j]! += a[off + i + j]!;
    let res = ((r[0]! + r[1]!) + (r[2]! + r[3]!)) + ((r[4]! + r[5]!) + (r[6]! + r[7]!));
    for (; i < n; i++) res += a[off + i]!;
    return res;
  }
  let n2 = Math.floor(n / 2);
  n2 -= n2 % 8;
  return pairwiseSum(a, off, n2) + pairwiseSum(a, off + n2, n - n2);
}


// ── solvers (LAPACK equivalents; agree with numpy to rounding, not bit for bit) ──
/** Lower and upper bandwidth of the nonzeros of the n x n row-major `a`. */
export function bandwidth(a: Float64Array, n: number): [number, number] {
  let kl = 0;
  let ku = 0;
  for (let i = 0; i < n; i++) {
    const r = i * n;
    for (let j = 0; j < i - kl; j++) if (a[r + j] !== 0) { kl = i - j; break; }
    for (let j = n - 1; j > i + ku; j--) if (a[r + j] !== 0) { ku = j - i; break; }
  }
  return [kl, ku];
}

/**
 * LU factorization with partial pivoting of the n x n row-major `a`, in place (multipliers below
 * the diagonal, rows interchanged as LAPACK's getrf leaves them). Returns the pivots. Elimination
 * stays inside the band of nonzeros, which gives the same result as the dense loops.
 */
export function luFactor(a: Float64Array, n: number): Int32Array {
  return run(luFactorWork(a, n));
}

/** Columns of elimination between yields in `luFactorWork`. */
const LU_COLUMNS_PER_YIELD = 32;

/** `luFactor` as cooperative work (see cooperate.ts): the same loop, yielding every few columns. */
export function* luFactorWork(a: Float64Array, n: number): Work<Int32Array> {
  const [kl, ku] = bandwidth(a, n);
  yield 0.003 * n * n;
  const piv = new Int32Array(n);
  for (let k = 0; k < n; k++) {
    const rowEnd = Math.min(n, k + kl + 1);
    const colEnd = Math.min(n, k + kl + ku + 1);
    let p = k;
    let best = Math.abs(a[k * n + k]!);
    for (let i = k + 1; i < rowEnd; i++) {
      const v = Math.abs(a[i * n + k]!);
      if (v > best) {
        best = v;
        p = i;
      }
    }
    piv[k] = p;
    if (p !== k) {
      for (let j = 0; j < n; j++) {
        const t = a[k * n + j]!;
        a[k * n + j] = a[p * n + j]!;
        a[p * n + j] = t;
      }
    }
    const d = a[k * n + k]!;
    if (d === 0) throw new LinAlgError("Singular matrix");
    const rk = k * n;
    for (let i = k + 1; i < rowEnd; i++) {
      const ri = i * n;
      const f = (a[ri + k]! /= d);
      if (f !== 0) for (let j = k + 1; j < colEnd; j++) a[ri + j]! -= f * a[rk + j]!;
    }
    if ((k + 1) % LU_COLUMNS_PER_YIELD === 0) yield 0.02 * LU_COLUMNS_PER_YIELD * (kl + 1) * (kl + ku + 1) + 0.01 * n * LU_COLUMNS_PER_YIELD;
  }
  return piv;
}

/** Solve with an LU factorization from `luFactor`; `b` (n values) is overwritten with x. */
export function luSolve(lu: Float64Array, piv: Int32Array, n: number, b: Float64Array): Float64Array {
  for (let k = 0; k < n; k++) {
    const p = piv[k]!;
    if (p !== k) {
      const t = b[k]!;
      b[k] = b[p]!;
      b[p] = t;
    }
  }
  for (let i = 1; i < n; i++) {
    let s = b[i]!;
    const ri = i * n;
    for (let j = 0; j < i; j++) s -= lu[ri + j]! * b[j]!;
    b[i] = s;
  }
  for (let i = n - 1; i >= 0; i--) {
    let s = b[i]!;
    const ri = i * n;
    for (let j = i + 1; j < n; j++) s -= lu[ri + j]! * b[j]!;
    b[i] = s / lu[ri + i]!;
  }
  return b;
}

/** `np.linalg.solve(A, b)` for a row-major n x n `a` (not modified). */
export function solve(a: Float64Array, n: number, b: Float64Array): Float64Array {
  return run(solveWork(a, n, b));
}

/** `solve` as cooperative work (see cooperate.ts). */
export function* solveWork(a: Float64Array, n: number, b: Float64Array): Work<Float64Array> {
  const lu = Float64Array.from(a);
  const piv = yield* luFactorWork(lu, n);
  const x = luSolve(lu, piv, n, Float64Array.from(b));
  yield 0.006 * n * n;
  return x;
}

/** `np.linalg.inv` of a row-major n x n `a`. */
export function inv(a: Float64Array, n: number): Float64Array {
  const lu = Float64Array.from(a);
  const piv = luFactor(lu, n);
  const out = new Float64Array(n * n);
  const col = new Float64Array(n);
  for (let j = 0; j < n; j++) {
    col.fill(0);
    col[j] = 1;
    luSolve(lu, piv, n, col);
    for (let i = 0; i < n; i++) out[i * n + j] = col[i]!;
  }
  return out;
}

/**
 * `V @ np.linalg.pinv(V)` for a full-column-rank m x k `v` (m >= k): the orthogonal projection onto
 * its column space, computed as Q Q^T from a Householder QR.
 */
export function projection(v: number[][]): number[][] {
  const m = v.length;
  const k = m ? v[0]!.length : 0;
  const a = v.map((r) => r.slice());
  const vs: number[][] = [];
  for (let j = 0; j < k; j++) {
    let s = 0;
    for (let i = j; i < m; i++) s += a[i]![j]! * a[i]![j]!;
    const alpha = a[j]![j]! > 0 ? -Math.sqrt(s) : Math.sqrt(s);
    const h = new Array<number>(m).fill(0);
    for (let i = j; i < m; i++) h[i] = a[i]![j]!;
    h[j]! -= alpha;
    let hn = 0;
    for (let i = j; i < m; i++) hn += h[i]! * h[i]!;
    if (hn > 0) {
      for (let c = j; c < k; c++) {
        let d = 0;
        for (let i = j; i < m; i++) d += h[i]! * a[i]![c]!;
        const f = (2 * d) / hn;
        for (let i = j; i < m; i++) a[i]![c]! -= f * h[i]!;
      }
    }
    vs.push(h.map((x) => x / Math.sqrt(hn || 1)));
  }
  // Q's first k columns: apply the reflections to e_0..e_{k-1}.
  const q: number[][] = Array.from({ length: m }, () => new Array<number>(k).fill(0));
  for (let c = 0; c < k; c++) {
    const e = new Array<number>(m).fill(0);
    e[c] = 1;
    for (let j = k - 1; j >= 0; j--) {
      const h = vs[j]!;
      let d = 0;
      for (let i = j; i < m; i++) d += h[i]! * e[i]!;
      for (let i = j; i < m; i++) e[i]! -= 2 * d * h[i]!;
    }
    for (let i = 0; i < m; i++) q[i]![c] = e[i]!;
  }
  return Array.from({ length: m }, (_, i) =>
    Array.from({ length: m }, (_, j) => {
      let s = 0;
      for (let c = 0; c < k; c++) s += q[i]![c]! * q[j]![c]!;
      return s;
    }));
}

/** numpy's LinAlgError: a singular matrix in a solve. */
export class LinAlgError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "LinAlgError";
  }
}
