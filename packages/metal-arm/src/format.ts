/** Python number formatting and rounding, so messages and simulated values read as the Python SDK's do. */

/** Python's round(x) on a float: nearest integer, ties to even. */
export function roundHalfEven(x: number): number {
  if (!Number.isFinite(x)) return x;
  const r = Math.round(x);
  return Math.abs(x % 1) === 0.5 && r % 2 !== 0 ? r - 1 : r;
}

/** Python's round(x, 1). */
export function round1(x: number): number {
  if (!Number.isFinite(x)) return x;
  const y = x * 10;
  if (Number.isInteger(y * 2) && !Number.isInteger(y)) return roundHalfEven(y) / 10; // exact tie
  return Number(x.toFixed(1));
}

function digitsAndExponent(x: number): [string, number] {
  const [mantissa, exp] = Math.abs(x).toExponential().split("e");
  return [mantissa!.replace(".", ""), Number(exp)];
}

/** Python's repr() of a float: shortest round-trip digits, "1.0", "1e-05", "1.5e+16". */
export function reprFloat(x: number): string {
  if (Number.isNaN(x)) return "nan";
  if (!Number.isFinite(x)) return x > 0 ? "inf" : "-inf";
  const sign = x < 0 || Object.is(x, -0) ? "-" : "";
  if (x === 0) return `${sign}0.0`;
  const [digits, exp] = digitsAndExponent(x);
  if (exp < -4 || exp >= 16) {
    const m = digits.length > 1 ? `${digits[0]}.${digits.slice(1)}` : digits;
    return `${sign}${m}e${exp < 0 ? "-" : "+"}${String(Math.abs(exp)).padStart(2, "0")}`;
  }
  const point = exp + 1;
  if (point <= 0) return `${sign}0.${"0".repeat(-point)}${digits}`;
  if (point >= digits.length) return `${sign}${digits}${"0".repeat(point - digits.length)}.0`;
  return `${sign}${digits.slice(0, point)}.${digits.slice(point)}`;
}

/** Python's format(x, "g"): six significant digits, trailing zeros removed. */
export function formatG(x: number): string {
  if (Number.isNaN(x)) return "nan";
  if (!Number.isFinite(x)) return x > 0 ? "inf" : "-inf";
  const sign = x < 0 || Object.is(x, -0) ? "-" : "";
  if (x === 0) return `${sign}0`;
  const [mantissa, expText] = Math.abs(x).toExponential(5).split("e");
  const exp = Number(expText);
  const strip = (s: string) => (s.includes(".") ? s.replace(/0+$/, "").replace(/\.$/, "") : s);
  if (exp < -4 || exp >= 6) {
    return `${sign}${strip(mantissa!)}e${exp < 0 ? "-" : "+"}${String(Math.abs(exp)).padStart(2, "0")}`;
  }
  return `${sign}${strip(Math.abs(x).toFixed(5 - exp))}`;
}

/** Python's repr() of a string. */
export const reprStr = (s: string): string => `'${s}'`;
