/**
 * Leader servo degrees -> follower joint radians, from a Star mapping (the product's
 * `defaultStarMapping`, or the same JSON shape from elsewhere).
 *
 * Leader and follower are zeroed in the same pose, so per joint
 * `follower_deg = unwrap(raw_deg) * direction + offset_deg`, clipped to `range_deg`. `direction`
 * carries a scale as well as the sign (leader and follower travel differ). The unwrap removes whole
 * servo turns, centring the window on the follower range expressed in leader degrees
 * ((range - offset) / direction). `"magnitude": true` maps the distance from zero instead of the
 * signed reading, for a trigger grip that some builds turn one way and some the other.
 *
 * JSON: `{"joints": [{"name", "servo", "direction", "range_deg": [lo, hi], "offset_deg"?, "magnitude"?}]}`
 * (keys stay snake_case, as in the shipped files).
 */

import { radians } from "./config.js";
import { ValueError } from "./errors.js";

/** One joint of a Star mapping, as written in the JSON file. */
export interface StarMappingJoint {
  readonly name?: string | null;
  readonly servo: number;
  readonly direction: number;
  readonly range_deg: readonly number[];
  readonly offset_deg?: number;
  readonly magnitude?: boolean | null;
}

/** A Star mapping file's content. */
export interface StarMapping {
  readonly joints: readonly StarMappingJoint[];
  readonly _comment?: string;
}

/** Servo id -> raw leader degrees. */
export type RawDegrees = ReadonlyMap<number, number> | Readonly<Record<number, number>>;

/** Python's round() of a float: half to even, an integer (never -0). */
function roundHalfEven(x: number): number {
  const r = Math.round(x);
  return (r - x === 0.5 && r % 2 !== 0 ? r - 1 : r) + 0;
}

const isFiniteNumber = (v: unknown): v is number => typeof v === "number" && Number.isFinite(v);

/** Throws ValueError naming entry `index` unless every field of `j` has its type and allowed values. */
function validateJoint(j: StarMappingJoint, index: number): void {
  if (j === null || typeof j !== "object") throw new ValueError(`mapping entry ${index}: expected an object`);
  const what = `mapping entry ${index}` + (typeof j.name === "string" && j.name !== "" ? ` (${j.name})` : "");
  const fail = (message: string): never => {
    throw new ValueError(`${what}: ${message}`);
  };
  if (!Number.isInteger(j.servo) || j.servo < 0 || j.servo > 253) fail(`servo must be an integer 0..253, got ${JSON.stringify(j.servo)}`);
  if (j.name !== undefined && j.name !== null && (typeof j.name !== "string" || j.name === "")) {
    fail(`name must be a non-empty string, got ${JSON.stringify(j.name)}`);
  }
  if (!isFiniteNumber(j.direction) || j.direction === 0) fail(`servo ${j.servo}: direction must be finite and non-zero`);
  if (!Array.isArray(j.range_deg) || j.range_deg.length !== 2 || !j.range_deg.every(isFiniteNumber)) {
    fail(`range_deg must be [lo, hi], two finite numbers, got ${JSON.stringify(j.range_deg)}`);
  }
  const [lo, hi] = j.range_deg as [number, number];
  if (!(lo < hi)) fail(`servo ${j.servo}: range_deg must satisfy lo < hi`);
  if (j.offset_deg !== undefined && !isFiniteNumber(j.offset_deg)) {
    fail(`offset_deg must be a finite number, got ${JSON.stringify(j.offset_deg)}`);
  }
  if (j.magnitude !== undefined && j.magnitude !== null && typeof j.magnitude !== "boolean") {
    fail(`magnitude must be true or false, got ${JSON.stringify(j.magnitude)}`);
  }
}

const pyMin = (a: number, b: number): number => (b < a ? b : a);
const pyMax = (a: number, b: number): number => (b > a ? b : a);

/** Leader servo degrees -> follower joint radians for one leader. */
export class JointMapper {
  #joints: StarMappingJoint[];
  #lastOut: number[];

  /**
   * Throws ValueError, naming the entry, for anything but: an integer servo id 0..253, a finite
   * non-zero `direction`, `range_deg` as two finite numbers lo < hi, an optional finite `offset_deg`,
   * an optional boolean `magnitude` and an optional non-empty `name`; servo ids and names distinct.
   * (A mapping read from JSON is checked here, so `"offset_deg": "0"` cannot reach the arithmetic.)
   */
  constructor(joints: readonly StarMappingJoint[]) {
    if (!Array.isArray(joints)) throw new ValueError("a Star mapping needs a joints list");
    joints.forEach((j, i) => validateJoint(j, i));
    const ids = joints.map((j) => j.servo);
    if (new Set(ids).size !== ids.length) throw new ValueError(`servo ids must be unique: [${ids.join(", ")}]`);
    const named = joints.filter((j) => j.name !== undefined && j.name !== null).map((j) => j.name!);
    if (new Set(named).size !== named.length) throw new ValueError(`joint names must be unique: [${named.join(", ")}]`);
    this.#joints = joints.map((j) => Object.freeze({ ...j, range_deg: Object.freeze([...j.range_deg]) }));
    this.#lastOut = new Array<number>(joints.length).fill(NaN);
  }

  /** From a mapping object, or its JSON text (validated as the constructor does). */
  static fromJson(source: StarMapping | string): JointMapper {
    const raw = (typeof source === "string" ? JSON.parse(source) : source) as unknown;
    if (raw === null || typeof raw !== "object" || !Array.isArray((raw as StarMapping).joints)) {
      throw new ValueError('a Star mapping is {"joints": [...]}');
    }
    return new JointMapper((raw as StarMapping).joints);
  }

  get servoIds(): number[] {
    return this.#joints.map((j) => j.servo);
  }

  get names(): string[] {
    return this.#joints.map((j) => j.name ?? `servo ${j.servo}`);
  }

  /** `value` less the whole turns that take it furthest from the middle of [lo, hi]. */
  static unwrap(value: number, lo: number, hi: number): number {
    const center = (lo + hi) / 2.0;
    return value - roundHalfEven((value - center) / 360.0) * 360.0;
  }

  /** One joint's follower target (rad) for a raw leader reading (deg). */
  mapJoint(index: number, deg: number): number {
    const j = this.#joints[index]!;
    const [lo, hi] = j.range_deg as [number, number];
    const direction = j.direction;
    const offset = j.offset_deg ?? 0.0;
    let position: number;
    if (j.magnitude) {
      position = Math.abs(JointMapper.unwrap(deg, -180.0, 180.0)) * direction + offset;
    } else {
      const a = (lo - offset) / direction;
      const b = (hi - offset) / direction;
      position = JointMapper.unwrap(deg, pyMin(a, b), pyMax(a, b)) * direction + offset;
    }
    return radians(pyMax(lo, pyMin(hi, position)));
  }

  /**
   * Leader raw degrees by servo id -> follower targets in radians. A joint without a reading keeps
   * its last output (NaN before its first reading, which Arm.setJointTargets rejects).
   */
  map(rawDeg: RawDegrees): number[] {
    const get = rawDeg instanceof Map
      ? (id: number) => (rawDeg as ReadonlyMap<number, number>).get(id)
      : (id: number) => (rawDeg as Readonly<Record<number, number>>)[id];
    const out = [...this.#lastOut];
    this.#joints.forEach((j, i) => {
      const deg = get(j.servo);
      if (typeof deg !== "number" || !Number.isFinite(deg)) return;
      out[i] = this.mapJoint(i, deg);
    });
    this.#lastOut = out;
    return out;
  }

  /** Forget the last output: every joint is NaN until its next reading. */
  reset(): void {
    this.#lastOut = new Array<number>(this.#joints.length).fill(NaN);
  }
}
