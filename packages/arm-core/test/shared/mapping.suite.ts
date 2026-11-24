// Port of tests/unit/test_mapping.py (identical in both Python SDKs).
import { describe, expect, it } from "vitest";
import { radians, ValueError } from "@makermods/arm-core";
import { JointMapper, type StarMapping, type StarMappingJoint } from "../../src/mapping.js";
import type { ArmFakes } from "./fakes.js";

const JOINTS: StarMappingJoint[] = [
  { servo: 0, direction: 1.5, range_deg: [-150, 150] },
  { servo: 6, direction: -2.0, range_deg: [-120, -2] },
];

const close = (a: number, b: number): boolean => Math.abs(a - b) <= Math.max(1e-6 * Math.abs(b), 1e-12);
function expectApprox(actual: readonly number[], expected: readonly number[]): void {
  expect(actual.length).toBe(expected.length);
  actual.forEach((a, i) => expect(close(a, expected[i]!), `[${i}] ${a} ≈ ${expected[i]}`).toBe(true));
}

/** Registers the tests; the last one checks `fk`'s shipped Star mapping. */
export function mappingSuite(fk: Pick<ArmFakes, "name" | "product">): void {
  describe(`${fk.name}: JointMapper`, () => {
    it("follower is raw times direction in radians", () => {
      expectApprox(new JointMapper(JOINTS).map({ 0: 40.0, 6: 30.0 }), [radians(60.0), radians(-60.0)]);
    });

    it("output is clipped to range", () => {
      expectApprox(new JointMapper(JOINTS).map({ 0: 110.0, 6: -10.0 }), [radians(150.0), radians(-2.0)]);
    });

    it("whole servo turns are unwrapped", () => {
      expectApprox(new JointMapper(JOINTS).map({ 0: 40.0 + 360.0, 6: 30.0 - 720.0 }), [radians(60.0), radians(-60.0)]);
    });

    it("the unwrap window is the follower range in leader degrees", () => {
      // Follower [0, 170] at scale 0.5 is leader [0, 340]: 300 leader deg is 150 follower deg (a window
      // centred on the sign-only 85 would unwrap it to -60 and clip the joint to 0).
      const m = new JointMapper([{ servo: 1, direction: 0.5, range_deg: [0, 170] }]);
      expectApprox(m.map({ 1: 300.0 }), [radians(150.0)]);
    });

    it("a missing reading holds the last and starts NaN", () => {
      const m = new JointMapper(JOINTS);
      const first = m.map({ 0: 10.0 });
      expect(Number.isNaN(first[1])).toBe(true);
      expect(m.map({ 6: 20.0 })[0]).toBe(first[0]);
    });

    it("a non-finite reading is a missing reading", () => {
      const m = new JointMapper(JOINTS);
      const first = m.map({ 0: 10.0, 6: 5.0 });
      expect(m.map({ 0: NaN, 6: 5.0 })).toEqual(first);
    });

    it("reset forgets the last output", () => {
      const m = new JointMapper(JOINTS);
      m.map({ 0: 10.0, 6: 5.0 });
      m.reset();
      expect(m.map({}).every((v) => Number.isNaN(v))).toBe(true);
    });

    it("offset shifts the follower and its unwrap window", () => {
      const m = new JointMapper([{ servo: 0, direction: 1.0, offset_deg: 90.0, range_deg: [0, 300] }]);
      expectApprox(m.map({ 0: 200.0 }), [radians(290.0)]); // window centred on 60 leader deg
    });

    it.each([-50.0, 50.0, 50.0 - 360.0])("a magnitude joint maps the distance pulled either way (%d)", (raw) => {
      const m = new JointMapper([{ servo: 6, direction: -1.2, magnitude: true, range_deg: [-120, 0] }]);
      expectApprox(m.map({ 6: raw }), [radians(-60.0)]);
    });

    it.each([
      [[{ servo: 0, direction: 0, range_deg: [-1, 1] }]],
      [[{ servo: 0, direction: 1, range_deg: [1, 1] }]],
      [[{ servo: 0, direction: 1, range_deg: [-1, 1] }, { servo: 0, direction: 1, range_deg: [-1, 1] }]],
    ] as StarMappingJoint[][][])("bad mappings are rejected (%#)", (joints) => {
      expect(() => new JointMapper(joints)).toThrow(ValueError);
    });

    it.each([
      [{ servo: 0, direction: 1, range_deg: [-90, 90], offset_deg: "0" }, /mapping entry 0 \(a\): offset_deg must be a finite number/],
      [{ servo: 1.5, direction: 1, range_deg: [-90, 90] }, /mapping entry 0 \(a\): servo must be an integer/],
      [{ servo: "1", direction: 1, range_deg: [-90, 90] }, /servo must be an integer/],
      [{ servo: 0, direction: "1", range_deg: [-90, 90] }, /direction must be finite and non-zero/],
      [{ servo: 0, direction: 1, range_deg: ["-90", 90] }, /range_deg must be \[lo, hi\], two finite numbers/],
      [{ servo: 0, direction: 1, range_deg: [-90, 90, 0] }, /range_deg must be \[lo, hi\]/],
      [{ servo: 0, direction: 1, range_deg: [-90, 90], magnitude: "yes" }, /magnitude must be true or false/],
      [{ servo: 0, direction: 1, range_deg: [-90, 90], offset_deg: null }, /offset_deg must be a finite number/],
    ])("a field of the wrong type or value is refused, naming the entry (%#)", (bad, message) => {
      const entry = { name: "a", ...(bad as object) } as unknown as StarMappingJoint;
      expect(() => new JointMapper([entry])).toThrow(message);
      expect(() => JointMapper.fromJson(JSON.stringify({ joints: [entry] }))).toThrow(ValueError);
    });

    it("names must be distinct non-empty strings", () => {
      const j = (servo: number, name: unknown) => ({ servo, name, direction: 1, range_deg: [-1, 1] }) as unknown as StarMappingJoint;
      expect(() => new JointMapper([j(0, "a"), j(1, "a")])).toThrow(/joint names must be unique/);
      expect(() => new JointMapper([j(0, "")])).toThrow(/mapping entry 0: name must be a non-empty string/);
      expect(() => new JointMapper([j(0, 7)])).toThrow(/name must be a non-empty string/);
    });

    it("fromJson refuses something that is not a mapping", () => {
      expect(() => JointMapper.fromJson("[]")).toThrow(ValueError);
      expect(() => JointMapper.fromJson("null")).toThrow(ValueError);
      expect(() => JointMapper.fromJson({ joints: "x" } as unknown as StarMapping)).toThrow(ValueError);
    });

    it("the shipped Star mapping loads and names seven servos", () => {
      const raw = fk.product.defaultStarMapping;
      const m = JointMapper.fromJson(raw);
      expect(m.servoIds.length).toBe(7);
      expect(new Set(m.servoIds).size).toBe(7);
      expect(m.names).toEqual(raw.joints.map((j) => j.name));
      const out = m.map(new Map(m.servoIds.map((sid) => [sid, 0.0])));
      out.forEach((value, i) => {
        const [lo, hi] = raw.joints[i]!.range_deg as [number, number];
        expect(radians(lo) <= value && value <= radians(hi)).toBe(true);
      });
      expect(JointMapper.fromJson(JSON.stringify(raw)).servoIds).toEqual(m.servoIds);
    });
  });
}
