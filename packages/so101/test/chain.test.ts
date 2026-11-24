// The SO-101 chain data (src/chain.ts) against dreamlabs' so101_faas forward kinematics (the console's
// so101-trajectory-ik.json fixture). The FK here is a test-only URDF walk, the console's planner/kinematics.ts walk.
import { describe, expect, it } from "vitest";
import { SO101_CHAINS, type So101Chain } from "../src/chain.js";
import fixture from "./fixtures/so101-trajectory-ik.json" with { type: "json" };

type M = number[];
const mul = (a: M, b: M): M => [0, 1, 2].flatMap((r) => [0, 1, 2].map((c) => a[r * 3]! * b[c]! + a[r * 3 + 1]! * b[3 + c]! + a[r * 3 + 2]! * b[6 + c]!));
const apply = (r: M, v: readonly number[]) => [0, 1, 2].map((i) => r[i * 3]! * v[0]! + r[i * 3 + 1]! * v[1]! + r[i * 3 + 2]! * v[2]!);
function rpy([r, p, y]: readonly number[]): M {
  const [cr, sr, cp, sp, cy, sy] = [Math.cos(r!), Math.sin(r!), Math.cos(p!), Math.sin(p!), Math.cos(y!), Math.sin(y!)];
  return [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, -sp, cp * sr, cp * cr];
}
function axisAngle([x, y, z]: readonly number[], q: number): M {
  const [c, s] = [Math.cos(q), Math.sin(q)];
  const t = 1 - c;
  return [t * x! * x! + c, t * x! * y! - s * z!, t * x! * z! + s * y!, t * x! * y! + s * z!, t * y! * y! + c, t * y! * z! - s * x!, t * x! * z! - s * y!, t * y! * z! + s * x!, t * z! * z! + c];
}
function tool(model: So101Chain, q: readonly number[]) {
  let rot: M = [1, 0, 0, 0, 1, 0, 0, 0, 1];
  let pos = [0, 0, 0];
  let k = 0;
  for (const j of model.chain) {
    if (j.branch) {
      k++;
      continue;
    }
    pos = pos.map((v, i) => v + apply(rot, j.xyz)[i]!);
    rot = mul(rot, rpy(j.rpy));
    if (j.axis) rot = mul(rot, axisAngle(j.axis, q[k++]!));
  }
  const off = apply(rot, model.toolOffset);
  return { xyz: pos.map((v, i) => v + off[i]!), rotation: mul(rot, [...model.toolRotation]) };
}

describe(`SO-101 chain data vs dreamlabs ${fixture.sdk_version} so101_faas FK`, () => {
  it("matches dream_ik's FK on both sides (joint6 moves nothing)", () => {
    expect(fixture.fk.length).toBeGreaterThan(0);
    for (const c of fixture.fk) {
      const model = SO101_CHAINS[c.side as "left" | "right"];
      const t = tool(model, c.q);
      t.xyz.forEach((v, j) => expect(Math.abs(v - c.xyz[j]!)).toBeLessThan(1e-6));
      t.rotation.forEach((v, k) => expect(Math.abs(v - c.rotation[Math.floor(k / 3)]![k % 3]!)).toBeLessThan(1e-5));
      expect(tool(model, [...c.q.slice(0, 5), 0.4])).toEqual(t);
    }
  });

  it("carries each side's URDF limits (the IK's are them less its margin)", () => {
    for (const side of ["left", "right"] as const) {
      const c = fixture.cases.find((x) => x.side === side)!;
      const revolute = SO101_CHAINS[side].chain.filter((j) => j.axis && !j.branch);
      revolute.forEach((j, i) => {
        expect(j.limit![0]).toBeLessThanOrEqual(c.lo[i]! + 1e-9);
        expect(j.limit![1]).toBeGreaterThanOrEqual(c.hi[i]! - 1e-9);
      });
    }
  });
});
