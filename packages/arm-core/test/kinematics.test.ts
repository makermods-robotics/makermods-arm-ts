import { afterAll, describe, expect, it } from "vitest";
import { radians } from "../src/config.js";
import { ValueError } from "../src/errors.js";
import {
  Chain, chainFromConfig, matrixToRpy, poseToXyzRpy, rotationError, rpyToMatrix, xyzRpyToPose,
} from "../src/kinematics.js";
import { ARMS, armFixture, compare, configOf, type Diff, type Json, loadFixture, Tally } from "./helpers/math-fixtures.js";

// ── a hand-checkable planar arm (port of tests/unit/test_kinematics.py) ──
const L1 = 0.3, LC1 = 0.12, LC2 = 0.1, M1 = 1.5, M2 = 0.8;
const JAW_M = 0.2, JAW_COM = 0.05;
const TOOL_X = 0.25;
const G = 9.81;

// Two pitch joints (axis y) in the x-z plane, a jaw on a prismatic joint and a massive fixed tool link
// under link2, plus a mast on its own branch of the base that is not part of the chain.
const PLANAR = `<?xml version="1.0"?>
<robot name="planar">
  <link name="world"/>
  <link name="base"><inertial><origin xyz="0 0 0.05"/><mass value="3"/></inertial></link>
  <link name="mast"><inertial><origin xyz="0 0 0.5"/><mass value="1"/></inertial></link>
  <link name="link1"><inertial><origin xyz="${LC1} 0 0" rpy="0.3 0 0"/><mass value="${M1}"/></inertial></link>
  <link name="link2"><inertial><origin xyz="${LC2 - 0.05} 0 0"/><mass value="${M2 / 2}"/></inertial></link>
  <link name="jaw"><inertial><origin xyz="${JAW_COM} 0 0"/><mass value="${JAW_M}"/></inertial></link>
  <link name="flange"><inertial><origin xyz="0.05 0 0"/><mass value="${M2 / 2}"/></inertial></link>
  <link name="tool"/>
  <joint name="mount" type="fixed"><parent link="world"/><child link="base"/></joint>
  <joint name="mast_joint" type="fixed"><parent link="base"/><child link="mast"/><origin xyz="0.2 0 0"/></joint>
  <joint name="j1" type="revolute"><parent link="base"/><child link="link1"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 1 0"/><limit lower="-2" upper="2"/></joint>
  <joint name="j2" type="revolute"><parent link="link1"/><child link="link2"/>
    <origin xyz="${L1} 0 0"/><axis xyz="0 1 0"/><limit lower="-2.5" upper="1"/></joint>
  <joint name="flange_joint" type="fixed"><parent link="link2"/><child link="flange"/><origin xyz="0.1 0 0"/></joint>
  <joint name="jaw_joint" type="prismatic"><parent link="link2"/><child link="jaw"/>
    <origin xyz="0 0 0"/><axis xyz="0 0 1"/><limit lower="0" upper="0.04"/></joint>
  <joint name="tool_joint" type="fixed"><parent link="link2"/><child link="tool"/><origin xyz="${TOOL_X} 0 0"/></joint>
</robot>
`;

/** Tool point by hand: a pitch about y turns +x toward -z. */
function planarTool(q: number[]): number[] {
  const t1 = q[0]!, t12 = q[0]! + q[1]!;
  return [L1 * Math.cos(t1) + TOOL_X * Math.cos(t12), 0.0, 0.1 - L1 * Math.sin(t1) - TOOL_X * Math.sin(t12)];
}

/** dV/dq with V = g * sum(m z): link2 carries its own mass, the flange and the jaw. */
function planarGravity(q: number[]): number[] {
  const t1 = q[0]!, t12 = q[0]! + q[1]!;
  const m2 = M2 + JAW_M;
  const m2c = M2 * LC2 + JAW_M * JAW_COM;
  return [-G * (M1 * LC1 * Math.cos(t1) + m2 * L1 * Math.cos(t1) + m2c * Math.cos(t12)), -G * m2c * Math.cos(t12)];
}

/** Seeded uniform numbers (mulberry32). */
function rng(seed: number): (lo?: number, hi?: number) => number {
  let a = seed >>> 0;
  return (lo = 0, hi = 1) => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return lo + (hi - lo) * (((t ^ (t >>> 14)) >>> 0) / 4294967296);
  };
}

const close = (a: number[], b: number[], tol = 1e-8) => {
  expect(a.length).toBe(b.length);
  a.forEach((v, i) => expect(Math.abs(v - b[i]!)).toBeLessThanOrEqual(tol + 1e-5 * Math.abs(b[i]!)));
};
const position = (pose: number[][]) => [pose[0]![3]!, pose[1]![3]!, pose[2]![3]!];
const rot3 = (pose: number[][]) => pose.slice(0, 3).map((r) => r.slice(0, 3));

function numericJacobian(chain: Chain, q: number[], h = 1e-6): number[][] {
  const jac = Array.from({ length: 6 }, () => new Array<number>(chain.n).fill(0));
  for (let i = 0; i < chain.n; i++) {
    const plus = chain.fk(q.map((v, j) => (j === i ? v + h : v)));
    const minus = chain.fk(q.map((v, j) => (j === i ? v - h : v)));
    const dp = position(plus).map((v, r) => (v - position(minus)[r]!) / (2 * h));
    const dr = rotationError(rot3(plus), rot3(minus)).map((v) => v / (2 * h));
    for (let r = 0; r < 3; r++) {
      jac[r]![i] = dp[r]!;
      jac[3 + r]![i] = dr[r]!;
    }
  }
  return jac;
}

function numericGravity(chain: Chain, q: number[], h = 1e-6): number[] {
  return q.map((_, i) => (chain.potentialEnergy(q.map((v, j) => (j === i ? v + h : v)))
    - chain.potentialEnergy(q.map((v, j) => (j === i ? v - h : v)))) / (2 * h));
}

describe("parsing", () => {
  it("default chain follows the revolute branch", () => {
    const chain = Chain.fromUrdf(PLANAR);
    expect(chain.jointNames).toEqual(["j1", "j2"]);
    expect(chain.lower).toEqual([-2, -2.5]);
    expect(chain.upper).toEqual([2, 1]);
  });

  it("off-chain links are lumped and the base branch carries no joint", () => {
    close(Chain.fromUrdf(PLANAR).masses, [3 + 1, M1, M2 + JAW_M], 1e-12);
  });

  it("explicit tip extends through fixed joints", () => {
    const chain = Chain.fromUrdf(PLANAR, { tip: "tool" });
    close(position(chain.fk([0.4, -0.7])), planarTool([0.4, -0.7]));
  });

  it("joint list holds the other movable joints at zero", () => {
    const chain = Chain.fromUrdf(PLANAR, { joints: ["j2"], tip: "tool" });
    const full = Chain.fromUrdf(PLANAR, { tip: "tool" });
    expect(chain.n).toBe(1);
    close(chain.fk([0.3]).flat(), full.fk([0.0, 0.3]).flat());
  });

  it.each([[["j2", "j1"]], [["jaw_joint"]], [["nope"]]])("bad joint list %j is refused", (bad) => {
    expect(() => Chain.fromUrdf(PLANAR, { joints: bad })).toThrow(ValueError);
  });

  it("a base or tip that is not in the URDF is refused", () => {
    expect(() => Chain.fromUrdf(PLANAR, { base: "nope" })).toThrow(ValueError);
    expect(() => Chain.fromUrdf(PLANAR, { tip: "nope" })).toThrow(ValueError);
    expect(() => Chain.fromUrdf(PLANAR, { base: "link2", tip: "link1" })).toThrow(/not below/);
  });
});

describe("kinematics", () => {
  it.each([[[0.0, 0.0]], [[0.4, -0.7]], [[-1.2, 0.9]]])("fk and gravity match the hand model at %j", (q) => {
    const chain = Chain.fromUrdf(PLANAR, { tcpXyz: [TOOL_X, 0, 0] });
    close(position(chain.fk(q)), planarTool(q));
    close(chain.gravityTorques(q), planarGravity(q));
  });

  it("sign and offset map joint coordinates onto URDF angles", () => {
    const plain = Chain.fromUrdf(PLANAR);
    const mapped = Chain.fromUrdf(PLANAR, { sign: [-1, 1], offset: [0.2, -0.1] });
    const q = [0.3, 0.5];
    const theta = [q[0]! * -1 + 0.2, q[1]! * 1 - 0.1];
    close(mapped.fk(q).flat(), plain.fk(theta).flat());
    close(mapped.gravityTorques(q), plain.gravityTorques(theta).map((v, i) => v * [-1, 1][i]!));
    close(mapped.jacobian(q).flat(), plain.jacobian(theta).map((r) => r.map((v, i) => v * [-1, 1][i]!)).flat());
    // URDF j1 limits [-2, 2] with sign -1, offset 0.2 -> q in [-1.8, 2.2]
    expect(mapped.lower[0]).toBeCloseTo(-1.8, 12);
    expect(mapped.upper[0]).toBeCloseTo(2.2, 12);
  });

  it("sign must be +1 or -1", () => {
    expect(() => Chain.fromUrdf(PLANAR, { sign: [2, 1] })).toThrow(/sign must be/);
    expect(() => Chain.fromUrdf(PLANAR, { sign: [1] })).toThrow(/sign and offset/);
  });

  it("base turn rotates the world frame", () => {
    const turned = Chain.fromUrdf(PLANAR, { baseRpy: [0, 0, Math.PI / 2], baseXyz: [1, 0, 0] });
    const [x, y, z] = position(Chain.fromUrdf(PLANAR).fk([0.2, 0.1]));
    close(position(turned.fk([0.2, 0.1])), [1 - y!, x!, z!]);
  });

  it("pose helpers round trip", () => {
    const u = rng(3);
    for (let k = 0; k < 200; k++) {
      const rpy = [u(-Math.PI, Math.PI), u(-1.5, 1.5), u(-Math.PI, Math.PI)];
      const [xyz, back] = poseToXyzRpy(xyzRpyToPose([1, 2, 3], rpy));
      close(xyz, [1, 2, 3], 1e-12);
      close(rpyToMatrix(back).flat(), rpyToMatrix(rpy).flat(), 1e-12);
    }
  });

  it.each([1e-8, 0.3, Math.PI - 1e-6, Math.PI])("rotation error is axis times angle at %f", (angle) => {
    const n = Math.hypot(1.0, -2.0, 0.5);
    const axis = [1.0 / n, -2.0 / n, 0.5 / n];
    const k = [[0, -axis[2]!, axis[1]!], [axis[2]!, 0, -axis[0]!], [-axis[1]!, axis[0]!, 0]];
    const kk = k.map((r) => [0, 1, 2].map((j) => r[0]! * k[0]![j]! + r[1]! * k[1]![j]! + r[2]! * k[2]![j]!));
    const rot = [0, 1, 2].map((i) => [0, 1, 2].map((j) => (i === j ? 1 : 0) + Math.sin(angle) * k[i]![j]! + (1 - Math.cos(angle)) * kk[i]![j]!));
    const err = rotationError(rot, [[1, 0, 0], [0, 1, 0], [0, 0, 1]]);
    expect(Math.abs(Math.hypot(...err) - angle)).toBeLessThan(1e-9);
    expect(Math.abs(Math.abs(err[0]! * axis[0]! + err[1]! * axis[1]! + err[2]! * axis[2]!) - angle)).toBeLessThan(1e-9);
  });

  it("matrixToRpy puts roll at 0 at the singularity", () => {
    const rpy = matrixToRpy(rpyToMatrix([0.4, Math.PI / 2, 0.1]));
    expect(rpy[0]).toBe(0);
    expect(rpy[1]).toBeCloseTo(Math.PI / 2, 12);
  });
});

// ── each arm, as the SDK's tests/unit/gravity_fixtures.json describes it ──
for (const arm of ARMS) {
  describe(`${arm}: repo chain`, () => {
    const spec = loadFixture(`math_${arm}_gravity_fixtures.json`).chain;
    const limits: [number[], number[]] = [spec.limits_deg.map((l: number[]) => radians(l[0]!)), spec.limits_deg.map((l: number[]) => radians(l[1]!))];
    const repoChain = (tcp = true) => Chain.fromUrdf(spec.urdf_text, {
      sign: spec.sign, offset: spec.offset_deg.map(radians), baseRpy: spec.base_rpy_deg.map(radians),
      tcpXyz: tcp ? spec.tcp_xyz : [0, 0, 0], tcpRpy: tcp ? spec.tcp_rpy_deg.map(radians) : [0, 0, 0],
    });
    const uniform = (u: ReturnType<typeof rng>) => limits[0].map((lo, i) => u(lo, limits[1][i]!));
    const mean = limits[0].map((lo, i) => (lo + limits[1][i]!) / 2);

    it("is the profile arm", () => {
      expect(repoChain().n).toBe(spec.sign.length);
    });

    it("Jacobian matches finite differences", () => {
      const chain = repoChain();
      const u = rng(11);
      for (let k = 0; k < 20; k++) {
        const q = uniform(u);
        close(chain.jacobian(q).flat(), numericJacobian(chain, q).flat(), 1e-6);
      }
    });

    it("gravity is the gradient of potential energy", () => {
      const chain = repoChain();
      const u = rng(12);
      for (let k = 0; k < 50; k++) {
        const q = uniform(u);
        close(chain.gravityTorques(q), numericGravity(chain, q), 1e-6);
      }
    });

    it("tcp moves the tool point along the tip frame", () => {
      const flange = repoChain(false).fk(mean);
      const t = spec.tcp_xyz as number[];
      const want = [0, 1, 2].map((r) => flange[r]![0]! * t[0]! + flange[r]![1]! * t[1]! + flange[r]![2]! * t[2]! + flange[r]![3]!);
      close(position(repoChain().fk(mean)), want, 1e-12);
    });

    const ikRate = (n: number, fullPose: boolean): number => {
      const chain = repoChain();
      const u = rng(5);
      let good = 0;
      for (let k = 0; k < n; k++) {
        const qTrue = uniform(u);
        const seed = qTrue.map((v, i) => Math.min(Math.max(v + u(-0.2, 0.2), limits[0][i]!), limits[1][i]!));
        const pose = chain.fk(qTrue);
        const result = chain.ik(fullPose ? pose : position(pose), seed, { limits });
        if (result.posErr < 1e-3 && (!fullPose || result.rotErr < radians(0.5))) good++;
        result.q.forEach((v, i) => {
          expect(v).toBeGreaterThanOrEqual(limits[0][i]! - 1e-12);
          expect(v).toBeLessThanOrEqual(limits[1][i]! + 1e-12);
        });
      }
      return good / n;
    };

    it("position IK converges from a nearby seed", () => {
      expect(ikRate(150, false)).toBeGreaterThan(0.99);
    });

    it("full-pose IK converges from a nearby seed", () => {
      expect(ikRate(150, true)).toBeGreaterThan(0.95);
    });

    it("IK result reports the solution it found", () => {
      const chain = repoChain();
      const target = chain.fk(mean);
      const result = chain.ik(position(target), mean.map((v) => v + 0.1), { rpy: matrixToRpy(rot3(target)), limits });
      expect(result.ok).toBe(true);
      expect(result.iters).toBeGreaterThan(0);
      const reached = position(chain.fk(result.q));
      expect(Math.abs(Math.hypot(...reached.map((v, i) => v - position(target)[i]!)) - result.posErr)).toBeLessThan(1e-12);
      expect(result.rotErr).toBeLessThan(1e-3);
    });

    it("position IK stays near the seed in the redundant directions", () => {
      const chain = repoChain();
      const result = chain.ik(position(chain.fk(mean)), mean, { limits });
      expect(result.ok).toBe(true);
      expect(result.iters).toBe(0);
      close(result.q, mean, 1e-12);
    });

    it("unreachable target is not ok but returns the closest pose", () => {
      const chain = repoChain();
      const result = chain.ik([5.0, 0.0, 0.0], mean, { limits });
      expect(result.ok).toBe(false);
      expect(result.posErr).toBeGreaterThan(3.0);
      expect(result.q.length).toBe(chain.n);
      expect(result.q.every(Number.isFinite)).toBe(true);
    });

    it.each([
      { target: [0, 0] }, { target: [0, 0, NaN] }, { q0: [0.0] },
    ])("IK refuses malformed input %j", (bad) => {
      const chain = repoChain();
      const args = { target: [0.2, 0.0, 0.2], q0: mean, ...bad };
      expect(() => chain.ik(args.target, args.q0)).toThrow(ValueError);
    });
  });
}

// ── parity with the Python SDK on each arm's shipped profile ──
// Angles go through Math.sin / Math.cos, which differ from the C library's in the last bit for a few
// percent of arguments, so results agree with Python's to a few units in the last place. Bounds are
// in ULPs of each result's largest |value| (element-wise ULPs are meaningless for entries that should
// be zero), set just above the worst measured over both arms' fixtures.
const BOUND = {
  mount: 1, fk: 2, jacobian: 3, gravityTorques: 4, potentialEnergy: 12, fkJacobianMany: 3, variantFk: 2, variantGravity: 2,
  rpyToMatrix: 1, xyzRpyToPose: 1,
} as const;
const tally = new Tally();
afterAll(() => tally.report("kinematics vs Python (per arm; bounds in ULPs of scale)"));

for (const arm of ARMS) {
  describe(`${arm}: parity with Python`, () => {
    const fx: Json = armFixture(arm);
    const chain = chainFromConfig(configOf(fx))!;
    const within = (label: keyof typeof BOUND, d: Diff) => {
      tally.add(`${arm} ${label}`, d);
      expect(d.scaled).toBeLessThanOrEqual(BOUND[label]);
    };

    it("builds the same chain", () => {
      expect(chain.jointNames).toEqual(fx.chain.joint_names);
      expect(compare(chain.masses, fx.chain.masses).ulps).toBe(0);
      expect(compare(chain.lower, fx.chain.lower).ulps).toBe(0);
      expect(compare(chain.upper, fx.chain.upper).ulps).toBe(0);
      within("mount", compare(chain.mount, fx.chain.mount));
      expect(chain.sign).toEqual(fx.chain.sign);
      expect(compare(chain.offset, fx.chain.offset).ulps).toBe(0);
    });

    it("gravity torques agree to a few ULPs", () => {
      for (const c of fx.kinematics) within("gravityTorques", compare(chain.gravityTorques(c.q), c.gravity));
      for (const c of fx.custom_gravity) within("gravityTorques", compare(chain.gravityTorques(c.q, c.g), c.gravity));
    });

    it("fk, Jacobian and potential energy agree to a few ULPs", () => {
      for (const c of fx.kinematics) {
        within("fk", compare(chain.fk(c.q), c.fk));
        within("jacobian", compare(chain.jacobian(c.q), c.jacobian));
        within("potentialEnergy", compare(chain.potentialEnergy(c.q), c.potential));
      }
      for (const c of fx.custom_gravity) within("potentialEnergy", compare(chain.potentialEnergy(c.q, c.g), c.potential));
    });

    it("fkJacobianMany agrees to a few ULPs", () => {
      const [poses, jacs] = chain.fkJacobianMany(fx.batch.qs);
      poses.forEach((p, i) => within("fkJacobianMany", compare(p, fx.batch.poses[i])));
      jacs.forEach((j, i) => within("fkJacobianMany", compare(j, fx.batch.jacobians[i])));
    });

    it("chains with default and partial joint lists agree", () => {
      for (const v of fx.variants) {
        const c = v.label === "bare"
          ? Chain.fromUrdf(fx.profile.urdf)
          : Chain.fromUrdf(fx.profile.urdf, { joints: v.joint_names });
        expect(c.jointNames).toEqual(v.joint_names);
        expect(compare(c.masses, v.masses).ulps).toBe(0);
        expect(compare(c.lower, v.lower).ulps).toBe(0);
        expect(compare(c.upper, v.upper).ulps).toBe(0);
        v.q.forEach((q: number[], i: number) => {
          within("variantGravity", compare(c.gravityTorques(q), v.gravity[i]));
          within("variantFk", compare(c.fk(q), v.fk[i]));
        });
      }
    });

    it("ik agrees", () => {
      for (const c of fx.ik) {
        const o = c.options;
        const result = chain.ik(c.target, c.q0, {
          rpy: o.rpy ?? null, positionOnly: o.position_only ?? null, limits: o.limits ?? null, maxIters: o.max_iters,
          tol: o.tol, rotTol: o.rot_tol, rotWeight: o.rot_weight, nullGain: o.null_gain, maxStep: o.max_step,
        });
        const d = compare(result.q, c.q);
        tally.add(c.ok ? "Chain.ik q (converged)" : "Chain.ik q (stalled, not ok)", d);
        tally.add("Chain.ik posErr", compare(result.posErr, c.pos_err));
        expect(result.ok).toBe(c.ok);
        // A converging solve takes the same steps. One that stalls (not ok) ends where the damping
        // gives up, which rounding in the 6x6 inverse (LAPACK in numpy) can move by a few iterations.
        const tol = c.ok ? 1e-12 : 2e-9;
        if (c.ok) expect(result.iters).toBe(c.iters);
        expect(d.abs).toBeLessThanOrEqual(tol);
        expect(Math.abs(result.posErr - c.pos_err)).toBeLessThanOrEqual(tol);
        expect(Math.abs(result.rotErr - c.rot_err)).toBeLessThanOrEqual(tol);
      }
    });
  });
}

describe("rotations: parity with Python", () => {
  const common: Json = loadFixture("math_common.json");

  it("rpyToMatrix, matrixToRpy and pose helpers agree", () => {
    for (const c of common.rotation) {
      const d = compare(rpyToMatrix(c.rpy), c.matrix);
      tally.add("rpyToMatrix", d);
      expect(d.scaled).toBeLessThanOrEqual(BOUND.rpyToMatrix);
      const back = compare(matrixToRpy(c.matrix), c.back);
      tally.add("matrixToRpy", back);
      expect(back.abs).toBeLessThanOrEqual(1e-12);
    }
    for (const p of common.poses) {
      expect(compare(xyzRpyToPose(p.xyz, p.rpy), p.pose).scaled).toBeLessThanOrEqual(BOUND.xyzRpyToPose);
      const [xyz, rpy] = poseToXyzRpy(p.pose);
      expect(compare(xyz, p.xyz_back).ulps).toBe(0);
      expect(compare(rpy, p.rpy_back).abs).toBeLessThanOrEqual(1e-12);
    }
  });

  it("rotationError agrees", () => {
    for (const c of common.rotation_error) {
      const d = compare(rotationError(c.target, c.current), c.out);
      tally.add("rotationError", d);
      expect(d.abs).toBeLessThanOrEqual(1e-12);
    }
  });
});

// ── cooperative (async) ik ──
for (const arm of ARMS) {
  describe(`${arm}: ikAsync`, () => {
    const fx: Json = armFixture(arm);
    const chain = chainFromConfig(configOf(fx))!;
    const optionsOf = (o: Json) => ({
      rpy: o.rpy ?? null, positionOnly: o.position_only ?? null, limits: o.limits ?? null, maxIters: o.max_iters,
      tol: o.tol, rotTol: o.rot_tol, rotWeight: o.rot_weight, nullGain: o.null_gain, maxStep: o.max_step,
    });

    it("returns exactly what ik returns", async () => {
      for (const c of fx.ik) {
        const a = await chain.ikAsync(c.target, c.q0, optionsOf(c.options));
        const b = chain.ik(c.target, c.q0, optionsOf(c.options));
        expect(compare([a.q, a.posErr, a.rotErr], [b.q, b.posErr, b.rotErr]).ulps).toBe(0);
        expect([a.ok, a.iters]).toEqual([b.ok, b.iters]);
      }
    });

    it("pauses during a long solve and stops when aborted", async () => {
      const q0 = chain.lower.map((l, i) => (l + chain.upper[i]!) / 2);
      let pauses = 0;
      const pause = async () => {
        pauses++;
        await new Promise<void>((resolve) => setImmediate(resolve));
      };
      const result = await chain.ikAsync([5, 0, 0], q0, { rpy: [0, 0, 0], pause });
      expect(result.iters).toBe(100);
      expect(pauses).toBeGreaterThan(5);
      const controller = new AbortController();
      const stop = async () => {
        controller.abort(new Error("stop"));
      };
      await expect(chain.ikAsync([5, 0, 0], q0, { rpy: [0, 0, 0], pause: stop, signal: controller.signal })).rejects.toThrow("stop");
      await expect(chain.ikAsync([5, 0, 0], q0, { signal: AbortSignal.abort(new Error("early")) })).rejects.toThrow("early");
    });
  });
}

