import { afterAll, describe, expect, it } from "vitest";
import { radians } from "../src/config.js";
import { ValueError } from "../src/errors.js";
import * as gravity from "../src/gravity.js";
import { GravityFeedforward, RAMP_S, RAMP_STEP_S, STALE_S } from "../src/gravity.js";
import { Chain, chainFromConfig } from "../src/kinematics.js";
import { ARMS, armFixture, compare, configOf, type Json, loadFixture, Tally } from "./helpers/math-fixtures.js";

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

const near = (a: number[], b: number[], tol = 1e-9) => {
  expect(a.length).toBe(b.length);
  a.forEach((v, i) => expect(Math.abs(v - b[i]!)).toBeLessThanOrEqual(tol + 1e-6 * Math.abs(b[i]!)));
};

// ── port of tests/unit/test_gravity.py, on each arm's reference chain ──
for (const arm of ARMS) {
  describe(`${arm}: gravity feed-forward`, () => {
    const fx: Json = loadFixture(`math_${arm}_gravity_fixtures.json`);
    const spec = fx.chain;
    const CAPS: number[] = spec.gravity_limit_nm;
    const FACTORS: number[] = spec.gravity_factor;
    const N = CAPS.length;
    const FRESH = new Array<number>(N).fill(0.0);
    const lower = spec.limits_deg.map((l: number[]) => radians(l[0]!)) as number[];
    const upper = spec.limits_deg.map((l: number[]) => radians(l[1]!)) as number[];
    const POSE = lower.map((lo, i) => (lo + upper[i]!) / 2);
    const repoChain = (tcp = true) => Chain.fromUrdf(spec.urdf_text, {
      sign: spec.sign, offset: spec.offset_deg.map(radians), baseRpy: spec.base_rpy_deg.map(radians),
      tcpXyz: tcp ? spec.tcp_xyz : [0, 0, 0], tcpRpy: tcp ? spec.tcp_rpy_deg.map(radians) : [0, 0, 0],
    });
    const chain = repoChain(false);
    const feedforward = (scale = 1.0, caps: number[] | null = CAPS, level?: number) => {
      const ff = new GravityFeedforward(chain, { scale, limitsNm: caps, jointScale: FACTORS });
      if (level !== undefined) ff.level = level;
      return ff;
    };
    const uniform = (u: ReturnType<typeof rng>) => lower.map((lo, i) => u(lo, upper[i]!));

    it("the model matches the reference", () => {
      for (const c of fx.cases) near(feedforward().model(c.q_deg.map(radians)), c.tau_nm, 1e-9);
    });

    it("the model is the gradient of potential energy times the factors", () => {
      const u = rng(4);
      for (let k = 0; k < 30; k++) {
        const q = uniform(u);
        const numeric = q.map((_, i) => (chain.potentialEnergy(q.map((v, j) => (j === i ? v + 1e-6 : v)))
          - chain.potentialEnergy(q.map((v, j) => (j === i ? v - 1e-6 : v)))) / 2e-6);
        near(feedforward().model(q), numeric.map((v, i) => v * FACTORS[i]!), 1e-6);
      }
    });

    it("the model is scaled and periodic", () => {
      near(feedforward(0.5).model(POSE), feedforward().model(POSE).map((t) => t / 2));
      const turned = POSE.map((v, i) => (i === 0 ? v + 2 * Math.PI : v));
      near(feedforward().model(turned), feedforward().model(POSE));
    });

    it("the model of a non-finite pose is NaN, not an error", () => {
      expect(feedforward().model([Infinity, ...POSE.slice(1)]).every(Number.isNaN)).toBe(true);
    });

    it("the cap is not reached inside the soft limits", () => {
      const u = rng(20261103);
      const ff = feedforward();
      const worst = new Array<number>(N).fill(0);
      for (let k = 0; k < 3000; k++) ff.model(uniform(u)).forEach((t, i) => (worst[i] = Math.max(worst[i]!, Math.abs(t))));
      worst.forEach((w, i) => expect(w).toBeLessThan(CAPS[i]!));
    });

    it("the ease-in starts at zero and takes the ramp time", () => {
      const ff = feedforward();
      const full = ff.model(POSE);
      near(ff.torques(POSE, FRESH, POSE, 10.0), new Array<number>(N).fill(0));
      const frames = Math.trunc(RAMP_S / 0.02);
      for (let k = 1; k < frames; k++) near(ff.torques(POSE, FRESH, POSE, 10.0 + k * 0.02), full.map((t) => (t * k) / frames));
      near(ff.torques(POSE, FRESH, POSE, 10.0 + frames * 0.02), full);
      expect(ff.level).toBe(1.0);
    });

    it("a pause never skips the ease-in", () => {
      const ff = feedforward();
      ff.torques(POSE, FRESH, POSE, 0.0);
      ff.torques(POSE, FRESH, POSE, 100.0);
      expect(ff.level).toBeCloseTo(RAMP_STEP_S / RAMP_S, 12);
    });

    it("a clock step backwards does not advance or undo the ease-in", () => {
      const ff = feedforward();
      ff.torques(POSE, FRESH, POSE, 10.0);
      ff.torques(POSE, FRESH, POSE, 10.02);
      const level = ff.level;
      ff.torques(POSE, FRESH, POSE, 5.0);
      expect(ff.level).toBe(level);
    });

    it("reset starts the ease-in again", () => {
      const ff = feedforward(1.0, CAPS, 1.0);
      ff.reset();
      expect(ff.torques(POSE, FRESH, POSE, 0.0).every((t) => t === 0)).toBe(true);
    });

    it("the measured pose is used while every joint is fresh", () => {
      const goal = POSE.map((q) => q + 0.3);
      const ages = new Array<number>(N).fill(STALE_S);
      near(feedforward(1.0, CAPS, 1).torques(POSE, ages, goal, 0.0), feedforward().model(POSE));
    });

    it.each(Array.from({ length: 6 }, (_, i) => i))("one stale joint (%i) falls back to the goal", (stale) => {
      const goal = POSE.map((q) => q + 0.3);
      const ages = new Array<number>(N).fill(0.0);
      ages[stale % N] = STALE_S + 0.001;
      near(feedforward(1.0, CAPS, 1).torques(POSE, ages, goal, 0.0), feedforward().model(goal));
    });

    it("a non-finite measurement falls back to the goal", () => {
      const measured = [NaN, ...POSE.slice(1)];
      near(feedforward(1.0, CAPS, 1).torques(measured, FRESH, POSE, 0.0), feedforward().model(POSE));
    });

    it("a non-finite pose everywhere gives zero torque", () => {
      const bad = new Array<number>(N).fill(NaN);
      expect(feedforward(1.0, CAPS, 1).torques(bad, FRESH, bad, 0.0)).toEqual(new Array<number>(N).fill(0.0));
    });

    it("the cap is per joint and both signs", () => {
      const ff = feedforward(1.0, CAPS, 1);
      ff.model = () => Array.from({ length: N }, (_, i) => 100.0 * (-1) ** i);
      expect(ff.torques(POSE, FRESH, POSE, 0.0)).toEqual(CAPS.map((c, i) => c * (-1) ** i));
    });

    it("a single non-finite torque becomes zero", () => {
      const ff = feedforward(1.0, CAPS, 1);
      const values = Array.from({ length: N }, (_, i) => 0.01 * (i + 1));
      values[1] = NaN;
      values[2] = Infinity;
      ff.model = () => values;
      expect(ff.torques(POSE, FRESH, POSE, 0.0)).toEqual(values.map((v, i) => (Number.isFinite(v) ? Math.min(v, CAPS[i]!) : 0.0)));
    });

    it.each([-0.1, 1.21, NaN])("scale %f out of range is refused", (scale) => {
      expect(() => feedforward(scale)).toThrow(/gravity_scale/);
    });

    it.each([[null], [[1.0]], [new Array(6).fill(Infinity)], [new Array(6).fill(-1.0)]])("caps %j must be one finite value per joint", (caps) => {
      expect(() => feedforward(1.0, caps)).toThrow(/limits/);
    });

    it("factors, ramp and stale times are checked", () => {
      expect(() => new GravityFeedforward(chain, { limitsNm: CAPS, jointScale: [3, ...FACTORS.slice(1)] })).toThrow(/factors/);
      expect(() => new GravityFeedforward(chain, { limitsNm: CAPS, rampS: 0 })).toThrow(/ramp_s/);
      expect(() => new GravityFeedforward(chain, { limitsNm: CAPS, staleS: NaN })).toThrow(ValueError);
    });
  });
}

// ── profile glue and parity with the Python SDK ──
// Within a few ULPs of each torque vector's largest |value| (Math.sin / Math.cos vs the C library);
// bounds just above the worst measured over both arms.
const BOUND = { model: 4, torques: 5 } as const;
const tally = new Tally();
afterAll(() => tally.report("gravity feed-forward vs Python (per arm; bounds in ULPs of scale)"));

for (const arm of ARMS) {
  describe(`${arm}: fromConfig and parity with Python`, () => {
    const fx: Json = armFixture(arm);
    const config = configOf(fx);
    const chain = chainFromConfig(config)!;

    it("fromConfig builds the profile model", () => {
      const ff = gravity.fromConfig(config)!;
      expect(ff.n).toBe(chain.n);
      const viaChain = gravity.fromConfig(config, chain)!;
      expect(viaChain.chain).toBe(chain);
    });

    it("fromConfig is off without a urdf or at scale zero", () => {
      expect(gravity.fromConfig(configOf(fx, { urdf: null }))).toBeNull();
      expect(gravity.fromConfig(configOf(fx, { urdf: "" }))).toBeNull();
      expect(gravity.fromConfig(configOf(fx, { gravity_scale: 0.0 }))).toBeNull();
    });

    it("fromConfig needs a cap on every arm joint", () => {
      const joints = structuredClone(fx.profile.joints);
      delete joints[1].gravity_limit_nm;
      expect(() => gravity.fromConfig(configOf(fx, { joints }))).toThrow(/gravity_limit_nm/);
    });

    it("model agrees to a few ULPs", () => {
      const ff = gravity.fromConfig(config, chain)!;
      for (const c of fx.gravity_model) {
        const d = compare(ff.model(c.q), c.out);
        tally.add(`${arm} model`, d);
        expect(d.scaled).toBeLessThanOrEqual(BOUND.model);
      }
    });

    it("torques over time agree to a few ULPs (ease-in, stale feedback, caps, reset)", () => {
      for (const seq of fx.gravity_sequences) {
        const init = seq.init;
        const ff = init === "from_config"
          ? gravity.fromConfig(config, chain)!
          : new GravityFeedforward(chain, {
            scale: init.scale, limitsNm: init.limits_nm, rampS: init.ramp_s, staleS: init.stale_s, jointScale: init.joint_scale,
          });
        for (const step of seq.steps) {
          if (step.op === "reset") {
            ff.reset();
            continue;
          }
          const out = ff.torques(step.measured, step.ages, step.goal, step.now);
          const d = compare(out, step.out);
          tally.add(`${arm} torques (${seq.label})`, d);
          expect(d.scaled).toBeLessThanOrEqual(BOUND.torques);
          expect(ff.level).toBe(step.level);
        }
      }
    });

    it("model and torques are bit for bit Python's when sin and cos are the C library's", () => {
      // The remaining differences above come only from Math.sin / Math.cos: with the values Python's
      // math module gave for the same angles (fixture gravity_trig), every torque has Python's bits.
      const table = new Map<number, [number, number]>();
      for (const [x, s, c] of fx.gravity_trig as [number, number, number][]) table.set(x, [s, c]);
      const { sin, cos } = Math;
      let missing = 0;
      Math.sin = (x: number) => table.get(x)?.[0] ?? (missing++, sin(x));
      Math.cos = (x: number) => table.get(x)?.[1] ?? (missing++, cos(x));
      try {
        const model = gravity.fromConfig(config, chain)!;
        for (const c of fx.gravity_model) expect(compare(model.model(c.q), c.out).ulps).toBe(0);
        for (const seq of fx.gravity_sequences) {
          const init = seq.init;
          const ff = init === "from_config"
            ? gravity.fromConfig(config, chain)!
            : new GravityFeedforward(chain, {
              scale: init.scale, limitsNm: init.limits_nm, rampS: init.ramp_s, staleS: init.stale_s, jointScale: init.joint_scale,
            });
          for (const step of seq.steps) {
            if (step.op === "reset") ff.reset();
            else expect(compare(ff.torques(step.measured, step.ages, step.goal, step.now), step.out).ulps).toBe(0);
          }
        }
      } finally {
        Math.sin = sin;
        Math.cos = cos;
      }
      expect(missing).toBe(0);
    });

    it("the float model follows changes to the chain's masses and signs", () => {
      const c = chainFromConfig(config)!;
      const q = [0.0, -15.0, 22.0, 0.0, 0.0, 0.0].slice(0, c.n).map(radians);
      c.gravityTorques(q);
      c.masses[3] = c.masses[3]! * 2.0;
      c.sign.forEach((v, i) => (c.sign[i] = -v));
      const numeric = q.map((_, i) => (c.potentialEnergy(q.map((v, j) => (j === i ? v + 1e-6 : v)))
        - c.potentialEnergy(q.map((v, j) => (j === i ? v - 1e-6 : v)))) / 2e-6);
      near(c.gravityTorques(q), numeric, 1e-6);
      expect(c.gravityTorques(q)).not.toEqual(chainFromConfig(config)!.gravityTorques(q));
    });
  });
}

// ── port of tests/unit/test_maker_gravity.py: the Maker model against holding torques measured on the arm ──
describe("maker_arm: gravity model against measured holding torques", () => {
  const measured: Json = loadFixture("math_maker_arm_gravity_measured.json");
  const model = gravity.fromConfig(configOf(armFixture("maker_arm")))!;
  const rms = (): number[] => {
    const sq = new Array<number>(6).fill(0);
    for (const c of measured.cases) {
      const tau = model.model(c.q_deg.slice(0, 6).map(radians));
      for (let i = 0; i < 6; i++) sq[i] = sq[i]! + (c.tau_nm[i] - tau[i]!) ** 2;
    }
    return sq.map((v) => Math.sqrt(v / measured.cases.length));
  };

  it("the shoulder term matches the measured holding torque", () => {
    expect(rms()[1]).toBeLessThan(0.2);   // N*m; the friction measured at these poses is about 0.06
  });

  it("elbow and wrist terms stay as close as measured", () => {
    const r = rms();
    expect(r[2]).toBeLessThan(0.25);
    expect(r[3]).toBeLessThan(0.2);
  });
});
