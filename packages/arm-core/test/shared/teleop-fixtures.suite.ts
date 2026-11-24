// Cross-language fixture contract/fixtures/teleop_math.json (tools/gen_teleop_math.py): JointMapper,
// the ArmLeader friction feed-forward, SpeedCap and the Teleop loop match the Python SDK exactly.
import { describe, expect, it } from "vitest";
import { ArmState } from "@makermods/arm-core";
import { VirtualClock } from "@makermods/arm-core/testing";
import { Friction } from "../../src/leaders/arm.js";
import { JointMapper, type StarMappingJoint } from "../../src/mapping.js";
import { SpeedCap, Teleop, type TeleopStep } from "../../src/teleop.js";
import { type Json, loadFixture } from "../helpers/math-fixtures.js";
import type { ArmFakes } from "./fakes.js";
import { RecordingLogger } from "./fakes.js";
import { FakeArm, joint, NAMES, ScriptedLeader } from "./teleop-fakes.js";

const FX: Json = loadFixture("teleop_math.json");
const nullToUndefined = <T>(v: T | null): T | undefined => (v === null ? undefined : v);

/** Registers the fixture checks for `fk`'s shipped mapping and profile, and the shared arithmetic. */
export function teleopFixtureSuite(fk: Pick<ArmFakes, "name" | "product" | "defaultConfig">): void {
  describe(`${fk.name}: teleop fixtures`, () => {
    it("the fixture's shipped mapping is the product's", () => {
      const fixture = FX.mapping.cases.find((c: Json) => c.label === `${fk.name}_shipped`);
      expect(fixture.joints).toEqual(fk.product.defaultStarMapping.joints);
    });

    for (const label of [`${fk.name}_shipped`, "custom"]) {
      it(`JointMapper.map matches exactly (${label})`, () => {
        const c = FX.mapping.cases.find((x: Json) => x.label === label);
        const mapper = new JointMapper(c.joints as StarMappingJoint[]);
        for (const step of c.steps) {
          if (step.reset) mapper.reset();
          expect(mapper.map(new Map(step.raw as Array<[number, number]>))).toEqual(step.out);
        }
      });
    }

    it("JointMapper.unwrap matches exactly", () => {
      for (const [value, lo, hi, out] of FX.mapping.unwrap) expect(JointMapper.unwrap(value, lo, hi)).toEqual(out);
    });

    it("the profile's friction terms and torques match exactly", () => {
      const expected = FX.friction[fk.name];
      const friction = Friction.fromConfig(fk.defaultConfig());
      if (expected === null) {
        expect(friction).toBeNull();
        return;
      }
      expect(friction).not.toBeNull();
      expect([friction!.viscous, friction!.coulomb, friction!.rest, friction!.clamp, friction!.deadzone])
        .toEqual([expected.viscous, expected.coulomb, expected.rest, expected.clamp, expected.deadzone]);
      for (const row of expected.rows) {
        expect(friction!.torques(row.velocities.map(() => 0.0), row.velocities)).toEqual(row.torques);
      }
    });

    it("custom friction torques match exactly", () => {
      const c = FX.friction.custom;
      const friction = new Friction(c.viscous, c.coulomb, c.rest, c.clamp, c.deadzone);
      for (const row of c.rows) expect(friction.torques([0, 0, 0], row.velocities)).toEqual(row.torques);
    });

    it("SpeedCap matches exactly", () => {
      for (const c of FX.speedcap) {
        const cap = new SpeedCap(c.start);
        for (const s of c.steps) {
          expect(cap.step(s.goal, s.dt, c.max_velocity, c.max_acceleration)).toEqual(s.command);
          expect(cap.velocity).toEqual(s.velocity);
        }
      }
    });

    for (const c of FX.teleop) {
      it(`the Teleop loop matches exactly (${c.label})`, async () => {
        const clock = new VirtualClock({ start: 100 });
        const stop = new AbortController();
        const steps: TeleopStep[] = [];
        let stopTime: number | null = null;
        const follower = new FakeArm({ positions: c.start, state: ArmState.ENABLED, joints: NAMES.map((n) => joint(n, -3, 3)) });
        const o = c.options;
        const loop = new Teleop(new ScriptedLeader(c.script), follower, {
          rateHz: o.rate_hz, maxVelocity: nullToUndefined(o.max_velocity ?? null), maxAcceleration: nullToUndefined(o.max_acceleration ?? null),
          syncThreshold: o.sync_threshold, syncVelocity: o.sync_velocity, missLimit: o.miss_limit, clock, logger: new RecordingLogger(),
          onStep: (step) => {
            steps.push(step);
            if (steps.length >= c.cycles) {
              stopTime = clock.now();
              stop.abort();
            }
          },
        });
        expect(await loop.run(stop.signal)).toBe(c.reason);
        expect(loop.startupGap).toEqual(c.startup_gap);
        expect(steps.map((s) => ({ elapsed_s: s.elapsedS, leader: s.leader, command: s.command, syncing: s.syncing })))
          .toEqual(c.steps);
        expect(stopTime).toEqual(c.stop_time);
      });
    }
  });
}
