// Teleop end to end on simulated hardware: a simulated Star leader, and a floated second arm,
// driving the product's simulated arm (shipped profile) through its real control loop.
import { describe, expect, it } from "vitest";
import { ArmState, degrees, radians } from "@makermods/arm-core";
import { VirtualClock } from "@makermods/arm-core/testing";
import { ArmLeader } from "../../src/leaders/arm.js";
import { simulatedStarLeader } from "../../src/leaders/sim.js";
import { Teleop } from "../../src/teleop.js";
import type { ArmFakes, FakeMotorLike } from "./fakes.js";
import { RecordingLogger } from "./fakes.js";

/** Registers the tests on `fk`'s simulated arm. */
export function teleopEndToEndSuite<M extends FakeMotorLike>(fk: Pick<ArmFakes<M>, "name" | "product" | "simulatedArm">): void {
  describe(`${fk.name}: teleop on simulated hardware`, () => {
    it("a simulated Star leader drives the simulated arm onto the mapped pose", async () => {
      const clock = new VirtualClock({ start: 1000 });
      const logger = new RecordingLogger();
      const [follower] = fk.simulatedArm({ clock, logger });
      const [leader, port] = simulatedStarLeader({ product: fk.product, clock, logger });
      const mapping = fk.product.defaultStarMapping.joints;
      // A pose inside both the mapping's range and the follower's limits, as servo degrees.
      const margin = degrees(follower.config.limitMargin) + 1;
      const wanted = mapping.map((j, i) => {
        const joint = follower.config.joints[i]!;
        const lo = Math.max(j.range_deg[0]!, degrees(joint.lo) + margin);
        const hi = Math.min(j.range_deg[1]!, degrees(joint.hi) - margin);
        return lo + 0.3 * (hi - lo);
      });
      await follower.connect();
      await follower.enable();
      await leader.connect();
      port.setAngles(mapping.map((j, i) => wanted[i]! / j.direction));
      const stop = new AbortController();
      let cycles = 0;
      const teleop = new Teleop(leader, follower, {
        clock, logger, syncVelocity: 1.0, onStep: () => (++cycles >= 1200 ? stop.abort() : undefined),
      });
      expect(await teleop.run(stop.signal)).toBe("stopped");
      expect(follower.state).toBe(ArmState.ENABLED);
      const reached = follower.getJointPositions();
      wanted.forEach((deg, i) => expect(Math.abs(reached[i]! - radians(deg)), `joint ${i}`).toBeLessThan(0.02));
      await leader.close();
      await follower.disable();
    });

    it("a floated leader arm moved by hand drives the follower", async () => {
      const clock = new VirtualClock({ start: 1000 });
      const logger = new RecordingLogger();
      const [follower] = fk.simulatedArm({ clock, logger });
      const [leaderArm, leaderSim] = fk.simulatedArm({ clock, logger });
      const leader = new ArmLeader(leaderArm, follower);
      await follower.connect();
      await follower.enable();
      await leader.connect();
      expect(leaderArm.config.joints.every((j) => j.kp === 0)).toBe(true);
      // The hand moves the elbow 0.4 rad from where it started.
      const elbow = leaderArm.config.joints[2]!;
      const start = leaderArm.getJointPositions()[2]!;
      const goal = Math.min(elbow.hi - 0.1, start + 0.4);
      leaderSim.motor(2).position = goal * elbow.direction + elbow.offset;
      const stop = new AbortController();
      let cycles = 0;
      const teleop = new Teleop(leader, follower, { clock, logger, onStep: () => (++cycles >= 400 ? stop.abort() : undefined) });
      expect(await teleop.run(stop.signal)).toBe("stopped");
      expect(Math.abs(follower.getJointPositions()[2]! - goal)).toBeLessThan(0.02);
      await leader.close();
      expect(leaderArm.config.joints.every((j) => j.kp > 0)).toBe(true); // the profile's gains are back
      await leaderArm.disable();
      await follower.disable();
    });
  });
}
