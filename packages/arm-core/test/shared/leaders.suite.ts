// Port of tests/unit/test_leaders.py (identical in both Python SDKs), on VirtualClock.
import { describe, expect, it } from "vitest";
import { ArmState, radians, ValueError } from "@makermods/arm-core";
import { VirtualClock } from "@makermods/arm-core/testing";
import { ArmLeader, Friction } from "../../src/leaders/arm.js";
import { isLeader, LeaderError } from "../../src/leaders/base.js";
import { StarLeader, type StarLeaderOptions } from "../../src/leaders/star.js";
import { SimulatedStarPort } from "../../src/leaders/sim.js";
import { JointMapper, type StarMappingJoint } from "../../src/mapping.js";
import type { ArmFakes } from "./fakes.js";
import { RecordingLogger } from "./fakes.js";
import { FakeArm, FakeServoBus, joint, NAMES } from "./teleop-fakes.js";

const JOINTS: StarMappingJoint[] = [
  { name: "a", servo: 0, direction: 1.0, range_deg: [-90, 90] },
  { name: "b", servo: 1, direction: -2.0, range_deg: [-120, 0] },
];

const close = (a: number, b: number): boolean => Math.abs(a - b) <= Math.max(1e-6 * Math.abs(b), 1e-12);
function expectApprox(actual: readonly number[] | null, expected: readonly number[]): void {
  expect(actual).not.toBeNull();
  expect(actual!.length).toBe(expected.length);
  actual!.forEach((a, i) => expect(close(a, expected[i]!), `[${i}] ${a} ≈ ${expected[i]}`).toBe(true));
}

async function star(bus: FakeServoBus, options: StarLeaderOptions = {}): Promise<StarLeader> {
  const leader = new StarLeader(new SimulatedStarPort([]), {
    mapping: new JointMapper(JOINTS), busFactory: () => bus, clock: new VirtualClock(), logger: new RecordingLogger(), ...options,
  });
  await leader.connect();
  return leader;
}

const FRICTION = { frictionViscous: 0.5, frictionCoulomb: 0.1, frictionRestNm: 0.06, frictionVelocityClamp: 1.0 };

function frictionArm(): FakeArm {
  const joints = [...NAMES.slice(0, 6).map((n) => joint(n)), joint("gripper", -2.0, 2.0, FRICTION)];
  joints[1] = joint("shoulder_lift", -2.0, 2.0, { frictionViscous: 1.0 });
  return new FakeArm({ joints, state: ArmState.ENABLED, frictionDeadzone: 0.05 });
}

/** Registers the tests on `fk`'s arm (the last test uses its simulated arm and product). */
export function leadersSuite(fk: Pick<ArmFakes, "name" | "product" | "makeArm">): void {
  describe(`${fk.name}: Star leader`, () => {
    it("the Leader interface is met", async () => {
      expect(isLeader(await star(new FakeServoBus({ 0: 0.0, 1: 0.0 })))).toBe(true);
      expect(isLeader(new ArmLeader(new FakeArm()))).toBe(true);
    });

    it("connect pings, then unlocks, then resets every servo", async () => {
      const bus = new FakeServoBus({ 0: 0.0, 1: 0.0 });
      await star(bus);
      expect(bus.calls.map((c) => c[0])).toEqual(["ping", "ping", "unlock", "unlock", "reset_multi_turn", "reset_multi_turn"]);
    });

    it("connect refuses a silent servo and closes the port", async () => {
      const bus = new FakeServoBus({ 0: 0.0, 1: 0.0 }, [], [1]);
      await expect(star(bus)).rejects.toThrow(LeaderError);
      await expect(star(new FakeServoBus({ 0: 0.0, 1: 0.0 }, [], [1]))).rejects.toThrow(/\[1\]/);
      expect(bus.closed).toBe(true);
    });

    it("read maps servo degrees to follower radians", async () => {
      const leader = await star(new FakeServoBus({ 0: 30.0, 1: 20.0 }));
      expectApprox(await leader.read(), [radians(30.0), radians(-40.0)]);
    });

    it("a missed servo keeps its last target", async () => {
      const bus = new FakeServoBus({ 0: 30.0, 1: 20.0 });
      const leader = await star(bus);
      const first = (await leader.read())!;
      bus.silent = new Set([1]);
      bus.angles.set(0, 40.0);
      expectApprox(await leader.read(), [radians(40.0), first[1]!]);
      expect(leader.misses).toEqual(new Map([[0, 0], [1, 1]]));
    });

    it("no target until every servo has replied once", async () => {
      const leader = await star(new FakeServoBus({ 0: 30.0, 1: 20.0 }, [1]));
      expect(await leader.read()).toBeNull();
    });

    it("a servo lost for missLimit reads ends the session", async () => {
      const bus = new FakeServoBus({ 0: 30.0, 1: 20.0 });
      const leader = await star(bus, { missLimit: 3 });
      await leader.read();
      bus.silent = new Set([0]);
      await leader.read();
      await leader.read();
      await expect(leader.read()).rejects.toThrow(/\[0\]/);
    });

    it("a bus error is a miss on every servo", async () => {
      const bus = new FakeServoBus({ 0: 30.0, 1: 20.0 });
      const leader = await star(bus, { missLimit: 2 });
      await leader.read();
      bus.raiseNext = new Error("serial timeout");
      expect(await leader.read()).toBeNull();
      bus.raiseNext = new Error("serial timeout");
      const reading = leader.read();
      await expect(reading).rejects.toThrow(LeaderError);
      await expect(reading).rejects.toThrow(/serial timeout/);
    });

    it("a reply after misses resets the count", async () => {
      const bus = new FakeServoBus({ 0: 30.0, 1: 20.0 });
      const leader = await star(bus, { missLimit: 2 });
      bus.silent = new Set([1]);
      await leader.read();
      bus.silent = new Set();
      await leader.read();
      bus.silent = new Set([1]);
      await leader.read(); // one miss again, not two
      expect(leader.misses.get(1)).toBe(1);
    });

    it("zero sets every origin and resets the turn counters", async () => {
      const bus = new FakeServoBus({ 0: 30.0, 1: 20.0 });
      const leader = await star(bus);
      await leader.read();
      bus.calls.length = 0;
      await leader.zero();
      expect(bus.calls.filter((c) => c[0] === "set_origin_point")).toEqual([["set_origin_point", 0], ["set_origin_point", 1]]);
      expect(bus.calls.slice(-2)).toEqual([["reset_multi_turn", 0], ["reset_multi_turn", 1]]);
      expectApprox(await leader.read(), [0.0, 0.0]);
    });

    it("close tolerates a vanished port", async () => {
      const bus = new FakeServoBus({ 0: 0.0, 1: 0.0 });
      const leader = await star(bus);
      bus.close = async () => {
        throw new Error("device gone");
      };
      await leader.close();
      expect(leader.bus).toBeNull();
    });

    it("the default mapping is the product's", () => {
      // Python: the default comes from _product and the servo library is imported lazily. arm-core
      // never names a product, so the follower's product is passed; without it there is no default.
      const leader = new StarLeader(new SimulatedStarPort([]), { product: fk.product });
      expect(leader.servoIds.length).toBe(7);
      expect(() => new StarLeader(new SimulatedStarPort([]))).toThrow(ValueError);
    });
    it("a mapping for another arm is refused for this follower", () => {
      const shipped = fk.product.defaultStarMapping;
      expect(() => new StarLeader(new SimulatedStarPort([]), { product: fk.product, mapping: shipped })).not.toThrow();
      const reversed = { joints: [...shipped.joints].reverse() };
      expect(() => new StarLeader(new SimulatedStarPort([]), { product: fk.product, mapping: reversed })).toThrow(/are not the/);
      expect(() => new StarLeader(new SimulatedStarPort([]), { product: fk.product, mapping: new JointMapper(JOINTS) }))
        .toThrow(ValueError);
      expect(() => new StarLeader(new SimulatedStarPort([]), { mapping: new JointMapper(JOINTS) })).not.toThrow();
    });
  });

  describe(`${fk.name}: arm leader`, () => {
    it("enables, then floats", async () => {
      const arm = new FakeArm({ state: ArmState.DISCONNECTED });
      await new ArmLeader(arm).connect();
      expect(arm.calls).toEqual(["connect", "enable", "float_mode"]);
      expect(arm.floatArgs).toEqual([]);
    });

    it("reads positions clipped to the follower", async () => {
      const follower = new FakeArm({ joints: NAMES.map((n) => joint(n, -1.0, 1.0)) });
      const arm = new FakeArm({ positions: [0.5, -1.5, 1.5, 0, 0, 0, 0.2], state: ArmState.ENABLED });
      expect(await new ArmLeader(arm, follower).read()).toEqual([0.5, -1.0, 1.0, 0, 0, 0, 0.2]);
    });

    it("has no reading unless enabled and finite", async () => {
      const arm = new FakeArm({ state: ArmState.CONNECTED });
      const leader = new ArmLeader(arm);
      expect(await leader.read()).toBeNull();
      arm.state = ArmState.ENABLED;
      arm.positions[2] = NaN;
      expect(await leader.read()).toBeNull();
    });

    it("a faulted leader arm ends the session", async () => {
      const arm = new FakeArm({ state: ArmState.ENABLED });
      arm.fault("rotor overheat");
      await expect(new ArmLeader(arm).read()).rejects.toThrow(/rotor overheat/);
      await expect(new ArmLeader(arm).read()).rejects.toThrow(LeaderError);
    });

    it("close holds, then restores the profile gains", async () => {
      const arm = new FakeArm({ state: ArmState.ENABLED });
      await new ArmLeader(arm).close();
      expect(arm.calls).toEqual(["hold", ["set_gains", null, null]]); // no arguments: the profile's gains
    });

    it("a follower of another size is refused", () => {
      expect(() => new ArmLeader(new FakeArm(), new FakeArm({ joints: [joint("gripper")] }))).toThrow(ValueError);
    });

    it("passes its signal to the enable", async () => {
      const arm = new FakeArm({ state: ArmState.CONNECTED });
      const cancel = new AbortController();
      await new ArmLeader(arm).connect(cancel.signal);
      expect(arm.signal).toBe(cancel.signal);
    });

    it("close stiffens a real arm with the profile gains", async () => {
      const { arm } = fk.makeArm();
      const leader = new ArmLeader(arm);
      await leader.connect();
      expect(arm.config.joints.map((j) => j.kp)).toEqual([0.0, 0.0]);
      await leader.close();
      expect(arm.config.joints.map((j) => [j.kp, j.kd])).toEqual([[60, 2.0], [20, 0.5]]);
      await arm.disable();
    });
  });

  describe(`${fk.name}: friction compensation`, () => {
    it("friction terms come from the profile and reach floatMode", async () => {
      const arm = frictionArm();
      const leader = new ArmLeader(arm);
      await leader.connect();
      expect(arm.floatArgs![1]).toBe(leader.friction!.torques);
      const tau = leader.friction!.torques(new Array<number>(7).fill(0.0), [0.0, 0.2, 0.0, 0.0, 0.0, 0.0, -3.0]);
      expect(close(tau[1]!, 0.2)).toBe(true); // viscous only
      expect(close(tau[6]!, 0.5 * -1.0 - 0.1)).toBe(true); // clamped viscous + Coulomb, both along the motion
      expect(tau[0]).toBe(0.0);
    });

    it("inside the deadzone the rest torque only", () => {
      const friction = Friction.fromConfig(frictionArm().config)!;
      expect(friction.torques(new Array<number>(7).fill(0.0), [0.01, 0.04, 0, 0, 0, 0, 0.049])).toEqual([0, 0, 0, 0, 0, 0, 0.06]);
    });

    it("off when the profile has none", () => {
      expect(Friction.fromConfig(new FakeArm().config)).toBeNull();
      expect(Friction.fromConfig(frictionArm().config)!.torques(new Array<number>(7).fill(0), new Array<number>(7).fill(NaN)))
        .toEqual(new Array<number>(7).fill(0.0));
    });
  });
}
