// Cross-language fixture contract/fixtures/teleop_star.json (tools/gen_teleop_star.py): the Python
// StarLeader on the real motorbridge-smart-servo library, against simulated servos behind a pty.
// The TypeScript StarLeader on FashionStarBus and SimulatedStarPort must write the same bytes, get
// the same replies and return the same results.
import { describe, expect, it } from "vitest";
import { VirtualClock } from "@makermods/arm-core/testing";
import { LeaderError } from "../src/leaders/base.js";
import { SimulatedStarPort } from "../src/leaders/sim.js";
import { StarLeader } from "../src/leaders/star.js";
import { JointMapper, type StarMappingJoint } from "../src/mapping.js";
import { type Json, loadFixture } from "./helpers/math-fixtures.js";
import { RecordingLogger } from "./shared/fakes.js";

const FX: Json = loadFixture("teleop_star.json");

const hex = (chunks: readonly Uint8Array[]): string =>
  chunks.map((c) => Array.from(c, (b) => b.toString(16).padStart(2, "0")).join("")).join("");

describe("Star leader on the wire", () => {
  for (const c of FX.cases) {
    it(`matches the Python library byte for byte (${c.label})`, async () => {
      const port = new SimulatedStarPort(new Map((c.ids as number[]).map((id) => [id, 0.0])));
      const leader = new StarLeader(port, {
        mapping: new JointMapper(c.joints as StarMappingJoint[]), missLimit: c.miss_limit, clock: new VirtualClock(),
        logger: new RecordingLogger(),
      });
      let written = 0;
      let replies = 0;
      for (const [i, step] of (c.steps as Json[]).entries()) {
        const where = `step ${i} (${step.action})`;
        if (step.action === "angles") {
          for (const [id, deg] of step.angles) port.setAngle(id, deg);
          continue;
        }
        if (step.action === "online") {
          for (const id of step.ids) port.setOnline(id, step.online);
          continue;
        }
        let result: unknown = null;
        let error: [string, string] | null = null;
        try {
          result = (await (leader as unknown as Record<string, () => Promise<unknown>>)[step.action]!.call(leader)) ?? null;
        } catch (e) {
          if (!(e instanceof LeaderError)) throw e;
          error = ["LeaderError", e.message.replace("the serial port", "<port>")];
        }
        expect(hex(port.written.slice(written)), where).toBe(step.written);
        expect(hex(port.replies.slice(replies)), where).toBe(step.replies);
        written = port.written.length;
        replies = port.replies.length;
        expect(result, where).toEqual(step.result);
        expect(error, where).toEqual(step.error);
        if (step.misses) expect([...leader.misses], where).toEqual(step.misses);
      }
      await leader.close();
      expect(port.isOpen).toBe(false);
    });
  }
});
