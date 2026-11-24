/** Which port an SO-101 is on (Swing to find) and which arm a port drives (Wiggle), lifted from the console's
hardware/so101.ts (MakerLab identify.py and wiggle.py). Swing to find never touches torque; Wiggle energizes only the
gripper, and switches it off again with read-back. */

import type { Clock } from "@makermods/arm-core";
import { msClock } from "./clock-ms.js";
import { openSo101Link, portLabel, type FeetechBus, type SerialPortLike, type So101Link } from "./bus.js";
import { FeetechError, STS3215_MODEL } from "./protocol.js";
import { DISABLE_RETRIES, MAX_TICK, So101Error, switchOff } from "./so101.js";

/** identify._SWING_THRESHOLD_TICKS: the base must turn this far both above and below where it started. */
export const SWING_TICKS = 120;
export const SWING_POLL_MS = 1000 / 15;
export const SWING_TIMEOUT_MS = 20_000;
/** The gripper's jog either side of where it is, ticks. */
export const WIGGLE_OFFSET_TICKS = 200;
export const WIGGLE_REPEATS = 3;
export const WIGGLE_DWELL_MS = 300;
/** Back where it was: polled for up to this long until within WIGGLE_ARRIVED_TICKS. */
export const WIGGLE_RETURN_MS = 2_000;
export const WIGGLE_ARRIVED_TICKS = 10;

export const SO101_NO_ARM = "No SO-101 answered on the USB serial ports. Check that the arms are powered and plugged in.";
export const SO101_NO_MOTION =
  "No motion detected within 20s — make sure the arm is powered and swing its base wide to the left AND the right (10–15° past where it started, each way).";

/** A port to look at, with an optional label for messages. */
export type PortCandidate = SerialPortLike & { readonly label?: string };

/** What Swing to find answers: the port whose base swung, and its index in the list given. */
export interface IdentifyResult {
  success: boolean;
  port?: PortCandidate;
  index?: number;
  message: string;
}

/** What Wiggle answers. */
export interface WiggleResult {
  success: boolean;
  message: string;
}

const labelOf = (c: PortCandidate) => c.label ?? portLabel(c);

/** One raw position of servo 1 (shoulder_pan). */
async function panTicks(bus: FeetechBus): Promise<number> {
  const read = await bus.syncRead([1], "Present_Position");
  const t = read.get(1);
  if (t === undefined) throw new FeetechError("No position from servo 1");
  return t;
}

/** A ping and a Model_Number check (LeRobot's handshake). */
async function handshake(bus: FeetechBus, id: number): Promise<void> {
  if (!(await bus.ping(id))) throw new FeetechError(`servo ${id} did not answer`);
  const model = await bus.read(id, "Model_Number");
  if (model !== STS3215_MODEL) throw new FeetechError(`servo ${id} is model ${model}, not an STS3215 (${STS3215_MODEL})`);
}

/** How Swing to find runs. */
export interface IdentifyOptions {
  clock: Clock;
  /** Aborting ends the watch at the next poll and answers "Stopped.". */
  signal?: AbortSignal;
  /** Seconds to watch (default 20). */
  timeout?: number;
  /** Seconds between sweeps (default 1/15). */
  pollInterval?: number;
  log?: (message: string) => void;
}

/** identify.identify_by_motion for an SO-101 (leader or follower alike, torque never touched): every port opened,
servo 1 watched, the first port whose base swung SWING_TICKS both ways from its first reading wins. Every port is
closed again before it resolves. */
export async function identifySo101(candidates: readonly PortCandidate[], options: IdentifyOptions): Promise<IdentifyResult> {
  const { signal, log } = options;
  const timeoutMs = options.timeout === undefined ? SWING_TIMEOUT_MS : options.timeout * 1000;
  const pollMs = options.pollInterval === undefined ? SWING_POLL_MS : options.pollInterval * 1000;
  if (!candidates.length) throw new So101Error("No USB serial port is plugged in. Plug in the arm and rescan.");
  const ms = msClock(options.clock);
  const links: So101Link[] = [];
  try {
    const watched: Array<{ candidate: PortCandidate; index: number; link: So101Link; baseline: number; low: number; high: number }> = [];
    await Promise.all(
      candidates.map(async (candidate, index) => {
        let link: So101Link | null = null;
        try {
          link = await openSo101Link(candidate, options.clock);
          links.push(link);
          const bus = link.bus;
          const first = await link.arbiter.run(async () => {
            await handshake(bus, 1);
            return panTicks(bus);
          });
          watched.push({ candidate, index, link, baseline: first, low: first, high: first });
        } catch (exc) {
          log?.(`so101 identify: skipping ${labelOf(candidate)}: ${exc instanceof Error ? exc.message : String(exc)}`);
          await link?.close().catch(() => undefined);
        }
      }),
    );
    if (signal?.aborted) return { success: false, message: "Stopped." };
    if (!watched.length) throw new So101Error(SO101_NO_ARM);
    const end = ms.now() + timeoutMs;
    while (ms.now() < end && !signal?.aborted) {
      for (const w of watched) {
        if (signal?.aborted) break;
        let t: number;
        try {
          t = await w.link.arbiter.run(() => panTicks(w.link.bus));
        } catch {
          continue; // a missed reply; read it again next sweep
        }
        w.low = Math.min(w.low, t);
        w.high = Math.max(w.high, t);
        if (w.high - w.baseline >= SWING_TICKS && w.baseline - w.low >= SWING_TICKS) {
          return { success: true, port: w.candidate, index: w.index, message: `Found the arm on ${labelOf(w.candidate)}.` };
        }
      }
      if (signal?.aborted) break;
      await ms.sleep(pollMs, signal);
    }
    return { success: false, message: signal?.aborted ? "Stopped." : SO101_NO_MOTION };
  } finally {
    await Promise.all(links.map((link) => link.close().catch(() => undefined)));
  }
}

/** wiggle.plan_wiggle: the jog's two ends inside the servo's EEPROM limits (the firmware clamps goals to them, so a
jog planned against 0-4095 can move the wrong way), and where it returns. */
export function planWiggle(current: number, minLimit: number, maxLimit: number, offset = WIGGLE_OFFSET_TICKS): { high: number; low: number; rest: number } {
  const lo = Math.max(minLimit, 0);
  const hi = Math.min(maxLimit, MAX_TICK);
  if (hi - lo < 2 * offset) throw new So101Error(`The gripper's limits (${lo}..${hi}) are too narrow to wiggle it; calibrate the arm first`);
  if (!(lo <= current && current <= hi)) throw new So101Error(`The gripper reads ${current}, outside its limits ${lo}..${hi}; it can't be wiggled and returned`);
  return { high: Math.min(current + offset, hi), low: Math.max(current - offset, lo), rest: current };
}

/** wiggle.wiggle_gripper on an open link: the gripper (servo 6) jogged open and shut WIGGLE_REPEATS times, so the
operator sees which arm it is; then back where it was and the torque off. Only the gripper is ever energized. The
caller holds the link's arbiter. `signal` ends the jog early (the gripper still returns before it is switched off);
`halt` is an e-stop: no return, the gripper switched off at once. */
export async function wiggleGripper(link: So101Link, label: string, signal?: AbortSignal, halt?: AbortSignal): Promise<WiggleResult> {
  const { bus, clock } = link;
  let rest: number | null = null;
  const either = new AbortController();
  for (const sig of [signal, halt]) {
    if (sig?.aborted) either.abort();
    else sig?.addEventListener("abort", () => either.abort(), { once: true });
  }
  const halted = () => {
    if (halt?.aborted) throw new So101Error("E-stop: the gripper was switched off");
  };
  try {
    await handshake(bus, 6);
    const current = (await bus.syncRead([6], "Present_Position")).get(6);
    if (current === undefined) throw new So101Error("The gripper (servo 6) did not report its position");
    const plan = planWiggle(current, await bus.read(6, "Min_Position_Limit"), await bus.read(6, "Max_Position_Limit"));
    // Its goal is where it is before the torque goes on: a stale goal would snap it.
    await bus.write(6, "Goal_Position", current);
    rest = plan.rest;
    halted();
    await bus.write(6, "Torque_Enable", 1);
    await bus.write(6, "Lock", 1);
    for (let i = 0; i < WIGGLE_REPEATS && !either.signal.aborted; i++) {
      await bus.write(6, "Goal_Position", plan.high);
      await clock.sleep(WIGGLE_DWELL_MS, either.signal);
      halted();
      await bus.write(6, "Goal_Position", plan.low);
      await clock.sleep(WIGGLE_DWELL_MS, either.signal);
    }
    halted();
    return { success: true, message: `Wiggled the gripper on ${label}.` };
  } catch (exc) {
    return { success: false, message: exc instanceof Error ? exc.message : String(exc) };
  } finally {
    if (rest !== null && !halt?.aborted) {
      // Back where it was: polled every 50 ms for up to 2 s until within 10 ticks, then the torque off (the default
      // disconnect's disable_torque: Torque_Enable 0, Lock 0).
      try {
        await bus.write(6, "Goal_Position", rest);
        const end = clock.now() + WIGGLE_RETURN_MS;
        while (clock.now() < end) {
          const at = (await bus.syncRead([6], "Present_Position")).get(6);
          if (at !== undefined && Math.abs(at - rest) <= WIGGLE_ARRIVED_TICKS) break;
          await clock.sleep(50, halt);
          if (halt?.aborted) break;
        }
      } catch {
        // switched off below either way
      }
    }
    if (rest !== null) {
      for (let attempt = 0; attempt < DISABLE_RETRIES; attempt++) {
        if (await switchOff(bus, 6).catch(() => false)) break;
      }
    }
  }
}

/** Opens `candidate`, wiggles its gripper (wiggleGripper), and closes it. */
export async function wiggleSo101(candidate: PortCandidate, options: { clock: Clock; signal?: AbortSignal }): Promise<WiggleResult> {
  const link = await openSo101Link(candidate, options.clock);
  try {
    return await link.arbiter.run(() => wiggleGripper(link, labelOf(candidate), options.signal));
  } finally {
    await link.close();
  }
}
