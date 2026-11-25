/**
 * Sine tracking check of one arm, the same routine as the Python SDK's `check --sine`: the
 * selected joints (default: all but the gripper) follow a sine around the current pose while
 * commanded and measured positions are logged at the control rate. Reports per joint the RMS and
 * max tracking error, a lag estimate, peak torque and temperature rise. Afterwards the arm returns
 * to rest and releases.
 *
 *     node examples/node/check-sine.ts --product metal --port gs_usb:<serial> --yes --sine 10 --period 2 \
 *         --cycles 3 --at shoulder_lift=-30,elbow_flex=40 [--joints wrist_roll,wrist_yaw] \
 *         [--gravity-scale 1] [--json out.json] [--csv out.csv] [--frames sent.jsonl]
 *
 * This energizes the arm and moves it: start it with the arm resting where it may be released,
 * keep the workspace clear and a hand on the power switch. Ctrl-C stops the motion and returns the
 * arm to rest; a second Ctrl-C ends the return where it is and the arm holds. If the return cannot
 * be verified the arm is left holding at full gains, never released in the air.
 * The adapter must be listed for that product in the rig file (see common.ts).
 */

import { writeFileSync } from "node:fs";
import { createInterface } from "node:readline";
import { Arm, ArmState } from "@makermods/arm-core";
import { COMMON_OPTIONS, deg, openTarget, parseArgs, rad, recordSends, type SentFrame, writeFrames } from "./common.ts";

type Row = [t: number, cmd: number[], pos: number[], tau: number[], temp: number[]];

/** Sine around q0; the amplitude eases in over the first half period. */
function sineTargets(q0: number[], selected: number[], amplitude: number, period: number, t: number): number[] {
  const s = amplitude * Math.min(1.0, t / (period / 2)) * Math.sin((2 * Math.PI * t) / period);
  return q0.map((q, i) => (selected.includes(i) ? q + s : q));
}

/** Linear interpolation of (xs, ys) at x, clamped at the ends (numpy.interp). */
function interp(x: number, xs: number[], ys: number[]): number {
  if (x <= xs[0]!) return ys[0]!;
  const last = xs.length - 1;
  if (x >= xs[last]!) return ys[last]!;
  let lo = 0;
  let hi = last;
  while (hi - lo > 1) {
    const mid = (lo + hi) >> 1;
    if (xs[mid]! <= x) lo = mid;
    else hi = mid;
  }
  return ys[lo]! + ((ys[hi]! - ys[lo]!) * (x - xs[lo]!)) / (xs[hi]! - xs[lo]!);
}

/** The delay (s) that best lines the measured trace up with the commanded one. */
function lagEstimate(t: number[], cmd: number[], pos: number[], period: number, windowStart: number): number {
  let best = 0.0;
  let bestErr = Infinity;
  for (let k = 0; k * 0.001 < period / 4; k++) {
    const lag = k * 0.001;
    let sum = 0;
    let n = 0;
    for (let i = 0; i < t.length; i++) {
      if (t[i]! < windowStart + lag) continue;
      const e = pos[i]! - interp(t[i]! - lag, t, cmd);
      sum += e * e;
      n++;
    }
    if (n < 3) break;
    const err = Math.sqrt(sum / n);
    if (err < bestErr) {
      best = lag;
      bestErr = err;
    }
  }
  return best;
}

interface JointReport {
  rms_error: number;
  max_error: number;
  lag_s: number;
  peak_torque: number;
  temperature_rise: number;
}

function analyse(log: Row[], names: string[], selected: number[], period: number): Record<string, JointReport> {
  const t = log.map((row) => row[0]);
  const window = period / 2; // after the ease-in
  const report: Record<string, JointReport> = {};
  for (const i of selected) {
    const cmd = log.map((row) => row[1][i]!);
    const pos = log.map((row) => row[2][i]!);
    const tau = log.map((row) => row[3][i]!);
    const temp = log.map((row) => row[4][i]!);
    const err = cmd.map((c, k) => c - pos[k]!).filter((_, k) => t[k]! >= window);
    report[names[i]!] = {
      rms_error: err.length ? Math.sqrt(err.reduce((a, e) => a + e * e, 0) / err.length) : NaN,
      max_error: err.length ? Math.max(...err.map(Math.abs)) : NaN,
      lag_s: lagEstimate(t, cmd, pos, period, window),
      peak_torque: Math.max(...tau.filter(Number.isFinite).map(Math.abs)),
      temperature_rise: temp[temp.length - 1]! - temp[0]!,
    };
  }
  return report;
}

/** How evenly the control loop sent its ticks, from the recorded frames (one command per tick to the first joint). */
function tickStats(frames: SentFrame[], firstId: number, from: number, to: number) {
  const marks: number[] = [];
  let previous = -1;
  for (const f of frames) {
    if (f.id === firstId && previous !== firstId && f.t >= from && f.t <= to) marks.push(f.t);
    previous = f.id;
  }
  const periods = marks.slice(1).map((m, i) => (m - marks[i]!) * 1000).sort((a, b) => a - b);
  const at = (p: number) => periods[Math.min(periods.length - 1, Math.floor(p * periods.length))] ?? NaN;
  return {
    ticks: marks.length,
    rate_hz: marks.length > 1 ? (marks.length - 1) / (marks[marks.length - 1]! - marks[0]!) : NaN,
    period_ms: { median: at(0.5), p99: at(0.99), max: periods[periods.length - 1] ?? NaN },
  };
}

const AT_SPEED = rad(20.0); // rad/s, the move to --at

/** The pose `text` (NAME=DEG,...) describes, starting from `q`; checked completely before anything moves. */
function parsePose(text: string, names: string[], q: number[]): number[] {
  const pose = [...q];
  for (const item of text.split(",")) {
    const [name, value, ...rest] = item.split("=");
    const i = names.indexOf((name ?? "").trim());
    const degrees = Number(value);
    if (i < 0 || rest.length || value === undefined || value.trim() === "" || !Number.isFinite(degrees)) {
      throw new Error(`--at expects NAME=DEG,...; got "${item}"; joints: ${names.join(", ")}`);
    }
    pose[i] = rad(degrees);
  }
  return pose;
}

interface SineArgs {
  sine: number;
  period: number;
  cycles: number;
  joints: string[] | null;
}

async function sine(arm: Arm, a: SineArgs, signal: AbortSignal): Promise<{ log: Row[]; selected: number[]; amplitude: number }> {
  const cfg = arm.config;
  const names = arm.jointNames;
  const wanted = a.joints ?? names.filter((n) => n !== "gripper");
  const selected = wanted.map((n) => names.indexOf(n));
  const amplitude = rad(a.sine);
  const q0 = arm.getJointPositions();
  for (const i of selected) {
    const j = cfg.joints[i]!;
    if (!(j.lo + cfg.limitMargin <= q0[i]! - amplitude && q0[i]! + amplitude <= j.hi - cfg.limitMargin)) {
      throw new Error(`${j.label}: ${deg(q0[i]!).toFixed(1)} deg +/- ${a.sine} leaves its limits [${deg(j.lo).toFixed(1)}, ${deg(j.hi).toFixed(1)}]`);
    }
  }
  const rate = cfg.controlRateHz;
  const log: Row[] = [];
  const clock = arm.clock;
  const t0 = clock.now();
  const duration = a.cycles * a.period;
  while (!signal.aborted) {
    const t = clock.now() - t0;
    if (t > duration) break;
    const cmd = sineTargets(q0, selected, amplitude, a.period, t);
    if (!arm.setJointTargets(cmd)) throw new Error(`targets refused: ${arm.state} ${arm.faultReason ?? ""}`);
    log.push([t, cmd, arm.getJointPositions(), arm.getJointTorques(), arm.getTemperatures()]);
    await clock.sleep(Math.max(0.0, log.length / rate - (clock.now() - t0)));
  }
  await arm.moveTo(q0, { timeout: 2.0, signal });
  return { log, selected, amplitude };
}

/** Resolves when RELEASE is typed; never when there is no input to read. */
function releaseTyped(): Promise<void> {
  return new Promise<void>((resolve) => {
    const lines = createInterface({ input: process.stdin });
    lines.on("line", (line) => {
      if (line.trim().toUpperCase() === "RELEASE") {
        lines.close();
        resolve();
      } else {
        console.error("torque remains enabled; type RELEASE only when the arm is supported");
      }
    });
  });
}

/**
 * Keep an energized arm holding its present pose without releasing it: detach when no stiff joint
 * would release on its watchdog, else keep this process holding until RELEASE is typed.
 */
async function leaveHolding(arm: Arm): Promise<string> {
  arm.holdCurrentPosition();
  await arm.clock.sleep(0.3); // the hold has gone out on fresh feedback before anything else is decided
  const unsafe = arm.detachUnsafe();
  if (unsafe.length) {
    console.error(`${unsafe.join(", ")} would release on their motor watchdog if this process stopped commanding them`);
    console.error("this process keeps holding the arm. Type RELEASE and press ENTER only when it is safe to release torque");
    await releaseTyped();
    await arm.disable();
    return arm.unconfirmedDisable.length ? "unconfirmed" : "released";
  }
  const releasing = await arm.detach();
  console.error(`arm left holding with torque on (watchdog releases: ${JSON.stringify(releasing)}). `
    + "Bring it down with the Python SDK's 'rest' command; 'release' turns torque off where it is");
  return "holding";
}

/**
 * Return to rest and release torque, or leave the arm holding. Aborting `signal` (a second
 * Ctrl-C) ends the return where it is: stop() then leaves the arm holding. Returns the outcome.
 */
async function windDown(arm: Arm, signal: AbortSignal): Promise<string> {
  if (arm.state !== ArmState.ENABLED && arm.state !== ArmState.FAULT) return "not_enabled";
  console.log("returning to the rest pose...");
  let verified = false;
  try {
    verified = await arm.stop({ signal });
  } catch (e) {
    console.error(`return to rest failed: ${e instanceof Error ? e.message : e}`);
  }
  if (verified) {
    if (arm.unconfirmedDisable.length) {
      console.error(`rest pose reached, but torque off was not confirmed on motors ${JSON.stringify(arm.unconfirmedDisable)}; cut motor power if they hold`);
      return "unconfirmed";
    }
    console.log("rest pose reached; torque released");
    return "rest";
  }
  console.error("could not verify the return to rest; the arm is holding its pose");
  return leaveHolding(arm);
}

const args = parseArgs(process.argv.slice(2), {
  ...COMMON_OPTIONS, sine: "value", period: "value", cycles: "value", at: "value", joints: "value", "gravity-scale": "value",
  json: "value", csv: "value", yes: "flag",
});
const a: SineArgs = {
  sine: Number(args.sine ?? 10),
  period: Number(args.period ?? 2),
  cycles: Number(args.cycles ?? 3),
  joints: typeof args.joints === "string" ? args.joints.split(",").map((n) => n.trim()) : null,
};
if (!(a.sine > 0 && a.sine <= 45) || !(a.period > 0) || !(Number.isInteger(a.cycles) && a.cycles >= 1)) {
  throw new Error("--sine must be in (0, 45] deg, --period > 0 and --cycles a whole number >= 1");
}
if (!args.yes) throw new Error("this energizes the motors and moves the arm: pass --yes once the workspace is clear");
const overrides = args["gravity-scale"] !== undefined ? { gravity_scale: Number(args["gravity-scale"]) } : {};
const { config, backend } = await openTarget(args, overrides);
const names = config.joints.map((j) => j.label);
const unknown = (a.joints ?? []).filter((n) => !names.includes(n));
if (unknown.length) throw new Error(`unknown joints ${JSON.stringify(unknown)}; joints: ${names.join(", ")}`);
const atPose = typeof args.at === "string" ? parsePose(args.at, names, new Array<number>(names.length).fill(0)) : null; // syntax check only
const peak = (rad(a.sine) * 2 * Math.PI) / a.period;
if (peak > config.maxVelocity) {
  throw new Error(`the sine's peak speed ${deg(peak).toFixed(0)} deg/s exceeds the arm's limit ${deg(config.maxVelocity).toFixed(0)} deg/s`);
}
const frames = typeof args.frames === "string" ? recordSends(backend) : null;
const arm = new Arm(config, backend);
console.log(`will enable ${args.product} (${backend.port}) and run a ${a.sine} deg, ${a.period} s sine for ${a.cycles} cycles `
  + `(peak ${deg(peak).toFixed(0)} deg/s), then: rest`);

// The first Ctrl-C or SIGTERM stops the motion; the arm then returns to rest. A second one ends the
// return where it is and the arm keeps holding at full gains. Nothing here releases an arm that is
// in the air.
const stop = new AbortController();
const interruptRest = new AbortController();
for (const name of ["SIGINT", "SIGTERM"] as const) {
  process.on(name, () => {
    if (!stop.signal.aborted) {
      stop.abort();
    } else if (!interruptRest.signal.aborted) {
      console.error("interrupted again: the return to rest stops here and the arm keeps holding");
      interruptRest.abort();
    } else {
      console.error("already stopping: the arm keeps holding");
    }
  });
}

let run: { log: Row[]; selected: number[]; amplitude: number } | null = null;
let window: [number, number] | null = null;
let failure: unknown = null;
let then = "failed";
try {
  await arm.connect();
  await arm.enable({ signal: stop.signal });
  await arm.clock.sleep(0.3);
  if (atPose && typeof args.at === "string") {
    console.log(`moving to ${args.at}`);
    const q = parsePose(args.at, arm.jointNames, arm.getJointPositions());
    if (!(await arm.moveTo(q, { speed: AT_SPEED, signal: stop.signal }))) throw new Error("could not reach the --at pose");
    await arm.clock.sleep(1.0);
  }
  const began = performance.now() / 1000;
  run = await sine(arm, a, stop.signal);
  window = [began, began + a.cycles * a.period];
} catch (e) {
  failure = e;
} finally {
  try {
    then = await windDown(arm, interruptRest.signal);
  } catch (e) {
    console.error(`wind-down failed: ${e instanceof Error ? e.message : e}`);
    then = "failed";
  }
  if (arm.state === ArmState.ENABLED || arm.state === ArmState.FAULT) {
    // Still energized after a failed wind-down: leave it holding rather than closing the bus on it.
    try {
      then = await leaveHolding(arm);
    } catch (e) {
      console.error(`could not leave the arm holding cleanly: ${e instanceof Error ? e.message : e}`);
    }
  } else if (arm.state !== ArmState.DISCONNECTED) {
    await arm.disconnect();
  }
}

// Everything below runs with the arm at rest or handed over: analysis and files never delay the control loop.
if (failure) console.error(failure instanceof Error ? failure.message : failure);
const result: Record<string, unknown> = { mode: "sine", period: a.period, cycles: a.cycles, then, units: "rad, s, N*m, C" };
if (run && run.log.length >= 10) {
  const { log, selected, amplitude } = run;
  const report = analyse(log, arm.jointNames, selected, a.period);
  const seconds = log[log.length - 1]![0];
  console.log(`${log.length} samples over ${seconds.toFixed(1)} s (${(log.length / seconds).toFixed(0)} Hz)`);
  console.log("joint            rms_deg  max_deg  lag_ms  peak_Nm  temp_rise_C");
  for (const [name, r] of Object.entries(report)) {
    console.log(`${name.padEnd(16)} ${deg(r.rms_error).toFixed(2).padStart(7)}  ${deg(r.max_error).toFixed(2).padStart(7)}  `
      + `${(r.lag_s * 1000).toFixed(0).padStart(6)}  ${r.peak_torque.toFixed(2).padStart(7)}  ${r.temperature_rise.toFixed(1).padStart(11)}`);
  }
  Object.assign(result, { amplitude, samples: log.length, joints: report });
  if (typeof args.csv === "string") {
    const head = ["t", ...selected.flatMap((i) => ["cmd", "pos", "torque", "temp"].map((k) => `${arm.jointNames[i]}_${k}`))];
    const rows = log.map(([t, cmd, pos, tau, temp]) =>
      [t.toFixed(4), ...selected.flatMap((i) => [cmd[i]!, pos[i]!, tau[i]!, temp[i]!].map((v) => v.toPrecision(6)))].join(","));
    writeFileSync(args.csv, [head.join(","), ...rows].join("\n") + "\n");
  }
  if (frames && window) {
    const ticks = tickStats(frames, config.joints[0]!.motorId, window[0], window[1]);
    result.control_loop = ticks;
    console.log(`control loop: ${ticks.rate_hz.toFixed(1)} Hz (target ${config.controlRateHz}); `
      + `tick period median ${ticks.period_ms.median.toFixed(2)} ms, p99 ${ticks.period_ms.p99.toFixed(2)} ms, max ${ticks.period_ms.max.toFixed(2)} ms`);
  }
} else if (!failure) {
  console.error("interrupted before enough samples");
}
if (frames && typeof args.frames === "string") writeFrames(args.frames, frames);
result.ok = !failure && then === "rest" && run !== null && run.log.length >= 10;
console.log(`then: ${then}`);
if (typeof args.json === "string") writeFileSync(args.json, JSON.stringify(result, null, 2));
process.exit(result.ok ? 0 : 1);
