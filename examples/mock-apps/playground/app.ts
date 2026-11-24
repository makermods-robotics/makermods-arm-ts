/**
 * The playground's loop in a page (makermods_playground.dream: client.py, runner.py, scheduling.py, simplified to one
 * arm): camera frames and the arm's tool pose go into the Dream client's observation window; when the pacer says a
 * chunk is due, getAction() goes through the proxy; the reply's end-effector head is solved to joint rows with the
 * arm's trajectory IK (decode.ts) and played to the arm at the action rate, its own control loop limiting each step.
 */

import { DREAM_1_BETA, DreamClient } from "@makermods/dreamlabs";
import { bytesToBase64, CONSOLE_FRAME, encodeJpegFrame } from "@makermods/dreamlabs/browser";
import {
  type Arm, ArmState, Camera, GsUsbBackend, IKStream, listCameras, requestAdapter, TrajectoryIK,
} from "@makermods/metal-arm";
import { createArm } from "@makermods/metal-arm";
import { $, errorText, logPane, paneLogger, renderJoints, say } from "../shared/ui.ts";
import { headFromReply, poseToRot6, solveHead } from "./decode.ts";

const ACTION_HZ = 20;
const EXECUTION_HORIZON = 30;
const SYNC_ABOVE_MS = 700;
const OBS_PERIOD_MS = 100;
/** rad/s cap on the solved rows (the playground's first chunk runs at 0.5). */
const SPEED_CAP = 0.5;
const SIDES = ["left", "right"] as const;
const PARTS: [string, number][] = [["arm", 6], ["gripper", 1], ["eef_xyz", 3], ["eef_rot6", 6], ["grip", 1]];
/** What the model reads: both arms' joints, grip and tool pose, and three views (client.py's embodiment()). */
const EMBODIMENT = {
  state: Object.fromEntries(SIDES.flatMap((side) => PARTS.map(([part, width]) => [`${side}_${part}`, { shape: [width], dtype: "float32" }]))),
  video: { overhead: {}, wrist_l: {}, wrist_r: {} },
  action: {},
};

const message = $("message");
const log = logPane($("log"));
const logger = paneLogger(log);
const video = $<HTMLVideoElement>("camera");
/** The scene camera (the SDK's Camera: newest frame kept, a dead camera reopened). */
let camera: Camera | null = null;
/** The size and quality the model is sent frames at (client.py: 320x176, quality 85). */
const FRAME = { width: CONSOLE_FRAME.width, height: CONSOLE_FRAME.height };
const ids = ["connect", "estop", "run", "stop"] as const;
const button = (id: (typeof ids)[number]) => $<HTMLButtonElement>(id);

let arm: Arm | null = null;
let episode: Episode | null = null;

function phase(text: string): void {
  $("phase").textContent = text;
}

function refreshButtons(): void {
  const state = arm?.state ?? ArmState.DISCONNECTED;
  button("connect").disabled = arm !== null;
  button("estop").disabled = arm === null;
  button("run").disabled = !(state === ArmState.CONNECTED || state === ArmState.ENABLED) || episode !== null;
  button("stop").disabled = arm === null || !(state === ArmState.ENABLED || state === ArmState.FAULT);
}

setInterval(() => {
  if (arm) {
    const s = arm.status();
    $("state").textContent = s.state;
    $("fault").textContent = s.faultReason ? `(${s.faultReason})` : "";
    renderJoints($<HTMLTableSectionElement>("joints"), s.joints);
  }
  refreshButtons();
}, 100);

/** A black frame for the views this robot has no camera for (as client.py sends them). */
async function blackFrame(): Promise<string> {
  const canvas = new OffscreenCanvas(320, 176);
  const g = canvas.getContext("2d")!;
  g.fillStyle = "#000";
  g.fillRect(0, 0, 320, 176);
  return encodeJpegFrame(canvas);
}

class Episode {
  readonly client: DreamClient;
  stopped = false;
  private rows: number[][] = [];
  private grips: number[] = [];
  private executed = 0;
  private inFlight = false;
  private chunks = 0;
  private played = 0;
  private observations = 0;
  private worstTracking = 0;
  /** Per chunk, the IK's worst position miss, m. */
  private misses: number[] = [];
  private stream: IKStream | null = null;
  private timers: number[] = [];
  private finished!: (outcome: string) => void;
  readonly done = new Promise<string>((resolve) => (this.finished = resolve));

  constructor(readonly arm: Arm, proxyUrl: string, readonly prompt: string, readonly maxChunks: number, readonly black: string) {
    // No key here: the proxy adds it.
    this.client = new DreamClient({ proxyUrl, onWarning: (w) => log(`policy warning: ${w.message}`) });
  }

  async start(): Promise<void> {
    const a = this.arm;
    phase("opening the policy");
    await this.client.policy({
      checkpoint: DREAM_1_BETA, embodiment: EMBODIMENT, robot: "other_embodiment", actionHz: ACTION_HZ,
      executionHorizon: EXECUTION_HORIZON, syncAboveMs: SYNC_ABOVE_MS, timeoutMs: 15_000,
    });
    if (a.state === ArmState.CONNECTED) await a.enable();
    const ik = TrajectoryIK.fromConfig(a.config, { chain: a.chain });
    // The model was trained from the arm's working pose: start there.
    phase("moving to the ready pose");
    const ready = [...ik.rest, a.getCommandedPositions()[6]!];
    if (!(await a.moveTo(ready, { speed: 0.8 }))) throw new Error("the arm did not reach the ready pose");
    if (this.stopped) return;
    this.stream = new IKStream(ik, a.getCommandedPositions().slice(0, ik.n), ACTION_HZ, SPEED_CAP);
    phase("running");
    this.observe();
    this.timers.push(window.setInterval(() => this.observe(), OBS_PERIOD_MS));
    this.timers.push(window.setInterval(() => this.tick(), 1000 / ACTION_HZ));
  }

  /** One camera tick: the frame and the arm's state into the client's window. */
  private observe(): void {
    const frame = camera?.newest(0.5);
    if (this.stopped || !frame) return frame?.close();
    const a = this.arm;
    const q = a.getJointPositions();
    const tool = a.chain.fk(q.slice(0, a.chain.n));
    const opening = Math.min(1, Math.max(0, a.gripperOpening()));
    void frame.toJpeg(CONSOLE_FRAME.quality, FRAME).finally(() => frame.close()).then((jpeg) => {
      if (this.stopped) return;
      const overhead = bytesToBase64(jpeg);
      const observation: Record<string, unknown> = { prompt: this.prompt };
      // The model reads both arms: the left (absent) one is reported parked at the right one's pose, gripper closed.
      for (const side of SIDES) {
        const open = side === "right" ? opening : 0;
        observation[`state.${side}_arm`] = [0, 0, 0, 0, 0, 0];
        observation[`state.${side}_gripper`] = [open];
        observation[`state.${side}_grip`] = [open];
        observation[`state.${side}_eef_xyz`] = [tool[0]![3]!, tool[1]![3]!, tool[2]![3]!];
        observation[`state.${side}_eef_rot6`] = poseToRot6(tool);
      }
      this.client.setObservation({ "video.overhead": overhead, "video.wrist_l": this.black, "video.wrist_r": this.black }, observation);
      $("observations").textContent = String(++this.observations);
    });
  }

  /** One action-rate tick: play the next row, and ask for the next chunk when the pacer says so. */
  private tick(): void {
    if (this.stopped) return;
    const a = this.arm;
    if (a.state !== ArmState.ENABLED) return void this.end(`the arm is ${a.state}${a.faultReason ? `: ${a.faultReason}` : ""}`, true);
    const measured = a.getJointPositions();
    const commanded = a.getCommandedPositions();
    this.worstTracking = Math.max(this.worstTracking, ...measured.slice(0, 6).map((m, i) => Math.abs(m - commanded[i]!)));
    $("tracking").textContent = `${this.worstTracking.toFixed(4)} rad`;
    if (this.executed < this.rows.length) {
      const row = [...this.rows[this.executed]!, a.gripperTarget(this.grips[this.executed]!)];
      a.setJointTargets(row);
      this.executed++;
      $("rows").textContent = String(++this.played);
      $("last").textContent = JSON.stringify(row.map((v) => Number(v.toFixed(4))));
    }
    const remaining = this.rows.length - this.executed;
    if (this.chunks >= this.maxChunks) {
      if (remaining === 0 && !this.inFlight) this.end(`Episode done: ${this.chunks} chunks, ${this.played} rows; holding.`, false);
      return;
    }
    if (!this.inFlight && this.observations > 0 && this.client.refillDue(this.executed, remaining)) void this.request();
  }

  private async request(): Promise<void> {
    this.inFlight = true;
    try {
      const action = (await this.client.getAction()) as { per_key?: Record<string, unknown> };
      if (this.stopped) return;
      const head = headFromReply(action.per_key, "right");
      // Continue from what was sent last (the IK reads the last three commands).
      const sent = this.rows.slice(Math.max(0, this.executed - 3), this.executed).reverse();
      if (sent.length) this.stream!.history = sent.map((r) => [...r]);
      const solved = await solveHead(this.stream!, head);
      if (this.stopped) return;
      // Recorded chunks are not continuations of each other, so a later one can start out of reach of the speed cap.
      this.misses.push(solved.posErr);
      $("ik").textContent = this.misses.map((m) => `${(m * 1000).toFixed(1)} mm`).join(", ");
      $("ik").dataset.misses = JSON.stringify(this.misses);
      this.rows = solved.q;
      this.grips = solved.grip;
      this.executed = 0;
      $("chunks-done").textContent = String(++this.chunks);
      log(`chunk ${this.chunks}: ${solved.q.length} rows, mode ${this.client.mode}, round trip ${this.client.lastTiming?.rttMs?.toFixed(0) ?? "?"} ms`);
    } catch (e) {
      if (this.stopped) return;
      // Nothing more is played: the arm holds its last target.
      this.rows = [];
      this.executed = 0;
      this.end(`Policy error: ${errorText(e)}; the arm is holding.`, true);
    } finally {
      this.inFlight = false;
    }
  }

  private end(text: string, error: boolean): void {
    if (this.stopped) return;
    this.halt();
    phase(error ? "stopped on an error" : "done");
    say(message, text, error ? "error" : "info");
    this.finished(text);
  }

  private halt(): void {
    this.stopped = true;
    for (const t of this.timers) clearInterval(t);
    this.timers = [];
  }

  async close(): Promise<void> {
    this.halt();
    this.finished("stopped");
    await this.client.close();
  }
}

button("connect").onclick = async () => {
  try {
    const device = await requestAdapter();
    if (!device) return say(message, "No adapter chosen.");
    const candidate = createArm(new GsUsbBackend(device, { logger }), { logger });
    await candidate.connect();
    arm = candidate;
    say(message, `Connected to ${device.serialNumber}.`);
    const [first] = await listCameras();
    if (!first) throw new Error("no camera found");
    camera = new Camera(first.id, { width: 640, height: 360 });
    await camera.open();
    video.srcObject = camera.stream as MediaStream;
    await video.play();
  } catch (e) {
    say(message, errorText(e), "error");
  }
  refreshButtons();
};

button("run").onclick = async () => {
  const a = arm!;
  const proxy = new URL($<HTMLInputElement>("proxy").value, location.href).href;
  const e = new Episode(a, proxy, $<HTMLInputElement>("prompt").value, Number($<HTMLInputElement>("chunks").value), await blackFrame());
  episode = e;
  refreshButtons();
  say(message, "Episode running.");
  try {
    await e.start();
    await e.done;
  } catch (err) {
    if (!e.stopped) {
      phase("stopped on an error");
      say(message, `Policy error: ${errorText(err)}; the arm is holding.`, "error");
    }
  }
  await e.client.close().catch(() => undefined);
  if (episode === e) episode = null;
  refreshButtons();
};

button("stop").onclick = async () => {
  const e = episode;
  episode = null;
  phase("stopping");
  await e?.close().catch(() => undefined);
  const a = arm!;
  if (await a.stop()) {
    phase("idle");
    say(message, "Stopped: at rest, torque released.");
  } else say(message, "Return to rest not verified: the arm keeps holding. Support it, then E-stop.", "error");
  refreshButtons();
};

button("estop").onclick = async () => {
  episode?.close().catch(() => undefined);
  episode = null;
  await arm?.estop();
  phase("idle");
  say(message, "E-stop: torque released on every motor.");
  refreshButtons();
};

window.addEventListener("pagehide", () => void arm?.estop());
refreshButtons();
