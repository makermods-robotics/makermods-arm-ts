import "../shared/mock-gate.ts";
import { type Arm, type ArmGroup, ArmState, adapterPort, requestAdapter, Rig, runMany, Teleop } from "@makermods/arm-core";
import { MAKER_ARM } from "@makermods/maker-arm";
import { METAL_ARM } from "@makermods/metal-arm";
import { starPanel, type StarPanel } from "../shared/star-panel.ts";
import { $, deg, errorText, logPane, paneLogger, renderJoints, say } from "../shared/ui.ts";

const SIDES = ["left", "right"] as const;
type Side = (typeof SIDES)[number];
const message = $("message");
const log = logPane($("log"));
const logger = paneLogger(log);
const button = (id: string) => $<HTMLButtonElement>(id);
const rigText = $<HTMLTextAreaElement>("rig-text");
const granted: string[] = [];
let group: ArmGroup | null = null;
let leaders: Record<Side, StarPanel> | null = null;
let teleop: AbortController | null = null;
let busy = false;

const arm = (side: Side): Arm => group!.get(side);
const state = (side: Side) => (group ? arm(side).state : ArmState.DISCONNECTED);
const all = (wanted: ArmState) => group !== null && SIDES.every((s) => state(s) === wanted);
const any = (wanted: ArmState) => group !== null && SIDES.some((s) => state(s) === wanted);

function refresh(): void {
  button("connect").disabled = group !== null || busy;
  button("disconnect").disabled = group === null || busy || teleop !== null;
  button("enable").disabled = !all(ArmState.CONNECTED) || busy;
  for (const side of SIDES) button(`move-${side}`).disabled = state(side) !== ArmState.ENABLED || busy || teleop !== null;
  button("sine").disabled = !all(ArmState.ENABLED) || busy || teleop !== null;
  button("stop").disabled = !(any(ArmState.ENABLED) || any(ArmState.FAULT)) || busy || teleop !== null;
  button("estop").disabled = group === null;
  button("teleop-start").disabled = !all(ArmState.ENABLED) || busy || teleop !== null;
  button("teleop-stop").disabled = teleop === null;
  if (!group) return;
  for (const side of SIDES) {
    const s = arm(side).status();
    $(`${side}-state`).textContent = s.state;
    $(`${side}-fault`).textContent = s.faultReason ?? "";
    renderJoints($<HTMLTableSectionElement>(`${side}-joints`), s.joints);
  }
}
setInterval(refresh, 100);

async function action(work: () => Promise<unknown>): Promise<void> {
  busy = true;
  refresh();
  try {
    await work();
  } catch (e) {
    say(message, errorText(e), "error");
  } finally {
    busy = false;
    refresh();
  }
}

/** The rig file for the adapters granted so far: the first is the left arm, the second the right. */
function suggestRig(): void {
  const arms = Object.fromEntries(granted.slice(0, 2).map((port, i) => [SIDES[i], { backend: "gs_usb", port }]));
  rigText.value = JSON.stringify({ arms }, null, 2);
}

button("grant").onclick = async () => {
  try {
    const device = await requestAdapter(); // one chooser per adapter
    if (!device) return say(message, "No adapter chosen.");
    if (!granted.includes(adapterPort(device))) granted.push(adapterPort(device));
    $("granted").textContent = granted.join(", ");
    suggestRig();
  } catch (e) {
    say(message, errorText(e), "error");
  }
};

button("connect").onclick = () =>
  action(async () => {
    const product = $<HTMLSelectElement>("product").value === "maker" ? MAKER_ARM : METAL_ARM;
    // Rig.group() finds each arm's adapter among the granted ones by the serial the rig names.
    const rig = Rig.fromRaw(rigText.value, product, { gsUsb: { logger }, armOptions: { logger }, logger, path: "this rig" });
    const g = await rig.group(SIDES);
    await g.connect();
    group = g;
    $<HTMLSelectElement>("joint").replaceChildren(...arm("left").jointNames.map((n) => new Option(n, n)));
    $<HTMLSelectElement>("joint").value = "wrist_flex";
    leaders = {
      left: starPanel($("leader-left"), product, "left-star"),
      right: starPanel($("leader-right"), product, "right-star"),
    };
    say(message, `Connected: left on ${rig.spec("left").port}, right on ${rig.spec("right").port}.`);
  });

button("disconnect").onclick = () =>
  action(async () => {
    const g = group!;
    group = null;
    await g.disconnect();
    for (const side of SIDES) $(`${side}-state`).textContent = "not connected";
    say(message, "Disconnected.");
  });

button("enable").onclick = () => action(async () => (await group!.enable(), say(message, "Both arms holding.")));

for (const side of SIDES) {
  button(`move-${side}`).onclick = () =>
    action(async () => {
      const a = arm(side);
      const joint = $<HTMLSelectElement>("joint").value;
      const q = a.getCommandedPositions();
      q[a.jointNames.indexOf(joint)]! += Number($<HTMLInputElement>("delta").value);
      if (!(await a.moveTo(q, { speed: 0.5 }))) throw new Error(`${side}: the move did not arrive.`);
      say(message, `${side} arrived: ${joint} at ${deg(a.getJointPositions()[a.jointNames.indexOf(joint)]!)} deg.`);
    });
}

button("sine").onclick = () =>
  action(async () => {
    // Both wrists roll, in opposite phase, streamed to both arms from one loop.
    const starts = Object.fromEntries(SIDES.map((s) => [s, arm(s).getCommandedPositions()])) as Record<Side, number[]>;
    const amplitude = (10 * Math.PI) / 180;
    const period = 2;
    const clock = arm("left").clock;
    const t0 = clock.now();
    const seen: Record<Side, [number, number]> = { left: [Infinity, -Infinity], right: [Infinity, -Infinity] };
    for (let t = 0; t < 2 * period && all(ArmState.ENABLED); t = clock.now() - t0) {
      for (const [k, side] of SIDES.entries()) {
        const a = arm(side);
        const i = a.jointNames.indexOf("wrist_roll");
        const q = [...starts[side]];
        q[i] = starts[side][i]! + (k === 0 ? 1 : -1) * amplitude * Math.sin((2 * Math.PI * t) / period);
        a.setJointTargets(q);
        const m = a.getJointPositions()[i]! - starts[side][i]!;
        seen[side] = [Math.min(seen[side][0], m), Math.max(seen[side][1], m)];
      }
      await clock.sleep(0.02);
    }
    await Promise.all(SIDES.map((s) => arm(s).moveTo(starts[s], { timeout: 2 })));
    if (!all(ArmState.ENABLED)) throw new Error(`sine stopped: left ${state("left")}, right ${state("right")}`);
    const amp = (s: Side) => (seen[s][1] - seen[s][0]) / 2;
    button("sine").dataset.amplitudes = JSON.stringify({ left: amp("left"), right: amp("right") });
    say(message, `Sine done: amplitude left ${deg(amp("left"))} deg, right ${deg(amp("right"))} deg.`);
  });

button("stop").onclick = () =>
  action(async () => {
    const result = await group!.stop();
    const text = SIDES.map((s) => `${s} ${result[s] ? "at rest, released" : "holding (return not verified)"}`).join("; ");
    say(message, `Stopped: ${text}.`, SIDES.every((s) => result[s]) ? "info" : "error");
  });

button("estop").onclick = async () => {
  teleop?.abort();
  try {
    await group!.estop();
    say(message, "E-stop: torque released on both arms.");
  } catch (e) {
    say(message, errorText(e), "error");
  }
};

button("teleop-start").onclick = async () => {
  const controller = new AbortController();
  teleop = controller;
  refresh();
  try {
    for (const side of SIDES) await leaders![side].leader.connect();
    const pairs = SIDES.map((side) => new Teleop(leaders![side].leader, arm(side), { logger }));
    $("teleop").textContent = "running";
    say(message, "Bimanual teleop running.");
    // When one pair stops, for any reason, both stop and hold.
    const reasons = await runMany(pairs, controller.signal);
    const text = SIDES.map((s, i) => `${s}: ${reasons[i]}`).join("; ");
    $("teleop").textContent = `stopped (${text})`;
    say(message, `Teleop ended (${text}); both arms are holding.`, reasons.every((r) => r === "stopped") ? "info" : "error");
  } catch (e) {
    say(message, errorText(e), "error");
  }
  teleop = null;
  refresh();
};

button("teleop-stop").onclick = () => teleop?.abort();

window.addEventListener("pagehide", () => void group?.estop());
refresh();
document.body.dataset.ready = "true";
