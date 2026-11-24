import "../shared/mock-gate.ts";
import { type Arm, ArmState, GsUsbBackend, requestAdapter, Teleop } from "@makermods/arm-core";
import { createArm as createMaker } from "@makermods/maker-arm";
import { createArm as createMetal } from "@makermods/metal-arm";
import { starPanel, type StarPanel } from "../shared/star-panel.ts";
import { $, errorText, logPane, paneLogger, renderJoints, say } from "../shared/ui.ts";

const message = $("message");
const log = logPane($("log"));
const logger = paneLogger(log);
const ids = ["connect", "enable", "rest", "estop", "start", "stop"] as const;
const button = (id: (typeof ids)[number]) => $<HTMLButtonElement>(id);

let arm: Arm | null = null;
let panel: StarPanel | null = null;
let running: AbortController | null = null;
let busy = false;

function refreshButtons(): void {
  const state = arm?.state ?? ArmState.DISCONNECTED;
  button("connect").disabled = arm !== null;
  button("enable").disabled = state !== ArmState.CONNECTED || busy;
  button("rest").disabled = !(state === ArmState.ENABLED || state === ArmState.FAULT) || busy || running !== null;
  button("estop").disabled = arm === null;
  button("start").disabled = state !== ArmState.ENABLED || running !== null || busy;
  button("stop").disabled = running === null;
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

async function action(work: () => Promise<unknown>): Promise<void> {
  busy = true;
  refreshButtons();
  try {
    await work();
  } catch (e) {
    say(message, errorText(e), "error");
  } finally {
    busy = false;
    refreshButtons();
  }
}

button("connect").onclick = () =>
  action(async () => {
    const device = await requestAdapter();
    if (!device) return say(message, "No adapter chosen.");
    const create = $<HTMLSelectElement>("product").value === "maker" ? createMaker : createMetal;
    const candidate = create(new GsUsbBackend(device, { logger }), { logger });
    await candidate.connect();
    arm = candidate;
    panel = starPanel($("leader"), arm.config.product);
    say(message, `Connected to ${device.serialNumber}.`);
  });

button("enable").onclick = () => action(async () => (await arm!.enable(), say(message, "Holding the present pose.")));

button("start").onclick = async () => {
  const a = arm!;
  const { leader } = panel!;
  const controller = new AbortController();
  running = controller;
  refreshButtons();
  try {
    await leader.connect(); // opens the leader's port and unlocks its servos (once)
  } catch (e) {
    running = null;
    await leader.close();
    say(message, `Leader: ${errorText(e)}`, "error");
    return refreshButtons();
  }
  const capText = $<HTMLInputElement>("cap").value;
  const teleop = new Teleop(leader, a, {
    maxVelocity: capText ? Number(capText) : null,
    logger,
    onStep: (step) => {
      $("teleop").textContent = step.syncing ? "syncing" : "following";
      if (step.leader) $("targets").textContent = JSON.stringify(step.leader.map((v) => Number(v.toFixed(4))));
    },
  });
  $("teleop").textContent = "starting";
  say(message, "Teleop running.");
  const reason = await teleop.run(controller.signal);
  running = null;
  $("gap").textContent = teleop.startupGap === null ? "" : `(startup gap ${teleop.startupGap.toFixed(2)} rad)`;
  $("teleop").textContent = `stopped: ${reason}`;
  // The leader's last bus error names the servo that went quiet (Teleop's reason may only count missed reads).
  const why = reason.startsWith("leader_lost") && leader.lastError ? `${reason}; last leader error: ${leader.lastError}` : reason;
  if (reason.startsWith("leader_lost")) await leader.close(); // reconnected (pinged again) on the next Start
  say(message, reason === "stopped" ? "Teleop stopped; the follower is holding." : `Teleop ended (${why}); the follower is holding.`,
    reason === "stopped" ? "info" : "error");
  refreshButtons();
};

button("stop").onclick = () => running?.abort();

button("rest").onclick = () =>
  action(async () => {
    if (await arm!.stop()) say(message, "At rest, torque released.");
    else say(message, "Return to rest not verified: the arm keeps holding. Support it, then E-stop.", "error");
  });

button("estop").onclick = async () => {
  running?.abort();
  await arm?.estop();
  say(message, "E-stop: torque released on every motor.");
};

window.addEventListener("pagehide", () => void arm?.estop());
refreshButtons();
document.body.dataset.ready = "true";
