import "../shared/mock-gate.ts";
import { type Arm, ArmLeader, ArmState, GsUsbBackend, requestAdapter, Teleop } from "@makermods/arm-core";
import { createArm as createMaker } from "@makermods/maker-arm";
import { createArm as createMetal } from "@makermods/metal-arm";
import { $, errorText, logPane, paneLogger, renderJoints, say } from "../shared/ui.ts";

const message = $("message");
const log = logPane($("log"));
const logger = paneLogger(log);
const button = (id: string) => $<HTMLButtonElement>(id);
const arms: { leader: Arm | null; follower: Arm | null } = { leader: null, follower: null };
let running: AbortController | null = null;

function refresh(): void {
  for (const role of ["leader", "follower"] as const) {
    const a = arms[role];
    button(`connect-${role}`).disabled = a !== null;
    if (!a) continue;
    const s = a.status();
    $(`${role}-state`).textContent = s.state + (s.faultReason ? ` (${s.faultReason})` : "");
    renderJoints($<HTMLTableSectionElement>(`${role}-joints`), s.joints);
  }
  const both = arms.leader !== null && arms.follower !== null;
  button("start").disabled = !both || running !== null;
  button("stop").disabled = running === null;
  button("rest").disabled = !both || running !== null;
  button("estop").disabled = !both;
}
setInterval(refresh, 100);

for (const role of ["leader", "follower"] as const) {
  button(`connect-${role}`).onclick = async () => {
    try {
      const device = await requestAdapter(); // one chooser per arm
      if (!device) return say(message, "No adapter chosen.");
      const create = $<HTMLSelectElement>("product").value === "maker" ? createMaker : createMetal;
      const arm = create(new GsUsbBackend(device, { logger }), { logger });
      await arm.connect();
      arms[role] = arm;
      $(`${role}-port`).textContent = device.serialNumber ?? "?";
      say(message, `${role} connected on ${device.serialNumber}.`);
    } catch (e) {
      say(message, errorText(e), "error");
    }
    refresh();
  };
}

button("start").onclick = async () => {
  const leaderArm = arms.leader!;
  const follower = arms.follower!;
  const controller = new AbortController();
  running = controller;
  refresh();
  try {
    const leader = new ArmLeader(leaderArm, follower);
    await leader.connect(); // enables the leader arm and floats it
    if (follower.state === ArmState.CONNECTED) await follower.enable();
    $("teleop").textContent = "running";
    say(message, "Following: move the leader by hand.");
    const reason = await new Teleop(leader, follower, {
      logger,
      onStep: (step) => {
        $("teleop").textContent = step.syncing ? "syncing" : "following";
      },
    }).run(controller.signal);
    await leader.close(); // the leader stiffens where the hand left it
    $("teleop").textContent = `stopped: ${reason}`;
    say(message, `Following ended (${reason}); both arms are holding.`, reason === "stopped" ? "info" : "error");
  } catch (e) {
    say(message, errorText(e), "error");
  }
  running = null;
  refresh();
};

button("stop").onclick = () => running?.abort();

button("rest").onclick = async () => {
  const results = await Promise.all([arms.leader!.stop(), arms.follower!.stop()]);
  say(message, results.every(Boolean) ? "Both at rest, torque released." : `Return to rest not verified (leader ${results[0]}, follower ${results[1]}).`,
    results.every(Boolean) ? "info" : "error");
};

button("estop").onclick = async () => {
  running?.abort();
  await Promise.allSettled([arms.leader?.estop(), arms.follower?.estop()]);
  say(message, "E-stop: torque released on both arms.");
};

window.addEventListener("pagehide", () => void Promise.allSettled([arms.leader?.estop(), arms.follower?.estop()]));
refresh();
document.body.dataset.ready = "true";
